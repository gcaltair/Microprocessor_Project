from __future__ import annotations

import argparse
import math
import random
import sys
from dataclasses import dataclass


if "--self-test" in sys.argv:
    class _TkStub:
        Tk = object

    tk = _TkStub()  # type: ignore[assignment]
    ttk = None  # type: ignore[assignment]
else:
    import tkinter as tk
    from tkinter import ttk


OGM_MAX_WIDTH_CELLS = 96
OGM_MAX_HEIGHT_CELLS = 96
NAVIGATION_GRID_DOWNSAMPLE = 2
NAVIGATION_MAX_WIDTH_CELLS = (OGM_MAX_WIDTH_CELLS + NAVIGATION_GRID_DOWNSAMPLE - 1) // NAVIGATION_GRID_DOWNSAMPLE
NAVIGATION_MAX_HEIGHT_CELLS = (OGM_MAX_HEIGHT_CELLS + NAVIGATION_GRID_DOWNSAMPLE - 1) // NAVIGATION_GRID_DOWNSAMPLE
NAVIGATION_MAX_CELL_COUNT = NAVIGATION_MAX_WIDTH_CELLS * NAVIGATION_MAX_HEIGHT_CELLS
NAVIGATION_MAX_PATH_POINTS = 128
NAVIGATION_COST_STRAIGHT = 10
NAVIGATION_COST_INF = 65535
NAVIGATION_PARENT_NONE = 0xFF
NAVIGATION_PARENT_START = 0xFE
NAVIGATION_FLAG_OPEN = 0x01
NAVIGATION_FLAG_CLOSED = 0x02
NAVIGATION_INFLATE_RADIUS_M = 0.15
NAVIGATION_REACH_DISTANCE_M = 0.1
NAVIGATION_MIN_SEGMENT_M = 0.1
NAVIGATION_OCCUPIED_CELL_THRESHOLD = 5

OGM_UNKNOWN_LOG_ODDS = 0
OGM_FREE_LOG_ODDS = -6
OGM_OCCUPIED_LOG_ODDS = 8

DX4 = (1, -1, 0, 0)
DY4 = (0, 0, 1, -1)
REVERSE_DIR4 = (1, 0, 3, 2)


@dataclass(frozen=True)
class GridPoint:
    x: int
    y: int


@dataclass(frozen=True)
class WorldPoint:
    x_m: float
    y_m: float


@dataclass(frozen=True)
class Pose2D:
    x_m: float
    y_m: float
    theta_deg: float = 0.0


@dataclass
class NavigationResult:
    status: str
    raw_path: list[GridPoint]
    smooth_grid_path: list[GridPoint]
    smooth_world_path: list[WorldPoint]
    target_pose: Pose2D | None
    distance_to_goal_m: float
    message: str


@dataclass(frozen=True)
class OrthogonalLink:
    points: tuple[GridPoint, ...]


class NavigationPlanner:
    """Python port of Core/Src/navigation_task.c planner behavior."""

    def __init__(
        self,
        cells: list[list[int]],
        resolution_m_per_cell: float = 0.05,
        origin_x_m: float = 0.0,
        origin_y_m: float = 0.0,
    ) -> None:
        self.cells = cells
        self.height_cells = len(cells)
        self.width_cells = len(cells[0]) if cells else 0
        self.resolution_m_per_cell = resolution_m_per_cell
        self.origin_x_m = origin_x_m
        self.origin_y_m = origin_y_m
        self.nav_width_cells = (self.width_cells + NAVIGATION_GRID_DOWNSAMPLE - 1) // NAVIGATION_GRID_DOWNSAMPLE
        self.nav_height_cells = (self.height_cells + NAVIGATION_GRID_DOWNSAMPLE - 1) // NAVIGATION_GRID_DOWNSAMPLE
        self.g_score: list[int] = []
        self.flags: list[int] = []
        self.parent_dir: list[int] = []
        self.open_set: list[int] = []
        self.raw_path: list[GridPoint] = []
        self.smooth_grid_path: list[GridPoint] = []
        self.smooth_world_path: list[WorldPoint] = []
        self.inflated_blocked = self._build_inflated_blocked_map()

    def update(self, current_pose: Pose2D, goal_pose: Pose2D | None, control_busy: bool = False) -> NavigationResult:
        if goal_pose is None:
            return NavigationResult("IDLE", [], [], [], None, 0.0, "no goal")

        dx_goal = goal_pose.x_m - current_pose.x_m
        dy_goal = goal_pose.y_m - current_pose.y_m
        distance_to_goal = math.hypot(dx_goal, dy_goal)
        if distance_to_goal <= NAVIGATION_REACH_DISTANCE_M:
            return NavigationResult("REACHED", [], [], [], None, distance_to_goal, "goal reached")

        if not self._map_dimensions_valid():
            return self._failed(current_pose, goal_pose, distance_to_goal, "invalid map dimensions")

        start_cell = self.world_to_nav_cell(current_pose.x_m, current_pose.y_m)
        if start_cell is None:
            return self._failed(current_pose, goal_pose, distance_to_goal, "start is outside the map")

        goal_cell = self.world_to_nav_cell(goal_pose.x_m, goal_pose.y_m)
        if goal_cell is None:
            return self._failed(current_pose, goal_pose, distance_to_goal, "goal is outside the map")

        if not self.find_path_map(start_cell, goal_cell):
            return self._failed(current_pose, goal_pose, distance_to_goal, "A* failed; goal may be inflated-blocked")

        if not self.build_reduced_turn_path_from_raw_path():
            return self._failed(current_pose, goal_pose, distance_to_goal, "reduced-turn path build failed")

        if len(self.smooth_world_path) >= 2:
            target_pose = self.select_axis_target(
                current_pose,
                start_cell,
                self.smooth_grid_path[1],
                self.smooth_world_path[1],
            )
        else:
            target_pose = self.select_axis_goal_target(current_pose, goal_pose)

        if control_busy:
            status = "BUSY"
            message = "control layer busy; target planned but not issued"
        else:
            target_distance = math.hypot(target_pose.x_m - current_pose.x_m, target_pose.y_m - current_pose.y_m)
            status = "OK"
            if target_distance >= NAVIGATION_MIN_SEGMENT_M:
                message = f"would Start_Relative_Move(dx={target_pose.x_m - current_pose.x_m:.3f}, dy={target_pose.y_m - current_pose.y_m:.3f})"
            else:
                message = f"target segment {target_distance:.3f} m is below minimum"

        return NavigationResult(
            status,
            list(self.raw_path),
            list(self.smooth_grid_path),
            list(self.smooth_world_path),
            target_pose,
            distance_to_goal,
            message,
        )

    def world_to_nav_cell(self, x_m: float, y_m: float) -> GridPoint | None:
        map_x = int((x_m - self.origin_x_m) / self.resolution_m_per_cell)
        map_y = int((y_m - self.origin_y_m) / self.resolution_m_per_cell)
        if not (0 <= map_x < self.width_cells and 0 <= map_y < self.height_cells):
            return None
        cell = GridPoint(map_x // NAVIGATION_GRID_DOWNSAMPLE, map_y // NAVIGATION_GRID_DOWNSAMPLE)
        return cell if self.is_inside(cell.x, cell.y) else None

    def nav_cell_to_world(self, cell: GridPoint) -> WorldPoint:
        map_cell_center_x = cell.x * NAVIGATION_GRID_DOWNSAMPLE + NAVIGATION_GRID_DOWNSAMPLE * 0.5
        map_cell_center_y = cell.y * NAVIGATION_GRID_DOWNSAMPLE + NAVIGATION_GRID_DOWNSAMPLE * 0.5
        return WorldPoint(
            self.origin_x_m + map_cell_center_x * self.resolution_m_per_cell,
            self.origin_y_m + map_cell_center_y * self.resolution_m_per_cell,
        )

    def find_path_map(self, start_cell: GridPoint, goal_cell: GridPoint) -> bool:
        if not self.is_inside(start_cell.x, start_cell.y) or not self.is_inside(goal_cell.x, goal_cell.y):
            return False
        if self.is_blocked_inflated(goal_cell.x, goal_cell.y, start_cell):
            return False

        cell_count = self.nav_width_cells * self.nav_height_cells
        start_index = self.cell_index(start_cell.x, start_cell.y)
        goal_index = self.cell_index(goal_cell.x, goal_cell.y)
        self.astar_reset(cell_count)

        if start_index == goal_index:
            self.raw_path = [start_cell]
            return True

        self.g_score[start_index] = 0
        self.parent_dir[start_index] = NAVIGATION_PARENT_START
        self.open_push(start_index)

        while self.open_set:
            current_index = self.open_pop_best(goal_cell)
            if current_index == goal_index:
                return self.build_raw_path(goal_index, start_index)

            current_cell = GridPoint(current_index % self.nav_width_cells, current_index // self.nav_width_cells)
            for direction in range(4):
                next_x = current_cell.x + DX4[direction]
                next_y = current_cell.y + DY4[direction]
                if not self.is_inside(next_x, next_y) or self.is_blocked_inflated(next_x, next_y, start_cell):
                    continue

                next_index = self.cell_index(next_x, next_y)
                if self.flags[next_index] & NAVIGATION_FLAG_CLOSED:
                    continue

                tentative_g = self.g_score[current_index] + NAVIGATION_COST_STRAIGHT
                if not (self.flags[next_index] & NAVIGATION_FLAG_OPEN) or tentative_g < self.g_score[next_index]:
                    self.g_score[next_index] = tentative_g
                    self.parent_dir[next_index] = REVERSE_DIR4[direction]
                    if not (self.flags[next_index] & NAVIGATION_FLAG_OPEN):
                        self.open_push(next_index)

        return False

    def build_reduced_turn_path_from_raw_path(self) -> bool:
        if not self.raw_path:
            return False

        raw_index = 0
        self.smooth_grid_path = [self.raw_path[0]]
        self.smooth_world_path = [self.nav_cell_to_world(self.raw_path[0])]

        while raw_index < len(self.raw_path) - 1:
            best_next = raw_index + 1
            best_link = OrthogonalLink((self.raw_path[best_next],))
            preferred_next = self.raw_path[raw_index + 1]
            for candidate in range(len(self.raw_path) - 1, raw_index, -1):
                link = self.orthogonal_link_free(
                    self.raw_path[raw_index],
                    self.raw_path[candidate],
                    preferred_next,
                )
                if link is not None:
                    best_next = candidate
                    best_link = link
                    break

            for point in best_link.points:
                if self.smooth_grid_path[-1] == point:
                    continue
                if len(self.smooth_grid_path) >= NAVIGATION_MAX_PATH_POINTS:
                    return False
                self.smooth_grid_path.append(point)
                self.smooth_world_path.append(self.nav_cell_to_world(point))

            raw_index = best_next

        return self.path_is_axis_aligned(self.smooth_grid_path)

    def orthogonal_link_free(
        self,
        start_cell: GridPoint,
        end_cell: GridPoint,
        preferred_next: GridPoint | None = None,
    ) -> OrthogonalLink | None:
        if self.axis_aligned_segment_free(start_cell, end_cell):
            return OrthogonalLink((end_cell,))

        corners = [
            GridPoint(start_cell.x, end_cell.y),
            GridPoint(end_cell.x, start_cell.y),
        ]
        if preferred_next is not None and preferred_next.y == start_cell.y:
            corners = [corners[1], corners[0]]
        elif preferred_next is not None and preferred_next.x == start_cell.x:
            corners = [corners[0], corners[1]]

        for corner in corners:
            if corner == start_cell or corner == end_cell:
                continue
            if not self.is_inside(corner.x, corner.y):
                continue
            if self.axis_aligned_segment_free(start_cell, corner) and self.axis_aligned_segment_free(corner, end_cell):
                return OrthogonalLink((corner, end_cell))

        return None

    def axis_aligned_segment_free(self, start_cell: GridPoint, end_cell: GridPoint) -> bool:
        if start_cell.x != end_cell.x and start_cell.y != end_cell.y:
            return False

        return self.line_free_map(start_cell, end_cell)

    @staticmethod
    def select_axis_target(
        current_pose: Pose2D,
        start_cell: GridPoint,
        target_cell: GridPoint,
        target_world: WorldPoint,
    ) -> Pose2D:
        if target_cell.x != start_cell.x and target_cell.y == start_cell.y:
            return Pose2D(target_world.x_m, current_pose.y_m, current_pose.theta_deg)
        if target_cell.y != start_cell.y and target_cell.x == start_cell.x:
            return Pose2D(current_pose.x_m, target_world.y_m, current_pose.theta_deg)
        if target_cell == start_cell:
            return Pose2D(current_pose.x_m, current_pose.y_m, current_pose.theta_deg)

        dx = abs(target_world.x_m - current_pose.x_m)
        dy = abs(target_world.y_m - current_pose.y_m)
        if dx >= dy:
            return Pose2D(target_world.x_m, current_pose.y_m, current_pose.theta_deg)
        return Pose2D(current_pose.x_m, target_world.y_m, current_pose.theta_deg)

    @staticmethod
    def select_axis_goal_target(current_pose: Pose2D, goal_pose: Pose2D) -> Pose2D:
        dx = abs(goal_pose.x_m - current_pose.x_m)
        dy = abs(goal_pose.y_m - current_pose.y_m)
        if dx >= dy:
            return Pose2D(goal_pose.x_m, current_pose.y_m, current_pose.theta_deg)
        return Pose2D(current_pose.x_m, goal_pose.y_m, current_pose.theta_deg)

    def is_inside(self, x: int, y: int) -> bool:
        return 0 <= x < self.nav_width_cells and 0 <= y < self.nav_height_cells

    def _inflate_radius_map_cells(self) -> int:
        if self.resolution_m_per_cell <= 0.0:
            return 0
        return math.ceil(NAVIGATION_INFLATE_RADIUS_M / self.resolution_m_per_cell)

    def _build_inflated_blocked_map(self) -> list[list[bool]]:
        inflated = [[False for _ in range(self.width_cells)] for _ in range(self.height_cells)]
        radius = self._inflate_radius_map_cells()
        for map_y, row in enumerate(self.cells):
            for map_x, value in enumerate(row):
                if value < NAVIGATION_OCCUPIED_CELL_THRESHOLD:
                    continue
                for dy in range(-radius, radius + 1):
                    for dx in range(-radius, radius + 1):
                        if dx * dx + dy * dy > radius * radius:
                            continue
                        inflated_x = map_x + dx
                        inflated_y = map_y + dy
                        if 0 <= inflated_x < self.width_cells and 0 <= inflated_y < self.height_cells:
                            inflated[inflated_y][inflated_x] = True
        return inflated

    @staticmethod
    def nav_cell_to_map_center_coord(nav_coord: int) -> int:
        return nav_coord * NAVIGATION_GRID_DOWNSAMPLE + NAVIGATION_GRID_DOWNSAMPLE // 2

    def map_cell_blocked_for_line(self, map_x: int, map_y: int, start_map_x: int, start_map_y: int) -> bool:
        if map_x == start_map_x and map_y == start_map_y:
            return False
        return not self.map_cell_known_free(map_x, map_y)

    def line_free_map(self, start_cell: GridPoint, end_cell: GridPoint) -> bool:
        start_map_x = self.nav_cell_to_map_center_coord(start_cell.x)
        start_map_y = self.nav_cell_to_map_center_coord(start_cell.y)
        end_map_x = self.nav_cell_to_map_center_coord(end_cell.x)
        end_map_y = self.nav_cell_to_map_center_coord(end_cell.y)

        dx = abs(end_map_x - start_map_x)
        dy = abs(end_map_y - start_map_y)
        step_x = 1 if start_map_x < end_map_x else -1
        step_y = 1 if start_map_y < end_map_y else -1
        err = dx - dy
        x = start_map_x
        y = start_map_y

        while True:
            if self.map_cell_blocked_for_line(x, y, start_map_x, start_map_y):
                return False
            if x == end_map_x and y == end_map_y:
                return True

            prev_x = x
            prev_y = y
            moved_x = False
            moved_y = False
            twice_err = 2 * err
            if twice_err > -dy:
                err -= dy
                x += step_x
                moved_x = True
            if twice_err < dx:
                err += dx
                y += step_y
                moved_y = True
            if moved_x and moved_y:
                if self.map_cell_blocked_for_line(x, prev_y, start_map_x, start_map_y):
                    return False
                if self.map_cell_blocked_for_line(prev_x, y, start_map_x, start_map_y):
                    return False

    def map_cell_known_free(self, map_x: int, map_y: int) -> bool:
        if map_x < 0 or map_y < 0 or map_x >= self.width_cells or map_y >= self.height_cells:
            return False
        if self.inflated_blocked[map_y][map_x]:
            return False
        return self.cells[map_y][map_x] < NAVIGATION_OCCUPIED_CELL_THRESHOLD

    def nav_cell_known_free(self, nav_x: int, nav_y: int) -> bool:
        if not self.is_inside(nav_x, nav_y):
            return False
        map_x_start = nav_x * NAVIGATION_GRID_DOWNSAMPLE
        map_y_start = nav_y * NAVIGATION_GRID_DOWNSAMPLE
        for map_y in range(map_y_start, map_y_start + NAVIGATION_GRID_DOWNSAMPLE):
            for map_x in range(map_x_start, map_x_start + NAVIGATION_GRID_DOWNSAMPLE):
                if not self.map_cell_known_free(map_x, map_y):
                    return False
        return True

    def is_blocked_inflated(self, nav_x: int, nav_y: int, start_cell: GridPoint | None) -> bool:
        if start_cell is not None and nav_x == start_cell.x and nav_y == start_cell.y:
            return False
        return not self.nav_cell_known_free(nav_x, nav_y)

    def build_blocked_overlay(self, start_cell: GridPoint | None) -> list[list[bool]]:
        return [
            [self.is_blocked_inflated(x, y, start_cell) for x in range(self.nav_width_cells)]
            for y in range(self.nav_height_cells)
        ]

    def astar_reset(self, cell_count: int) -> None:
        self.g_score = [NAVIGATION_COST_INF] * cell_count
        self.flags = [0] * cell_count
        self.parent_dir = [NAVIGATION_PARENT_NONE] * cell_count
        self.open_set = []
        self.raw_path = []
        self.smooth_grid_path = []
        self.smooth_world_path = []

    def open_push(self, index: int) -> None:
        if len(self.open_set) < NAVIGATION_MAX_CELL_COUNT:
            self.open_set.append(index)
            self.flags[index] |= NAVIGATION_FLAG_OPEN

    def open_pop_best(self, goal: GridPoint) -> int:
        best_open_pos = 0
        best_index = self.open_set[0]
        best_f = NAVIGATION_COST_INF
        for open_pos, index in enumerate(self.open_set):
            cell = GridPoint(index % self.nav_width_cells, index // self.nav_width_cells)
            f_score = self.g_score[index] + self.heuristic(cell, goal)
            if f_score < best_f:
                best_f = f_score
                best_index = index
                best_open_pos = open_pos

        self.open_set[best_open_pos] = self.open_set[-1]
        self.open_set.pop()
        self.flags[best_index] &= ~NAVIGATION_FLAG_OPEN
        self.flags[best_index] |= NAVIGATION_FLAG_CLOSED
        return best_index

    def build_raw_path(self, goal_index: int, start_index: int) -> bool:
        reversed_path: list[GridPoint] = []
        current_index = goal_index
        while len(reversed_path) < NAVIGATION_MAX_PATH_POINTS:
            current = GridPoint(current_index % self.nav_width_cells, current_index // self.nav_width_cells)
            reversed_path.append(current)
            if current_index == start_index:
                break

            parent_dir = self.parent_dir[current_index]
            if parent_dir in (NAVIGATION_PARENT_NONE, NAVIGATION_PARENT_START):
                return False
            current_index = self.cell_index(current.x + DX4[parent_dir], current.y + DY4[parent_dir])

        if not reversed_path or reversed_path[-1] != GridPoint(start_index % self.nav_width_cells, start_index // self.nav_width_cells):
            return False

        forward_path = list(reversed(reversed_path))
        self.raw_path = [forward_path[0]]
        last_forward_dir: int | None = None
        previous = forward_path[0]

        for current in forward_path[1:]:
            forward_dir = self.direction_between(previous, current)
            if forward_dir is None:
                return False

            if last_forward_dir is not None and forward_dir != last_forward_dir:
                if self.raw_path[-1] != previous:
                    if len(self.raw_path) >= NAVIGATION_MAX_PATH_POINTS:
                        return False
                    self.raw_path.append(previous)

            last_forward_dir = forward_dir
            previous = current

        if self.raw_path[-1] != forward_path[-1]:
            if len(self.raw_path) >= NAVIGATION_MAX_PATH_POINTS:
                return False
            self.raw_path.append(forward_path[-1])

        return self.path_is_axis_aligned(self.raw_path)

    def cell_index(self, x: int, y: int) -> int:
        return y * self.nav_width_cells + x

    @staticmethod
    def direction_between(start: GridPoint, end: GridPoint) -> int | None:
        if end.x == start.x + 1 and end.y == start.y:
            return 0
        if end.x == start.x - 1 and end.y == start.y:
            return 1
        if end.x == start.x and end.y == start.y + 1:
            return 2
        if end.x == start.x and end.y == start.y - 1:
            return 3
        return None

    @staticmethod
    def path_is_axis_aligned(path: list[GridPoint]) -> bool:
        return all(start.x == end.x or start.y == end.y for start, end in zip(path, path[1:]))

    @staticmethod
    def heuristic(cell: GridPoint, goal: GridPoint) -> int:
        return (abs(cell.x - goal.x) + abs(cell.y - goal.y)) * NAVIGATION_COST_STRAIGHT

    def _map_dimensions_valid(self) -> bool:
        if self.width_cells <= 0 or self.height_cells <= 0:
            return False
        if self.width_cells > OGM_MAX_WIDTH_CELLS or self.height_cells > OGM_MAX_HEIGHT_CELLS:
            return False
        return self.nav_width_cells <= NAVIGATION_MAX_WIDTH_CELLS and self.nav_height_cells <= NAVIGATION_MAX_HEIGHT_CELLS

    def _failed(self, current_pose: Pose2D, goal_pose: Pose2D, distance_to_goal: float, message: str) -> NavigationResult:
        return NavigationResult("FAILED", [], [], [], None, distance_to_goal, message)


class NavigationSimulatorApp(tk.Tk):
    def __init__(self, width: int, height: int, resolution: float) -> None:
        super().__init__()
        self.title("navigation_task.c Simulator")
        self.geometry("1180x780")
        self.minsize(980, 660)

        self.resolution = resolution
        self.origin_x_m = 0.0
        self.origin_y_m = 0.0
        self.cells = [[OGM_UNKNOWN_LOG_ODDS for _ in range(width)] for _ in range(height)]
        self.robot_pose = Pose2D(0.35, 0.35, 0.0)
        self.goal_pose = Pose2D((width - 8) * resolution, (height - 8) * resolution, 0.0)
        self.paint_value = tk.StringVar(value="occupied")
        self.click_mode = tk.StringVar(value="paint")
        self.brush_radius = tk.IntVar(value=1)
        self.busy_var = tk.BooleanVar(value=False)
        self.show_nav_grid = tk.BooleanVar(value=True)
        self.show_inflation = tk.BooleanVar(value=True)
        self.status_text = tk.StringVar(value="Ready")
        self.run_after_id: str | None = None
        self.last_result: NavigationResult | None = None
        self.cell_px = 8
        self.margin = 20

        self._build_demo_map()
        self._build_ui()
        self.plan_and_draw()

    def _build_ui(self) -> None:
        self.configure(bg="#101419")

        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(root, bg="#11161c", highlightthickness=1, highlightbackground="#31404f")
        self.canvas.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        self.canvas.bind("<Button-1>", self._handle_canvas_click)
        self.canvas.bind("<B1-Motion>", self._handle_canvas_drag)
        self.canvas.bind("<Button-3>", self._handle_canvas_right_click)
        self.canvas.bind("<Configure>", lambda _event: self.redraw())

        panel = ttk.Frame(root, width=330)
        panel.grid(row=0, column=1, sticky="nsew")
        panel.columnconfigure(0, weight=1)

        controls = ttk.LabelFrame(panel, text="Controls", padding=10)
        controls.grid(row=0, column=0, sticky="ew")

        ttk.Label(controls, text="Click mode").grid(row=0, column=0, sticky="w")
        mode_row = ttk.Frame(controls)
        mode_row.grid(row=1, column=0, sticky="ew", pady=(2, 8))
        for text, value in (("Paint", "paint"), ("Start", "start"), ("Goal", "goal")):
            ttk.Radiobutton(mode_row, text=text, value=value, variable=self.click_mode).pack(side=tk.LEFT, padx=(0, 8))

        ttk.Label(controls, text="Paint value").grid(row=2, column=0, sticky="w")
        paint_row = ttk.Frame(controls)
        paint_row.grid(row=3, column=0, sticky="ew", pady=(2, 8))
        for text, value in (("Obstacle", "occupied"), ("Free", "free"), ("Unknown", "unknown")):
            ttk.Radiobutton(paint_row, text=text, value=value, variable=self.paint_value).pack(side=tk.LEFT, padx=(0, 8))

        ttk.Label(controls, text="Brush radius").grid(row=4, column=0, sticky="w")
        ttk.Scale(controls, from_=0, to=5, orient=tk.HORIZONTAL, variable=self.brush_radius).grid(row=5, column=0, sticky="ew")

        button_grid = ttk.Frame(controls)
        button_grid.grid(row=6, column=0, sticky="ew", pady=(10, 0))
        for col in range(2):
            button_grid.columnconfigure(col, weight=1)
        ttk.Button(button_grid, text="Plan", command=self.plan_and_draw).grid(row=0, column=0, sticky="ew", padx=(0, 4), pady=3)
        ttk.Button(button_grid, text="Step", command=self.step_robot).grid(row=0, column=1, sticky="ew", padx=(4, 0), pady=3)
        ttk.Button(button_grid, text="Run", command=self.start_running).grid(row=1, column=0, sticky="ew", padx=(0, 4), pady=3)
        ttk.Button(button_grid, text="Stop", command=self.stop_running).grid(row=1, column=1, sticky="ew", padx=(4, 0), pady=3)
        ttk.Button(button_grid, text="Demo Map", command=self.load_demo_map).grid(row=2, column=0, sticky="ew", padx=(0, 4), pady=3)
        ttk.Button(button_grid, text="Random Map", command=self.load_random_map).grid(row=2, column=1, sticky="ew", padx=(4, 0), pady=3)
        ttk.Button(button_grid, text="Clear Map", command=self.clear_map).grid(row=3, column=0, sticky="ew", padx=(0, 4), pady=3)
        ttk.Button(button_grid, text="Free All", command=self.free_all).grid(row=3, column=1, sticky="ew", padx=(4, 0), pady=3)

        ttk.Checkbutton(controls, text="Control busy", variable=self.busy_var, command=self.plan_and_draw).grid(row=7, column=0, sticky="w", pady=(8, 0))
        ttk.Checkbutton(controls, text="Show 2x nav grid", variable=self.show_nav_grid, command=self.redraw).grid(row=8, column=0, sticky="w")
        ttk.Checkbutton(controls, text="Show inflated blocked cells", variable=self.show_inflation, command=self.redraw).grid(row=9, column=0, sticky="w")

        pose_box = ttk.LabelFrame(panel, text="Pose / Goal", padding=10)
        pose_box.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        self.pose_label = ttk.Label(pose_box, text="", justify=tk.LEFT)
        self.pose_label.grid(row=0, column=0, sticky="w")

        info_box = ttk.LabelFrame(panel, text="NavigationTask_Update", padding=10)
        info_box.grid(row=2, column=0, sticky="nsew", pady=(10, 0))
        panel.rowconfigure(2, weight=1)
        self.info = tk.Text(info_box, height=20, width=42, wrap=tk.WORD, bg="#0e1216", fg="#dce3ea", insertbackground="#dce3ea")
        self.info.pack(fill=tk.BOTH, expand=True)

        status = ttk.Label(panel, textvariable=self.status_text, wraplength=320)
        status.grid(row=3, column=0, sticky="ew", pady=(10, 0))

        legend = ttk.Label(
            panel,
            text="Left click uses selected mode. Right click sets goal. Grid cells are original OGM cells; local targets only move along x or y.",
            wraplength=320,
        )
        legend.grid(row=4, column=0, sticky="ew", pady=(8, 0))

    def plan_and_draw(self) -> None:
        planner = NavigationPlanner(self.cells, self.resolution, self.origin_x_m, self.origin_y_m)
        self.last_result = planner.update(self.robot_pose, self.goal_pose, self.busy_var.get())
        self._update_info(planner)
        self.redraw()

    def step_robot(self) -> None:
        self.plan_and_draw()
        if self.last_result is None:
            return
        if self.last_result.status == "REACHED":
            self.status_text.set("Reached goal")
            self.stop_running()
            return
        if self.last_result.status not in ("OK", "BUSY") or self.last_result.target_pose is None:
            self.status_text.set(self.last_result.message)
            return
        if self.busy_var.get():
            self.status_text.set("Control busy is enabled; robot will not move")
            return

        target = self.last_result.target_pose
        self.robot_pose = Pose2D(target.x_m, target.y_m, self.robot_pose.theta_deg)
        self.status_text.set(f"Moved to local target ({target.x_m:.3f}, {target.y_m:.3f})")
        self.plan_and_draw()

    def start_running(self) -> None:
        if self.run_after_id is not None:
            return
        self._run_tick()

    def stop_running(self) -> None:
        if self.run_after_id is not None:
            self.after_cancel(self.run_after_id)
            self.run_after_id = None

    def _run_tick(self) -> None:
        self.run_after_id = None
        self.step_robot()
        if self.last_result is not None and self.last_result.status not in ("REACHED", "FAILED") and not self.busy_var.get():
            self.run_after_id = self.after(450, self._run_tick)

    def redraw(self) -> None:
        self.canvas.delete("all")
        width = len(self.cells[0])
        height = len(self.cells)
        canvas_w = max(1, self.canvas.winfo_width() - self.margin * 2)
        canvas_h = max(1, self.canvas.winfo_height() - self.margin * 2)
        self.cell_px = max(4, int(min(canvas_w / width, canvas_h / height)))
        map_w = self.cell_px * width
        map_h = self.cell_px * height
        left = (self.canvas.winfo_width() - map_w) // 2
        top = (self.canvas.winfo_height() - map_h) // 2
        self.map_left = left
        self.map_top = top

        for y, row in enumerate(self.cells):
            for x, value in enumerate(row):
                fill = self._cell_color(value)
                sx = left + x * self.cell_px
                sy = top + (height - 1 - y) * self.cell_px
                self.canvas.create_rectangle(sx, sy, sx + self.cell_px, sy + self.cell_px, fill=fill, outline=fill)

        planner = NavigationPlanner(self.cells, self.resolution, self.origin_x_m, self.origin_y_m)
        start_cell = planner.world_to_nav_cell(self.robot_pose.x_m, self.robot_pose.y_m)
        if self.show_inflation.get():
            for map_y, row in enumerate(planner.inflated_blocked):
                for map_x, is_blocked in enumerate(row):
                    if is_blocked:
                        self._draw_map_cell(map_x, map_y, "#d95f5f", stipple="gray25")

        if self.show_nav_grid.get():
            self._draw_nav_grid(planner.nav_width_cells, planner.nav_height_cells)

        if self.last_result is not None:
            self._draw_grid_path(self.last_result.raw_path, "#f6c343", width=2, dash=(5, 3))
            self._draw_grid_path(self.last_result.smooth_grid_path, "#2f80ed", width=4)
            if self.last_result.target_pose is not None:
                self._draw_world_cross(self.last_result.target_pose.x_m, self.last_result.target_pose.y_m, "#2dd4bf", 7)

        self._draw_world_cross(self.goal_pose.x_m, self.goal_pose.y_m, "#3b82f6", 9)
        self._draw_robot()

    def _update_info(self, planner: NavigationPlanner) -> None:
        result = self.last_result
        if result is None:
            return
        start_cell = planner.world_to_nav_cell(self.robot_pose.x_m, self.robot_pose.y_m)
        goal_cell = planner.world_to_nav_cell(self.goal_pose.x_m, self.goal_pose.y_m)
        target_text = "none"
        if result.target_pose is not None:
            target_text = f"({result.target_pose.x_m:.3f}, {result.target_pose.y_m:.3f})"

        lines = [
            f"status              : {result.status}",
            f"message             : {result.message}",
            f"distance_to_goal_m  : {result.distance_to_goal_m:.3f}",
            f"map_cells           : {planner.width_cells}x{planner.height_cells}",
            f"nav_cells           : {planner.nav_width_cells}x{planner.nav_height_cells}",
            f"downsample          : {NAVIGATION_GRID_DOWNSAMPLE}x{NAVIGATION_GRID_DOWNSAMPLE}",
            "motion_model        : x-only/y-only segments",
            f"occupied_threshold  : >= {NAVIGATION_OCCUPIED_CELL_THRESHOLD}",
            f"inflate_radius_m    : {NAVIGATION_INFLATE_RADIUS_M:.2f}",
            f"start_nav_cell      : {self._point_text(start_cell)}",
            f"goal_nav_cell       : {self._point_text(goal_cell)}",
            f"raw_path_len        : {len(result.raw_path)}",
            f"reduced_turn_len    : {len(result.smooth_grid_path)}",
            f"target              : {target_text}",
            "",
            "raw path:",
            " -> ".join(self._point_text(p) for p in result.raw_path) or "none",
            "",
            "reduced-turn path:",
            " -> ".join(self._point_text(p) for p in result.smooth_grid_path) or "none",
        ]
        self.info.delete("1.0", tk.END)
        self.info.insert(tk.END, "\n".join(lines))
        self.pose_label.configure(
            text=(
                f"robot: ({self.robot_pose.x_m:.3f}, {self.robot_pose.y_m:.3f}) m\n"
                f"goal : ({self.goal_pose.x_m:.3f}, {self.goal_pose.y_m:.3f}) m"
            )
        )

    def load_demo_map(self) -> None:
        self.stop_running()
        self.cells = [[OGM_UNKNOWN_LOG_ODDS for _ in row] for row in self.cells]
        self._build_demo_map()
        self.robot_pose = Pose2D(0.35, 0.35, 0.0)
        self.goal_pose = Pose2D((len(self.cells[0]) - 8) * self.resolution, (len(self.cells) - 8) * self.resolution, 0.0)
        self.plan_and_draw()

    def load_random_map(self) -> None:
        self.stop_running()
        height = len(self.cells)
        width = len(self.cells[0])
        self.cells = [[OGM_FREE_LOG_ODDS for _ in range(width)] for _ in range(height)]
        random.seed()
        for _ in range(14):
            x0 = random.randint(6, max(6, width - 16))
            y0 = random.randint(6, max(6, height - 16))
            length = random.randint(10, 30)
            horizontal = random.choice((True, False))
            gap = random.randint(2, max(3, length - 4))
            for i in range(length):
                if gap <= i < gap + 4:
                    continue
                x = x0 + i if horizontal else x0
                y = y0 if horizontal else y0 + i
                if 0 <= x < width and 0 <= y < height:
                    self._paint_cell_block(x, y, OGM_OCCUPIED_LOG_ODDS, radius=1)
        self.robot_pose = Pose2D(0.35, 0.35, 0.0)
        self.goal_pose = Pose2D((width - 8) * self.resolution, (height - 8) * self.resolution, 0.0)
        self.plan_and_draw()

    def clear_map(self) -> None:
        self.stop_running()
        width = len(self.cells[0])
        height = len(self.cells)
        self.cells = [[OGM_UNKNOWN_LOG_ODDS for _ in range(width)] for _ in range(height)]
        self.plan_and_draw()

    def free_all(self) -> None:
        self.stop_running()
        width = len(self.cells[0])
        height = len(self.cells)
        self.cells = [[OGM_FREE_LOG_ODDS for _ in range(width)] for _ in range(height)]
        self.plan_and_draw()

    def _build_demo_map(self) -> None:
        height = len(self.cells)
        width = len(self.cells[0])
        for y in range(height):
            for x in range(width):
                self.cells[y][x] = OGM_FREE_LOG_ODDS
        for x in range(width):
            self.cells[0][x] = OGM_OCCUPIED_LOG_ODDS
            self.cells[height - 1][x] = OGM_OCCUPIED_LOG_ODDS
        for y in range(height):
            self.cells[y][0] = OGM_OCCUPIED_LOG_ODDS
            self.cells[y][width - 1] = OGM_OCCUPIED_LOG_ODDS
        for x in range(12, width - 12):
            if not (30 <= x <= 38):
                self._paint_cell_block(x, height // 2, OGM_OCCUPIED_LOG_ODDS, 1)
        for y in range(12, height - 12):
            if not (36 <= y <= 44):
                self._paint_cell_block(width // 2, y, OGM_OCCUPIED_LOG_ODDS, 1)
        for x in range(20, 32):
            for y in range(18, 28):
                self.cells[y][x] = OGM_UNKNOWN_LOG_ODDS

    def _handle_canvas_click(self, event: tk.Event) -> None:
        self._apply_canvas_action(event.x, event.y)

    def _handle_canvas_drag(self, event: tk.Event) -> None:
        if self.click_mode.get() == "paint":
            self._apply_canvas_action(event.x, event.y)

    def _handle_canvas_right_click(self, event: tk.Event) -> None:
        cell = self._canvas_to_cell(event.x, event.y)
        if cell is None:
            return
        self.goal_pose = self._cell_center_pose(cell.x, cell.y)
        self.plan_and_draw()

    def _apply_canvas_action(self, sx: int, sy: int) -> None:
        cell = self._canvas_to_cell(sx, sy)
        if cell is None:
            return
        mode = self.click_mode.get()
        if mode == "start":
            self.robot_pose = self._cell_center_pose(cell.x, cell.y)
        elif mode == "goal":
            self.goal_pose = self._cell_center_pose(cell.x, cell.y)
        else:
            value = {
                "occupied": OGM_OCCUPIED_LOG_ODDS,
                "free": OGM_FREE_LOG_ODDS,
                "unknown": OGM_UNKNOWN_LOG_ODDS,
            }[self.paint_value.get()]
            self._paint_cell_block(cell.x, cell.y, value, self.brush_radius.get())
        self.plan_and_draw()

    def _canvas_to_cell(self, sx: int, sy: int) -> GridPoint | None:
        width = len(self.cells[0])
        height = len(self.cells)
        x = (sx - self.map_left) // self.cell_px
        y_screen = (sy - self.map_top) // self.cell_px
        y = height - 1 - y_screen
        if 0 <= x < width and 0 <= y < height:
            return GridPoint(int(x), int(y))
        return None

    def _cell_center_pose(self, x: int, y: int) -> Pose2D:
        return Pose2D(
            self.origin_x_m + (x + 0.5) * self.resolution,
            self.origin_y_m + (y + 0.5) * self.resolution,
            0.0,
        )

    def _paint_cell_block(self, center_x: int, center_y: int, value: int, radius: int) -> None:
        width = len(self.cells[0])
        height = len(self.cells)
        for y in range(center_y - radius, center_y + radius + 1):
            for x in range(center_x - radius, center_x + radius + 1):
                if 0 <= x < width and 0 <= y < height:
                    self.cells[y][x] = value

    def _draw_map_cell(self, map_x: int, map_y: int, fill: str, stipple: str = "") -> None:
        x0 = self.map_left + map_x * self.cell_px
        y0 = self.map_top + (len(self.cells) - 1 - map_y) * self.cell_px
        self.canvas.create_rectangle(x0, y0, x0 + self.cell_px, y0 + self.cell_px, fill=fill, outline="", stipple=stipple)

    def _draw_nav_cell(self, nav_x: int, nav_y: int, fill: str, stipple: str = "") -> None:
        x0 = self.map_left + nav_x * NAVIGATION_GRID_DOWNSAMPLE * self.cell_px
        y0 = self.map_top + (len(self.cells) - (nav_y + 1) * NAVIGATION_GRID_DOWNSAMPLE) * self.cell_px
        x1 = x0 + NAVIGATION_GRID_DOWNSAMPLE * self.cell_px
        y1 = y0 + NAVIGATION_GRID_DOWNSAMPLE * self.cell_px
        self.canvas.create_rectangle(x0, y0, x1, y1, fill=fill, outline="", stipple=stipple)

    def _draw_nav_grid(self, nav_width: int, nav_height: int) -> None:
        map_h = len(self.cells) * self.cell_px
        map_w = len(self.cells[0]) * self.cell_px
        step = NAVIGATION_GRID_DOWNSAMPLE * self.cell_px
        for i in range(nav_width + 1):
            x = self.map_left + i * step
            self.canvas.create_line(x, self.map_top, x, self.map_top + map_h, fill="#283747")
        for i in range(nav_height + 1):
            y = self.map_top + i * step
            self.canvas.create_line(self.map_left, y, self.map_left + map_w, y, fill="#283747")

    def _draw_grid_path(self, path: list[GridPoint], color: str, width: int, dash: tuple[int, int] | None = None) -> None:
        if len(path) < 2:
            return
        points: list[float] = []
        for point in path:
            sx, sy = self._nav_cell_screen_center(point)
            points.extend([sx, sy])
        self.canvas.create_line(*points, fill=color, width=width, dash=dash, capstyle=tk.ROUND, joinstyle=tk.ROUND)

    def _draw_world_cross(self, x_m: float, y_m: float, color: str, radius: int) -> None:
        sx, sy = self._world_to_screen(x_m, y_m)
        self.canvas.create_oval(sx - radius, sy - radius, sx + radius, sy + radius, outline=color, width=2)
        self.canvas.create_line(sx - radius - 3, sy, sx + radius + 3, sy, fill=color, width=2)
        self.canvas.create_line(sx, sy - radius - 3, sx, sy + radius + 3, fill=color, width=2)

    def _draw_robot(self) -> None:
        sx, sy = self._world_to_screen(self.robot_pose.x_m, self.robot_pose.y_m)
        radius = 8
        self.canvas.create_oval(sx - radius, sy - radius, sx + radius, sy + radius, fill="#f25f5c", outline="#ffffff", width=1)
        angle = math.radians(self.robot_pose.theta_deg)
        self.canvas.create_line(sx, sy, sx + math.cos(angle) * 18, sy - math.sin(angle) * 18, fill="#ffffff", width=2)

    def _nav_cell_screen_center(self, point: GridPoint) -> tuple[float, float]:
        map_x = point.x * NAVIGATION_GRID_DOWNSAMPLE + NAVIGATION_GRID_DOWNSAMPLE * 0.5
        map_y = point.y * NAVIGATION_GRID_DOWNSAMPLE + NAVIGATION_GRID_DOWNSAMPLE * 0.5
        sx = self.map_left + map_x * self.cell_px
        sy = self.map_top + (len(self.cells) - map_y) * self.cell_px
        return sx, sy

    def _world_to_screen(self, x_m: float, y_m: float) -> tuple[float, float]:
        cell_x = (x_m - self.origin_x_m) / self.resolution
        cell_y = (y_m - self.origin_y_m) / self.resolution
        sx = self.map_left + cell_x * self.cell_px
        sy = self.map_top + (len(self.cells) - cell_y) * self.cell_px
        return sx, sy

    @staticmethod
    def _cell_color(value: int) -> str:
        if value >= NAVIGATION_OCCUPIED_CELL_THRESHOLD:
            return "#1f2933"
        if value <= -3:
            return "#edf2f7"
        if value < 0:
            return "#cbd5df"
        if value > 0:
            return "#708090"
        return "#8b98a8"

    @staticmethod
    def _point_text(point: GridPoint | None) -> str:
        if point is None:
            return "none"
        return f"({point.x},{point.y})"


def run_self_test() -> None:
    width = 40
    height = 40
    cells = [[OGM_FREE_LOG_ODDS for _ in range(width)] for _ in range(height)]
    for y in range(height):
        cells[y][0] = OGM_OCCUPIED_LOG_ODDS
        cells[y][width - 1] = OGM_OCCUPIED_LOG_ODDS
    for x in range(width):
        cells[0][x] = OGM_OCCUPIED_LOG_ODDS
        cells[height - 1][x] = OGM_OCCUPIED_LOG_ODDS
    for y in range(5, height - 5):
        if 12 <= y <= 28:
            continue
        cells[y][20] = OGM_OCCUPIED_LOG_ODDS

    planner = NavigationPlanner(cells)
    result = planner.update(Pose2D(0.3, 0.3), Pose2D(1.6, 1.6))
    if result.status != "OK":
        raise AssertionError(result.message)
    if len(result.raw_path) < 2 or len(result.smooth_grid_path) < 2:
        raise AssertionError("expected a non-trivial path")
    for start, end in zip(result.smooth_grid_path, result.smooth_grid_path[1:]):
        if start.x != end.x and start.y != end.y:
            raise AssertionError(f"non-axis-aligned segment: {start} -> {end}")
    if result.target_pose is None:
        raise AssertionError("expected a local target")
    if result.target_pose.x_m != 0.3 and result.target_pose.y_m != 0.3:
        raise AssertionError(f"target moves both axes: {result.target_pose}")
    print(f"self-test ok: raw={len(result.raw_path)} smooth={len(result.smooth_grid_path)} target={result.target_pose}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Simulate Core/Src/navigation_task.c on an editable occupancy grid.")
    parser.add_argument("--width", type=int, default=OGM_MAX_WIDTH_CELLS, help="Original OGM width cells, max 96")
    parser.add_argument("--height", type=int, default=OGM_MAX_HEIGHT_CELLS, help="Original OGM height cells, max 96")
    parser.add_argument("--resolution", type=float, default=0.05, help="Meters per original OGM cell")
    parser.add_argument("--self-test", action="store_true", help="Run a headless planner smoke test")
    args = parser.parse_args()

    if args.self_test:
        run_self_test()
        return 0

    width = max(4, min(OGM_MAX_WIDTH_CELLS, args.width))
    height = max(4, min(OGM_MAX_HEIGHT_CELLS, args.height))
    app = NavigationSimulatorApp(width, height, args.resolution)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
