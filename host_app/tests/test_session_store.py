import struct

from host_app.protocol.telemetry import FRAME_TYPE_MAP_DATA_V2, FRAME_TYPE_MAP_META_V2, FRAME_TYPE_PATH_V2, TelemetryFrame
from host_app.services.store import SessionStore


def test_store_updates_map_and_path() -> None:
    store = SessionStore()

    map_meta_payload = struct.pack("<HHfff", 4, 4, 0.05, -0.1, -0.1)
    map_rows_payload = struct.pack("<HHHH", 4, 4, 0, 2) + struct.pack("<8b", -20, -20, 0, 15, 0, 0, 10, 10)
    path_payload = struct.pack("<BBHHhhhhf", 1, 0, 1, 2, 3, 4, 1, 2, 0.25) + struct.pack("<hhhh", 1, 2, 3, 4)

    store.apply_event(TelemetryFrame(FRAME_TYPE_MAP_META_V2, 1, map_meta_payload))
    store.apply_event(TelemetryFrame(FRAME_TYPE_MAP_DATA_V2, 2, map_rows_payload))
    store.apply_event(TelemetryFrame(FRAME_TYPE_PATH_V2, 3, path_payload))

    assert store.state.map_snapshot.meta.width_cells == 4
    assert store.state.map_snapshot.cells[:4] == [-20, -20, 0, 15]
    assert store.state.navigation.goal_cell == (3, 4)
    assert store.state.navigation.path_cells == [(1, 2), (3, 4)]
