import struct

from host_app.models.state import RuntimeStats
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


def test_store_updates_mapping_gate_status() -> None:
    store = SessionStore()
    payload = struct.pack(
        "<I9f4BfHHhhf7IHf4B3I4B",
        123,
        0.1,
        0.2,
        1.0,
        0.3,
        0.4,
        2.0,
        0.5,
        0.6,
        3.0,
        1,
        2,
        3,
        4,
        0.25,
        5,
        6,
        7,
        8,
        0.75,
        10,
        11,
        12,
        13,
        14,
        15,
        16,
        17,
        0.018,
        0,
        2,
        0,
        0,
        21,
        22,
        23,
        1,
        1,
        1,
        0,
    )

    store.apply_event(TelemetryFrame(16, 1, payload))

    assert isinstance(store.state.runtime, RuntimeStats)
    assert store.state.runtime.map_update_active is False
    assert store.state.runtime.map_last_skip_reason == 2
    assert store.state.runtime.mapping_skipped_turning_count == 21
    assert store.state.runtime.mapping_skipped_settle_count == 22
    assert store.state.runtime.mapping_skipped_quality_count == 23
