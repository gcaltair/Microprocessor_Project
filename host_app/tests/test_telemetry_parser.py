from host_app.protocol.telemetry import MAGIC, FRAME_TYPE_STATUS_V2, TelemetryFrame, TelemetryStreamParser, crc16_ccitt


def build_frame(frame_type: int, sequence: int, payload: bytes) -> bytes:
    header = bytearray()
    header.extend(MAGIC)
    header.append(1)
    header.append(frame_type)
    header.extend(sequence.to_bytes(2, "little"))
    header.extend(len(payload).to_bytes(2, "little"))
    body = bytes(header) + payload
    crc = crc16_ccitt(body).to_bytes(2, "little")
    return body + crc


def test_parser_can_extract_text_and_frame() -> None:
    parser = TelemetryStreamParser()
    payload = b"\x00" * 72
    stream = b"hello world\r\n" + build_frame(FRAME_TYPE_STATUS_V2, 7, payload)
    events = parser.feed(stream)

    assert len(events) == 2
    assert getattr(events[0], "text") == "hello world"
    assert isinstance(events[1], TelemetryFrame)
    assert events[1].frame_type == FRAME_TYPE_STATUS_V2
    assert events[1].sequence == 7
    assert events[1].payload == payload
