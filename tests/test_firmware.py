"""Compile and exercise the actual C parser, application and sensor state machines."""
import ctypes as c
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import threading
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent / "VisserLab"))
from visserlab.gwproto import Frame, Parser, Type, samples


@pytest.fixture
def native(tmp_path_factory):
    compiler = os.environ.get("HOST_CC") or ("C:/msys64/ucrt64/bin/gcc.exe" if Path("C:/msys64/ucrt64/bin/gcc.exe").exists() else shutil.which("gcc"))
    if not compiler:
        pytest.fail("Set HOST_CC to a native GCC compiler")
    out = tmp_path_factory.mktemp("firmware") / ("firmware.dll" if os.name == "nt" else "firmware.so")
    cmd = [compiler, "-shared", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O1"]
    if os.name != "nt":
        cmd += ["-fPIC"]
    for folder in ["tests/host", "config", "src", "lib/wch_low_level_drivers/CRC", "lib/wch_low_level_drivers"]:
        cmd += ["-I", str(ROOT / folder)]
    cmd += [str(ROOT / f) for f in ["tests/host/mock.c", "src/protocol.c", "src/app.c", "src/sensors.c", "lib/wch_low_level_drivers/CRC/crc.c"]]
    subprocess.run(cmd + ["-o", str(out)], check=True, capture_output=True)
    dll = c.CDLL(str(out))
    dll.proto_encode.argtypes = [c.c_void_p, c.c_uint16, c.c_uint32, c.c_uint64, c.c_void_p, c.c_uint16]
    dll.proto_encode.restype = c.c_size_t
    dll.host_step.argtypes = [c.c_uint64]
    dll.host_feed.argtypes = [c.c_void_p, c.c_size_t]
    dll.host_take.argtypes = [c.c_void_p]
    dll.host_take.restype = c.c_size_t
    dll.sensirion_crc.argtypes = [c.c_void_p, c.c_uint]
    dll.sensirion_crc.restype = c.c_uint8
    return dll


def test_c_python_wire_compatibility(native):
    for payload in [b"", b"VSLB\x00BLSV", bytes(range(256)) * 4, b"\x80"]:
        frame = Frame(Type.DATA, 0xFEDCBA98, 845678901234567, payload)
        out = c.create_string_buffer(1052)
        length = native.proto_encode(out, frame.type, frame.sequence, frame.time, payload, len(payload))
        assert out.raw[:length] == frame.encode()
    assert native.sensirion_crc(b"\xbe\xef", 2) == 0x92


def test_firmware_commands_sensors_button_ping_and_sync(native):
    native.host_init()
    parser, now, seq = Parser(), 0, 0

    def read():
        out = c.create_string_buffer(65536)
        n = native.host_take(out)
        return parser.feed(out.raw[:n])

    def step(delta=0, pong=True):
        nonlocal now, seq
        now += delta
        native.host_step(now)
        frames = read()
        if pong:
            for frame in frames:
                if frame.type == Type.PING:
                    packet = Frame(Type.PONG, seq, 845678901234567, struct.pack("<I", frame.sequence)).encode()
                    seq += 1
                    native.host_feed(packet, len(packet))
            native.host_step(now)
            frames += read()
        return frames

    def command(kind, payload=b"", timestamp=845678901234567, prefix=b""):
        nonlocal seq
        packet = prefix + Frame(kind, seq, timestamp, payload).encode()
        requested = seq
        seq += 1
        native.host_feed(packet, len(packet))
        frames = step()
        return requested, frames

    _, frames = command(Type.HELLO_REQ, struct.pack("<H", 1), prefix=b"noise\x00VSL")
    assert next(f for f in frames if f.type == Type.HELLO).payload[4:6] == b"\x01\x00"
    corrupt = bytearray(Frame(Type.PING, 555, 0).encode())
    corrupt[8] ^= 1
    _, frames = command(Type.GET_CONFIG, b"\x00\x00", prefix=bytes(corrupt))
    assert any(f.type == Type.CONFIG for f in frames)
    _, frames = command(Type.SET_CONFIG, struct.pack("<HHHI", 0, 1, 1, 0))
    assert struct.unpack("<IH", next(f for f in frames if f.type == Type.RESULT).payload)[1] == 3
    _, frames = command(Type.GET_CONFIG, b"\x00\x00")
    assert struct.unpack_from("<I", next(f for f in frames if f.type == Type.CONFIG).payload, 10)[0] == 1000
    _, frames = command(Type.START, b"\x00\x00")
    assert any(f.type == Type.RESULT for f in frames)
    frames = step(10000)
    assert any(samples(f)[0][0] == 2 for f in frames if f.type == Type.DATA)
    step(490000)
    assert native.host_asc() == 0
    step(2000)
    step(5000000)
    step(2000)
    frames = step(2000)
    assert any(samples(f)[0][2][0] == 600 for f in frames if f.type == Type.DATA)
    _, frames = command(Type.ENUM_REQ)
    enum = next(f for f in frames if f.type == Type.ENUM)
    assert struct.unpack_from("<H", enum.payload, 18)[0] == 1  # SCD presence
    _, frames = command(Type.TIME_SYNC_REQ)
    sync = next(f for f in frames if f.type == Type.TIME_SYNC_RESP)
    assert sync.time == now and struct.unpack_from("<Q", sync.payload, 12)[0] == now
    _, frames = command(Type.TIME_SYNC_SET, struct.pack("<q", 845678901234567 - now))
    assert next(f for f in frames if f.type == Type.RESULT).time == 845678901234567
    # A truncated long candidate must not trap the following valid request forever.
    truncated = Frame(Type.DATA, 123, 0, bytes(1000)).encode()[:20]
    request = Frame(Type.GET_CONFIG, seq, 0, b"\x00\x00").encode()
    seq += 1
    native.host_feed(truncated + request, len(truncated + request))
    assert not any(f.type == Type.CONFIG for f in step())
    assert any(f.type == Type.CONFIG for f in step(250001))
    # Button bounce does not toggle; one stable press toggles once, release doesn't.
    native.host_button(1); step(1000)
    native.host_button(0); step(10000)
    native.host_button(1); step(10000); step(30000)
    assert native.host_led() == 1
    step(100000)
    assert native.host_led() == 1
    native.host_button(0); step(1000); step(30000)
    _, frames = command(Type.STOP, b"\x00\x00")
    # Host STOP leaves automatic acquisition active; missing one PONG marks link down.
    step(1000000, pong=False)
    frames = step(1000000, pong=False)
    assert any(f.type == Type.PING for f in frames)
    _, frames = command(Type.PING)
    step(1000000)  # status after recovery
    _, frames = command(Type.GET_CONFIG, b"\x00\x00")
    assert any(f.type == Type.CONFIG for f in frames)
    native.host_button(1); step(1000); step(30000)
    assert native.host_led() == 0
    # Bad sensor CRC yields an event and no corrupted measurement.
    command(Type.START, struct.pack("<HH", 1, 2))
    native.host_fail(0, 1)
    frames = step(1000000) + step(10000) + step(1000000) + step(10000)
    assert any(f.type == Type.EVENT and struct.unpack("<HH", f.payload) == (2, 2) for f in frames)
    assert not any(f.type == Type.DATA for f in frames)


def test_visserlab_gateway_against_c_firmware(native, monkeypatch):
    """Run the real Python gateway against the real C application over a fake serial wire."""
    from visserlab.drivers import vgw

    class SerialWire:
        def __init__(self):
            native.host_init()
            self.base = time.monotonic_ns()
            self.lock = threading.Lock()
            self.rx = bytearray()
            self.closed = False

        def pump(self):
            native.host_step((time.monotonic_ns() - self.base) // 1000)
            data = c.create_string_buffer(65536)
            n = native.host_take(data)
            self.rx.extend(data.raw[:n])

        @property
        def in_waiting(self):
            with self.lock:
                self.pump()
                return len(self.rx)

        def write(self, data):
            with self.lock:
                native.host_feed(data, len(data))
                self.pump()
            return len(data)

        def read(self, count):
            deadline = time.monotonic() + 0.02
            while True:
                with self.lock:
                    self.pump()
                    if self.rx:
                        data = bytes(self.rx[:count])
                        del self.rx[:count]
                        return data
                if time.monotonic() >= deadline:
                    return b""
                time.sleep(0.001)

        def close(self):
            self.closed = True

    wire = SerialWire()
    monkeypatch.setattr(vgw.serial, "serial_for_url", lambda *args, **kwargs: wire)

    class Context:
        def __init__(self):
            self.data = []

        def online(self, ok, reason="", dev=None):
            pass

        def emit(self, values, t=None, dev=None):
            self.data.append((dev, t, values))

    ctx = Context()
    gateway = vgw.Gateway("gw", "gw", {"port": "fake"}, ctx)
    gateway.attach(vgw.SCD41("scd", "scd", {}, ctx))
    gateway.attach(vgw.SHT41("sht", "sht", {}, ctx))
    gateway.open()
    stop = threading.Event()
    failures = []

    def run():
        try:
            gateway.run(stop)
        except Exception as exc:
            failures.append(exc)

    thread = threading.Thread(target=run)
    thread.start()
    try:
        deadline = time.monotonic() + 7
        while {i for i, _, _ in ctx.data} != {"scd", "sht"} and time.monotonic() < deadline and not failures:
            time.sleep(0.02)
        assert not failures
        assert {i for i, _, _ in ctx.data} == {"scd", "sht"}
        assert all(abs(t - time.time()) < 10 for _, t, _ in ctx.data)
        assert next(v for i, _, v in ctx.data if i == "scd")["co2"] == 600
    finally:
        stop.set()
        thread.join(timeout=2)
        gateway.close()
    assert wire.closed


def test_batch_offsets_and_one_missed_ping(native):
    native.host_init()
    seq = 0
    parser = Parser()

    def step(now):
        native.host_step(now)
        buf = c.create_string_buffer(65536)
        n = native.host_take(buf)
        return parser.feed(buf.raw[:n])

    def command(kind, data):
        nonlocal seq
        packet = Frame(kind, seq, 845678901234567, data).encode()
        seq += 1
        native.host_feed(packet, len(packet))
        return step(0)

    command(Type.HELLO_REQ, struct.pack("<H", 1))
    command(Type.SET_CONFIG, struct.pack("<HHHI", 0, 1, 3, 100000))
    command(Type.SET_CONFIG, struct.pack("<HHHI", 2, 1, 16, 10000))
    command(Type.START, struct.pack("<HH", 1, 2))
    assert not any(f.type == Type.DATA for f in step(10000))
    step(10000)  # begin next conversion at the same mock time
    assert not any(f.type == Type.DATA for f in step(20000))
    frames = step(110000)
    records = samples(next(f for f in frames if f.type == Type.DATA))
    assert records[0][1] == 10000 and records[1][1] == 20000
    assert all(r[0] == 2 for r in records)
    assert any(f.type == Type.PING for f in step(1000000))
    step(2000000)  # no PONG: one missed ping must mark the link down
    native.host_button(1)
    step(2000001)
    frames = step(2030001)
    status = next(f for f in frames if f.type == Type.STATUS)
    auto, linked, mask, missed = struct.unpack_from("<BBHI", status.payload)
    assert (auto, linked, mask, missed) == (1, 0, 3, 1)
