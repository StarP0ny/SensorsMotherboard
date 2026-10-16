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
def native(tmp_path_factory, request):
    compiler = os.environ.get("HOST_CC") or ("C:/msys64/ucrt64/bin/gcc.exe" if Path("C:/msys64/ucrt64/bin/gcc.exe").exists() else shutil.which("gcc"))
    if not compiler:
        pytest.fail("Set HOST_CC to a native GCC compiler")
    out = tmp_path_factory.mktemp("firmware") / ("firmware.dll" if os.name == "nt" else "firmware.so")
    common = ["-Wall", "-Wextra", "-Werror", "-O1"]
    if getattr(request, "param", None) is not None:
        common += [f"-DMUX_ENABLED={request.param}"]
    if os.name != "nt":
        common += ["-fPIC"]
    for folder in ["tests/host", "config", "src", "lib/wch_low_level_drivers/CRC", "lib/wch_low_level_drivers"]:
        common += ["-I", str(ROOT / folder)]
    cpp = str(Path(compiler).with_name(Path(compiler).name.replace("gcc", "g++")))
    objects = []
    for name in ["tests/host/mock.c", "src/protocol.c", "src/app.cpp", "src/sensors.cpp", "lib/wch_low_level_drivers/CRC/crc.c"]:
        obj = out.parent / (Path(name).name + ".o")
        is_cpp = name.endswith(".cpp")
        command = [cpp if is_cpp else compiler, "-std=c++17" if is_cpp else "-std=c11", *common]
        if is_cpp:
            command += ["-fno-exceptions", "-fno-rtti"]
        result = subprocess.run(command + ["-c", str(ROOT / name), "-o", str(obj)], capture_output=True)
        assert result.returncode == 0, result.stderr.decode(errors="replace")
        objects.append(str(obj))
    subprocess.run([cpp, "-shared", *objects, "-o", str(out)], check=True, capture_output=True)
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
    gateway.attach(vgw.SDP810("sdp", "sdp", {}, ctx))
    gateway.open()
    scan = gateway.link.scan_i2c()
    assert scan["paths"] == [{"bus": 1, "mux": 0, "channel": 255}, {"bus": 2, "mux": 0, "channel": 255}]
    assert [r["address"] for r in scan["devices"]] == [0x44, 0x62, 0x25]
    assert [r["bus"] for r in scan["devices"]] == [1, 1, 2]
    native.host_fail_scan_bus(1)
    scan = gateway.link.scan_i2c()
    assert scan["devices"] == [{"bus": 2, "mux": 0, "channel": 255, "address": 0x25, "hint": "SDP810 / SDP800"}]
    assert scan["errors"][0]["bus"] == 1
    native.host_fail_scan_bus(0)
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
        while {i for i, _, _ in ctx.data} != {"scd", "sht", "sdp"} and time.monotonic() < deadline and not failures:
            time.sleep(0.02)
        assert not failures
        assert {i for i, _, _ in ctx.data} == {"scd", "sht", "sdp"}
        assert all(abs(t - time.time()) < 10 for _, t, _ in ctx.data)
        assert next(v for i, _, v in ctx.data if i == "scd")["co2"] == 600
        assert next(v for i, _, v in ctx.data if i == "sdp") == {"dp": -10, "t": 25}
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
    assert (auto, linked, mask, missed) == (1, 0, 7, 1)


def test_sdp_start_stop_crc_and_recovery(native):
    native.host_init()
    parser = Parser()
    seq = 0

    def step(now):
        native.host_step(now)
        buf = c.create_string_buffer(65536)
        length = native.host_take(buf)
        return parser.feed(buf.raw[:length])

    def command(kind, payload, now):
        nonlocal seq
        packet = Frame(kind, seq, 0, payload).encode()
        seq += 1
        native.host_feed(packet, len(packet))
        return step(now)

    command(Type.HELLO_REQ, struct.pack("<H", 1), 0)
    frames = command(Type.ENUM_REQ, b"", 0)
    descriptor = next(f for f in frames if f.type == Type.ENUM).payload
    assert struct.unpack_from("<H", descriptor, 4)[0] == 3
    assert struct.unpack_from("<HHBBBB", descriptor, 38) == (3, 3, 2, 0, 255, 0x25)
    command(Type.START, struct.pack("<HH", 1, 3), 0)
    assert not any(f.type == Type.DATA for f in step(24999))
    step(25000)  # stop before setup
    step(26000)  # start continuous
    frames = step(46000)
    assert samples(next(f for f in frames if f.type == Type.DATA))[0][2] == (64936, 5000, 60)
    native.host_fail(0, 1)
    frames = step(146000)
    assert not any(f.type == Type.DATA for f in frames)
    assert any(f.type == Type.EVENT and struct.unpack("<HH", f.payload) == (3, 2) for f in frames)
    native.host_fail(0, 0)
    command(Type.PING, b"", 1145000)  # maintain the host session during retry
    step(1146000)
    step(1147000)
    assert any(f.type == Type.DATA for f in step(1167000))
    command(Type.STOP, struct.pack("<HH", 1, 3), 1167000)
    assert not any(f.type == Type.DATA for f in step(1267000))
    assert not any(f.type == Type.DATA for f in step(1367000))


def test_i2c_scan_validation_busy_and_bus_error(native):
    native.host_init()
    parser, seq = Parser(), 0

    def step():
        native.host_step(0)
        buf = c.create_string_buffer(65536)
        length = native.host_take(buf)
        return parser.feed(buf.raw[:length])

    def command(kind, data):
        nonlocal seq
        packet = Frame(kind, seq, 0, data).encode()
        seq += 1
        native.host_feed(packet, len(packet))
        return step()

    def code(frames):
        return struct.unpack("<IH", next(f for f in frames if f.type == Type.RESULT).payload)[1]

    command(Type.HELLO_REQ, struct.pack("<H", 1))
    assert code(command(Type.I2C_SCAN_REQ, b"")) == 1
    assert code(command(Type.I2C_SCAN_REQ, b"\x00")) == 3
    command(Type.I2C_SCAN_REQ, b"\xff")
    assert code(command(Type.I2C_SCAN_REQ, b"\xff")) == 4
    frames = []
    for _ in range(112):
        frames.extend(step())
    reply = next(f for f in frames if f.type == Type.I2C_SCAN)
    assert reply.payload[4:] == bytes([255, 2, 0x44, 0x62])
    assert not any(f.type == Type.DATA for f in frames)
    assert code(command(Type.I2C_SCAN_REQ, bytes([3, 255]))) == 3
    assert code(command(Type.I2C_SCAN_REQ, bytes([2, 0]))) == 3
    command(Type.I2C_SCAN_REQ, bytes([2, 255]))
    frames = []
    for _ in range(112):
        frames.extend(step())
    reply = next(f for f in frames if f.type == Type.I2C_SCAN)
    assert reply.payload[4:] == bytes([2, 255, 1, 0x25])
    native.host_fail(1, 0)
    assert code(command(Type.I2C_SCAN_REQ, b"\xff")) == 5
    native.host_fail(0, 0)


def test_mux_removal_isolates_hardware_and_keeps_route_on_write_failure(tmp_path):
    compiler = os.environ.get("HOST_CC") or shutil.which("gcc")
    assert compiler, "Set HOST_CC to a native GCC compiler"
    out = tmp_path / ("mux-test.exe" if os.name == "nt" else "mux-test")
    command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
               "-I", str(ROOT / "tests/i2c_host"), "-I", str(ROOT / "src"), "-I", str(ROOT / "config"),
               str(ROOT / "src/i2c_bus.c"), str(ROOT / "tests/i2c_host/mux_test.c"), "-o", str(out)]
    result = subprocess.run(command, capture_output=True)
    assert result.returncode == 0, result.stderr.decode(errors="replace")
    subprocess.run([str(out)], check=True)


@pytest.fixture
def wire(native):
    from types import SimpleNamespace
    from visserlab.core.driver import DriverError
    native.host_init()
    parser, seq, now = Parser(), 0, 0

    def step(t):
        nonlocal now
        now = t
        native.host_step(t)
        buf = c.create_string_buffer(65536)
        length = native.host_take(buf)
        return parser.feed(buf.raw[:length])

    def request(kind, payload=b"", expected=Type.RESULT, **kwargs):
        nonlocal seq
        packet = Frame(kind, seq, 0, payload).encode()
        request_id = seq
        seq += 1
        native.host_feed(packet, len(packet))
        frames = step(now)
        if expected is None:
            return frames, 0
        frame = next(f for f in frames if f.type in (expected, Type.RESULT) and struct.unpack_from("<I", f.payload)[0] == request_id)
        if frame.type == Type.RESULT:
            code = struct.unpack_from("<H", frame.payload, 4)[0]
            if code:
                raise DriverError(f"RESULT={code}")
        return frame, 0

    request(Type.HELLO_REQ, struct.pack("<H", 1), Type.HELLO)
    return SimpleNamespace(request=request, step=step, native=native)


@pytest.mark.parametrize("stop_fails", [False, True])
def test_sdp_stop_does_not_wait_for_next_measurement(wire, stop_fails):
    from visserlab.core.driver import DriverError
    wire.request(Type.SET_CONFIG, struct.pack("<HHHI", 3, 1, 16, 60000000))
    wire.request(Type.START, struct.pack("<HH", 1, 3))
    for t in (25000, 26000, 46000, 47000):
        wire.step(t)
    wire.native.host_fail(int(stop_fails), 0)
    wire.request(Type.STOP, struct.pack("<HH", 1, 3))
    if stop_fails:
        with pytest.raises(DriverError, match="RESULT=3"):
            wire.request(Type.SENSOR_REMOVE, struct.pack("<H", 3))
        wire.native.host_fail(0, 0)
        wire.step(1047000)
    wire.request(Type.SENSOR_REMOVE, struct.pack("<H", 3))
    # Reassign and identify are accepted without waiting for the 60-second period.
    wire.request(Type.SENSOR_ASSIGN, struct.pack("<HHBBBBI", 3, 3, 2, 0, 255, 0x25, 100000))
    frames, _ = wire.request(Type.IDENTIFY_REQ, bytes([2, 0, 255, 0x25]), expected=None)
    assert not any(f.type == Type.RESULT for f in frames)
    base = 1047000 if stop_fails else 47000
    wire.step(base + 1000)
    frames = wire.step(base + 21000)
    assert any(f.type == Type.IDENTIFY and struct.unpack_from("<H", f.payload, 8)[0] == 3 for f in frames)


def test_configure_children_migrates_default_id_and_keeps_other_sensors(wire):
    from visserlab.drivers.vgw import Link, SCD41
    link = Link.__new__(Link)
    link.request = wire.request
    link.configure_children([SCD41("scd4", "scd4", {"sensor_id": 4}, None)])
    sensors = {r["sensor_id"]: r for r in link.topology()["sensors"]}
    assert set(sensors) == {2, 3, 4}
    assert sensors[4]["address"] == 0x62
    # Restored topology survives a simulated application restart through config_store.
    wire.native.host_init()
    wire.request(Type.HELLO_REQ, struct.pack("<H", 1), Type.HELLO)
    assert link.topology()["sensors"] == list(sensors.values())


@pytest.mark.parametrize("native", [1], indirect=True)
def test_blank_flash_uses_mux_defaults_from_board_config(wire):
    from visserlab.drivers.vgw import Link
    link = Link.__new__(Link)
    link.request = wire.request
    topology = link.topology()
    assert topology["muxes"] == [{"bus": 1, "address": 0x70}]
    assert [(r["sensor_id"], r["bus"], r["mux"], r["channel"]) for r in topology["sensors"]] == [
        (1, 1, 0x70, 0), (2, 1, 0x70, 1), (3, 2, 0, 255)]
