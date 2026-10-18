"""The ground link of the quad_fc SIL, end to end (decision 0053; ground-segment unit 6, LINK-SIL; invariants K-1 and G7-1's
store half). A MAVLink client of the dialect generated here by pymavlink's mavgen (a test tool; dev dependency) talks over a
plain UDP socket to the SIL's link service (fw/link/src/service.cpp) inside the gz-sim 8 lockstep plugin.

Skipped unless `gz sim --force-version 8` reports 8.x and the host-gz-quadfc build holds libmarv_gz_lockstep.so
(`cmake --preset host-gz-quadfc && cmake --build --preset host-gz-quadfc`); the gz CI step fails on a skip. Every gz process gets
its own GZ_PARTITION and GZ_IP=127.0.0.1.

The world is built as in test_quad_fc_plumbing.py, on the FC3v2 card (vehicles/marv_5in_fc3v2.yaml): the L2 hover world (member lo,
m = 1) with the model gyro at the nominal corner, without the open-loop DShot overrides, with <launch_pad/>, a
<manual_control_script> (link on, arm switch low unless a check raises it) and the link elements <config_dir> (a fresh directory
per check: the SIL's config storage, fw/hal/sim/include/marv/hal_sim/storage.hpp) and <link_udp_port> (a free port). The SIL's
link answers the sender of its latest datagram, so the client sends an empty datagram every POKE_S while it waits. PARAM_VALUE's
value is read as the raw 32 bits of the payload (byte-wise encoding), never as the decoded float.

What is checked:
  - A session on the factory config (real-time factor 1): heartbeat; identity (factory, OK, the image hash = SHA-256 of the
    loaded SIL library, the config hash = the factory .mcfg's config hash, tools/config/mcfg.py); the parameter list: every
    PARAM_VALUE bit-exact to the factory .mcfg (marv_params_fc3v2.mcfg), with the manifest's link id, type and count; an
    accepted write (ACCEPTED_PARAM one ulp up) and a refused one (LOCKED_PARAM, locked: K-1), each followed by a read by index
    (the accepted value iff its bits are the requested ones; the locked one's status text arrives and its value is unchanged);
    MARV_PARAM_SOURCE: a source text set in chunks is read back in chunks and the record carries its length and the staged
    provenance; activation (a) with a zero report id is DENIED_REPORT and (b) with the REAL report id (the id of the PASS report
    that tools/validate/validate_config.py writes for this image and the staged config, which the test validates beforehand: the
    staged config is computed with mcfg.py exactly as the store stages it, and the test asserts the report is for the image hash
    the SIL reports and for the config hash the activation carries) and the staged config's hash is ACCEPTED, and the run ends with
    gz exit EX_TEMPFAIL ("REBOOT REQUESTED"); the relaunch of the same world on the same config dir reports config source ACTIVE,
    OK, the staged config hash, that report id and ACTIVATED, and the accepted value is active; a further run on the activated
    config dir is not refused and arms on the raise (G7-1's allowed half: the stored config bound to this image and a report).
  - Arming (uncapped, the plumbing test's arm raise after calibration and link recovery, read from the DShot values of the log and
    from the heartbeat's armed flag): (c) a stored slot the test writes unbound (mcfg.py factory config, no image, no report) is
    reported ACTIVE, REFUSED, IMAGE_BINDING and the raised switch never arms; (d) the same slot with one file byte changed (its
    SHA-256 no longer matches the commit record) is reported FACTORY, REFUSED, FORMAT and never arms; control: the factory config
    (an empty config dir) arms on the same raise.
"""

import hashlib
import json
import os
import re
import signal
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path
from xml.sax.saxutils import escape

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tools" / "card"))
sys.path.insert(0, str(ROOT / "tools" / "sim"))
sys.path.insert(0, str(ROOT / "tools" / "config"))
import gen_world  # noqa: E402
import lockstep_log  # noqa: E402
import mcfg  # noqa: E402
import run_scenario  # noqa: E402

CARD = ROOT / "vehicles" / "marv_5in_fc3v2.yaml"
HOVER = ROOT / "scenarios" / "quad" / "L02" / "hover.yaml"
REGISTER = ROOT / "fw" / "compositions" / "quad_fc" / "params" / "quad_fc_register.yaml"
BUILD = ROOT / "build" / "host-gz-quadfc"
PLUGIN_DIR = BUILD / "sim" / "gz" / "plugin"
PLUGIN = PLUGIN_DIR / "libmarv_gz_lockstep.so"
SIL_LIBRARY = BUILD / "sim" / "quad_fc" / "libmarv_sil_quad_fc.so"
DIALECT_XML = BUILD / "mavlink" / "defs" / "marv.xml"
PARAM_SET = BUILD / "generated" / "marv_params_fc3v2"
FACTORY_MCFG = PARAM_SET / "marv_params_fc3v2.mcfg"
MANIFEST = PARAM_SET / "marv" / "params" / "params_manifest.json"
CMAKE_CACHE = BUILD / "CMakeCache.txt"

TIMEOUT_S = 300  # labelled harness value: bounds a hung gz process only
REPLY_S = 20  # labelled harness value: how long the client waits for an answer
POKE_S = 0.05  # labelled harness value: the client's empty-datagram period while it waits
SESSION_S = 300  # labelled harness value: the most SIL (= wall, real-time factor 1) seconds a session run lasts; the test stops it sooner
SEED = 1
NUM_US, DEN = 625, 4  # the scenario's tick period, us (scenarios/quad/L02/hover.yaml)
FRAME_PERIOD_US = 8000  # 125 frames/s, measured on Luis's ExpressLRS link (docs/m2-bench-card.md)
ARM_CHANNEL, ANGLE_CHANNEL = 5, 8  # the register's rc_channel_arm and rc_channel_angle (read below and asserted equal)
DSHOT_THROTTLE_MIN = 48  # DShot protocol: 0 is stop, 1..47 are commands, 48..2047 are throttle values
SWITCH_LOW, SWITCH_HIGH = -1.0, 1.0  # normalised channel values, [-1, 1] (fw/types manual_control.hpp)
MARGIN_US = 1_000_000  # labelled harness value: the margin after calibration + recovery before the raise (the plumbing test's)
ARMED_US = 1_500_000  # labelled harness value: sim time after the raise: more than one heartbeat period (1 s, service.cpp)
GCS_SYSTEM, GCS_COMPONENT = 255, 190  # the client's address: the protocol's ground-station convention (MAV_COMP_ID_MISSIONPLANNER)
VEHICLE = (1, 1)  # the link service's system and component (fw/link/src/service.cpp kSystemId, kComponentId)
ACCEPTED_PARAM = "att_kp"  # an unlocked f32 parameter; the write is one ulp up, which the validator accepts
LOCKED_PARAM = "rotor_speed_max"  # a locked f32 parameter (Luis-signed lock list, decision 0053 Q2)
SOURCE_TEXT = ("test_link_sil.py: one ulp above the factory value, staged over the link to check the staged echo, the source "
               "round trip in chunks and the activation through a reboot; not a tuning")
VALIDATOR = ROOT / "tools" / "validate" / "validate_config.py"
VALIDATE_TIMEOUT_S = 3600  # labelled harness value: bounds a hung validator run only (two gz runs and the config-validation ctests)
COMMIT_MAGIC = b"MARVSLOT"  # fw/store/include/marv/store/store.hpp kCommitMagic; record: magic, sequence u32, length u32, SHA-256
SLOT_FILE = "slot_a.bin"  # hal_sim's file of slot A (marv/hal_sim/storage.hpp)
ERASED = 0xFF  # a slot byte past the file reads as erased flash (marv/hal_sim/storage.hpp)

pytestmark = pytest.mark.skipif(not (PLUGIN.exists() and run_scenario.gz8_available()),
                                reason="gz-sim 8 or the host-gz-quadfc build of libmarv_gz_lockstep.so is absent")


def register_value(name):
    return yaml.safe_load(REGISTER.read_text(encoding="utf-8"))[name]["value"]


READY_US = register_value("gyro_cal_duration_us") + register_value("failsafe_recovery_us") + MARGIN_US
assert register_value("rc_channel_arm") == ARM_CHANNEL and register_value("rc_channel_angle") == ANGLE_CHANNEL


def iterations(duration_us):
    return -(-duration_us * DEN // NUM_US)  # ticks of m = 1 that reach duration_us


def seg(start_us, duration_us, arm):
    return (f"<segment><start_us>{start_us}</start_us><duration_us>{duration_us}</duration_us><link>on</link>"
            f"<roll>0.0</roll><pitch>0.0</pitch><yaw>0.0</yaw><throttle>0.0</throttle><arm>{arm}</arm><angle>1.0</angle></segment>")


def script(segments):
    return (f"<manual_control_script><frame_period_us>{FRAME_PERIOD_US}</frame_period_us><arm_channel>{ARM_CHANNEL}</arm_channel>"
            f"<angle_channel>{ANGLE_CHANNEL}</angle_channel>{''.join(segments)}</manual_control_script>")


def world(path, mode, log, script_text, config_dir, port):
    text = gen_world.generate(CARD, HOVER, mode, 1, "lo", str(log) if log else None,
                              sensors=gen_world.SensorSet(gyro="model", bias_signs=(0,) * 6))[1]
    text = re.sub(r'\s*<sil_override param="ol_dshot_m\d" type="i32">\d+</sil_override>', "", text)
    add = (f"<launch_pad/>\n{script_text}\n<config_dir>{escape(str(config_dir))}</config_dir>\n"
           f"<link_udp_port>{port}</link_udp_port>\n")
    assert text.count("<gyro_source>") == 1
    path.write_text(text.replace("<gyro_source>", add + "<gyro_source>", 1), encoding="utf-8")
    return path


def free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Gz:
    """One gz process on a world, in its own process group; stop() ends it (SIGTERM, then SIGKILL)."""

    def __init__(self, sdf, n, tmp):
        self.err_path = tmp / f"{sdf.stem}.{time.monotonic_ns()}.stderr"
        self._err = open(self.err_path, "w", encoding="utf-8")
        self.proc = subprocess.Popen([*run_scenario.GZ_ARGS, "-s", "-r", "--iterations", str(n), "--seed", str(SEED), str(sdf)],
                                     env=run_scenario.gz_env(PLUGIN_DIR), stdout=subprocess.DEVNULL, stderr=self._err,
                                     start_new_session=True)

    def stderr(self):
        self._err.flush()
        return self.err_path.read_text(encoding="utf-8", errors="replace")

    def wait(self, timeout=TIMEOUT_S):
        try:
            return self.proc.wait(timeout)
        finally:
            self._err.flush()

    def stop(self):
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGTERM)
            try:
                self.proc.wait(TIMEOUT_S)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()
        self._err.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.stop()


@pytest.fixture(scope="module")
def dialect(tmp_path_factory):
    import importlib.util
    out = tmp_path_factory.mktemp("dialect") / "marv_dialect.py"
    subprocess.run([sys.executable, "-m", "pymavlink.tools.mavgen", "--lang", "Python3", "--wire-protocol", "2.0", "--output",
                    str(out), str(DIALECT_XML)], check=True, capture_output=True)
    spec = importlib.util.spec_from_file_location("marv_dialect", out)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def manifest():
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def factory():
    return mcfg.decode(FACTORY_MCFG.read_bytes())


class Client:
    """The ground station's side: a UDP socket to the SIL's port; every message received is kept, in order."""

    def __init__(self, d, port):
        self.d = d
        self.peer = ("127.0.0.1", port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(POKE_S)
        self.mav = d.MAVLink(None, srcSystem=GCS_SYSTEM, srcComponent=GCS_COMPONENT)
        self.parser = d.MAVLink(None)
        self.parser.robust_parsing = True
        self.got = []

    def close(self):
        self.sock.close()

    def send(self, message):
        self.sock.sendto(message.pack(self.mav), self.peer)

    def pump(self):
        """One empty datagram (the SIL learns this socket as its peer), then whatever arrives within POKE_S."""
        try:
            self.sock.sendto(b"", self.peer)
            while True:
                data = self.sock.recv(65536)
                self.got.extend(self.parser.parse_buffer(data) or [])
                self.sock.settimeout(0.0)
        except (BlockingIOError, socket.timeout, ConnectionRefusedError):
            pass
        finally:
            self.sock.settimeout(POKE_S)

    def mark(self):
        return len(self.got)

    def wait(self, predicate, what, after=0):
        """The first message from index `after` on that satisfies predicate; pumps until it arrives or REPLY_S passes."""
        deadline = time.monotonic() + REPLY_S
        i = after
        while True:
            for k in range(i, len(self.got)):
                if predicate(self.got[k]):
                    return self.got[k]
            i = len(self.got)
            if time.monotonic() > deadline:
                raise AssertionError(f"no {what} within {REPLY_S} s")
            self.pump()

    def since(self, after, name):
        return [m for m in self.got[after:] if m.get_type() == name]

    # ---- requests ---------------------------------------------------------------------------------------------------

    def connect(self):
        """The first heartbeat: from then on the SIL knows this socket and a request is not sent before the SIL listens."""
        return self.wait(lambda m: m.get_type() == "HEARTBEAT", "HEARTBEAT")

    def identity(self):
        at = self.mark()
        self.send(self.d.MAVLink_command_long_message(*VEHICLE, self.d.MAV_CMD_REQUEST_MESSAGE, 0,
                                                      float(self.d.MAVLINK_MSG_ID_MARV_IDENTITY), 0, 0, 0, 0, 0, 0))
        return self.wait(lambda m: m.get_type() == "MARV_IDENTITY", "MARV_IDENTITY", at)

    def read(self, index):
        at = self.mark()
        self.send(self.d.MAVLink_param_request_read_message(*VEHICLE, b"", index))
        value = self.wait(lambda m: m.get_type() == "PARAM_VALUE" and m.param_index == index, f"PARAM_VALUE {index}", at)
        record = self.wait(lambda m: m.get_type() == "MARV_PARAM_RECORD" and m.param_index == index, f"MARV_PARAM_RECORD {index}", at)
        return value, record

    def set(self, link_id, bits):
        assert struct.pack("<f", f32_of(bits)) == struct.pack("<I", bits), "PARAM_SET carries a float: use bits that are not a NaN"
        at = self.mark()
        self.send(self.d.MAVLink_param_set_message(*VEHICLE, link_id.encode(), f32_of(bits), self.d.MAV_PARAM_TYPE_REAL32))
        return self.wait(lambda m: m.get_type() == "PARAM_VALUE" and m.param_id == link_id, f"PARAM_VALUE {link_id}", at), at

    def config(self, action, report_id, expected, schema_hash):
        at = self.mark()
        self.send(self.d.MAVLink_marv_config_command_message(schema_hash, list(report_id), list(expected), *VEHICLE, action))
        return self.wait(lambda m: m.get_type() == "MARV_CONFIG_ACK", "MARV_CONFIG_ACK", at), at


def f32_of(bits):
    return struct.unpack("<f", struct.pack("<I", bits))[0]


def payload(d, m):
    """The message's payload bytes, MAVLink 2's trailing-zero truncation undone."""
    frame = bytes(m.get_msgbuf())
    assert frame[0] == d.PROTOCOL_MARKER_V2
    return frame[d.HEADER_LEN_V2:d.HEADER_LEN_V2 + frame[1]].ljust(type(m).unpacker.size, b"\0")


def value_bits(d, m):
    """PARAM_VALUE's param_value as the raw 32 bits it carries (byte-wise encoding), never through the float."""
    assert type(m).ordered_fieldnames[0] == "param_value"
    return struct.unpack_from("<I", payload(d, m), 0)[0]


def text_of(chunk, length):
    raw = chunk.encode() if isinstance(chunk, str) else bytes(chunk)
    return raw[:length].decode()


def staged_config(factory_cfg, manifest, changes):
    """The factory config with `changes` {index: Record} applied: what the store stages and hashes."""
    section = factory_cfg.section("quad")
    records = mcfg.section_records(section)
    for i, r in changes.items():
        records[i] = r
    return mcfg.Config(0, bytes(mcfg.DIGEST), bytes(mcfg.DIGEST),
                       (mcfg.build_section("quad", mcfg.manifest_schema_hash(manifest), records),))


def slot_bytes():
    m = re.search(r"^MARV_CONFIG_SLOT_BYTES:\w+=(\d+)$", CMAKE_CACHE.read_text(encoding="utf-8"), re.M)
    assert m, "MARV_CONFIG_SLOT_BYTES is not in the build's CMake cache"
    return int(m.group(1))


def write_slot(config_dir, file, corrupt=False):
    """Slot A: the .mcfg file at offset 0, erased bytes, the commit record in the last bytes (store.hpp slot layout). With corrupt,
    one byte of the file changes after the record was computed, so its SHA-256 no longer matches."""
    sequence = struct.unpack_from("<I", file, len(mcfg.MAGIC) + 4)[0]
    record = COMMIT_MAGIC + struct.pack("<II", sequence, len(file)) + hashlib.sha256(file).digest()
    size = slot_bytes()
    body = bytearray(file)
    if corrupt:
        body[-1] ^= 1
    region = bytes(body) + bytes([ERASED]) * (size - len(body) - len(record)) + record
    assert len(region) == size
    (config_dir / SLOT_FILE).write_bytes(region)


def sha256_of(path):
    return hashlib.sha256(Path(path).read_bytes()).digest()


def validated_report_id(staged, tmp):
    """Validates `staged` (an mcfg.Config) with tools/validate/validate_config.py for the build's SIL library, as the helper does,
    and returns the report's id (the SHA-256 of the report's bytes) as 32 bytes. The report must be a PASS for exactly that
    image and that config."""
    tmp.mkdir(exist_ok=True)
    config, out = tmp / "staged.mcfg", tmp / "report.json"
    config.write_bytes(mcfg.encode(staged))
    proc = subprocess.run([sys.executable, str(VALIDATOR), "--config", str(config), "--build", str(BUILD), "--work-dir", str(tmp / "work"),
                           "--out", str(out)], capture_output=True, text=True, timeout=VALIDATE_TIMEOUT_S, check=False)
    assert proc.returncode == 0, f"the validator did not pass the staged config (exit {proc.returncode}):\n{proc.stdout[-2000:]}\n{proc.stderr[-3000:]}"
    data = out.read_bytes()
    report = json.loads(data)
    assert report["verdict"] == "PASS" and all(c["result"] == "PASS" for c in report["checks"])
    assert report["config"]["config_hash"] == mcfg.config_hash(staged).hex()
    assert report["image"]["sha256"] == sha256_of(SIL_LIBRARY).hex()
    return hashlib.sha256(data).digest()


# ---- the session on the factory config, the activation and the relaunch ---------------------------------------------------

def test_session_parameters_writes_sources_and_activation_through_a_reboot(dialect, manifest, factory, tmp_path):
    d = dialect
    schema = mcfg.manifest_schema_hash(manifest)
    names = list(manifest["params"])
    records = mcfg.section_records(factory.section("quad"))
    accepted, locked = names.index(ACCEPTED_PARAM), names.index(LOCKED_PARAM)
    assert not manifest["params"][ACCEPTED_PARAM]["lock"] and manifest["params"][LOCKED_PARAM]["lock"]
    assert records[accepted].type == records[locked].type == "f32"
    new_bits = records[accepted].value_bits + 1
    staged = staged_config(factory, manifest, {accepted: mcfg.Record("f32", new_bits, 0, "Choice", "scenario", "Manual", False, SOURCE_TEXT)})
    expected = mcfg.config_hash(staged)
    report_id = validated_report_id(staged, tmp_path / "validation")
    store = tmp_path / "store"
    store.mkdir()
    port = free_udp_port()
    duration = SESSION_S * 1_000_000
    sdf = world(tmp_path / "session.sdf", "pilot", None, script([seg(0, duration, SWITCH_LOW)]), store, port)
    client = Client(d, port)
    try:
        with Gz(sdf, iterations(duration), tmp_path) as gz:
            hb = client.connect()
            assert hb.get_srcSystem() == VEHICLE[0] and hb.get_srcComponent() == VEHICLE[1]
            assert not hb.base_mode & d.MAV_MODE_FLAG_SAFETY_ARMED

            ident = client.identity()
            assert ident.config_source == d.MARV_CONFIG_SOURCE_FACTORY and ident.config_state == d.MARV_CONFIG_STATE_OK
            assert ident.config_refusal == d.MARV_CONFIG_REFUSAL_NONE and ident.staged_differs == 0
            assert ident.param_schema_hash == schema
            assert bytes(ident.image_sha256) == sha256_of(SIL_LIBRARY), "the image hash is not the SHA-256 of the SIL library"
            assert bytes(ident.config_sha256) == mcfg.config_hash(factory), "the running config hash is not the factory file's"
            image = bytes(ident.image_sha256)

            # the list: every value bit-exact to the factory file
            at = client.mark()
            client.send(d.MAVLink_param_request_list_message(*VEHICLE))
            for i in range(len(records)):
                client.wait(lambda m, i=i: m.get_type() == "PARAM_VALUE" and m.param_index == i, f"PARAM_VALUE {i} of the list", at)
            values = {m.param_index: m for m in client.since(at, "PARAM_VALUE")}
            assert len(values) == len(records) == manifest["count"]
            for i, name in enumerate(names):
                m = values[i]
                assert m.param_count == manifest["count"] and m.param_id == manifest["params"][name]["link_id"], name
                want_type = d.MAV_PARAM_TYPE_REAL32 if records[i].type == "f32" else d.MAV_PARAM_TYPE_INT32
                assert m.param_type == want_type, name
                assert value_bits(d, m) == records[i].value_bits, f"{name}: {value_bits(d, m):#010x} != {records[i].value_bits:#010x}"

            # an accepted write, then a read by index: the staged value iff its bits are the requested ones
            link_id = manifest["params"][ACCEPTED_PARAM]["link_id"]
            echo, at = client.set(link_id, new_bits)
            assert value_bits(d, echo) == new_bits and not client.since(at, "STATUSTEXT")
            value, record = client.read(accepted)
            assert value_bits(d, value) == new_bits
            assert record.staged_bits == new_bits and record.active_bits == records[accepted].value_bits and record.lock == 0
            assert (record.origin, record.method, record.sigma_kind) == (d.MARV_PARAM_ORIGIN_MANUAL, d.MARV_PARAM_METHOD_SCENARIO,
                                                                         d.MARV_PARAM_SIGMA_KIND_CHOICE)

            # a refused write of a locked parameter (K-1): its status text, the unchanged value, and the read agrees
            locked_id = manifest["params"][LOCKED_PARAM]["link_id"]
            echo, at = client.set(locked_id, records[locked].value_bits + 1)
            assert value_bits(d, echo) == records[locked].value_bits
            texts = [m.text for m in client.since(at, "STATUSTEXT")]
            assert any(t.startswith(f"{locked_id}: ") for t in texts), texts
            value, record = client.read(locked)
            assert value_bits(d, value) == records[locked].value_bits and record.lock == 1
            assert record.staged_bits == record.active_bits == records[locked].value_bits

            # the source text of the accepted parameter: set in chunks, read back in chunks, reported by the record
            text = SOURCE_TEXT.encode()
            cls = d.MAVLink_marv_param_source_message
            size = dict(zip(cls.ordered_fieldnames, cls.array_lengths))["chunk"]
            assert len(text) > 2 * size, "the text must span chunks"
            at = client.mark()
            for off in range(0, len(text), size):
                part = text[off:off + size]
                client.send(cls(schema, accepted, off, len(text), *VEHICLE, d.MARV_PARAM_SOURCE_OP_SET, len(part), part))
            read_back = b""
            for off in range(0, len(text), size):
                asked = client.mark()
                client.send(cls(schema, accepted, off, 0, *VEHICLE, d.MARV_PARAM_SOURCE_OP_READ, 0, b""))
                m = client.wait(lambda m, off=off: m.get_type() == "MARV_PARAM_SOURCE" and m.offset == off
                                and m.operation == d.MARV_PARAM_SOURCE_OP_DATA, f"source chunk at {off}", asked)
                assert m.total_length == len(text) and m.param_schema_hash == schema
                read_back += text_of(m.chunk, m.chunk_length).encode()
            assert read_back == text and not client.since(at, "STATUSTEXT")
            _, record = client.read(accepted)
            assert record.source_length == len(text) and record.staged_bits == new_bits
            assert text_of(record.source_chunk, record.source_chunk_length).encode() == text[:record.source_chunk_length]

            assert client.identity().staged_differs == 1

            # (a) no report id: denied
            ack, _ = client.config(d.MARV_CONFIG_ACTION_ACTIVATE, bytes(mcfg.DIGEST), expected, schema)
            assert (ack.action, ack.result) == (d.MARV_CONFIG_ACTION_ACTIVATE, d.MARV_CONFIG_RESULT_DENIED_REPORT)
            assert gz.proc.poll() is None
            # (b) the real report id and the staged hash (the activation checks the staged config is the one the report is for): accepted,
            # and the run stops for the reboot
            ack, _ = client.config(d.MARV_CONFIG_ACTION_ACTIVATE, report_id, expected, schema)
            assert (ack.action, ack.result) == (d.MARV_CONFIG_ACTION_ACTIVATE, d.MARV_CONFIG_RESULT_ACCEPTED)
            assert gz.wait(REPLY_S) == os.EX_TEMPFAIL, run_scenario.stderr_tail(gz.stderr())
            assert "marv_gz_lockstep: REBOOT REQUESTED" in gz.stderr()
    finally:
        client.close()

    # the relaunch: the same world on the same config dir
    client = Client(d, port)
    try:
        with Gz(sdf, iterations(duration), tmp_path):
            client.connect()
            ident = client.identity()
            assert ident.config_source == d.MARV_CONFIG_SOURCE_ACTIVE and ident.config_state == d.MARV_CONFIG_STATE_OK
            assert ident.config_refusal == d.MARV_CONFIG_REFUSAL_NONE
            assert ident.last_activation_result == d.MARV_ACTIVATION_RESULT_ACTIVATED
            assert bytes(ident.config_sha256) == expected, "the reported config hash is not the staged config's"
            assert bytes(ident.validation_report_id) == report_id and bytes(ident.image_sha256) == image
            assert ident.staged_differs == 0
            value, record = client.read(accepted)
            assert value_bits(d, value) == new_bits and record.active_bits == record.staged_bits == new_bits
            assert record.source_length == len(SOURCE_TEXT)
    finally:
        client.close()

    # G7-1, the allowed half: the activated config is bound to this image and its report, so a further run on the same config dir is
    # not refused and arms on the raise (the control of the refused runs below)
    ident, heartbeats, run = arming_run(d, tmp_path, "activated", store)
    assert ident.config_source == d.MARV_CONFIG_SOURCE_ACTIVE and ident.config_state == d.MARV_CONFIG_STATE_OK
    assert bytes(ident.config_sha256) == expected and bytes(ident.validation_report_id) == report_id
    ticks = armed_ticks(run)
    assert ticks and ticks[0]["sil_t_us"] >= READY_US, "the validated, activated config did not arm on the raise"
    assert any(h.base_mode & d.MAV_MODE_FLAG_SAFETY_ARMED for h in heartbeats), "no heartbeat reported the vehicle armed"


# ---- arming with a stored config the store refuses --------------------------------------------------------------------------

def arming_run(d, tmp, name, config_dir):
    """The plumbing test's raise (after calibration and recovery) with the link served; (identity, heartbeats, log)."""
    tmp = tmp / name
    tmp.mkdir(exist_ok=True)
    port = free_udp_port()
    duration = READY_US + ARMED_US
    log = tmp / "run.bin"
    n = iterations(duration)
    sdf = world(tmp / "world.sdf", "test", log, script([seg(0, READY_US, SWITCH_LOW), seg(READY_US, ARMED_US, SWITCH_HIGH)]),
                config_dir, port)
    client = Client(d, port)
    try:
        with Gz(sdf, n, tmp) as gz:
            client.connect()
            ident = client.identity()
            deadline = time.monotonic() + TIMEOUT_S
            while gz.proc.poll() is None and time.monotonic() < deadline:
                client.pump()
            assert gz.wait() == 0 and "REFUSED" not in gz.stderr(), run_scenario.stderr_tail(gz.stderr())
            client.pump()
    finally:
        client.close()
    run = lockstep_log.read(log)
    assert run["trailer"] == {"steps": n, "ticks": n, "applied": n}
    raised = [m for m in run["manuals"] if m["channels"][ARM_CHANNEL - 1] == SWITCH_HIGH]
    assert raised and raised[0]["t_us"] >= READY_US
    return ident, [m for m in client.got if m.get_type() == "HEARTBEAT"], run


def armed_ticks(run):
    return [t for t in run["ticks"] if all(v >= DSHOT_THROTTLE_MIN for v in t["dshot"])]


def test_control_the_factory_config_arms_on_the_raise(dialect, tmp_path):
    d = dialect
    store = tmp_path / "store"
    store.mkdir()
    ident, heartbeats, run = arming_run(d, tmp_path, "factory", store)
    assert ident.config_source == d.MARV_CONFIG_SOURCE_FACTORY and ident.config_state == d.MARV_CONFIG_STATE_OK
    ticks = armed_ticks(run)
    assert ticks and ticks[0]["sil_t_us"] >= READY_US, "the factory config did not arm on the raise"
    assert any(h.base_mode & d.MAV_MODE_FLAG_SAFETY_ARMED for h in heartbeats), "no heartbeat reported the vehicle armed"


def test_c_an_unbound_stored_config_is_refused_and_never_arms(dialect, manifest, tmp_path):
    d = dialect
    store = tmp_path / "store"
    store.mkdir()
    write_slot(store, mcfg.encode(mcfg.factory_config(manifest, sequence=1)))
    ident, heartbeats, run = arming_run(d, tmp_path, "unbound", store)
    assert ident.config_source == d.MARV_CONFIG_SOURCE_ACTIVE and ident.config_state == d.MARV_CONFIG_STATE_REFUSED
    assert ident.config_refusal == d.MARV_CONFIG_REFUSAL_IMAGE_BINDING
    assert bytes(ident.validation_report_id) == bytes(mcfg.DIGEST)
    assert not armed_ticks(run) and all(all(v == 0 for v in t["dshot"]) for t in run["ticks"])
    assert heartbeats and not any(h.base_mode & d.MAV_MODE_FLAG_SAFETY_ARMED for h in heartbeats)


def test_d_a_corrupt_stored_config_is_refused_as_format_and_never_arms(dialect, manifest, tmp_path):
    d = dialect
    store = tmp_path / "store"
    store.mkdir()
    write_slot(store, mcfg.encode(mcfg.factory_config(manifest, sequence=1)), corrupt=True)
    ident, heartbeats, run = arming_run(d, tmp_path, "corrupt", store)
    assert ident.config_source == d.MARV_CONFIG_SOURCE_FACTORY and ident.config_state == d.MARV_CONFIG_STATE_REFUSED
    assert ident.config_refusal == d.MARV_CONFIG_REFUSAL_FORMAT
    assert not armed_ticks(run) and all(all(v == 0 for v in t["dshot"]) for t in run["ticks"])
    assert heartbeats and not any(h.base_mode & d.MAV_MODE_FLAG_SAFETY_ARMED for h in heartbeats)
