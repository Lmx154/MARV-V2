"""The sim channel's run-definition side and a reference decoder (ground-segment unit GX-8, decision 0056).

The lockstep plugin (sim/gz/plugin/src/sim_telemetry.hpp has the record layout and the argument that the channel does not perturb
the run) publishes a versioned record per period of SIM time when its world has the two elements this module writes:

  <sim_telemetry_udp_port>P</sim_telemetry_udp_port>      127.0.0.1:P, where the ground-station helper listens
  <sim_telemetry_period_us>T</sim_telemetry_period_us>    SIM microseconds between records, T = 1e6 / rate_hz

run_quad_fc.py (--sim-telemetry-udp-port, --sim-telemetry-hz) and run_hil_s4.py write them; without the port the world is byte-
identical to one without the option.

decode() is the Python reference decoder of record version 1: tests and evidence use it, the GCS has its own in
gcs/src/lib/sim/record.ts, and the two are checked against each other on the same bytes (a real plugin record is a fixture of
the GCS tests). The helper does not use it: it relays the datagrams unparsed.
"""

from __future__ import annotations

import struct

MAGIC = b"MSIM"
VERSION = 1
# Labelled harness value (decision 0056): the sim channel's default rate. A display rate: the GCS plots a few tens of points
# per second; it decides no result. PLACEHOLDER until Luis wants another rate.
DEFAULT_RATE_HZ = 20.0
MICROSECONDS_PER_SECOND = 1_000_000

HEADER = struct.Struct("<4sHHIIQQd")
TRUTH = struct.Struct("<3d3d4d3d3d4d")
INJECTED = struct.Struct("<3f3fI4H4H")
GROUND = struct.Struct("<If")
HIL = struct.Struct("<9I4f16f4f")
RECORD_BYTES = HEADER.size + TRUTH.size + INJECTED.size + GROUND.size + HIL.size

FLAGS = {"imu_model": 0, "launch_pad": 1, "ground": 2, "hil": 3, "pad_held": 4, "pad_released": 5}
HIL_MODES = ("unknown", "acro", "angle")


def period_us(rate_hz: float) -> int:
    """The record period in SIM microseconds of a rate: round(1e6 / rate_hz); ValueError for a rate that is not positive and finite."""
    if not isinstance(rate_hz, (int, float)) or isinstance(rate_hz, bool) or not 0.0 < rate_hz < float("inf"):
        raise ValueError(f"the sim channel rate must be a positive finite number of Hz, not {rate_hz!r}")
    return max(1, round(MICROSECONDS_PER_SECOND / rate_hz))


def sim_telemetry_xml(port: int | None, rate_hz: float = DEFAULT_RATE_HZ) -> str:
    """The plugin's two sim channel elements; "" without a port."""
    if port is None:
        return ""
    if isinstance(port, bool) or not isinstance(port, int) or not 0 < port < 65536:
        raise ValueError(f"the sim channel port must be a UDP port 1..65535, not {port!r}")
    return (f"<sim_telemetry_udp_port>{port}</sim_telemetry_udp_port>\n"
            f"<sim_telemetry_period_us>{period_us(rate_hz)}</sim_telemetry_period_us>\n")


def decode(data: bytes) -> dict:
    """The fields of one record (ValueError if the magic, the version or the length is not a record this decoder knows)."""
    if len(data) < HEADER.size:
        raise ValueError(f"{len(data)} bytes: shorter than the sim record header")
    magic, version, length, seq, flags, sim_time_ns, tick, rtf = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise ValueError(f"magic {magic!r} is not {MAGIC!r}")
    if version != VERSION:
        raise ValueError(f"sim record version {version} is not {VERSION}")
    if length != len(data) or length < RECORD_BYTES:
        raise ValueError(f"sim record length field {length}, datagram {len(data)} bytes, version {VERSION} needs {RECORD_BYTES}")
    off = HEADER.size
    t = TRUTH.unpack_from(data, off)
    off += TRUTH.size
    inj = INJECTED.unpack_from(data, off)
    off += INJECTED.size
    touchdowns, last_touchdown = GROUND.unpack_from(data, off)
    off += GROUND.size
    h = HIL.unpack_from(data, off)
    return {
        "seq": seq, "flags": {name: bool(flags >> bit & 1) for name, bit in FLAGS.items()}, "sim_time_ns": sim_time_ns,
        "tick": tick, "rtf": rtf,
        "pos_ned_m": t[0:3], "vel_ned_m_s": t[3:6], "q_wxyz": t[6:10], "omega_frd_rad_s": t[10:13],
        "specific_force_frd_m_s2": t[13:16], "rotor_speed_rad_s": t[16:20],
        "imu_gyro_rad_s": inj[0:3], "imu_accel_m_s2": inj[3:6], "imu_flags": inj[6],
        "dshot_commanded": inj[7:11], "dshot_applied": inj[11:15],
        "touchdowns": touchdowns, "last_touchdown_down_m_s": last_touchdown,
        "hil": {"arming_state": h[0], "status_records": h[1], "receiver_frames": h[2], "frame_counter": h[3],
                "mode": HIL_MODES[h[4]] if h[4] < len(HIL_MODES) else "unknown", "deadline_misses": h[5], "link_faults": h[6],
                "reports_used": h[7], "lag_events": h[8], "sticks": h[9:13], "channels": h[13:29], "link_quality": h[29],
                "margin_min_us": h[30], "margin_max_us": h[31], "lag_us": h[32]},
    }
