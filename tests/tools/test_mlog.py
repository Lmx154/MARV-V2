"""The onboard flight-log decoder (tools/log/mlog.py), invariant L-4 of the architect ruling 2026-10-18.

Files are built here by an encoder written from the format description (fw/log/include/marv/log/format.hpp), independent of the
C++ writer; the C++ writer and this decoder meet in the ctest log_roundtrip_decode (tools/log/roundtrip_check.py).

L-4: power loss leaves every sector before the last completed write decodable, and stale sectors (another session's) are
rejected. Covered: a cut at every sector boundary and inside a sector, a torn CRC, a stale sector of another session, a hole
(a missing sector) with resync, a repeated sector, a trailing zero region after the End record, records spanning sectors, the
unwrapping of the 32-bit stamps, the header and the CSV output.
"""

from __future__ import annotations

import struct
import sys
import zlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "log"))
import mlog  # noqa: E402

SESSION = 0x1122334455667788
OTHER = 0x99AABBCCDDEEFF00
PAYLOAD = mlog.PAYLOAD


def sector(session: int, seq: int, first: int, payload: bytes, version: int = 1) -> bytes:
    assert len(payload) <= PAYLOAD
    body = b"MLOG" + struct.pack("<QIHH", session, seq, first, version) + payload.ljust(PAYLOAD, b"\0")
    return body + struct.pack("<I", zlib.crc32(body))


def rec_lead(kind: int, sub: int, body: bytes) -> bytes:
    return struct.pack("<BBH", kind, sub, 4 + len(body)) + body


def rec_info(start_us: int = 0) -> bytes:
    body = struct.pack("<HHQ", 1, 512, start_us) + bytes(range(32)) + bytes(range(32, 64)) + struct.pack(
        "<QIIfI", 0xABCD, 1953125, 14649, 7500.3, 5) + bytes([1, 2, 3, 4, 5, 6, 0, 0])
    return rec_lead(8, 0, body)


def rec_field(kind: int, off: int, size: int, ftype: int, name: str, unit: str) -> bytes:
    return rec_lead(11, kind, struct.pack("<HBB", off, size, ftype) + name.encode() + b"\0" + unit.encode() + b"\0")


def rec_param(pid: int, name: str, value: float, unit: str) -> bytes:
    return rec_lead(10, 0, struct.pack("<HHf", pid, 0, value) + name.encode() + b"\0" + unit.encode() + b"\0")


def header_records() -> list[bytes]:
    f = [rec_field(1, 2, 2, 1, "flags", "1"), rec_field(1, 4, 4, 2, "t_lo_us", "us")]
    for i, n in enumerate(("gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z")):
        f.append(rec_field(1, 8 + 4 * i, 4, 4, n, "rad/s" if n.startswith("gyro") else "m/s2"))
    f += [rec_field(5, 1, 1, 0, "state", "1"), rec_field(5, 4, 8, 3, "t_us", "us"), rec_field(5, 12, 4, 2, "drops_imu", "1"),
          rec_field(6, 1, 1, 0, "reason", "1"), rec_field(6, 4, 8, 3, "t_us", "us"), rec_field(6, 12, 4, 2, "sectors", "1"),
          rec_field(4, 4, 4, 2, "t_lo_us", "us"), rec_field(4, 8, 4, 2, "dropped_imu", "1"),
          rec_field(4, 12, 4, 2, "dropped_rate_tick", "1"), rec_field(4, 16, 4, 2, "dropped_manual_control", "1")]
    return [rec_info(), *f, rec_param(7, "sd_log_write_stall_max_s", 0.5, "s")]


def rec_imu(t: int) -> bytes:
    return struct.pack("<BBHI6f", 1, 0, 0x180, t & 0xFFFFFFFF, t * 0.5, 1.0, -2.0, 0.25, 0.5, -9.5)


def rec_status(t: int, drops: int = 0) -> bytes:
    return rec_lead(5, 2, struct.pack("<QI", t, drops))


def rec_end(t: int, sectors: int) -> bytes:
    return rec_lead(6, 0, struct.pack("<QII", t, sectors, 0))


def rec_gap(t_lo: int, imu: int, rate: int = 0, man: int = 0) -> bytes:
    return struct.pack("<B3xI3I", 4, t_lo, imu, rate, man).ljust(32, b"\0")


def frame(records: list[bytes], session: int = SESSION, first_seq: int = 0) -> list[bytes]:
    """Records into sectors the way the framer does: spanning, first-record offset, zero padding at the end."""
    sectors: list[bytes] = []
    cur = bytearray()
    first = mlog.NO_RECORD
    seq = first_seq
    for r in records:
        begun = False
        done = 0
        while done < len(r):
            if not begun and first == mlog.NO_RECORD:
                first = len(cur)
            begun = True
            take = min(len(r) - done, PAYLOAD - len(cur))
            cur += r[done: done + take]
            done += take
            if len(cur) == PAYLOAD:
                sectors.append(sector(session, seq, first, bytes(cur)))
                seq += 1
                cur = bytearray()
                first = mlog.NO_RECORD
    if cur:
        sectors.append(sector(session, seq, first, bytes(cur)))
    return sectors


def good_log(n_imu: int = 60, t0: int = 1000) -> list[bytes]:
    recs = header_records() + [rec_status(t0)] + [rec_imu(t0 + 133 * i) for i in range(n_imu)]
    recs += [rec_status(t0 + 133 * n_imu), rec_end(t0 + 133 * n_imu, 0)]
    return frame(recs)


def test_a_valid_log_decodes_completely():
    s = good_log()
    dec = mlog.decode(b"".join(s))
    assert dec.clean, [str(p) for p in dec.problems]
    assert dec.session == SESSION
    assert dec.sectors_ok == len(s) == dec.sectors_total
    imus = mlog.records_of(dec, "imu")
    assert [r["t_us"] for r in imus] == [1000 + 133 * i for i in range(60)]
    assert imus[3]["gyro_x"] == pytest.approx(1399 * 0.5)
    assert imus[0]["accel_z"] == -9.5 and imus[0]["flags"] == 0x180
    assert len(mlog.records_of(dec, "status")) == 2
    assert mlog.records_of(dec, "end")[0]["reason"] == "disarm"
    assert mlog.records_of(dec, "status")[0]["state"] == "Logging"


def test_the_header_is_decoded():
    dec = mlog.decode(b"".join(good_log()))
    info = dec.header["info"]
    assert info["format_version"] == 1 and info["sector_bytes"] == 512
    assert info["tick_num_us"] == 1953125 and info["tick_den"] == 14649
    assert info["odr_hz"] == pytest.approx(7500.3, rel=1e-6)
    assert info["image_hash"] == bytes(range(32)).hex()
    assert info["boot"] == {"config_source": 1, "slot": 2, "config_status": 3, "fault": 4, "request": 5, "request_fault": 6,
                            "sequence": 5}
    assert dec.header["params"] == [{"id": 7, "name": "sd_log_write_stall_max_s", "type": "f32", "value": 0.5, "unit": "s",
                                     "locked": False}]


def test_records_span_sectors_and_none_is_lost():
    # 488-byte payloads and 32-byte records: records straddle sector edges (488 is not a multiple of 32).
    s = good_log(n_imu=200)
    assert len(s) > 10
    dec = mlog.decode(b"".join(s))
    assert dec.clean
    assert len(mlog.records_of(dec, "imu")) == 200


@pytest.mark.parametrize("k", range(0, 14))
def test_a_cut_at_a_sector_boundary_decodes_the_prefix(k):
    s = good_log(n_imu=300)
    assert len(s) >= 13
    dec = mlog.decode(b"".join(s[:k]))
    assert dec.stopped_at is None
    assert not dec.problems
    assert dec.sectors_ok == k
    assert not dec.ended
    # Every record decoded from the prefix is the genuine one (the same as in the whole file's first records).
    whole = [r for r in mlog.decode(b"".join(s)).records]
    for got, want in zip(dec.records, whole):
        assert got == want


@pytest.mark.parametrize("extra", [1, 20, 24, 300, 507, 511])
def test_a_cut_inside_a_sector_keeps_the_sectors_before_it(extra):
    s = good_log(n_imu=120)
    k = 6
    data = b"".join(s[:k]) + s[k][:extra]
    dec = mlog.decode(data)
    assert dec.sectors_ok == k
    assert [p.reason for p in dec.problems if p.reason == "partial_sector"] == ["partial_sector"]
    clean_prefix = mlog.decode(b"".join(s[:k]))
    assert dec.records == clean_prefix.records


@pytest.mark.parametrize("where", [0, 4, 12, 16, 18, 20, 100, 400, 507, 508, 511])
def test_a_torn_sector_stops_the_strict_decoder_exactly_there(where):
    s = good_log(n_imu=120)
    k = 5
    torn = bytearray(s[k])
    torn[where] ^= 0x40
    data = b"".join(s[:k]) + bytes(torn) + b"".join(s[k + 1:])
    dec = mlog.decode(data)
    assert dec.stopped_at == k
    assert dec.problems[0].sector == k
    assert dec.problems[0].reason in ("crc", "magic")
    assert dec.sectors_ok == k
    assert dec.records == mlog.decode(b"".join(s[:k])).records, "nothing after the torn sector is trusted"
    assert not dec.clean


def test_resync_skips_a_torn_sector_and_continues_at_a_record_boundary():
    s = good_log(n_imu=120)
    k = 5
    torn = bytearray(s[k])
    torn[100] ^= 0x01
    data = b"".join(s[:k]) + bytes(torn) + b"".join(s[k + 1:])
    dec = mlog.decode(data, resync=True)
    reasons = [p.reason for p in dec.problems]
    assert "crc" in reasons and "hole" in reasons
    assert dec.ended, "the End record is still found"
    imus = mlog.records_of(dec, "imu")
    ts = [r["t_us"] for r in imus]
    assert ts == sorted(ts) and len(set(ts)) == len(ts)
    assert 100 < len(imus) < 120  # one sector (at most 15 records) lost, the rest recovered
    assert all(t in {1000 + 133 * i for i in range(120)} for t in ts), "no record is garbled"


def test_a_stale_sector_of_another_session_is_rejected():
    s = good_log(n_imu=120)
    stale = frame(header_records() + [rec_imu(5)] * 40, session=OTHER, first_seq=5)
    k = 5
    data = b"".join(s[:k]) + stale[0] + b"".join(s[k + 1:])
    dec = mlog.decode(data)
    assert dec.stopped_at == k
    assert dec.problems[0].reason == "session"
    assert dec.sectors_ok == k
    # In resync mode the foreign sector is skipped as a stale one, and the file's own sectors go on.
    dec2 = mlog.decode(data, resync=True)
    assert [p.reason for p in dec2.problems][0] == "session"
    assert dec2.ended


def test_a_stale_region_after_the_end_record_is_never_read():
    s = good_log(n_imu=40)
    stale = frame([rec_imu(i) for i in range(200)], session=OTHER, first_seq=len(s))
    dec = mlog.decode(b"".join(s) + b"".join(stale))
    assert dec.clean, [str(p) for p in dec.problems]


def test_a_zero_filled_tail_after_the_end_is_not_a_problem_but_without_an_end_it_is_a_stop():
    s = good_log(n_imu=40)
    dec = mlog.decode(b"".join(s) + bytes(4 * 512))
    assert dec.clean
    dec2 = mlog.decode(b"".join(s[:-1]) + bytes(4 * 512))
    assert dec2.stopped_at == len(s) - 1 and dec2.problems[0].reason == "magic" and not dec2.ended


def test_a_missing_sector_is_a_hole_the_strict_decoder_stops_at_and_resync_crosses():
    s = good_log(n_imu=120)
    k = 7
    data = b"".join(s[:k]) + b"".join(s[k + 1:])
    strict = mlog.decode(data)
    assert strict.stopped_at == k and strict.problems[0].reason == "hole"
    assert strict.records == mlog.decode(b"".join(s[:k])).records
    dec = mlog.decode(data, resync=True)
    assert [p.reason for p in dec.problems if p.reason == "hole"] == ["hole"]
    assert dec.ended
    ts = [r["t_us"] for r in mlog.records_of(dec, "imu")]
    assert ts == sorted(ts) and len(ts) > 90
    missing = [1000 + 133 * i for i in range(120) if 1000 + 133 * i not in set(ts)]
    assert 0 < len(missing) < 40, "only the records of the lost sector (and the one cut by it) are missing"


def test_a_repeated_or_stale_sequence_number_is_rejected():
    s = good_log(n_imu=120)
    data = b"".join(s[:6]) + s[3] + b"".join(s[6:])
    dec = mlog.decode(data)
    assert dec.stopped_at == 6 and dec.problems[0].reason == "sequence"


def test_a_wrong_version_is_refused():
    s = good_log(n_imu=40)
    s[2] = sector(SESSION, 2, 0, b"", version=3)  # versions 1 and 2 are known (format version 2: the raw-count IMU record)
    dec = mlog.decode(b"".join(s))
    assert dec.stopped_at == 2 and dec.problems[0].reason == "version"


def test_the_first_record_offset_is_range_checked():
    s = good_log(n_imu=40)
    s[1] = sector(SESSION, 1, PAYLOAD + 1, b"")
    dec = mlog.decode(b"".join(s))
    assert dec.stopped_at == 1 and dec.problems[0].reason == "first_offset"


def test_the_32_bit_stamps_are_unwrapped_against_the_full_stamps():
    t0 = (1 << 32) - 3 * 133
    recs = header_records() + [rec_status(t0)] + [rec_imu(t0 + 133 * i) for i in range(10)] + [rec_end(t0 + 1330, 0)]
    dec = mlog.decode(b"".join(frame(recs)))
    ts = [r["t_us"] for r in mlog.records_of(dec, "imu")]
    assert ts == [t0 + 133 * i for i in range(10)]
    assert ts[-1] > (1 << 32)


def test_a_gap_record_is_decoded_with_its_counts():
    recs = header_records() + [rec_status(1000), rec_imu(1000), rec_gap(5000, 17, 3, 1), rec_imu(5000), rec_end(5000, 0)]
    dec = mlog.decode(b"".join(frame(recs)))
    gap = mlog.records_of(dec, "gap")[0]
    assert (gap["dropped_imu"], gap["dropped_rate_tick"], gap["dropped_manual_control"]) == (17, 3, 1)
    assert gap["t_us"] == 5000


def test_a_record_cut_by_a_lost_continuation_is_reported_not_garbled():
    # A record that begins in sector k and would continue in k+1, where k+1 begins a new record: the first is dropped.
    recs = header_records() + [rec_status(1000)] + [rec_imu(1000 + 133 * i) for i in range(30)] + [rec_end(9999, 0)]
    s = frame(recs)
    # Remove the continuation sector's leading bytes by faking first-record offset 0 on a copy of the next sector.
    k = 1
    payload = s[k][mlog.SEC_HEADER: mlog.SEC_HEADER + PAYLOAD]
    first = struct.unpack_from("<H", s[k], 16)[0]
    assert first != mlog.NO_RECORD
    forged = sector(SESSION, k, 0, payload[first:] + bytes(first))
    dec = mlog.decode(b"".join([s[0], forged, *s[2:]]))
    assert any(p.reason == "record" for p in dec.problems)
    assert all(r["gyro_x"] == pytest.approx(r["t_us"] * 0.5) for r in mlog.records_of(dec, "imu"))


def test_cli_exit_status_csv_and_header(tmp_path, capsys):
    good = tmp_path / "good.mlog"
    good.write_bytes(b"".join(good_log(n_imu=30)))
    out = tmp_path / "csv"
    assert mlog.main([str(good), "--csv-dir", str(out), "--header"]) == 0
    text = capsys.readouterr().out
    assert "end record: yes" in text and '"sd_log_write_stall_max_s"' in text
    assert (out / "imu.csv").read_text().splitlines()[0].startswith("kind,sector,flags,t_lo_us,gyro_x")
    assert len((out / "imu.csv").read_text().splitlines()) == 31
    assert (out / "header.json").is_file()
    cut = tmp_path / "cut.mlog"
    cut.write_bytes(good.read_bytes()[: 3 * 512 + 9])
    assert mlog.main([str(cut)]) == 1
    assert "partial_sector" in capsys.readouterr().out


# ---- format version 2: the raw-count IMU record (unit U4-3; invariant V2-1) --------------------------------------------------
# The records are built here from the description in format.hpp (imu_raw, header_imu_raw), independent of the C++ writer; the C++
# writer meets this decoder in the ctest log_imu_raw_roundtrip_* (tools/log/roundtrip_check.py --imu-raw).

import math  # noqa: E402

# The scales of lsm6dsv_scales() at the full scales the firmware runs the part at (boards/parts/lsm6dsv.hpp: gyro +-2000 dps,
# 70 mdps/LSB; accel +-16 g, 0.488 mg/LSB), in binary32.
GYRO_SCALE = struct.unpack("<f", struct.pack("<f", 0.070 * math.pi / 180.0))[0]
ACCEL_SCALE = struct.unpack("<f", struct.pack("<f", 0.488e-3 * 9.80665))[0]
FULL_SCALES = {"gyro 2000 dps / accel 16 g": (GYRO_SCALE, ACCEL_SCALE)}
BOARD_SIGNS = (-1, 1, -1)  # the madflight FC3v2 Rev-C mounting map
GYRO_VALID, ACCEL_VALID, TEMP_VALID = 1 << 6, 1 << 7, 1 << 8  # imu_flag bits of the 32-byte record
RAW_GYRO, RAW_ACCEL, RAW_TEMP = 1, 2, 4  # flag byte of the imu_raw record
SAT_X_GYRO, SAT_X_ACCEL = 0, 3


def f32(x: float) -> float:
    return struct.unpack("<f", struct.pack("<f", x))[0]


def rec_hdr_imu_raw(scales=(GYRO_SCALE, ACCEL_SCALE), signs=BOARD_SIGNS) -> bytes:
    return rec_lead(14, 0, struct.pack("<ff3b", scales[0], scales[1], *signs))


def header_records_v2(scales=(GYRO_SCALE, ACCEL_SCALE), signs=BOARD_SIGNS) -> list[bytes]:
    names = ("gyro_x_raw", "gyro_y_raw", "gyro_z_raw", "accel_x_raw", "accel_y_raw", "accel_z_raw")
    f = [rec_field(13, 1, 1, 0, "flags", "1"), rec_field(13, 2, 2, 1, "t_lo16_us", "us")]
    f += [rec_field(13, 4 + 2 * i, 2, 6, n, "lsb") for i, n in enumerate(names)]
    f += [rec_field(2, 4, 4, 2, "now_lo_us", "us"), rec_field(2, 1, 1, 0, "arm_state", "1")]
    base = header_records()
    return [base[0], *f, *base[1:], rec_hdr_imu_raw(scales, signs)]


def rec_imu_raw(t16: int, counts, flags: int = RAW_GYRO | RAW_ACCEL) -> bytes:
    return struct.pack("<BBH6h", 13, flags, t16 & 0xFFFF, *counts)


def rec_rate(t_lo: int) -> bytes:
    return struct.pack("<BBBBI", 2, 1, 2, 0, t_lo & 0xFFFFFFFF).ljust(16, b"\0")


def rec_imu_v1_from_counts(t: int, counts, flags_raw: int, scales, signs) -> bytes:
    """The 32-byte record the firmware writes for the same sample: lsm6dsv_decode.cpp to_frd (sign x count x scale in float) and
    the flag rules of imu_sample.hpp, computed here from the description, not by mlog."""
    gyro, accel = counts[:3], counts[3:]
    flags = 0
    values = []
    for valid_raw, valid_bit, sat_x, cs, scale in ((RAW_GYRO, GYRO_VALID, SAT_X_GYRO, gyro, scales[0]),
                                                  (RAW_ACCEL, ACCEL_VALID, SAT_X_ACCEL, accel, scales[1])):
        ok = bool(flags_raw & valid_raw)
        if ok:
            flags |= valid_bit
        for i, c in enumerate(cs):
            values.append(f32(float(signs[i]) * float(c) * scale) if ok else 0.0)
            if ok and c in (-32768, 32767):
                flags |= 1 << (sat_x + i)
    if flags_raw & RAW_TEMP:
        flags |= TEMP_VALID
    return struct.pack("<BBHI6f", 1, 0, flags, t & 0xFFFFFFFF, *values)


def sector_v2(session: int, seq: int, first: int, payload: bytes) -> bytes:
    return sector(session, seq, first, payload, version=2)


def frame_v2(records: list[bytes], session: int = SESSION) -> list[bytes]:
    return [sector_v2(session, i, int.from_bytes(s[16:18], "little"), s[mlog.SEC_HEADER: mlog.SEC_HEADER + PAYLOAD])
            for i, s in enumerate(frame(records, session))]


def v2_log(records: list[bytes], t0: int = 1000, scales=(GYRO_SCALE, ACCEL_SCALE), signs=BOARD_SIGNS) -> bytes:
    recs = header_records_v2(scales, signs) + [rec_status(t0)] + records + [rec_end(t0, 0)]
    return b"".join(frame_v2(recs))


def bits_of(rows: list[dict]) -> list[tuple]:
    keys = ("flags", "gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z")
    return [(r["flags"], *(struct.pack("<f", r[k]) for k in keys[1:])) for r in rows]


@pytest.mark.parametrize("signs", [BOARD_SIGNS, (1, 1, 1), (-1, -1, -1)])
@pytest.mark.parametrize("fs", sorted(FULL_SCALES))
def test_v2_decodes_to_the_v1_values_bit_for_bit_for_every_int16_count(fs, signs):
    # V2-1. Every int16 count passes every axis of both sensors (each axis is offset by a third of the range), at the full scales
    # the firmware uses and each mounting sign; the reference is the 32-byte record built from the description of the firmware's
    # float expression. Equal means the same binary32 bit patterns, so -0.0 (a negative sign on a zero count) is checked too.
    scales = FULL_SCALES[fs]
    third = 65536 // 3
    v1, v2 = [], []
    for i in range(65536):
        counts = [((i + k * third) % 65536) - 32768 for k in range(6)]
        v1.append(rec_imu_v1_from_counts(5000 + 133 * i, counts, RAW_GYRO | RAW_ACCEL, scales, signs))
        v2.append(rec_imu_raw(5000 + 133 * i, counts))
    d1 = mlog.decode(v2_log(v1 + [rec_rate(5000 + 133 * 65536)], 5000, scales, signs))
    d2 = mlog.decode(v2_log(v2 + [rec_rate(5000 + 133 * 65536)], 5000, scales, signs))
    assert d1.clean, [str(p) for p in d1.problems]
    assert d2.clean, [str(p) for p in d2.problems]
    r1, r2 = mlog.records_of(d1, "imu"), mlog.records_of(d2, "imu")
    assert len(r1) == len(r2) == 65536
    assert bits_of(r2) == bits_of(r1)
    assert [r["t_us"] for r in r2] == [r["t_us"] for r in r1] == [5000 + 133 * i for i in range(65536)]
    assert [r["t_lo_us"] for r in r2] == [r["t_lo_us"] for r in r1]
    assert list(r2[0]) == list(r1[0]), "the same columns in the same order, so a reader cannot tell the kinds apart"
    assert any(r["flags"] & (7 << SAT_X_GYRO) for r in r2) and any(r["flags"] & (7 << SAT_X_ACCEL) for r in r2)


def test_v2_flags_and_a_sensor_that_is_not_valid_read_like_v1():
    cases = [(RAW_GYRO, [100, -200, 300, 0, 0, 0]), (RAW_ACCEL, [0, 0, 0, 32767, -32768, 5]),
             (RAW_GYRO | RAW_ACCEL | RAW_TEMP, [-32768, 32767, 0, 7, 8, 9]), (0, [0] * 6), (RAW_TEMP, [0] * 6)]
    v1 = [rec_imu_v1_from_counts(2000 + 133 * i, c, f, (GYRO_SCALE, ACCEL_SCALE), BOARD_SIGNS) for i, (f, c) in enumerate(cases)]
    v2 = [rec_imu_raw(2000 + 133 * i, c, f) for i, (f, c) in enumerate(cases)]
    a = mlog.records_of(mlog.decode(v2_log(v1 + [rec_rate(2000 + 133 * 5)])), "imu")
    b = mlog.records_of(mlog.decode(v2_log(v2 + [rec_rate(2000 + 133 * 5)])), "imu")
    assert bits_of(a) == bits_of(b)
    assert b[1]["gyro_x"] == 0.0 and math.copysign(1.0, b[1]["gyro_x"]) == 1.0, "an invalid sensor reads +0, not -0"
    assert b[0]["flags"] == GYRO_VALID and b[2]["flags"] & TEMP_VALID


def test_a_v1_file_and_a_file_with_both_imu_kinds_still_decode():
    # The sample of unit U1 (version 1 sectors) is untouched by version 2.
    dec = mlog.decode(b"".join(good_log(n_imu=60)))
    assert dec.clean and len(mlog.records_of(dec, "imu")) == 60
    mixed = [rec_imu_raw(1000, [1, 2, 3, 4, 5, 6]), rec_imu(1133), rec_imu_raw(1266, [-1, -2, -3, -4, -5, -6]), rec_rate(1266)]
    d = mlog.decode(v2_log(mixed))
    assert d.clean, [str(p) for p in d.problems]
    assert [r["t_us"] for r in mlog.records_of(d, "imu")] == [1000, 1133, 1266]


def test_the_16_bit_times_unwrap_exactly_across_16_and_32_bit_wraps():
    # Steps up to 65535 us (the longest the chain follows), over many 2^16 wraps and across 2^32; a rate tick after each sample
    # is the 32-bit stamp that anchors the chain, and the first sample lies 0.3 s before the status stamp (a pre-roll).
    import random
    rng = random.Random(3)
    t0 = (1 << 32) - 200_000
    truth = [t0 - 300_000]
    for _ in range(3000):
        truth.append(truth[-1] + rng.choice([1, 133, 134, 2000, 40000, 65535, rng.randrange(1, 65536)]))
    assert truth[-1] > (1 << 32) + (1 << 20)
    recs = []
    for t in truth:
        recs += [rec_imu_raw(t, [1, 2, 3, 4, 5, 6]), rec_rate(t)]
    d = mlog.decode(v2_log(recs, t0))
    assert d.clean, [str(p) for p in d.problems]
    assert [r["t_us"] for r in mlog.records_of(d, "imu")] == truth
    assert [r["t_us"] for r in mlog.records_of(d, "rate_tick")] == truth


def _stream(n: int, step: int = 133, t0: int = 100_000) -> list[tuple[int, bytes]]:
    out = []
    for i in range(n):
        t = t0 + step * i
        out.append((t, rec_imu_raw(t, [i % 100] * 6)))
        if i % 2 == 1:
            out.append((t, rec_rate(t)))
    return out


def _decode_stream(items: list[bytes]):
    return mlog.decode(v2_log(items, 100_000))


def test_a_loss_over_65_ms_without_a_gap_record_is_reported_and_the_controls_are_not():
    # Negative control (V2-1): the planted file loses 600 ticks = 79.8 ms of every record with no Gap record. The 16-bit chain
    # cannot see the 65.536 ms wrap it missed; the 32-bit stamp of the rate tick must.
    s = _stream(2000)
    lost = [r for t, r in s if not 100_000 + 133 * 700 <= t < 100_000 + 133 * 1300]
    planted = _decode_stream(lost)
    assert [p.reason for p in planted.problems] == ["imu_time"], [str(p) for p in planted.problems]
    assert "65 ms" in planted.problems[0].detail
    # Control 1: the same loss announced by a Gap record (the writer's way) is no problem, and every time is exact.
    t_after = 100_000 + 133 * 1300
    announced = []
    for t, r in s:
        if t_after - 133 * 600 <= t < t_after:
            if t == t_after - 133 * 600 and r[0] == 13:
                announced.append(rec_gap(t_after, 600))
            continue
        announced.append(r)
    d = _decode_stream(announced)
    assert d.clean, [str(p) for p in d.problems]
    assert [r["t_us"] for r in mlog.records_of(d, "imu")] == [t for t, r in s if r[0] == 13 and not (t_after - 133 * 600 <= t < t_after)]
    # Control 2: a loss under 65.536 ms without a Gap record (300 ticks = 39.9 ms) is followed exactly and is not reported.
    short = [r for t, r in s if not 100_000 + 133 * 700 <= t < 100_000 + 133 * 1000]
    d2 = _decode_stream(short)
    assert d2.clean, [str(p) for p in d2.problems]
    assert [r["t_us"] for r in mlog.records_of(d2, "imu")] == [t for t, r in s if r[0] == 13 and not 100_000 + 133 * 700 <= t < 100_000 + 133 * 1000]
    # Control 3: no loss at all.
    d3 = _decode_stream([r for t, r in s])
    assert d3.clean and len(mlog.records_of(d3, "imu")) == 2000


@pytest.mark.parametrize("lost_ticks", [493, 494, 600, 1000, 4000])
def test_every_loss_over_65_ms_is_reported_wherever_it_falls(lost_ticks):
    # 493 ticks of 133 us are 65.57 ms, the shortest loss that skips a wrap of the 16-bit time.
    s = _stream(7000)
    for start in (500, 777, 1500, 2001):
        lost = [r for t, r in s if not 100_000 + 133 * start <= t < 100_000 + 133 * (start + lost_ticks)]
        d = _decode_stream(lost)
        assert any(p.reason == "imu_time" for p in d.problems), (lost_ticks, start)


def test_a_loss_just_under_65_ms_is_followed_exactly():
    s = _stream(3000)
    lost = [r for t, r in s if not 100_000 + 133 * 800 <= t < 100_000 + 133 * 1290]  # 490 ticks = 65.2 ms of lost samples
    d = _decode_stream(lost)
    assert d.clean, [str(p) for p in d.problems]
    assert [r["t_us"] for r in mlog.records_of(d, "imu")] == [t for t, r in s if r[0] == 13 and not 100_000 + 133 * 800 <= t < 100_000 + 133 * 1290]


def test_imu_raw_records_with_no_stamp_to_anchor_them_are_reported():
    d = mlog.decode(v2_log([rec_imu_raw(5000, [1] * 6), rec_imu_raw(5133, [2] * 6)]))
    assert [p.reason for p in d.problems] == ["imu_time"]
    assert len(mlog.records_of(d, "imu")) == 2


def test_a_missing_header_record_is_reported_not_guessed():
    recs = [r for r in header_records_v2() if r[0] != 14] + [rec_status(1000), rec_imu_raw(1000, [1] * 6), rec_end(1000, 0)]
    d = mlog.decode(b"".join(frame_v2(recs)))
    assert any(p.reason == "record" for p in d.problems)
    assert mlog.records_of(d, "imu") == []


def test_the_v2_header_is_decoded():
    d = mlog.decode(v2_log([rec_imu_raw(1000, [1] * 6), rec_rate(1000)]))
    assert d.header["imu_raw"] == {"gyro_rad_s_per_lsb": GYRO_SCALE, "accel_m_s2_per_lsb": ACCEL_SCALE, "sign": [-1, 1, -1]}
