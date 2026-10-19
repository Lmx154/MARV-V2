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
    s[2] = sector(SESSION, 2, 0, b"", version=2)
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
