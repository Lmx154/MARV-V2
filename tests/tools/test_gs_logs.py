"""The helper's log library (tools/gs/logs.py, ground-segment unit GX-15): .mlog files of the onboard logger pulled from the SD card.

The sample is tests/tools/data/u1_sample.mlog, built by the independent encoder of tests/tools/test_mlog.py (logger unit U1); the
decoder is U1's tools/log/mlog.py, and the expected values here come from calling it directly.

  - the summary and the series the helper returns equal the decoder's own records (with gap records too);
  - a copy with a corrupted sector is imported, with the decoder's report of it attached, and its series equal the decoder's;
  - the SHA-256 is recorded (the file name, the sidecar, the reply) and the source is not modified;
  - path refusals, each with the accepted control: relative, traversal, not normalised, NUL, not *.mlog, a directory, a symbolic link,
    a link into the repository, a path inside the repository, inside the library, a missing, an empty, an oversized file and a FIFO;
  - W-1: the repository's git status (ignored files included) is the same after an import, and the library is outside the repository;
  - the card listing, a decoder failure that is reported and does not stop the import, and the refusals of a series request.
"""

import dataclasses
import hashlib
import json
import os
import struct
import subprocess
import sys
from collections import Counter
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "tools" / "gs"))
sys.path.insert(0, str(ROOT / "tools" / "log"))
import logs as logs_mod  # noqa: E402
import mlog  # noqa: E402
import test_gs_helper as gh  # noqa: E402
import test_mlog as tm  # noqa: E402

SAMPLE = HERE / "data" / "u1_sample.mlog"
SAMPLE_IMU = 600  # records of the sample (tests/tools/data/u1_sample.mlog = good_log(600))
CORRUPT_SECTOR = 5  # labelled harness value: the sector the corrupted copy is damaged in


def sample_bytes() -> bytes:
    return SAMPLE.read_bytes()


def corrupt(data: bytes, sector: int = CORRUPT_SECTOR) -> bytes:
    """One payload byte of the sector flipped: its CRC no longer matches."""
    i = sector * mlog.SECTOR + mlog.SEC_HEADER + 7
    return data[:i] + bytes([data[i] ^ 0xFF]) + data[i + 1:]


def with_gaps() -> bytes:
    recs = tm.header_records() + [tm.rec_status(1000)]
    for i in range(30):
        recs.append(tm.rec_imu(1000 + 133 * i))
        if i in (9, 19):
            recs.append(tm.rec_gap((1000 + 133 * i) & 0xFFFFFFFF, imu=3 + i, rate=1, man=0))
    recs += [tm.rec_status(6000), tm.rec_end(6000, 0)]
    return b"".join(tm.frame(recs))


def run(tmp_path, body, **kw):
    return gh.run_with_helper(tmp_path, lambda h: with_ws(h, body), **kw)


async def with_ws(h, body):
    async with gh.ws_connect(h) as ws:
        return await body(h, ws)


def put(tmp_path, name, data: bytes) -> str:
    p = tmp_path / "src" / name
    p.parent.mkdir(exist_ok=True)
    p.write_bytes(data)
    return str(p)


def expected_summary(data: bytes) -> dict:
    dec = mlog.decode(data)
    kinds = {}
    for kind in {r["kind"] for r in dec.records}:
        rows = mlog.records_of(dec, kind)
        ts = [r["t_us"] for r in rows if "t_us" in r]
        kinds[kind] = {"count": len(rows), "t_min_us": min(ts) if ts else None, "t_max_us": max(ts) if ts else None}
    gaps = mlog.records_of(dec, "gap")
    return {"sectors_ok": dec.sectors_ok, "sectors_total": dec.sectors_total, "session": f"{dec.session:#018x}", "ended": dec.ended,
            "stopped_at": dec.stopped_at, "clean": dec.clean,
            "problems": [{"sector": p.sector, "reason": p.reason, "detail": p.detail} for p in dec.problems],
            "records": dict(Counter(r["kind"] for r in dec.records)), "kinds": kinds, "gap_count": len(gaps),
            "dropped": {"imu": sum(g["dropped_imu"] for g in gaps), "rate_tick": sum(g["dropped_rate_tick"] for g in gaps),
                        "manual_control": sum(g["dropped_manual_control"] for g in gaps)} if gaps else {}}


def check_summary(summary: dict, data: bytes) -> None:
    exp = expected_summary(data)
    for key in ("sectors_ok", "sectors_total", "session", "ended", "stopped_at", "clean", "problems", "records"):
        assert summary[key] == exp[key], key
    assert {k: {f: v[f] for f in ("count", "t_min_us", "t_max_us")} for k, v in summary["kinds"].items()} == exp["kinds"]
    assert summary["gaps"]["count"] == exp["gap_count"] and summary["gaps"]["dropped"] == exp["dropped"]
    assert len(summary["gaps"]["shown"]) == exp["gap_count"]


def expected_series(data: bytes, kind: str, fields: list, t0: int, t1: int, buckets: int) -> dict:
    """Min and max per equal time bucket, from the decoder's records: bucket i is t0 + i*w <= t_us < t0 + (i+1)*w, w = ceil((t1-t0+1)/N)."""
    width = -(-(t1 - t0 + 1) // buckets)
    rows = [r for r in mlog.decode(data).records if r["kind"] == kind and t0 <= r["t_us"] <= t1]
    out = {"bucket_us": width, "n": [0] * buckets, "fields": {f: {"min": [None] * buckets, "max": [None] * buckets} for f in fields}}
    for r in rows:
        i = (r["t_us"] - t0) // width
        out["n"][i] += 1
        for f in fields:
            lo, hi = out["fields"][f]["min"], out["fields"][f]["max"]
            lo[i] = r[f] if lo[i] is None else min(lo[i], r[f])
            hi[i] = r[f] if hi[i] is None else max(hi[i], r[f])
    return out


def check_series(reply: dict, data: bytes, kind: str, fields: list, t0: int, t1: int, buckets: int) -> None:
    assert reply["ok"], reply
    s = reply["series"]
    exp = expected_series(data, kind, fields, t0, t1, buckets)
    assert (s["kind"], s["t0_us"], s["t1_us"], s["buckets"]) == (kind, t0, t1, buckets)
    assert s["bucket_us"] == exp["bucket_us"] and s["n"] == exp["n"] and s["records"] == sum(exp["n"])
    for f in fields:
        assert s["fields"][f]["min"] == exp["fields"][f]["min"] and s["fields"][f]["max"] == exp["fields"][f]["max"], f


def test_the_sample_is_what_the_u1_encoder_builds():
    assert sample_bytes() == b"".join(tm.good_log(SAMPLE_IMU))
    assert mlog.decode(sample_bytes()).clean


def test_summary_and_series_equal_what_the_decoder_returns(tmp_path):
    data = sample_bytes()
    src = put(tmp_path, "sample.mlog", data)

    async def body(h, ws):
        r = await gh.control(ws, cmd="log_import", path=src)
        assert r["ok"] and r["size"] == len(data) and r["already_imported"] is False
        check_summary(r["summary"], data)
        assert r["summary"]["clean"] and r["summary"]["records"]["imu"] == SAMPLE_IMU
        assert "gyro_x" in r["summary"]["kinds"]["imu"]["fields"]
        t_min, t_max = r["summary"]["kinds"]["imu"]["t_min_us"], r["summary"]["kinds"]["imu"]["t_max_us"]
        sha = r["sha256"]
        # The whole range, a sub-range, one bucket, more buckets than records, and two kinds.
        for kind, fields, t0, t1, n in (("imu", ["gyro_x", "accel_z"], t_min, t_max, 7), ("imu", ["gyro_x"], t_min + 5000, t_max - 9000, 13),
                                         ("imu", ["gyro_y", "accel_x", "gyro_z"], t_min, t_max, 1), ("imu", ["gyro_x"], t_min, t_max, 2000),
                                         ("status", ["drops_imu"], 0, t_max + 1, 5)):
            check_series(await gh.control(ws, cmd="log_series", sha256=sha, kind=kind, fields=fields, t0_us=t0, t1_us=t1, buckets=n),
                         data, kind, fields, t0, t1, n)
        # The defaults: the kind's whole range at the default bucket count.
        d = await gh.control(ws, cmd="log_series", sha256=sha, kind="imu", fields=["gyro_x"])
        check_series(d, data, "imu", ["gyro_x"], t_min, t_max, logs_mod.SERIES_BUCKETS_DEFAULT)
        # The listing carries the same summary.
        listed = await gh.control(ws, cmd="log_list")
        assert [e["sha256"] for e in listed["logs"]] == [sha] and listed["logs"][0]["summary"] == r["summary"]

    run(tmp_path, body)


def test_gap_records_are_counted_as_the_decoder_reads_them(tmp_path):
    data = with_gaps()
    src = put(tmp_path, "gaps.mlog", data)

    async def body(h, ws):
        r = await gh.control(ws, cmd="log_import", path=src)
        assert r["ok"]
        check_summary(r["summary"], data)
        assert r["summary"]["gaps"]["count"] == 2 and r["summary"]["gaps"]["dropped"]["imu"] == (3 + 9) + (3 + 19)

    run(tmp_path, body)


def test_a_non_finite_value_is_skipped_and_counted_not_sent(tmp_path):
    nan = struct.pack("<BBHI6f", 1, 0, 0x180, 1133, float("nan"), 1.0, -2.0, 0.25, 0.5, -9.5)
    recs = tm.header_records() + [tm.rec_status(1000), tm.rec_imu(1000), nan, tm.rec_imu(1266), tm.rec_status(1400), tm.rec_end(1400, 0)]
    src = put(tmp_path, "nan.mlog", b"".join(tm.frame(recs)))

    async def body(h, ws):
        sha = (await gh.control(ws, cmd="log_import", path=src))["sha256"]
        r = await gh.control(ws, cmd="log_series", sha256=sha, kind="imu", fields=["gyro_x", "gyro_y"], buckets=1)
        assert r["ok"] and r["series"]["n"] == [3]
        assert r["series"]["fields"]["gyro_x"] == {"min": [500.0], "max": [633.0], "skipped": 1}
        assert r["series"]["fields"]["gyro_y"]["skipped"] == 0
        json.dumps(r, allow_nan=False)  # the reply is strict JSON

    run(tmp_path, body)


def test_a_corrupted_sector_is_reported_and_the_import_still_succeeds(tmp_path):
    good = sample_bytes()
    bad = corrupt(good)
    assert bad != good
    src = put(tmp_path, "cut.mlog", bad)

    async def body(h, ws):
        r = await gh.control(ws, cmd="log_import", path=src)
        assert r["ok"], r
        s = r["summary"]
        check_summary(s, bad)
        assert s["clean"] is False and s["ended"] is False
        assert s["problems"] == [{"sector": CORRUPT_SECTOR, "reason": "crc", "detail": ""}] and s["stopped_at"] == CORRUPT_SECTOR
        assert s["sectors_ok"] == CORRUPT_SECTOR and 0 < s["records"]["imu"] < SAMPLE_IMU
        # Control: the same file undamaged is clean.
        assert (await gh.control(ws, cmd="log_import", path=put(tmp_path, "good.mlog", good)))["summary"]["problems"] == []
        # What the decoder could read is still plotted, equal to the decoder's.
        t0, t1 = s["kinds"]["imu"]["t_min_us"], s["kinds"]["imu"]["t_max_us"]
        check_series(await gh.control(ws, cmd="log_series", sha256=r["sha256"], kind="imu", fields=["gyro_x"], t0_us=t0, t1_us=t1, buckets=9),
                     bad, "imu", ["gyro_x"], t0, t1, 9)
        assert len((await gh.control(ws, cmd="log_list"))["logs"]) == 2

    run(tmp_path, body)


def test_the_sha256_is_recorded_and_the_source_is_not_modified(tmp_path):
    data = sample_bytes()
    src = put(tmp_path, "sample.mlog", data)
    os.chmod(src, 0o440)
    before = os.stat(src)
    sha = hashlib.sha256(data).hexdigest()

    async def body(h, ws):
        r = await gh.control(ws, cmd="log_import", path=src)
        assert r["sha256"] == sha and r["source"] == src and r["name"] == "sample.mlog"
        lib = h.run_dir / "logs"
        assert (lib / f"{sha}.mlog").read_bytes() == data
        assert json.loads((lib / f"{sha}.json").read_text())["sha256"] == sha
        assert sorted(p.name for p in lib.iterdir()) == [f"{sha}.json", f"{sha}.mlog"]  # no partial file left behind
        # The same bytes again: the same library file, told so.
        again = await gh.control(ws, cmd="log_import", path=src)
        assert again["sha256"] == sha and again["already_imported"] is True
        assert len((await gh.control(ws, cmd="log_list"))["logs"]) == 1

    run(tmp_path, body)
    after = os.stat(src)
    assert Path(src).read_bytes() == data
    assert (after.st_mtime_ns, after.st_ctime_ns, after.st_mode, after.st_size, after.st_ino) == (
        before.st_mtime_ns, before.st_ctime_ns, before.st_mode, before.st_size, before.st_ino)


def git_state():
    return subprocess.run(["git", "status", "--porcelain", "--ignored"], cwd=ROOT, check=True, capture_output=True, text=True).stdout


def test_an_import_leaves_the_repository_as_it_was(tmp_path):
    src = put(tmp_path, "sample.mlog", sample_bytes())
    before = git_state()

    async def body(h, ws):
        r = await gh.control(ws, cmd="log_import", path=src)
        assert r["ok"]
        await gh.control(ws, cmd="log_series", sha256=r["sha256"], kind="imu", fields=["gyro_x"])
        lib = (h.run_dir / "logs").resolve()
        assert ROOT.resolve() not in lib.parents, "the library is outside the repository"

    run(tmp_path, body)
    assert git_state() == before


def test_paths_are_refused_and_nothing_is_written(tmp_path):
    data = sample_bytes()
    good = put(tmp_path, "good.mlog", data)
    (tmp_path / "dir.mlog").mkdir()
    os.symlink(good, tmp_path / "link.mlog")
    os.symlink(SAMPLE.parent, tmp_path / "into_repo")
    empty = put(tmp_path, "empty.mlog", b"")
    notmlog = put(tmp_path, "sample.bin", data)
    fifo = tmp_path / "fifo.mlog"
    os.mkfifo(fifo)
    repo_file = str(SAMPLE)
    repo_other = str(ROOT / "tools" / "log" / "mlog.py")
    refused = {
        "relative": "src/good.mlog",
        "empty path": "",
        "traversal": f"{tmp_path}/src/../src/good.mlog",
        "traversal out": f"{tmp_path}/src/../../../etc/x.mlog",
        "doubled slash": f"{tmp_path}//src/good.mlog",
        "dot": f"{tmp_path}/./src/good.mlog",
        "trailing slash": good + "/",
        "NUL": good + "\0",
        "not .mlog": notmlog,
        "only the suffix": str(tmp_path / ".mlog"),
        "a directory": str(tmp_path / "dir.mlog"),
        "a symbolic link": str(tmp_path / "link.mlog"),
        "through a link into the repository": str(tmp_path / "into_repo" / "u1_sample.mlog"),
        "inside the repository": repo_file,
        "inside the repository, other name": repo_other,
        "missing": str(tmp_path / "src" / "missing.mlog"),
        "empty file": empty,
        "a FIFO": str(fifo),
    }

    async def body(h, ws):
        for why, path in refused.items():
            r = await gh.control(ws, cmd="log_import", path=path)
            assert r["ok"] is False and r["error"], why
        assert (await gh.control(ws, cmd="log_import"))["ok"] is False  # no path at all
        assert (await gh.control(ws, cmd="log_import", path=5))["ok"] is False
        lib = h.run_dir / "logs"
        assert not lib.exists() or list(lib.iterdir()) == [], "a refusal writes nothing"
        # Control: the accepted path of the same bytes imports.
        assert (await gh.control(ws, cmd="log_import", path=good))["ok"] is True
        # The library's own files are not imported again.
        sha = hashlib.sha256(data).hexdigest()
        inside = await gh.control(ws, cmd="log_import", path=str(lib / f"{sha}.mlog"))
        assert inside["ok"] is False and "library" in inside["error"]

    run(tmp_path, body)


def test_the_size_limit_refuses_and_the_control_imports(tmp_path, monkeypatch):
    src = put(tmp_path, "sample.mlog", sample_bytes())

    async def body(h, ws):
        monkeypatch.setattr(logs_mod, "IMPORT_MAX_BYTES", SAMPLE.stat().st_size - 1)
        r = await gh.control(ws, cmd="log_import", path=src)
        assert r["ok"] is False and "limit" in r["error"]
        lib = h.run_dir / "logs"
        assert not lib.exists() or list(lib.iterdir()) == []
        monkeypatch.setattr(logs_mod, "IMPORT_MAX_BYTES", SAMPLE.stat().st_size)
        assert (await gh.control(ws, cmd="log_import", path=src))["ok"] is True

    run(tmp_path, body)


def test_mounted_cards_are_listed_and_a_listed_file_imports(tmp_path):
    cards = tmp_path / "cards"
    log_dir = cards / "MARVSD" / "marv" / "log"
    log_dir.mkdir(parents=True)
    (log_dir / "00000001.mlog").write_bytes(sample_bytes())
    (log_dir / "00000002.mlog").write_bytes(corrupt(sample_bytes()))
    (log_dir / "notes.txt").write_text("not a log")
    os.symlink(log_dir / "00000001.mlog", log_dir / "00000003.mlog")  # a link is not offered
    (cards / "OTHER").mkdir()
    (cards / "OTHER" / "stray.mlog").write_bytes(b"x")  # not in marv/log

    async def body(h, ws):
        listed = await gh.control(ws, cmd="log_list")
        assert [(c["name"], c["mount"], c["size"]) for c in listed["cards"]] == [
            ("00000001.mlog", str(cards / "MARVSD"), SAMPLE.stat().st_size), ("00000002.mlog", str(cards / "MARVSD"), SAMPLE.stat().st_size)]
        assert listed["logs"] == []
        r = await gh.control(ws, cmd="log_import", path=listed["cards"][1]["path"])
        assert r["ok"] and r["summary"]["problems"][0]["reason"] == "crc"
        assert (log_dir / "00000002.mlog").read_bytes() == corrupt(sample_bytes())  # the card's file is as it was

    run(tmp_path, body, log_card_dirs=[cards, tmp_path / "no-such-dir"])


def test_a_decoder_failure_is_reported_and_the_decoder_is_put_back(tmp_path, monkeypatch):
    src = put(tmp_path, "sample.mlog", sample_bytes())
    original = mlog.Decoded

    def broken(data, **kw):
        raise ValueError("planted")

    async def body(h, ws):
        monkeypatch.setattr(mlog, "decode", broken)
        r = await gh.control(ws, cmd="log_import", path=src)
        assert r["ok"] is True and r["summary"] == {"decoder_error": "ValueError: planted"}
        assert mlog.Decoded is original
        # A series of a log the decoder cannot read is refused with the reason (it has no kinds).
        again = await gh.control(ws, cmd="log_series", sha256=r["sha256"], kind="imu", fields=["gyro_x"])
        assert again["ok"] is False

    run(tmp_path, body)
    assert mlog.Decoded is original


def test_series_requests_are_checked(tmp_path):
    src = put(tmp_path, "sample.mlog", sample_bytes())

    async def body(h, ws):
        sha = (await gh.control(ws, cmd="log_import", path=src))["sha256"]
        base = {"sha256": sha, "kind": "imu", "fields": ["gyro_x"]}
        assert (await gh.control(ws, cmd="log_series", **base))["ok"] is True  # control
        bad = {
            "no sha": {**base, "sha256": None}, "short sha": {**base, "sha256": sha[:10]}, "upper sha": {**base, "sha256": sha.upper()},
            "unknown sha": {**base, "sha256": "0" * 64}, "unknown kind": {**base, "kind": "nope"},
            "unknown field": {**base, "fields": ["nope"]}, "field of another kind": {**base, "fields": ["drops_imu"]},
            "no fields": {**base, "fields": []}, "fields not a list": {**base, "fields": "gyro_x"}, "repeated field": {**base, "fields": ["gyro_x"] * 2},
            "too many fields": {**base, "fields": ["gyro_x"] * (logs_mod.SERIES_FIELDS_MAX + 1)},
            "zero buckets": {**base, "buckets": 0}, "too many buckets": {**base, "buckets": logs_mod.SERIES_BUCKETS_MAX + 1},
            "buckets a string": {**base, "buckets": "9"}, "buckets a bool": {**base, "buckets": True}, "t0 a float": {**base, "t0_us": 1.5},
            "range reversed": {**base, "t0_us": 5000, "t1_us": 4000},
        }
        for why, req in bad.items():
            r = await gh.control(ws, cmd="log_series", **req)
            assert r["ok"] is False and r["error"], why
        # The largest bucket count is accepted.
        assert (await gh.control(ws, cmd="log_series", **base, buckets=logs_mod.SERIES_BUCKETS_MAX))["ok"] is True

    run(tmp_path, body)

