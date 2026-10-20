"""The helper's log library (ground-segment unit GX-15, "logs by card pull now, IF-7 later"): onboard-logger .mlog files copied from a
mounted SD card (or a path the operator gives) into <run dir>/logs, with their SHA-256, and read through the logger's own decoder
(tools/log/mlog.py: this module has no second decoder, it only gives the decoder a sink for its records).

  - import_file(path): copies one .mlog into the library as <sha256>.mlog, writes <sha256>.json (name, size, source, import time and the
    decoder's summary) next to it and never opens the source for writing. The source must be an absolute, normalised path to a regular
    file (not a symbolic link) named *.mlog, outside this repository and outside the library. A file the decoder reports problems in
    (a corrupt sector, a cut) is imported all the same, with the report attached.
  - list_logs(): the library, each with its sha, size and summary, and the *.mlog files of the mounted cards (<card dir>/<mount>/marv/log,
    the logger's directory, firmware fw/log).
  - series(request): for a kind and fields of one imported log, the minimum and maximum of each field in each of N equal time buckets of
    a range, from one pass of the decoder; the browser never receives the records.

Decoding is strict (the decoder's default: it stops at the first invalid sector and says where). Everything here is standard library.
"""

from __future__ import annotations

import collections
import contextlib
import dataclasses
import hashlib
import json
import math
import mmap
import os
import re
import stat
import sys
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "tools" / "log"))
import mlog  # noqa: E402

# Labelled harness values (PLACEHOLDER: the size and count limits bound a request, they decide no vehicle result).
IMPORT_MAX_BYTES = 2 << 30  # PLACEHOLDER: the largest .mlog imported; the soak files are 0.6 GiB, the logger's file is 1 GiB (log_bench README)
SERIES_BUCKETS_DEFAULT = 600  # PLACEHOLDER: buckets of a series when the request names none (a plot is about this many pixels wide)
SERIES_BUCKETS_MAX = 4096  # PLACEHOLDER: the most buckets a series returns
SERIES_FIELDS_MAX = 8  # PLACEHOLDER: the most fields one series request names
GAPS_SHOWN_MAX = 200  # PLACEHOLDER: the gap records a summary lists (the totals count them all)
COPY_CHUNK = 1 << 20  # labelled harness value: bytes read per step of the copy
SUFFIX = ".mlog"
CARD_LOG_DIR = ("marv", "log")  # the logger's directory on the card (architect-onboard-logger: /marv/log/NNNNNNNN.mlog)
SHA256_RE = re.compile(r"[0-9a-f]{64}")
# The decoder reads Decoded by this module-level name; a sink replaces it for one decode at a time.
_DECODE_LOCK = threading.Lock()
_NON_DATA_KEYS = ("kind", "sector")


class LogError(ValueError):
    """A refused request; the text is the operator's reason."""


def default_card_dirs() -> list[Path]:
    user = os.environ.get("USER")
    return [Path(base) / user for base in ("/media", "/run/media") if user]


def _within(path: Path, root: Path) -> bool:
    return path == root or root in path.parents


def _utc() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _is_number(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


# ---- the decoder with a sink for its records ------------------------------------------------------------------------------


def decode_into(path: Path, sink: Any) -> mlog.Decoded:
    """mlog.decode() of the whole file, strict, with `sink.append(row)` standing for the record list (as tests/regression/quad/L10/
    results/log_bench/soak_analyze.py does), so that a file of millions of records is read without being held."""
    @dataclasses.dataclass
    class Sunk(mlog.Decoded):
        records: Any = dataclasses.field(default_factory=lambda: sink)

    with path.open("rb") as fh, mmap.mmap(fh.fileno(), 0, access=mmap.ACCESS_READ) as data, _DECODE_LOCK:
        original = mlog.Decoded
        mlog.Decoded = Sunk
        try:
            return mlog.decode(data)  # type: ignore[arg-type]
        finally:
            mlog.Decoded = original


class SummarySink:
    """Counts the records by kind and keeps what a report needs: each kind's time range and numeric fields, the gap records and the
    End record."""

    def __init__(self) -> None:
        self.counts: collections.Counter[str] = collections.Counter()
        self.t_min: dict[str, int] = {}
        self.t_max: dict[str, int] = {}
        self.fields: dict[str, list[str]] = {}
        self.gaps: list[dict] = []
        self.gap_count = 0
        self.dropped: collections.Counter[str] = collections.Counter()
        self.end: dict | None = None

    def append(self, row: dict) -> None:
        kind = row["kind"]
        self.counts[kind] += 1
        if kind not in self.fields:
            self.fields[kind] = [k for k, v in row.items() if k not in _NON_DATA_KEYS and _is_number(v)]
        t = row.get("t_us")
        if isinstance(t, int):
            if kind not in self.t_min or t < self.t_min[kind]:
                self.t_min[kind] = t
            if kind not in self.t_max or t > self.t_max[kind]:
                self.t_max[kind] = t
        if kind == "gap":
            self.gap_count += 1
            for k, v in row.items():
                if k.startswith("dropped_") and _is_number(v):
                    self.dropped[k[len("dropped_"):]] += v
            if len(self.gaps) < GAPS_SHOWN_MAX:
                self.gaps.append({k: v for k, v in row.items() if k != "kind"})
        elif kind == "end":
            self.end = {k: v for k, v in row.items() if k != "kind"}


def summarise(path: Path) -> dict:
    """The decoder's report of a file as JSON-ready facts (what mlog.summary() prints, structured)."""
    sink = SummarySink()
    try:
        dec = decode_into(path, sink)
    except (ValueError, IndexError, KeyError, OverflowError) as e:  # a record the decoder cannot read: reported, not fatal
        return {"decoder_error": f"{type(e).__name__}: {e}"}
    info = dec.header.get("info", {})
    header = {
        "format_version": info.get("format_version"),
        "session_start_us": info.get("session_start_us"),
        "image_hash": info.get("image_hash"),
        "config_hash": info.get("config_hash"),
        "schema_hash": f"{info['schema_hash']:#018x}" if "schema_hash" in info else None,
        "odr_hz": info.get("odr_hz"),
        "params": len(dec.header.get("params", [])),
    }
    return {
        "sectors_ok": dec.sectors_ok,
        "sectors_total": dec.sectors_total,
        "session": f"{dec.session:#018x}" if dec.session is not None else None,
        "ended": dec.ended,
        "ends_inside_record": dec.ends_inside_record,
        "stopped_at": dec.stopped_at,
        "clean": dec.clean,
        "problems": [{"sector": p.sector, "reason": p.reason, "detail": p.detail} for p in dec.problems],
        "records": dict(sink.counts),
        "kinds": {k: {"count": n, "t_min_us": sink.t_min.get(k), "t_max_us": sink.t_max.get(k), "fields": sink.fields.get(k, [])}
                  for k, n in sorted(sink.counts.items())},
        "gaps": {"count": sink.gap_count, "dropped": dict(sink.dropped), "shown": sink.gaps},
        "end": sink.end,
        "header": header,
    }


# ---- series ---------------------------------------------------------------------------------------------------------------


class SeriesSink:
    """Min and max of the named fields of one kind in equal time buckets: bucket i holds t0 + i*width <= t_us < t0 + (i+1)*width."""

    def __init__(self, kind: str, fields: list[str], t0: int, t1: int, buckets: int) -> None:
        self.kind, self.fields, self.t0, self.t1 = kind, fields, t0, t1
        self.buckets = buckets
        self.width = max(1, -(-(t1 - t0 + 1) // buckets))
        self.n = [0] * buckets
        self.lo: dict[str, list[float | None]] = {f: [None] * buckets for f in fields}
        self.hi: dict[str, list[float | None]] = {f: [None] * buckets for f in fields}
        self.skipped: dict[str, int] = {f: 0 for f in fields}
        self.in_range = 0

    def append(self, row: dict) -> None:
        if row["kind"] != self.kind:
            return
        t = row.get("t_us")
        if not isinstance(t, int) or t < self.t0 or t > self.t1:
            return
        i = (t - self.t0) // self.width
        self.n[i] += 1
        self.in_range += 1
        for f in self.fields:
            v = row.get(f)
            if not _is_number(v) or (isinstance(v, float) and not math.isfinite(v)):
                self.skipped[f] += 1
                continue
            lo, hi = self.lo[f], self.hi[f]
            if lo[i] is None or v < lo[i]:
                lo[i] = v
            if hi[i] is None or v > hi[i]:
                hi[i] = v


def _int_arg(request: dict, key: str, default: int | None) -> int:
    v = request.get(key, default)
    if not isinstance(v, int) or isinstance(v, bool):
        raise LogError(f"{key} must be an integer")
    return v


# ---- the library ----------------------------------------------------------------------------------------------------------


class LogLibrary:
    def __init__(self, run_dir: Path, repo_root: Path = ROOT, card_dirs: list[Path] | None = None) -> None:
        self.dir = Path(run_dir).resolve() / "logs"
        self.repo_root = Path(repo_root).resolve()
        self.card_dirs = [Path(d) for d in (default_card_dirs() if card_dirs is None else card_dirs)]
        self._import_lock = threading.Lock()

    # -- paths ----------------------------------------------------------------------------------------------------------
    def check_source(self, path: Any) -> str:
        """The path, if it may be imported; else LogError with the reason. Nothing is opened here."""
        if not isinstance(path, str) or path == "":
            raise LogError("log_import needs a path")
        if "\0" in path:
            raise LogError("the path holds a NUL character")
        if not os.path.isabs(path):
            raise LogError("the path must be absolute")
        if ".." in Path(path).parts or os.path.normpath(path) != path:
            raise LogError("the path must be normalised: no '..', '.', doubled or trailing '/'")
        if not path.endswith(SUFFIX) or os.path.basename(path) == SUFFIX:
            raise LogError(f"the file must be named *{SUFFIX}")
        real = Path(os.path.realpath(path))
        if _within(real, self.repo_root):
            raise LogError(f"the path is inside the repository ({self.repo_root}): copy the file out of it first (W-1)")
        if _within(real, self.dir):
            raise LogError("the path is inside the log library: it is imported already")
        return path

    # -- import ---------------------------------------------------------------------------------------------------------
    def import_file(self, path: Any) -> dict:
        path = self.check_source(path)
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC | os.O_NONBLOCK)  # NONBLOCK: a FIFO is refused below, not waited for
        except FileNotFoundError:
            raise LogError("no such file") from None
        except OSError as e:  # ELOOP for a symbolic link, EACCES, ...
            raise LogError(f"cannot open the file read-only: {e.strerror}") from None
        with os.fdopen(fd, "rb") as src:
            st = os.fstat(src.fileno())
            if not stat.S_ISREG(st.st_mode):
                raise LogError("not a regular file")
            if st.st_size == 0:
                raise LogError("the file is empty")
            if st.st_size > IMPORT_MAX_BYTES:
                raise LogError(f"the file is {st.st_size} bytes, over the limit of {IMPORT_MAX_BYTES} bytes")
            with self._import_lock:
                return self._copy_in(src, path)

    def _copy_in(self, src, source: str) -> dict:
        self.dir.mkdir(parents=True, exist_ok=True)
        part = self.dir / f".import-{os.getpid()}-{threading.get_ident()}.part"
        digest = hashlib.sha256()
        size = 0
        try:
            with part.open("wb") as out:
                while chunk := src.read(COPY_CHUNK):
                    size += len(chunk)
                    if size > IMPORT_MAX_BYTES:
                        raise LogError(f"the file grew over the limit of {IMPORT_MAX_BYTES} bytes while it was copied")
                    digest.update(chunk)
                    out.write(chunk)
            sha = digest.hexdigest()
            final = self.dir / f"{sha}{SUFFIX}"
            existed = final.exists()
            if existed:
                part.unlink()
            else:
                os.replace(part, final)
        finally:
            with contextlib.suppress(FileNotFoundError):
                part.unlink()
        sidecar = self.dir / f"{sha}.json"
        entry = self._read_entry(sidecar) if existed else None
        if entry is None:
            entry = {"sha256": sha, "size": size, "name": os.path.basename(source), "source": source, "imported_utc": _utc(),
                     "summary": summarise(final)}
            tmp = sidecar.with_suffix(".json.part")
            tmp.write_text(json.dumps(entry, indent=1))
            os.replace(tmp, sidecar)
        return {**entry, "already_imported": existed}

    # -- listing --------------------------------------------------------------------------------------------------------
    @staticmethod
    def _read_entry(sidecar: Path) -> dict | None:
        try:
            return json.loads(sidecar.read_text())
        except (OSError, ValueError):
            return None

    def cards(self) -> list[dict]:
        found = []
        for root in self.card_dirs:
            try:
                mounts = [root] if root.joinpath(*CARD_LOG_DIR).is_dir() else sorted(p for p in root.iterdir() if p.is_dir())
            except OSError:
                continue
            for mount in mounts:
                directory = mount.joinpath(*CARD_LOG_DIR)
                try:
                    files = sorted(directory.glob("*" + SUFFIX))
                except OSError:
                    continue
                for f in files:
                    try:
                        st = f.lstat()
                    except OSError:
                        continue
                    if stat.S_ISREG(st.st_mode):
                        found.append({"path": str(f), "name": f.name, "mount": str(mount), "size": st.st_size})
        return found

    def list_logs(self) -> dict:
        logs = []
        if self.dir.is_dir():
            for sidecar in self.dir.glob("*.json"):
                entry = self._read_entry(sidecar)
                if entry is not None and (self.dir / f"{entry.get('sha256')}{SUFFIX}").is_file():
                    logs.append(entry)
        logs.sort(key=lambda e: (e["imported_utc"], e["sha256"]))
        return {"logs": logs, "cards": self.cards(), "library": str(self.dir)}

    def lookup(self, sha: Any) -> tuple[Path, dict]:
        if not isinstance(sha, str) or not SHA256_RE.fullmatch(sha):
            raise LogError("sha256 must be 64 lower-case hex digits")
        entry = self._read_entry(self.dir / f"{sha}.json")
        path = self.dir / f"{sha}{SUFFIX}"
        if entry is None or not path.is_file():
            raise LogError("no such log in the library")
        return path, entry

    # -- series ---------------------------------------------------------------------------------------------------------
    def series(self, request: dict) -> dict:
        path, entry = self.lookup(request.get("sha256"))
        kinds = entry["summary"].get("kinds", {})
        kind = request.get("kind")
        if kind not in kinds:
            raise LogError(f"kind must be one of {sorted(kinds)}")
        info = kinds[kind]
        if info["t_min_us"] is None:
            raise LogError(f"records of kind {kind} carry no time")
        fields = request.get("fields")
        if (not isinstance(fields, list) or not fields or len(fields) > SERIES_FIELDS_MAX or len(set(map(str, fields))) != len(fields)
                or not all(isinstance(f, str) and f in info["fields"] for f in fields)):
            raise LogError(f"fields must name 1 to {SERIES_FIELDS_MAX} distinct fields of {kind}: {info['fields']}")
        t0 = _int_arg(request, "t0_us", info["t_min_us"])
        t1 = _int_arg(request, "t1_us", info["t_max_us"])
        buckets = _int_arg(request, "buckets", SERIES_BUCKETS_DEFAULT)
        if t1 < t0:
            raise LogError("t1_us is before t0_us")
        if not 1 <= buckets <= SERIES_BUCKETS_MAX:
            raise LogError(f"buckets must be 1 to {SERIES_BUCKETS_MAX}")
        sink = SeriesSink(kind, fields, t0, t1, buckets)
        try:
            decode_into(path, sink)
        except (ValueError, IndexError, KeyError, OverflowError) as e:
            raise LogError(f"the decoder failed: {type(e).__name__}: {e}") from None
        return {
            "sha256": entry["sha256"], "kind": kind, "t0_us": t0, "t1_us": t1, "bucket_us": sink.width, "buckets": buckets,
            "records": sink.in_range, "n": sink.n,
            "fields": {f: {"min": sink.lo[f], "max": sink.hi[f], "skipped": sink.skipped[f]} for f in fields},
        }
