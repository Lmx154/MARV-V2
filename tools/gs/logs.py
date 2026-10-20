"""The helper's log library (ground-segment unit GX-15, "logs by card pull now, IF-7 later"): onboard-logger .mlog files copied from a
mounted SD card (or a path the operator gives) into <run dir>/logs, with their SHA-256, and read through the logger's own decoder
(tools/log/mlog.py: this module has no second decoder, it only gives the decoder a sink for its records).

  - start_import(path): checks the path and starts an import in a worker thread, one at a time; answers at once. The worker copies the file
    (SHA-256 on the way), decodes the copy ONCE and, from that pass, writes the decoder's report and a columnar cache; it pushes
    log_import_progress events (rate-capped) and one log_import_done event. Nothing is in the library until the import is complete: the copy,
    the cache and the sidecar are written under temporary names and renamed, the sidecar last; a cancelled or failed import leaves none.
        <sha256>.mlog        the file, as copied
        <sha256>.cols/       one file per numeric field of each record kind and its t_us column, little-endian, fixed dtype (f32 as
                             "f", unsigned integers as "Q", t_us as "q"), and meta.json (rows, whether t_us never goes back, the dtype
                             and the count of non-finite values of each column)
        <sha256>.json        the sidecar: name, size, source, import time and the decoder's summary
    The source is opened read-only and must be an absolute, normalised path to a regular file (not a symbolic link) named *.mlog, outside this
    repository and outside the library. A file the decoder reports problems in (a corrupt sector, a cut) is imported all the same, with the
    report attached.
  - list_logs(): the library, each with its sha, size and summary, the import that is going, and the *.mlog files of the mounted cards
    (<card dir>/<mount>/marv/log, the logger's directory, firmware fw/log).
  - series(request): for a kind and fields of one imported log, the minimum and maximum of each field in each of N equal time buckets of a
    range, read from the columns with mmap (no decoding); the browser never receives the records.

Decoding is strict (the decoder's default: it stops at the first invalid sector and says where). Everything here is standard library.
"""

from __future__ import annotations

import array
import bisect
import collections
import contextlib
import dataclasses
import hashlib
import json
import math
import mmap
import os
import re
import shutil
import stat
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterator

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "tools" / "log"))
import mlog  # noqa: E402

# Labelled harness values (PLACEHOLDER: the size, count and rate limits bound a request, they decide no vehicle result).
IMPORT_MAX_BYTES = 2 << 30  # PLACEHOLDER: the largest .mlog imported; the soak files are 0.6 GiB, the logger's file is 1 GiB (log_bench README)
SERIES_BUCKETS_DEFAULT = 600  # PLACEHOLDER: buckets of a series when the request names none (a plot is about this many pixels wide)
SERIES_BUCKETS_MAX = 4096  # PLACEHOLDER: the most buckets a series returns
SERIES_FIELDS_MAX = 8  # PLACEHOLDER: the most fields one series request names
GAPS_SHOWN_MAX = 200  # PLACEHOLDER: the gap records a summary lists (the totals count them all)
PROGRESS_INTERVAL_S = 0.5  # PLACEHOLDER: at most one log_import_progress event per this many seconds (phase changes are always told)
FLUSH_ROWS = 1 << 16  # PLACEHOLDER: rows of a column held before they are written (memory against write calls)
COPY_CHUNK = 1 << 20  # labelled harness value: bytes read per step of the copy
CHECK_EVERY_ROWS = 1 << 10  # labelled harness value: the decoder's sink looks at the clock and the cancel flag every this many records
SUFFIX = ".mlog"
CARD_LOG_DIR = ("marv", "log")  # the logger's directory on the card (architect-onboard-logger: /marv/log/NNNNNNNN.mlog)
SHA256_RE = re.compile(r"[0-9a-f]{64}")
# Column dtypes (array typecodes): f32 values are exact in "f" (the decoder's floats are f32); the format's integers are unsigned.
FLOAT_CODE, UINT_CODE, TIME_CODE = "f", "Q", "q"
COLUMNS_VERSION = 1
# The decoder reads Decoded by this module-level name; a sink replaces it for one decode at a time.
_DECODE_LOCK = threading.Lock()
_NON_DATA_KEYS = ("kind", "sector")


class LogError(ValueError):
    """A refused request; the text is the operator's reason."""


class ImportCancelled(Exception):
    """Raised inside the decoder's sink to end an import that was cancelled (the helper is stopping)."""


def default_card_dirs() -> list[Path]:
    user = os.environ.get("USER")
    return [Path(base) / user for base in ("/media", "/run/media") if user]


def _within(path: Path, root: Path) -> bool:
    return path == root or root in path.parents


def _utc() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _is_number(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _is_time_name(name: str) -> bool:
    """t_us is the time axis and the *_lo_us stamps are what it was unwrapped from: neither is a field to plot."""
    return name == "t_us" or name.endswith("_lo_us")


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
            self.fields[kind] = [k for k, v in row.items() if k not in _NON_DATA_KEYS and not _is_time_name(k) and _is_number(v)]
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


# ---- the columnar cache ---------------------------------------------------------------------------------------------------


class _KindColumns:
    def __init__(self, directory: Path, kind: str, row: dict) -> None:
        self.kind = kind
        self.names = [k for k, v in row.items() if k not in _NON_DATA_KEYS and not _is_time_name(k) and _is_number(v)]
        self.codes = {n: FLOAT_CODE if isinstance(row[n], float) else UINT_CODE for n in self.names}
        self.nonfinite = {n: 0 for n in self.names}
        self.rows = 0
        self.monotone = True
        self.last_t: int | None = None
        files = {"t_us": TIME_CODE, **self.codes}
        self.buffers = {n: array.array(c) for n, c in files.items()}
        self.handles = {n: (directory / f"{kind}.{n}.col").open("wb") for n in files}
        self.files = files

    def add(self, row: dict) -> None:
        t = row.get("t_us")
        if type(t) is not int or not -(1 << 63) <= t < (1 << 63):
            return
        values = []
        for n in self.names:
            v = row.get(n)
            if self.codes[n] == FLOAT_CODE:
                if type(v) is not float:
                    return
            elif type(v) is not int or not 0 <= v < (1 << 64):
                return
            values.append(v)
        try:
            self.buffers["t_us"].append(t)
            for n, v in zip(self.names, values):
                self.buffers[n].append(v)
        except OverflowError:  # an f32 field cannot hold more than f32; the decoder's values never do
            raise LogError(f"{self.kind}.{n}: a value does not fit its column") from None
        for n, v in zip(self.names, values):
            if self.codes[n] == FLOAT_CODE and not math.isfinite(v):
                self.nonfinite[n] += 1
        if self.last_t is not None and t < self.last_t:
            self.monotone = False
        self.last_t = t
        self.rows += 1
        if len(self.buffers["t_us"]) >= FLUSH_ROWS:
            self.flush()

    def flush(self) -> None:
        for n, buf in self.buffers.items():
            buf.tofile(self.handles[n])
            del buf[:]

    def close(self) -> None:
        self.flush()
        for fh in self.handles.values():
            fh.close()

    def meta(self) -> dict:
        return {"rows": self.rows, "monotone": self.monotone,
                "columns": {"t_us": {"dtype": TIME_CODE, "nonfinite": 0},
                            **{n: {"dtype": self.codes[n], "nonfinite": self.nonfinite[n]} for n in self.names}}}


class ColumnWriter:
    """Writes the numeric fields of every record kind that has a t_us, one file per field, as the decoder's records arrive."""

    def __init__(self, directory: Path) -> None:
        if sys.byteorder != "little":
            raise LogError("the column cache is little-endian; this host is not")
        directory.mkdir(parents=True)
        self.directory = directory
        self.kinds: dict[str, _KindColumns | None] = {}

    def add(self, row: dict) -> None:
        kind = row["kind"]
        k = self.kinds.get(kind, False)
        if k is False:
            k = _KindColumns(self.directory, kind, row) if type(row.get("t_us")) is int else None
            self.kinds[kind] = k
        if k is not None:
            k.add(row)

    def close(self) -> None:
        kinds = {}
        for kind, k in self.kinds.items():
            if k is not None:
                k.close()
                if k.rows > 0:
                    kinds[kind] = k.meta()
        (self.directory / "meta.json").write_text(json.dumps({"version": COLUMNS_VERSION, "byte_order": "little", "kinds": kinds}))

    def abort(self) -> None:
        for k in self.kinds.values():
            if k is not None:
                for fh in k.handles.values():
                    fh.close()


class ImportSink(SummarySink):
    """The decoder's record sink of an import: the summary, the columns, the progress and the cancel flag."""

    def __init__(self, columns: ColumnWriter, cancel: threading.Event, progress: Callable[[int, bool], None]) -> None:
        super().__init__()
        self.columns, self.cancel, self.progress = columns, cancel, progress
        self._rows = 0

    def append(self, row: dict) -> None:
        super().append(row)
        self.columns.add(row)
        self._rows += 1
        if not self._rows % CHECK_EVERY_ROWS:
            if self.cancel.is_set():
                raise ImportCancelled()
            self.progress((row["sector"] + 1) * mlog.SECTOR, False)


def summarise(path: Path, sink: SummarySink) -> dict:
    """The decoder's report of a file as JSON-ready facts (what mlog.summary() prints, structured), the records going to `sink`."""
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


# ---- series from the columns ----------------------------------------------------------------------------------------------


@contextlib.contextmanager
def _column(path: Path, code: str, rows: int) -> Iterator[memoryview]:
    """The column file as a read-only memoryview of `rows` items of `code` (mmap; nothing is read until it is touched)."""
    size = rows * array.array(code).itemsize
    if path.stat().st_size != size:
        raise LogError(f"the column cache {path.name} is damaged: {path.stat().st_size} bytes, expected {size}")
    with path.open("rb") as fh, mmap.mmap(fh.fileno(), 0, access=mmap.ACCESS_READ) as mm:
        view = memoryview(mm).cast(code)
        try:
            yield view
        finally:
            view.release()


def bucket_series(t: memoryview, columns: dict[str, memoryview], nonfinite: dict[str, int], monotone: bool, t0: int, t1: int,
                  buckets: int) -> dict:
    """Min and max of each column in equal time buckets: bucket i holds t0 + i*w <= t_us < t0 + (i+1)*w, w = ceil((t1-t0+1)/N), and
    t0 <= t_us <= t1. When t_us never goes back the bucket's rows are a slice found by bisection (and the built-in min and max run over
    it); otherwise, and for a column with non-finite values, every row is visited."""
    width = max(1, -(-(t1 - t0 + 1) // buckets))
    n = [0] * buckets
    lo: dict[str, list[float | None]] = {f: [None] * buckets for f in columns}
    hi: dict[str, list[float | None]] = {f: [None] * buckets for f in columns}
    skipped = {f: 0 for f in columns}
    slow = [f for f in columns if not monotone or nonfinite.get(f, 0) > 0]
    if monotone:
        edges = [min(t0 + i * width, t1 + 1) for i in range(buckets + 1)]
        at = [bisect.bisect_left(t, e) for e in edges]
        for i in range(buckets):
            a, b = at[i], at[i + 1]
            n[i] = b - a
            if b > a:
                for f, col in columns.items():
                    if f not in slow:
                        seg = col[a:b]
                        lo[f][i], hi[f][i] = min(seg), max(seg)
        if slow:
            rows = range(at[0], at[buckets])
        else:
            rows = range(0)
    else:
        rows = range(len(t))
    if slow:
        for r in rows:
            ts = t[r]
            if not monotone:
                if ts < t0 or ts > t1:
                    continue
                n[(ts - t0) // width] += 1
            i = (ts - t0) // width
            for f in slow:
                v = columns[f][r]
                if isinstance(v, float) and not math.isfinite(v):
                    skipped[f] += 1
                    continue
                if lo[f][i] is None or v < lo[f][i]:  # type: ignore[operator]
                    lo[f][i] = v
                if hi[f][i] is None or v > hi[f][i]:  # type: ignore[operator]
                    hi[f][i] = v
    return {"bucket_us": width, "n": n, "fields": {f: {"min": lo[f], "max": hi[f], "skipped": skipped[f]} for f in columns}}


def _int_arg(request: dict, key: str, default: int | None) -> int:
    v = request.get(key, default)
    if not isinstance(v, int) or isinstance(v, bool):
        raise LogError(f"{key} must be an integer")
    return v


# ---- the library ----------------------------------------------------------------------------------------------------------


class LogLibrary:
    def __init__(self, run_dir: Path, repo_root: Path = ROOT, card_dirs: list[Path] | None = None,
                 emit: Callable[[dict], None] | None = None) -> None:
        self.dir = Path(run_dir).resolve() / "logs"
        self.repo_root = Path(repo_root).resolve()
        self.card_dirs = [Path(d) for d in (default_card_dirs() if card_dirs is None else card_dirs)]
        self.emit = emit or (lambda event: None)  # thread-safe: called from the import's worker thread
        self._state = threading.Lock()
        self._active: dict | None = None
        self._next_id = 0
        self._cancel = threading.Event()

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
    def start_import(self, path: Any) -> dict:
        """Checks the path and the file, then starts the import in a worker thread and answers {"started", "import_id"}; the worker's progress and
        end are the events log_import_progress and log_import_done. One import at a time."""
        path = self.check_source(path)
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC | os.O_NONBLOCK)  # NONBLOCK: a FIFO is refused below, not waited for
        except FileNotFoundError:
            raise LogError("no such file") from None
        except OSError as e:  # ELOOP for a symbolic link, EACCES, ...
            raise LogError(f"cannot open the file read-only: {e.strerror}") from None
        src = os.fdopen(fd, "rb")
        try:
            st = os.fstat(src.fileno())
            if not stat.S_ISREG(st.st_mode):
                raise LogError("not a regular file")
            if st.st_size == 0:
                raise LogError("the file is empty")
            if st.st_size > IMPORT_MAX_BYTES:
                raise LogError(f"the file is {st.st_size} bytes, over the limit of {IMPORT_MAX_BYTES} bytes")
            with self._state:
                if self._active is not None:
                    raise LogError(f"import {self._active['id']} ({os.path.basename(self._active['path'])}) is still going: one import at a time")
                self._next_id += 1
                import_id = self._next_id
                self._cancel.clear()
                thread = threading.Thread(target=self._run, args=(import_id, src, path, st.st_size), name=f"log-import-{import_id}", daemon=True)
                self._active = {"id": import_id, "path": path, "thread": thread}
        except BaseException:
            src.close()
            raise
        thread.start()
        return {"started": True, "import_id": import_id}  # not "id": the helper echoes the request's id in the reply

    def cancel(self) -> None:
        self._cancel.set()

    def shutdown(self) -> None:
        """Cancels the import that is going and waits for its worker: it leaves no file of its own behind."""
        self._cancel.set()
        with self._state:
            active = self._active
        if active is not None:
            active["thread"].join()

    def _run(self, import_id: int, src, source: str, size: int) -> None:
        try:
            with src:
                entry, existed = self._import(import_id, src, source, size)
            event: dict = {"event": "log_import_done", "id": import_id, "ok": True, "already_imported": existed, **entry}
        except ImportCancelled:
            event = {"event": "log_import_done", "id": import_id, "ok": False, "cancelled": True, "error": "cancelled"}
        except (LogError, OSError) as e:
            event = {"event": "log_import_done", "id": import_id, "ok": False, "error": str(e)}
        except Exception as e:  # the worker must tell its end whatever happened
            event = {"event": "log_import_done", "id": import_id, "ok": False, "error": f"{type(e).__name__}: {e}"}
        with self._state:
            self._active = None
        self.emit(event)

    def _import(self, import_id: int, src, source: str, size: int) -> tuple[dict, bool]:
        self.dir.mkdir(parents=True, exist_ok=True)
        tag = f".import-{os.getpid()}-{import_id}"
        part, parts = self.dir / f"{tag}.part", self.dir / f"{tag}.cols"
        last = [0.0]

        def progress(phase: str, done: int, force: bool) -> None:
            now = time.monotonic()
            if force or now - last[0] >= PROGRESS_INTERVAL_S:
                last[0] = now
                self.emit({"event": "log_import_progress", "id": import_id, "phase": phase, "bytes_done": min(done, size), "bytes_total": size})

        writer: ColumnWriter | None = None
        try:
            digest = hashlib.sha256()
            copied = 0
            progress("copy", 0, True)
            with part.open("wb") as out:
                while chunk := src.read(COPY_CHUNK):
                    if self._cancel.is_set():
                        raise ImportCancelled()
                    copied += len(chunk)
                    if copied > IMPORT_MAX_BYTES:
                        raise LogError(f"the file grew over the limit of {IMPORT_MAX_BYTES} bytes while it was copied")
                    digest.update(chunk)
                    out.write(chunk)
                    progress("copy", copied, False)
            sha = digest.hexdigest()
            final, sidecar, cols = self.dir / f"{sha}{SUFFIX}", self.dir / f"{sha}.json", self.dir / f"{sha}.cols"
            existing = self._read_entry(sidecar)
            if existing is not None and final.is_file() and (cols / "meta.json").is_file():
                return existing, True
            progress("decode", 0, True)
            writer = ColumnWriter(parts)
            summary = summarise(part, ImportSink(writer, self._cancel, lambda done, force: progress("decode", done, force)))
            if "decoder_error" in summary:
                writer.abort()
                shutil.rmtree(parts)
                writer = None
            else:
                writer.close()
                writer = None
            progress("decode", size, True)
            if self._cancel.is_set():
                raise ImportCancelled()
            entry = {"sha256": sha, "size": copied, "name": os.path.basename(source), "source": source, "imported_utc": _utc(),
                     "summary": summary}
            tmp = sidecar.with_suffix(".json.part")
            tmp.write_text(json.dumps(entry, indent=1))
            if parts.exists():
                shutil.rmtree(cols, ignore_errors=True)
                os.replace(parts, cols)
            os.replace(part, final)
            os.replace(tmp, sidecar)  # last: the library lists a log by its sidecar
            return entry, False
        finally:
            if writer is not None:
                writer.abort()
            shutil.rmtree(parts, ignore_errors=True)
            with contextlib.suppress(FileNotFoundError):
                part.unlink()
            with contextlib.suppress(FileNotFoundError):
                (self.dir / f"{tag}.json.part").unlink()

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
        with self._state:
            active = self._active
        return {"logs": logs, "cards": self.cards(), "library": str(self.dir),
                "importing": None if active is None else {"id": active["id"], "name": os.path.basename(active["path"])}}

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
        cols = path.with_suffix(".cols")
        meta = self._read_entry(cols / "meta.json")
        if meta is None or meta.get("version") != COLUMNS_VERSION or kind not in meta["kinds"]:
            raise LogError("the log has no column cache for this kind")
        kmeta = meta["kinds"][kind]
        rows = kmeta["rows"]
        with contextlib.ExitStack() as stack:
            t = stack.enter_context(_column(cols / f"{kind}.t_us.col", TIME_CODE, rows))
            columns = {f: stack.enter_context(_column(cols / f"{kind}.{f}.col", kmeta["columns"][f]["dtype"], rows)) for f in fields}
            result = bucket_series(t, columns, {f: kmeta["columns"][f]["nonfinite"] for f in fields}, kmeta["monotone"], t0, t1, buckets)
        return {"sha256": entry["sha256"], "kind": kind, "t0_us": t0, "t1_us": t1, "buckets": buckets, "records": sum(result["n"]), **result}
