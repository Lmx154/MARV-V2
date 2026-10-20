"""DERIVE from the GCS (ground-segment unit GX-13, unit 10 of the params-first ruling, kind 3 of the sim-parameter ruling).

A card copy with edits and their provenance text becomes a `.mcfg`, by the build that CMake runs for the parameter set
`marv_params_fc3v2` (fw/params/CMakeLists.txt, marv_add_card_param_set), and nothing else of it is reimplemented: the same three
scripts with the same arguments, in the same order, run as processes of the repository's own tools:

  1. tools/card/flatten_cached.py   lints the card, the design budget, the scenario register and the sensor profile, then writes the
                                    flattened card, register, mixer, scenario, rate, attitude, gyro-chain and vertical files and the
                                    derivation reports (`--rate-guard mean_period`, the board's)
  2. tools/gen/params_gen.py        the flattened files and the registers of the set -> params_manifest.json (and the generated headers, and the
                                    readable table of every record, <set>_product_table.txt)
  3. tools/config/mcfg.py encode    params_manifest.json -> <set>.mcfg, refused above half a config slot (design/budget.yaml,
                                    config_slot_bytes; the regular expression is the one of fw/hal/CMakeLists.txt)

Where it runs (W-1: the repository is never written). Each derivation has a work directory <run dir>/derive/<id>/ holding

  root/                 a mirror of the repository paths the flatten reads, as COPIES: vehicles/<card>.yaml (edited), design/budget.yaml,
                        the set's scenario register and the sensor profile the card names; `--root` is this directory, so that every
                        path in a report header is the repository-relative one a CMake build prints
  flatten/, gen/        the outputs of steps 1 and 2 (flatten/*_report.txt are the derivation reports)
  <set>.mcfg            the result; derive.json: the request, the edits (old and new value, provenance text), the SHA-256 of every
                        input, `git describe --dirty`, the step commands, the config hash and the images it can be validated against
  derive.log            the output of the steps

An edit is {key, value, source}: `key` names a card entry that has a value (the entries tools/card/lint.py knows: mass, inertia_diag,
drag, rotors.thrust_coeff, rotors.torque_ratio, rotors.speed_range, rotors.motor_lag.tau, rotors.m<n>.position, rotors.m<n>.spin), `value`
replaces the entry's value and `source` becomes the entry's source text (the provenance of the new value). A locked entry changes
only in its source entry by the lock editor, so it is refused here. The edited copy is linted before anything runs; a source that is
empty, or equal to the text it replaces, is a finding like any other, and a finding refuses the derivation (nothing heavy runs, no lock
is taken). A card with no edit is copied byte for byte. The edited copy is written by PyYAML (comments are not kept).

The heavy part (step 1 uses every CPU for about a minute) takes the host-wide heavy lock the way tools/gs/build.py does, and is refused
when the lock is held, when free memory is below DERIVE_MIN_FREE_BYTES or when another derivation runs.

The result joins the helper's config library: tools/gs/helper.py's `validate` takes {"config": <config hash>} for it, and the report
store (tools/gs/artifacts.py ReportLibrary) then keeps config.mcfg and report.json for the (image, config) pair, which is what the
G7 flash gate and the Parameters page's activation read.

Commands (tools/gs/helper.py): {"cmd": "cards"} (the helper's existing one), {"cmd": "derive_card", "card"}, {"cmd": "derive", "card", "edits"},
{"cmd": "derive_status"}, {"cmd": "derive_cancel"}, {"cmd": "derive_report", "derive_id", "name"}, {"cmd": "derive_values", "derive_id"}. Events:
{"event": "derive_started", ...}, {"event": "derive_step", ...}, {"event": "derive_done", ...}.
"""

from __future__ import annotations

import asyncio
import errno
import fcntl
import hashlib
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "tools" / "config"))
sys.path.append(str(ROOT / "tools" / "card"))  # appended: only `schema` and `lint` are taken from it
import hoststate  # noqa: E402
import lint  # noqa: E402
import mcfg  # noqa: E402
import schema  # noqa: E402
from artifacts import HASH_RE  # noqa: E402
from build import git_describe  # noqa: E402
from device import DeviceError  # noqa: E402
from flash import FlashRefused, config_hash_of  # noqa: E402
from runs import list_cards, resolve_card  # noqa: E402

DERIVE_SCHEMA = 1
PARAMS_SET = "marv_params_fc3v2"  # the parameter set derive builds (the flight composition's: the board image and the quad_fc SIL link it)
RECORD = "derive.json"
LOG = "derive.log"
BUDGET = "design/budget.yaml"  # fw/params/CMakeLists.txt MARV_DESIGN_BUDGET
# fw/params/CMakeLists.txt, marv_add_card_param_set(marv_params_fc3v2 ...): the scenario register, the rate guard and the extra
# registers (in order) of the set; tests/tools/test_gs_derive.py compares this table with that call.
SCENARIO = "fw/compositions/quad_fc/params/fc3v2_scenario_values.yaml"
RATE_GUARD = "mean_period"
EXTRA_REGISTERS = (
    "fw/compositions/quad_fc/params/fc3v2_rate_timing_register.yaml",
    "fw/compositions/quad_fc/params/quad_fc_register.yaml",
    "fw/link/params/telemetry_register.yaml",
)
FLATTEN_PARTS = ("card", "mixer", "scenario", "rate", "attitude", "gyro_chain", "vertical")  # the order of marv_add_card_param_set's CARD list
SLOT_RE = re.compile(r"\nconfig_slot_bytes:[ \t]*\n[ \t]+value:[ \t]*([0-9]+)[ \t]*\n")  # fw/hal/CMakeLists.txt
EDIT_KEYS = ("key", "value", "source")
REQUEST_KEYS = ("cmd", "id", "card", "edits")
REPORT_RE = re.compile(r"^[A-Za-z0-9_]+_report\.txt$")
ID_RE = re.compile(r"^[0-9]{8}T[0-9]{6}Z-[0-9]+$")

# PLACEHOLDER (design budget, unmeasured): a derivation starts only when this much memory is available (MemAvailable). The flatten
# runs numpy and one worker per usable CPU; its peak is not measured. Replaced by the peak a measured derive records; until then it
# blocks nothing but a starved host. Decides no result.
DERIVE_MIN_FREE_BYTES = 2 << 30
# PLACEHOLDER (harness value): the longest one step may run before it is killed; the flatten took 40 s on 16 CPUs under the heavy lock
# (tests/regression/quad/L10/results/gcs/gx13/README.md), several times that on one.
DERIVE_STEP_TIMEOUT_S = 3600.0
# PLACEHOLDER (harness value): how long a cancelled or timed-out step gets after SIGTERM before SIGKILL.
DERIVE_STOP_S = 10.0
# PLACEHOLDER (harness value): the lint of a card copy is a short Python run; it is killed after this long.
LINT_TIMEOUT_S = 120.0
# PLACEHOLDER (harness value): the last output lines the status keeps for the GCS; the whole output is in derive.log.
DERIVE_TAIL_LINES = 40
# PLACEHOLDER (harness value): at most this many edits in one request (a card has about 20 entries with a value).
MAX_EDITS = 64
MAX_SOURCE_CHARS = 4000  # PLACEHOLDER (harness value): the longest provenance text of one edit
READ_SIZE = 65536  # implementation constant: bytes per pipe read
POLL_S = 0.05  # labelled harness value: the poll period of the stop wait


def utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


class DeriveRefused(DeviceError):
    """A derive command the helper refuses; `extra` is merged into the reply (findings, refusals)."""

    def __init__(self, message: str, **extra):
        super().__init__(message)
        self.extra = extra


def sha256_of(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


# ---- the card: which entries can be edited, and the edit ----------------------------------------------------------------------------

def entry_keys(doc: dict) -> list[str]:
    """The entry paths of a card that carry a value (tools/card/lint.py's structure), in the card's order."""
    keys = [k for k in (*lint.TOP_UNITS, "drag") if isinstance(doc.get(k), dict) and "value" in doc[k]]
    rotors = doc.get("rotors")
    if isinstance(rotors, dict):
        keys += [f"rotors.{k}" for k in lint.ROTOR_SCALARS if isinstance(rotors.get(k), dict) and "value" in rotors[k]]
        lag = rotors.get("motor_lag")
        if isinstance(lag, dict) and isinstance(lag.get("tau"), dict) and "value" in lag["tau"]:
            keys.append("rotors.motor_lag.tau")
        for m in lint.QUADRANT:
            body = rotors.get(m)
            if isinstance(body, dict):
                keys += [f"rotors.{m}.{k}" for k in ("position", "spin") if isinstance(body.get(k), dict) and "value" in body[k]]
    return keys


def entry_at(doc: dict, key: str) -> dict:
    node = doc
    for part in key.split("."):
        node = node[part]
    return node


def card_text_doc(card: str) -> tuple[Path, dict]:
    path = ROOT / card
    try:
        doc = schema.load_yaml(path)
    except (OSError, yaml.YAMLError) as e:
        raise DeriveRefused(f"the card {card} cannot be read: {e}") from None
    if not isinstance(doc, dict):
        raise DeriveRefused(f"the card {card} is not a mapping")
    return path, doc


def jsonable(value):
    """A YAML value as JSON would carry it (dates as ISO text)."""
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, list):
        return [jsonable(v) for v in value]
    if hasattr(value, "isoformat"):
        return value.isoformat()
    return value


def card_entries(card: str) -> dict:
    """{"card", "vehicle", "entries": [{key, value, unit, shape, method, source, sigma, status, locked, lock}]}: what the Derive panel may edit."""
    _, doc = card_text_doc(resolve_card(card))
    entries = []
    for key in entry_keys(doc):
        e = entry_at(doc, key)
        entries.append({"key": key, "value": jsonable(e.get("value")), "unit": e.get("unit"), "shape": e.get("shape"), "method": e.get("method"),
                        "source": e.get("source"), "sigma": jsonable(e.get("sigma")), "status": e.get("status"),
                        "locked": bool(e.get("lock")), "lock": jsonable(e.get("lock"))})
    return {"card": resolve_card(card), "vehicle": doc.get("vehicle"), "entries": entries}


def is_finite_number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def parse_edits(card: str, doc: dict, edits) -> list[dict]:
    """The edits of a request, checked against the card: a list of {key, value, source}, each key an entry of the card with a value that is not
    locked, each once, each value of the entry's JSON kind. Nothing else is judged here (the lint of the edited copy judges units, sigma,
    shape and the source text)."""
    if not isinstance(edits, list):
        raise DeriveRefused("edits must be a list of {key, value, source}")
    if len(edits) > MAX_EDITS:
        raise DeriveRefused(f"at most {MAX_EDITS} edits in one request")
    keys = entry_keys(doc)
    seen, out = set(), []
    for i, edit in enumerate(edits):
        if not isinstance(edit, dict) or set(edit) != set(EDIT_KEYS):
            raise DeriveRefused(f"edit {i}: must be exactly {{key, value, source}}")
        key, value, source = edit["key"], edit["value"], edit["source"]
        if not isinstance(key, str) or key not in keys:
            shown = key if isinstance(key, str) and len(key) <= 64 else type(key).__name__
            raise DeriveRefused(f"edit {i}: {shown!r} is not an editable entry of {card}: {', '.join(keys)}")
        if key in seen:
            raise DeriveRefused(f"edit {i}: {key} is edited twice")
        seen.add(key)
        entry = entry_at(doc, key)
        if entry.get("lock"):
            lock = entry["lock"]
            raise DeriveRefused(f"edit {i}: {key} is locked by {lock.get('by')} on {lock.get('on')}: a lock changes only in its source entry "
                                "(the lock editor), not in a derivation")
        old = entry["value"]
        if isinstance(old, str) and old != schema.UNKNOWN:
            ok = isinstance(value, str)
        else:
            ok = is_finite_number(value) or (isinstance(value, list) and bool(value) and all(is_finite_number(v) for v in value))
        if not ok:
            raise DeriveRefused(f"edit {i}: the value of {key} must be {'text' if isinstance(old, str) and old != schema.UNKNOWN else 'a finite number or a list of finite numbers'}")
        if not isinstance(source, str) or len(source) > MAX_SOURCE_CHARS:
            raise DeriveRefused(f"edit {i}: the source of {key} must be text of at most {MAX_SOURCE_CHARS} characters")
        out.append({"key": key, "value": value, "source": source, "old_value": jsonable(old), "old_source": entry.get("source")})
    return out


def write_card_copy(src: Path, doc: dict, edits: list[dict], dest: Path) -> None:
    """The card as a copy: byte for byte without edits; with edits, the loaded card with the edits made, dumped by PyYAML."""
    dest.parent.mkdir(parents=True, exist_ok=True)
    if not edits:
        shutil.copyfile(src, dest)
        return
    for edit in edits:
        entry = entry_at(doc, edit["key"])
        entry["value"] = edit["value"]
        entry["source"] = edit["source"]
    dest.write_text(yaml.safe_dump(doc, sort_keys=False, allow_unicode=True, width=float("inf")), encoding="utf-8")


def provenance_findings(card: str, edits: list[dict]) -> list[dict]:
    """A source text equal to the one it replaces says nothing about the new value: a finding in the lint's format."""
    return [{"file": card, "entry": e["key"], "reason": "source: an edit needs its own provenance text; it equals the text of the value it replaces"}
            for e in edits if e["source"].strip() == str(e["old_source"] or "").strip() and e["source"].strip()]


def parse_findings(stderr: str, base: Path) -> list[dict]:
    """tools/card/lint.py prints '<file>: <entry path>: <reason>' per finding on stderr."""
    out = []
    for line in stderr.splitlines():
        if not line.strip():
            continue
        parts = line.split(": ", 2)
        if len(parts) == 3:
            file = parts[0]
            try:
                file = Path(file).relative_to(base).as_posix()
            except ValueError:
                pass
            out.append({"file": file, "entry": parts[1], "reason": parts[2]})
        else:
            out.append({"file": "", "entry": "", "reason": line.strip()})
    return out


# ---- the heavy lock, the way ci/heavy.sh and tools/gs/build.py take it ---------------------------------------------------------------

def take_heavy_lock(path: Path, what: str) -> int:
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o666)  # ci/heavy.sh: exec 9>>"${lock}"
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError as e:
        os.close(fd)
        if e.errno not in (errno.EWOULDBLOCK, errno.EAGAIN):
            raise DeriveRefused(f"the heavy lock {path} cannot be taken: {e}", refusals=[f"the heavy lock {path} cannot be taken: {e}"]) from None
        holder = hoststate.heavy_lock_state(path)["holder"]
        why = f"the heavy lock is held ({holder or 'unknown holder'}): heavy work waits for it (H-3)"
        raise DeriveRefused(why, refusals=[why]) from None
    user = os.environ.get("USER") or str(os.getuid())
    os.ftruncate(fd, 0)
    os.write(fd, f"pid={os.getpid()} user={user} since={utc_iso()} cmd=GCS {what}\n".encode())
    return fd


class DeriveManager:
    def __init__(self, *, root: Path, run_dir: Path, build_root: Path, library, log: Callable[[str], None], push: Callable[[dict], None],
                 lock_path: Callable[[], Path], min_free_bytes: int = DERIVE_MIN_FREE_BYTES, python: str | None = None):
        self.root, self.run_dir, self.build_root = (Path(os.path.abspath(p)) for p in (root, run_dir, build_root))  # the steps run in the work directory
        self.library, self.log, self.push, self.lock_path = library, log, push, lock_path
        self.min_free_bytes = min_free_bytes
        self.python = python or sys.executable
        self.stop_s = DERIVE_STOP_S
        self.step_timeout_s = DERIVE_STEP_TIMEOUT_S
        self.state: dict | None = None
        self._task: asyncio.Task | None = None
        self._proc: asyncio.subprocess.Process | None = None
        self._cancel = False
        self._lock_fd: int | None = None
        self._preparing = False

    # ---- what the browser may ask ---------------------------------------------------------------------------------------

    @property
    def derive_dir(self) -> Path:
        return self.run_dir / "derive"

    def busy(self) -> bool:
        return self._preparing or (self._task is not None and not self._task.done())

    def view(self) -> dict | None:
        return None if self.state is None else dict(self.state)

    def refusals(self, *, own: bool = False) -> list[str]:
        """Every reason a derivation cannot start now; `own`: the running state of the request being started is not one of them."""
        reasons = []
        if self.busy() and not own:
            reasons.append("a derivation is running: one at a time")
        lock = hoststate.heavy_lock_state(self.lock_path())
        if lock["held"]:
            reasons.append(f"the heavy lock is held ({lock['holder'] or 'unknown holder'}): heavy work waits for it (H-3)")
        free = hoststate.meminfo()["mem_available_bytes"]
        if free is None:
            reasons.append("free memory cannot be read: a derivation needs it to be known")
        elif free < self.min_free_bytes:
            reasons.append(f"free memory {free} B is below the derivation budget {self.min_free_bytes} B (PLACEHOLDER)")
        return reasons

    def status(self) -> dict:
        return {"params_set": PARAMS_SET, "running": self.busy(), "current": self.view(), "derivations": self.list_derivations(),
                "limits": {"min_free_bytes": self.min_free_bytes, "mem_available_bytes": hoststate.meminfo()["mem_available_bytes"],
                           "max_edits": MAX_EDITS, "max_source_chars": MAX_SOURCE_CHARS},
                "refusals": self.refusals()}

    # ---- the library of derived configs ---------------------------------------------------------------------------------

    def records(self) -> list[dict]:
        out = []
        if self.derive_dir.is_dir():
            for d in sorted(p for p in self.derive_dir.iterdir() if p.is_dir() and ID_RE.match(p.name)):
                try:
                    doc = json.loads((d / RECORD).read_text(encoding="utf-8"))
                except (OSError, ValueError):
                    continue
                if isinstance(doc, dict) and doc.get("id") == d.name:
                    out.append(doc)
        return out

    def list_derivations(self) -> list[dict]:
        keep = ("id", "state", "ok", "card", "started_utc", "finished_utc", "error", "config_hash", "schema_hash")
        return [{**{k: r.get(k) for k in keep}, "edits": [e["key"] for e in r.get("edits", [])]} for r in self.records()]

    def directory(self, derive_id) -> Path:
        if not isinstance(derive_id, str) or not ID_RE.match(derive_id) or not (self.derive_dir / derive_id / RECORD).is_file():
            raise DeriveRefused(f"{str(derive_id)[:40]!r} is not a derivation of this helper run")
        return self.derive_dir / derive_id

    def find_config(self, config_hash: str) -> Path | None:
        """The .mcfg of a finished derivation with this config hash, or None. The hash is recomputed from the file."""
        if not isinstance(config_hash, str) or not HASH_RE.match(config_hash):
            return None
        for rec in self.records():
            if rec.get("ok") and rec.get("config_hash") == config_hash:
                path = self.derive_dir / rec["id"] / f"{PARAMS_SET}.mcfg"
                try:
                    cfg = mcfg.decode(path.read_bytes())
                    if config_hash_of(cfg, int(rec["schema_hash"], 16)) == config_hash:
                        return path
                except (OSError, mcfg.McfgError, FlashRefused, ValueError, KeyError):
                    continue
        return None

    def report(self, derive_id, name) -> dict:
        d = self.directory(derive_id)
        if not isinstance(name, str) or not REPORT_RE.match(name):
            raise DeriveRefused(f"{str(name)[:40]!r} is not the name of a derivation report")
        path = d / "flatten" / name
        if not path.is_file():
            raise DeriveRefused(f"the derivation has no report {name}")
        return {"derive_id": derive_id, "name": name, "text": path.read_text(encoding="utf-8", errors="replace")}

    def values(self, derive_id) -> dict:
        """The parameters of a finished derivation as the Parameters page stages them: [{name, link_id, type, unit, bits, method, locked, source}]."""
        d = self.directory(derive_id)
        rec = json.loads((d / RECORD).read_text(encoding="utf-8"))
        if not rec.get("ok"):
            raise DeriveRefused("the derivation has no result")
        manifest = mcfg.load_manifest(d / "gen" / "marv" / "params" / "params_manifest.json")
        params = [{"name": n, "link_id": e["link_id"], "type": e["type"], "unit": e["unit"], "bits": int(e["default_bits"], 16), "method": e["method"],
                   "locked": bool(e["lock"]), "source": e["source"]} for n, e in manifest["params"].items()]
        return {"derive_id": derive_id, "config_hash": rec["config_hash"], "schema_hash": rec["schema_hash"], "params": params}

    # ---- start ----------------------------------------------------------------------------------------------------------

    async def start(self, msg: dict) -> dict:
        for k in msg:
            if k not in REQUEST_KEYS:
                raise DeriveRefused(f"derive: unknown key {k!r}")
        try:
            card = resolve_card(msg.get("card"))
        except ValueError as e:
            raise DeriveRefused(str(e)) from None
        if self.busy():
            raise DeriveRefused("a derivation is running: one at a time", refusals=self.refusals())
        self._preparing = True
        try:
            return await self._start(card, msg.get("edits", []))
        finally:
            self._preparing = False

    def _new_id(self) -> tuple[str, Path]:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.derive_dir.mkdir(parents=True, exist_ok=True)
        n = 0
        while True:
            derive_id = f"{stamp}-{n}"
            try:
                (self.derive_dir / derive_id).mkdir()
                return derive_id, self.derive_dir / derive_id
            except FileExistsError:
                n += 1

    def _write_record(self, work: Path, rec: dict) -> None:
        tmp = work / (RECORD + ".tmp")
        tmp.write_text(json.dumps(rec, indent=1, sort_keys=True) + "\n", encoding="utf-8")
        os.replace(tmp, work / RECORD)

    def _mirror(self, work: Path, card: str, doc: dict, src: Path, edits: list[dict]) -> dict:
        """The inputs as copies under <work>/root, at their repository-relative paths; {role: path} of what the steps read."""
        mirror = work / "root"
        profile = doc.get("sensor_profile")
        files = {"card": card, "budget": BUDGET, "scenario": SCENARIO}
        if isinstance(profile, str) and lint.IDENTIFIER.fullmatch(profile):
            files["profile"] = str(lint.PROFILE_DIR / f"{profile}.yaml")
        for role, rel in files.items():
            dest = mirror / rel
            if role == "card":
                write_card_copy(src, doc, edits, dest)
            elif (self.root / rel).is_file():
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(self.root / rel, dest)
        return {"root": mirror, **{role: mirror / rel for role, rel in files.items()}}

    async def _lint(self, paths: dict, work: Path) -> tuple[int | None, str, str]:
        argv = [self.python, str(self.root / "tools" / "card" / "lint.py"), "--card", str(paths["card"]), "--budget", str(paths["budget"]),
                "--scenario", str(paths["scenario"]), "--root", str(paths["root"])]
        proc = await asyncio.create_subprocess_exec(*argv, cwd=work, stdin=asyncio.subprocess.DEVNULL, stdout=asyncio.subprocess.PIPE,
                                                    stderr=asyncio.subprocess.PIPE, start_new_session=True)
        try:
            out, err = await asyncio.wait_for(proc.communicate(), LINT_TIMEOUT_S)
        except asyncio.TimeoutError:
            await self._kill(proc)
            return None, "", f"the lint did not finish in {LINT_TIMEOUT_S:g} s and was killed"
        return proc.returncode, out.decode("utf-8", "replace"), err.decode("utf-8", "replace")

    async def _start(self, card: str, edits_in) -> dict:
        src, doc = await asyncio.to_thread(card_text_doc, card)
        edits = parse_edits(card, doc, edits_in)
        derive_id, work = await asyncio.to_thread(self._new_id)
        rec = {"schema": DERIVE_SCHEMA, "id": derive_id, "state": "linting", "ok": None, "card": card, "params_set": PARAMS_SET, "edits": edits,
               "started_utc": utc_iso(), "finished_utc": None, "error": None, "findings": [], "steps": [], "inputs": {}, "describe": None,
               "config_hash": None, "schema_hash": None, "mcfg": None, "reports": [], "images": [], "baseline": None, "changes": None}
        paths = await asyncio.to_thread(self._mirror, work, card, doc, src, edits)
        rc, _, err = await self._lint(paths, work)
        findings = provenance_findings(card, edits)
        if rc != 0:
            findings = parse_findings(err, paths["root"]) + findings
            if not findings:
                findings = [{"file": "", "entry": "", "reason": err.strip() or f"the lint exited with status {rc}"}]
        rec["findings"] = findings
        if findings:
            rec.update(state="refused", ok=False, finished_utc=utc_iso(), error=f"the card copy does not lint: {len(findings)} finding{'s' * (len(findings) != 1)}")
            await asyncio.to_thread(self._write_record, work, rec)
            self.state = dict(rec)
            self.log(f"derive refused ({derive_id}): {rec['error']}")
            raise DeriveRefused(rec["error"], findings=findings, derive=self.view())
        reasons = self.refusals(own=True)
        if reasons:
            rec.update(state="refused", ok=False, finished_utc=utc_iso(), error=reasons[0])
            await asyncio.to_thread(self._write_record, work, rec)
            self.log(f"derive refused ({derive_id}): {'; '.join(reasons)}")
            raise DeriveRefused(reasons[0], refusals=reasons)
        try:
            describe = await asyncio.to_thread(git_describe, self.root)
        except (OSError, subprocess.SubprocessError) as e:
            raise DeriveRefused(f"the derivation is not started: {e}", refusals=[str(e)]) from None
        self._lock_fd = take_heavy_lock(self.lock_path(), f"derive {derive_id}")  # the lock itself is the arbiter: the refusals above are a courtesy
        try:
            rec.update(state="running", describe=describe, steps=self.plan_steps(paths, work), inputs=await asyncio.to_thread(self._hash_inputs, paths))
            await asyncio.to_thread(self._write_record, work, rec)
            self.state = {**rec, "running": True, "step": None, "tail": [], "log": str(work / LOG)}
            self._cancel = False
            self.log(f"derive started: {derive_id} card {card} edits {[e['key'] for e in edits]} describe {describe}")
            self.push({"event": "derive_started", "id": derive_id, "card": card, "edits": [e["key"] for e in edits], "describe": describe})
            self._task = asyncio.create_task(self._run(work, rec))
        except BaseException:
            self._release_lock()
            raise
        return {"started": True, "derive": self.view()}

    # ---- the steps: the commands of fw/params/CMakeLists.txt --------------------------------------------------------------

    def slot_half(self) -> int:
        match = SLOT_RE.search((self.root / BUDGET).read_text(encoding="utf-8"))
        if not match:
            raise DeriveRefused(f"config_slot_bytes has no integer value in {BUDGET}")
        return int(match.group(1)) // 2  # fw/params/CMakeLists.txt: MARV_CONFIG_SLOT_BYTES / 2

    def plan_steps(self, paths: dict, work: Path) -> list[dict]:
        flat = work / "flatten"
        stem = f"{PARAMS_SET}_"
        out = {role: flat / f"{stem}{role}.yaml" for role in (*FLATTEN_PARTS, "register")}
        gen_dir = work / "gen" / "marv" / "params"
        flatten = [self.python, str(self.root / "tools" / "card" / "flatten_cached.py"),
                   "--card", str(paths["card"]), "--budget", str(paths["budget"]),
                   "--out-card", str(out["card"]), "--out-register", str(out["register"]), "--out-mixer", str(out["mixer"]),
                   "--scenario", str(paths["scenario"]), "--out-scenario", str(out["scenario"]),
                   "--out-rate-lead", str(out["rate"]), "--rate-guard", RATE_GUARD,
                   "--out-attitude-lead", str(out["attitude"]), "--out-gyro-chain", str(out["gyro_chain"]), "--out-vertical", str(out["vertical"]),
                   "--root", str(paths["root"])]
        gen = [self.python, str(self.root / "tools" / "gen" / "params_gen.py")]
        for part in FLATTEN_PARTS:
            gen += ["--card", str(out[part])]
        gen += ["--register", str(out["register"]), *[a for r in EXTRA_REGISTERS for a in ("--register", str(self.root / r))]]
        # the readable table of every record (PRODUCT_TABLE): its header names these repository-relative inputs, as in a CMake build
        rel = lambda key: Path(paths[key]).relative_to(paths["root"]).as_posix()  # noqa: E731
        table_inputs = [rel("card"), *([rel("profile")] if "profile" in paths else []), BUDGET, SCENARIO, *EXTRA_REGISTERS, "tools/card/*.py"]
        gen += [a for t in table_inputs for a in ("--table-input", t)]
        gen += ["--out-table", str(work / f"{PARAMS_SET}_product_table.txt"), "--out-dir", str(gen_dir)]
        encode = [self.python, str(self.root / "tools" / "config" / "mcfg.py"), "encode", "--manifest", str(gen_dir / "params_manifest.json"),
                  "--section", "quad", "--max-bytes", str(self.slot_half()), "--out", str(work / f"{PARAMS_SET}.mcfg")]
        return [{"name": "flatten", "argv": flatten}, {"name": "params_gen", "argv": gen}, {"name": "mcfg", "argv": encode}]

    def _hash_inputs(self, paths: dict) -> dict:
        """repository-relative path -> SHA-256 of every file the steps read (the mirrored copies, then the registers read in place)."""
        mirror = Path(paths["root"])
        out = {p.relative_to(mirror).as_posix(): sha256_of(p) for p in sorted(mirror.rglob("*")) if p.is_file()}
        out.update({r: sha256_of(self.root / r) for r in EXTRA_REGISTERS})
        return out

    async def _run(self, work: Path, rec: dict) -> None:
        st = self.state
        error, ok = None, False
        try:
            env = {**os.environ, "MARV_HEAVY_HELD": str(self.lock_path())}
            (work / "gen" / "marv" / "params").mkdir(parents=True, exist_ok=True)
            (work / "flatten").mkdir(exist_ok=True)
            steps = rec["steps"]
            with open(work / LOG, "wb") as out:
                for i, step in enumerate(steps, 1):
                    if self._cancel:
                        break
                    st["step"] = step["name"]
                    out.write(f"+ {' '.join(step['argv'])}\n".encode())
                    out.flush()
                    self.push({"event": "derive_step", "id": rec["id"], "step": step["name"], "index": i, "steps": len(steps)})
                    status = await self._step(step["argv"], work, env, out)
                    step["status"] = status
                    if status != 0:
                        error = f"the {step['name']} step exited with status {status}" if status is not None else f"the {step['name']} step did not finish"
                        break
                else:
                    ok = not self._cancel
            if ok:
                await asyncio.to_thread(self._finish, work, rec)
        except OSError as e:
            error = f"a step did not start: {e}"
        except (mcfg.McfgError, FlashRefused, KeyError, ValueError) as e:
            error = f"the result cannot be read: {e}"
            ok = False
        except asyncio.CancelledError:
            error = "the helper stopped the derivation"
            raise
        finally:
            await self._reap()
            self._release_lock()
            self._proc = None
            if self._cancel:
                ok, error = False, "cancelled"
            elif not ok and error is None:
                error = "the derivation did not finish"
            rec.update(state="done" if ok else ("cancelled" if self._cancel else "failed"), ok=ok, error=error, finished_utc=utc_iso())
            self._write_record(work, rec)
            st.update(rec, running=False)
            self.log(f"derive done: {rec['id']} ok {ok} {error or ''}")
            self.push({"event": "derive_done", "id": rec["id"], "ok": ok, "state": rec["state"], "error": error, "findings": rec["findings"],
                       "config_hash": rec["config_hash"], "schema_hash": rec["schema_hash"], "reports": rec["reports"], "images": rec["images"]})

    def _finish(self, work: Path, rec: dict) -> None:
        """Reads the result: the config hash of the .mcfg for the schema of its manifest, the reports, the images the config can be validated
        against and the parameters that differ from the factory config of the first of them."""
        manifest = mcfg.load_manifest(work / "gen" / "marv" / "params" / "params_manifest.json")
        schema_hash = mcfg.manifest_schema_hash(manifest)
        data = (work / f"{PARAMS_SET}.mcfg").read_bytes()
        cfg = mcfg.decode(data)
        rec.update(config_hash=config_hash_of(cfg, schema_hash), schema_hash=f"{schema_hash:#018x}",
                   mcfg={"name": f"{PARAMS_SET}.mcfg", "size": len(data), "sha256": hashlib.sha256(data).hexdigest()},
                   reports=sorted(p.name for p in (work / "flatten").glob("*_report.txt")))
        images, baseline = [], None
        for entry in self.library.scan()["images"]:
            art = entry["artifacts"]
            if "params_manifest" not in art:
                continue
            try:
                other = mcfg.load_manifest(self.build_root / art["params_manifest"])
                if mcfg.manifest_schema_hash(other) != schema_hash:
                    continue
            except (mcfg.McfgError, KeyError, ValueError):
                continue
            images.append({"hash": entry["hash"], "name": entry["name"], "kind": entry["kind"], "tree": entry["tree"]})
            if baseline is None:
                baseline = (entry, other)
        rec["images"] = images
        if baseline is not None:
            entry, other = baseline
            rec["baseline"] = {"hash": entry["hash"], "name": entry["name"], "tree": entry["tree"]}
            rec["changes"] = [{"name": n, "unit": e["unit"], "type": e["type"], "from_bits": other["params"][n]["default_bits"], "to_bits": e["default_bits"],
                               "method": e["method"], "locked": bool(e["lock"])}
                              for n, e in manifest["params"].items() if n in other["params"] and other["params"][n]["default_bits"] != e["default_bits"]]

    async def _step(self, argv: list[str], cwd: Path, env: dict, out) -> int | None:
        proc = await asyncio.create_subprocess_exec(*argv, cwd=cwd, env=env, stdin=asyncio.subprocess.DEVNULL, stdout=asyncio.subprocess.PIPE,
                                                    stderr=asyncio.subprocess.STDOUT, start_new_session=True)
        self._proc = proc
        if self._cancel:
            self._signal(proc.pid, signal.SIGKILL)
        reader = asyncio.create_task(self._read(proc, out))
        try:
            await asyncio.wait_for(proc.wait(), self.step_timeout_s)
        except asyncio.TimeoutError:
            await self._terminate()
            self.state["tail"] = [*self.state["tail"], f"killed after {self.step_timeout_s:g} s (PLACEHOLDER)"][-DERIVE_TAIL_LINES:]
            return None
        finally:
            try:
                await asyncio.wait_for(asyncio.shield(reader), 1.0)  # a descendant that escaped the group may hold the pipe
            except asyncio.TimeoutError:
                reader.cancel()
                await asyncio.gather(reader, return_exceptions=True)
        return proc.returncode

    async def _read(self, proc, out) -> None:
        st, pending = self.state, b""
        while chunk := await proc.stdout.read(READ_SIZE):
            out.write(chunk)
            pending += chunk
            *lines, pending = re.split(rb"[\r\n]", pending)
            for raw in lines:
                text = raw.decode("utf-8", "replace").rstrip()
                if text:
                    st["tail"] = [*st["tail"][-(DERIVE_TAIL_LINES - 1):], text]
        if pending.strip():
            st["tail"] = [*st["tail"][-(DERIVE_TAIL_LINES - 1):], pending.decode("utf-8", "replace").rstrip()]

    # ---- cancel and stop --------------------------------------------------------------------------------------------------

    @staticmethod
    def _signal(group: int, sig: int) -> None:
        try:
            os.killpg(group, sig)
        except ProcessLookupError:
            pass

    async def _kill(self, proc) -> None:
        self._signal(proc.pid, signal.SIGKILL)
        await proc.wait()

    async def _terminate(self) -> None:
        """SIGTERM to the step's process group, SIGKILL after stop_s; returns when the group is empty and the leader is reaped."""
        proc = self._proc
        if proc is None:
            return
        start = time.monotonic()
        self._signal(proc.pid, signal.SIGTERM)
        sent_kill = False
        while self._group_alive(proc.pid) or proc.returncode is None:
            waited = time.monotonic() - start
            if waited >= self.stop_s and not sent_kill:
                sent_kill = True
                self._signal(proc.pid, signal.SIGKILL)
            elif waited >= 2 * self.stop_s:
                return  # SIGKILL has not taken effect: do not wait for ever
            await asyncio.sleep(POLL_S)

    @staticmethod
    def _group_alive(pgid: int) -> bool:
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    async def _reap(self) -> None:
        proc = self._proc
        if proc is not None:
            self._signal(proc.pid, signal.SIGKILL)

    def _release_lock(self) -> None:
        if self._lock_fd is not None:
            os.close(self._lock_fd)
            self._lock_fd = None

    async def cancel(self) -> dict:
        if not (self._task is not None and not self._task.done()):
            raise DeviceError("no derivation is running")
        self._cancel = True
        self.log(f"derive cancel: {self.state['id']}")
        await self._terminate()
        await asyncio.wait_for(asyncio.shield(self._task), self.stop_s * 3)
        return {"cancelled": True, "derive": self.view()}

    async def stop(self) -> None:
        """The helper is stopping: a derivation still going is cancelled with its processes, the lock released."""
        if self._task is not None and not self._task.done():
            self._cancel = True
            await self._terminate()
            try:
                await asyncio.wait_for(asyncio.shield(self._task), self.stop_s * 3)
            except (asyncio.TimeoutError, asyncio.CancelledError):
                self._task.cancel()
                await asyncio.gather(self._task, return_exceptions=True)
        self._release_lock()
