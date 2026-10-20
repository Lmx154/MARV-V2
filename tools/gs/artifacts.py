"""The image library: the build trees' images keyed by image hash (ground spec section 3, decision 0053 Q5).

Image hash = SHA-256 of the image file:
  board  the raw binary `marv_fc3v2.bin` (the hash the board computes of its own image, unit 14). A tree with an ELF and no
         .bin (the m33 build writes none) is keyed by the SHA-256 of the ELF's flash bytes, tools/config/image_hash.py (the same
         bytes; its `image` artifact is then the ELF). A tree with only a UF2, or an ELF that has no flash image, has no image
         hash here and is listed under `unkeyed`: a UF2 is a different byte string than the binary.
  SIL    the SIL shared library `libmarv_sil_<name>.so` (the file the SIL loads and hashes).
Beside the image, in the same entry, when present: the ELF and the UF2 of a board image, and the image's parameter set files
(params_manifest.json, params_provenance.json, the factory <set>.mcfg) from <tree>/generated/<set>/.

A GCS build (tools/gs/build.py) leaves <tree>/gs-build.json with the `git describe --dirty` of its source and the hashes of the images
it produced; an entry whose hash is in it carries `build`: {preset, describe, finished_utc}, else `build` is null (an image another
build left in the tree has no GCS describe string).

Which parameter set an image links is not written in the build tree. PARAM_SET_OF names it for the images whose set is not
`marv_params_<name>` (CMake: sim/quad_fc and fw/hal/target link marv_params_fc3v2); an image not listed uses
`marv_params_<name after marv_sil_>` when that set exists, else it has no parameter files (`params_set` null).
"""

from __future__ import annotations

import hashlib
import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "config"))
import image_hash as board_image  # noqa: E402
import mcfg  # noqa: E402

PARAM_SET_OF = {
    "marv_sil_quad_fc": "marv_params_fc3v2",
    "marv_fc3v2": "marv_params_fc3v2",
}
BOARD_NAME = "marv_fc3v2"
BUILD_RECORD = "gs-build.json"  # tools/gs/build.py writes it into the build tree
HASH_RE = re.compile(r"^[0-9a-f]{64}$")
ARTIFACT_KEYS = ("image", "elf", "uf2", "params_manifest", "params_provenance", "factory_mcfg")
CHUNK = 1 << 20  # file read size for hashing: an implementation constant


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(CHUNK):
            h.update(chunk)
    return h.hexdigest()


class ImageLibrary:
    def __init__(self, build_root: Path):
        self.root = Path(build_root)
        self._hash_cache: dict[tuple[str, int, int], str] = {}

    def _hash(self, path: Path) -> str:
        st = path.stat()
        key = (str(path), st.st_mtime_ns, st.st_size)
        if key not in self._hash_cache:
            self._hash_cache[key] = sha256_file(path)
        return self._hash_cache[key]

    def _elf_hash(self, path: Path) -> str:
        """SHA-256 of the flash bytes of an ELF (the board's image hash); board_image.ImageError when it has none."""
        st = path.stat()
        key = (str(path), st.st_mtime_ns, st.st_size)
        if key not in self._hash_cache:
            with open(path, "rb") as f:
                if f.read(len(board_image.ELF_MAGIC)) != board_image.ELF_MAGIC:
                    raise board_image.ImageError("it is not an ELF file")
            self._hash_cache[key] = hashlib.sha256(board_image.flash_image(path).data).hexdigest()
        return self._hash_cache[key]

    def _param_files(self, tree: Path, set_name: str | None) -> dict[str, Path]:
        if set_name is None:
            return {}
        d = tree / "generated" / set_name
        files = {
            "params_manifest": d / "marv" / "params" / "params_manifest.json",
            "params_provenance": d / "marv" / "params" / "params_provenance.json",
            "factory_mcfg": d / f"{set_name}.mcfg",
        }
        return {k: p for k, p in files.items() if p.is_file()}

    def _set_name(self, tree: Path, image_name: str) -> str | None:
        name = PARAM_SET_OF.get(image_name)
        if name is None and image_name.startswith("marv_sil_"):
            name = "marv_params_" + image_name[len("marv_sil_"):]
        return name if name and (tree / "generated" / name).is_dir() else None

    @staticmethod
    def _build_record(tree: Path) -> dict:
        try:
            record = json.loads((tree / BUILD_RECORD).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}
        return record if isinstance(record, dict) and isinstance(record.get("images"), dict) else {}

    def scan(self) -> dict:
        """{"images": [entry, ...] sorted by tree then name, "unkeyed": [{"tree", "name", "reason"}]}. Paths are relative to the build root."""
        images, unkeyed = [], []
        trees = sorted(p for p in self.root.iterdir() if p.is_dir()) if self.root.is_dir() else []
        for tree in trees:
            record = self._build_record(tree)
            candidates = []  # (kind, name, image path, extra artifacts, image hash when not the SHA-256 of the file)
            for lib in sorted(tree.glob("sim/**/libmarv_sil_*.so")):
                candidates.append(("sil", lib.stem[len("lib"):], lib, {}, None))
            boards = sorted(tree.glob(f"fw/**/{BOARD_NAME}.elf")) + sorted(tree.glob(f"fw/**/{BOARD_NAME}.uf2")) \
                + sorted(tree.glob(f"fw/**/{BOARD_NAME}.bin"))
            for d in sorted({p.parent for p in boards}):
                extra = {k: d / f"{BOARD_NAME}.{k}" for k in ("elf", "uf2") if (d / f"{BOARD_NAME}.{k}").is_file()}
                binary, elf = d / f"{BOARD_NAME}.bin", d / f"{BOARD_NAME}.elf"
                if binary.is_file():
                    candidates.append(("board", BOARD_NAME, binary, extra, None))
                elif elf.is_file():
                    try:
                        candidates.append(("board", BOARD_NAME, elf, extra, self._elf_hash(elf)))
                    except (board_image.ImageError, OSError) as e:
                        unkeyed.append({"tree": tree.name, "name": BOARD_NAME, "reason": f"the ELF has no flash image: {e}"})
                else:
                    unkeyed.append({"tree": tree.name, "name": BOARD_NAME, "reason": "no .bin or ELF: the board image hash is the SHA-256 of the .bin"})
            for kind, name, image, extra, known_hash in candidates:
                set_name = self._set_name(tree, name)
                files = {"image": image, **extra, **self._param_files(tree, set_name)}
                image_hash = known_hash or self._hash(image)
                images.append({
                    "hash": image_hash,
                    "tree": tree.name,
                    "name": name,
                    "kind": kind,
                    "image_file": image.name,
                    "size": image.stat().st_size,
                    "params_set": set_name,
                    "build": {k: record.get(k) for k in ("preset", "describe", "finished_utc")} if image_hash in record.get("images", {}) else None,
                    "artifacts": {k: str(p.relative_to(self.root)) for k, p in files.items()},
                })
        return {"images": images, "unkeyed": unkeyed}

    def lookup(self, image_hash: str, artifact: str) -> Path | None:
        """The file of `artifact` for the image with this hash, or None. The first entry (by tree name) wins a duplicate hash."""
        if not HASH_RE.match(image_hash) or artifact not in ARTIFACT_KEYS:
            return None
        for entry in self.scan()["images"]:
            if entry["hash"] == image_hash and artifact in entry["artifacts"]:
                path = (self.root / entry["artifacts"][artifact]).resolve()
                if path.is_file() and self.root.resolve() in path.parents:
                    return path
        return None


class ReportLibrary:
    """Validation reports keyed by (image hash, config hash) (ground-segment unit 9, G7): <root>/<image hash>/<config hash>/
    holds config.mcfg (the config validated) and report.json (tools/validate/validate_config.py's report, whose id is the
    SHA-256 of the file's bytes)."""

    def __init__(self, root: Path):
        self.root = Path(root)

    def directory(self, image_hash: str, config_hash: str) -> Path | None:
        if not (HASH_RE.match(image_hash) and HASH_RE.match(config_hash)):
            return None
        return self.root / image_hash / config_hash

    def report(self, image_hash: str, config_hash: str) -> Path | None:
        d = self.directory(image_hash, config_hash)
        path = None if d is None else d / "report.json"
        return path if path is not None and path.is_file() else None

    def find_config(self, config_hash: str) -> Path | None:
        """A stored config.mcfg whose config hash is `config_hash` (any image's directory), or None. The hash is recomputed from the
        file, so a file that does not hold what its directory name says is not returned."""
        if not HASH_RE.match(config_hash) or not self.root.is_dir():
            return None
        for path in sorted(self.root.glob(f"*/{config_hash}/config.mcfg")):
            try:
                cfg = mcfg.decode(path.read_bytes())
            except (mcfg.McfgError, OSError):
                continue
            if mcfg.config_hash(cfg).hex() == config_hash:
                return path
        return None
