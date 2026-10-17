"""The image library: the build trees' images keyed by image hash (ground spec section 3, decision 0053 Q5).

Image hash = SHA-256 of the image file:
  board  the raw binary `marv_fc3v2.bin` (the hash the board computes of its own image, unit 14). A tree with only an ELF or
         a UF2 has no image hash here and is listed under `unkeyed`: a UF2 is a different byte string than the binary.
  SIL    the SIL shared library `libmarv_sil_<name>.so` (the file the SIL loads and hashes).
Beside the image, in the same entry, when present: the ELF and the UF2 of a board image, and the image's parameter set files
(params_manifest.json, params_provenance.json, the factory <set>.mcfg) from <tree>/generated/<set>/.

Which parameter set an image links is not written in the build tree. PARAM_SET_OF names it for the images whose set is not
`marv_params_<name>` (CMake: sim/quad_fc and fw/hal/target link marv_params_fc3v2); an image not listed uses
`marv_params_<name after marv_sil_>` when that set exists, else it has no parameter files (`params_set` null).
"""

from __future__ import annotations

import hashlib
import re
from pathlib import Path

PARAM_SET_OF = {
    "marv_sil_quad_fc": "marv_params_fc3v2",
    "marv_fc3v2": "marv_params_fc3v2",
}
BOARD_NAME = "marv_fc3v2"
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

    def scan(self) -> dict:
        """{"images": [entry, ...] sorted by tree then name, "unkeyed": [{"tree", "name", "reason"}]}. Paths are relative to the build root."""
        images, unkeyed = [], []
        trees = sorted(p for p in self.root.iterdir() if p.is_dir()) if self.root.is_dir() else []
        for tree in trees:
            candidates = []  # (kind, name, image path, extra artifacts)
            for lib in sorted(tree.glob("sim/**/libmarv_sil_*.so")):
                candidates.append(("sil", lib.stem[len("lib"):], lib, {}))
            boards = sorted(tree.glob(f"fw/**/{BOARD_NAME}.elf")) + sorted(tree.glob(f"fw/**/{BOARD_NAME}.uf2")) \
                + sorted(tree.glob(f"fw/**/{BOARD_NAME}.bin"))
            for d in sorted({p.parent for p in boards}):
                extra = {k: d / f"{BOARD_NAME}.{k}" for k in ("elf", "uf2") if (d / f"{BOARD_NAME}.{k}").is_file()}
                binary = d / f"{BOARD_NAME}.bin"
                if binary.is_file():
                    candidates.append(("board", BOARD_NAME, binary, extra))
                else:
                    unkeyed.append({"tree": tree.name, "name": BOARD_NAME, "reason": "no .bin: the board image hash is the SHA-256 of the .bin"})
            for kind, name, image, extra in candidates:
                set_name = self._set_name(tree, name)
                files = {"image": image, **extra, **self._param_files(tree, set_name)}
                images.append({
                    "hash": self._hash(image),
                    "tree": tree.name,
                    "name": name,
                    "kind": kind,
                    "image_file": image.name,
                    "size": image.stat().st_size,
                    "params_set": set_name,
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
