#!/usr/bin/env python3
"""Every constant in constants.hpp carries a citation and a kind (core contracts 7.4, G1; decision 0002).

constants.hpp is the one file G1 exempts from the numeric-literal rule, so it holds physics, mathematics and published
standards only. Vehicle numbers belong in the card. The file is split into paragraphs by blank lines. Every
namespace-scope constant declaration (constexpr variable, or any variable defined with a numeric initialiser) must sit
in a paragraph whose `//` comment lines before its first declaration contain

  - a citation: `Citation:` or `Citation for every entry below:` followed by non-empty text, and
  - exactly one kind tag: `Kind: math`, `Kind: physics` or `Kind: standard` (standard = a published standard or
    specification such as SI, WGS 84 or the DShot protocol),

and whose comment does not refer to vehicle data (vehicles/, sensors/profiles/, design/budget, or the words card,
vehicle card, profile).

Usage: check_constants.py [FILE]   (default: fw/prim/include/marv/prim/constants.hpp)
Output: constants: <file>:<line>: <name>: <reason>. Exit status: 0 clean, 1 findings, 2 usage error.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT = ROOT / "fw" / "prim" / "include" / "marv" / "prim" / "constants.hpp"
KINDS = ("math", "physics", "standard")

_CITATION = re.compile(r"Citation(?: for every entry below)?:")
_KIND = re.compile(r"Kind:[ \t]*(\w*)")
_VEHICLE = re.compile(r"vehicles/|sensors/profiles/|design/budget|\b(?:card|vehicle card|profile)\b", re.IGNORECASE)
_NUMBER = re.compile(r"(?<![A-Za-z_0-9])(?:[0-9]|\.[0-9])")
_NAMESPACE_OPEN = re.compile(r"\s*(?:inline\s+)?namespace\b[^;{}=()]*$|\s*extern\s*$")
_SKIP_HEAD = re.compile(r"\s*(?:using|typedef|struct|class|enum|union|friend|static_assert)\b")
_TEMPLATE_HEAD = re.compile(r"\s*template\s*<[^<>]*>")
_ATTRIBUTE = re.compile(r"\[\[[^\]]*\]\]")


def blank_comments(text: str) -> str:
    """Text with comments, string and char literals and preprocessor lines blanked; newlines are kept."""
    out = []
    n = len(text)
    i = 0
    at_line_start = True
    while i < n:
        c = text[i]
        if text.startswith("//", i):
            while i < n and text[i] != "\n":
                out.append(" ")
                i += 1
            continue
        if text.startswith("/*", i):
            end = text.find("*/", i + 2)
            end = n if end < 0 else end + 2
            out.extend("\n" if ch == "\n" else " " for ch in text[i:end])
            i = end
            at_line_start = False
            continue
        if c == "\n":
            out.append(c)
            at_line_start = True
            i += 1
            continue
        if c in " \t\r":
            out.append(c)
            i += 1
            continue
        if c == "#" and at_line_start:
            while i < n and text[i] != "\n":
                out.append(" ")
                i += 1
            continue
        at_line_start = False
        if c in "\"'":
            j = i + 1
            while j < n and text[j] != c and text[j] != "\n":
                j += 2 if text[j] == "\\" else 1
            j = min(j + 1, n)
            out.extend("\n" if ch == "\n" else " " for ch in text[i:j])
            i = j
            continue
        out.append(c)
        i += 1
    return "".join(out)


def _skip_block(text: str, i: int) -> int:
    depth = 0
    n = len(text)
    while i < n:
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return i + 1
        i += 1
    return n


def _is_function_head(head: str) -> bool:
    m = re.search(r"[=({]", head)
    return m is not None and m.group() == "("


def statements(stripped: str) -> list[tuple[int, str]]:
    """(line, text) of every statement at namespace scope; function bodies and other blocks are folded in."""
    found: list[tuple[int, str]] = []
    buf: list[str] = []
    start_line = 0
    line = 1
    depth_paren = 0
    ns_depth = 0
    n = len(stripped)
    i = 0

    def flush() -> None:
        nonlocal buf
        text = "".join(buf).strip()
        if text:
            found.append((start_line, text))
        buf = []

    while i < n:
        c = stripped[i]
        if not buf and not c.isspace():
            start_line = line
        if c == "\n":
            line += 1
        if c == "(":
            depth_paren += 1
        elif c == ")":
            depth_paren -= 1
        if depth_paren <= 0 and c == ";":
            flush()
            i += 1
            continue
        if depth_paren <= 0 and c == "{":
            head = "".join(buf)
            if _NAMESPACE_OPEN.match(head):
                ns_depth += 1
                buf = []
                i += 1
                continue
            end = _skip_block(stripped, i)
            block = stripped[i:end]
            line += block.count("\n")
            buf.append(block)
            i = end
            if _is_function_head(head):
                flush()
            continue
        if c == "}" and ns_depth > 0:
            ns_depth -= 1
            buf = []
            i += 1
            continue
        if buf or not c.isspace():
            buf.append(c)
        i += 1
    flush()
    return found


def declarations(stripped: str) -> list[tuple[int, str, str]]:
    """(line, name, what) per constant: what is 'constexpr', or 'numeric' (non-constexpr, numeric initialiser)."""
    found = []
    for line, text in statements(stripped):
        while True:
            m = _TEMPLATE_HEAD.match(text)
            if not m:
                break
            text = text[m.end():]
        text = _ATTRIBUTE.sub(" ", text).strip()
        if not text or _SKIP_HEAD.match(text):
            continue
        m = re.search(r"[=({]", text)
        if m is not None and m.group() == "(":
            continue
        idx = m.start() if m else len(text)
        head, init = text[:idx], text[idx:]
        name_m = re.search(r"(\w+)\s*(?:\[[^\]]*\]\s*)*$", head)
        if name_m is None:
            continue
        if re.search(r"\bconstexpr\b", head):
            found.append((line, name_m.group(1), "constexpr"))
        elif _NUMBER.search(init):
            found.append((line, name_m.group(1), "numeric"))
    return found


def paragraphs(lines: list[str]) -> list[tuple[int, int]]:
    """Inclusive 1-based (first, last) line ranges separated by blank lines."""
    out = []
    first = None
    for no, raw in enumerate(lines, start=1):
        if raw.strip():
            if first is None:
                first = no
        elif first is not None:
            out.append((first, no - 1))
            first = None
    if first is not None:
        out.append((first, len(lines)))
    return out


def check_text(text: str, shown: str) -> list[str]:
    lines = text.splitlines()
    decls = declarations(blank_comments(text))
    if not decls:
        return [f"constants: {shown}:1: <none>: no constant declaration found; the check would be vacuous"]
    findings: list[str] = []

    for first, last in paragraphs(lines):
        para = [d for d in decls if first <= d[0] <= last]
        if not para:
            continue
        first_decl = min(d[0] for d in para)
        comment = "\n".join(
            lines[no - 1].strip()[2:].strip() for no in range(first, first_decl) if lines[no - 1].strip().startswith("//")
        )
        reasons: list[str] = []
        cite = _CITATION.search(comment)
        if not cite:
            reasons.append("no 'Citation:' before the first declaration of its paragraph")
        elif not re.search(r"\w", _KIND.sub("", comment[cite.end():])):
            reasons.append("'Citation:' has no text")
        kinds = _KIND.findall(comment)
        if not kinds:
            reasons.append("no 'Kind: math|physics|standard' before the first declaration of its paragraph")
        elif len(kinds) > 1:
            reasons.append(f"more than one Kind tag ({', '.join(kinds)}); exactly one is required")
        elif kinds[0] not in KINDS:
            reasons.append(f"Kind '{kinds[0]}' is not one of math, physics, standard")
        if _VEHICLE.search(_KIND.sub("", comment)):
            reasons.append("comment refers to vehicle data; vehicle numbers belong in the card, not in constants.hpp")
        for line, name, what in para:
            for reason in reasons:
                findings.append(f"constants: {shown}:{line}: {name}: {reason}")
            if what == "numeric":
                findings.append(
                    f"constants: {shown}:{line}: {name}: non-constexpr variable with a numeric initialiser; "
                    "constants are constexpr here"
                )
    return findings


def main(argv: list[str]) -> int:
    if len(argv) > 1 or (argv and argv[0].startswith("-")):
        print("usage: check_constants.py [FILE]", file=sys.stderr)
        return 2
    path = Path(argv[0]) if argv else DEFAULT
    if not path.is_file():
        print(f"constants: {path}: not a file", file=sys.stderr)
        return 2
    try:
        shown = str(path.resolve().relative_to(ROOT))
    except ValueError:
        shown = str(path)
    findings = check_text(path.read_text(encoding="utf-8"), shown)
    for f in findings:
        print(f)
    return 1 if findings else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
