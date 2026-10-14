#!/usr/bin/env python3
"""The repository files a reference generator depends on, found statically (python stdlib only).

    closure(generator, depends=(), unresolved_ok=()) -> (files, problems)

files     sorted repo-relative paths: the generator; every repo-local Python module reachable from it by import (Import and
          ImportFrom at any depth in any file of the closure, relative imports included; a module is looked up in the
          directory of every file of the closure and in every directory a sys.path.insert / append names); every repo file a
          path expression in those files names (os.path.join, Path, joinpath, `/`, or a string with a `/` that is a repo
          file), Python files so named joining the closure; and the files matched by `depends` (repo-relative globs).
problems  texts for what could not be resolved, which the caller must treat as a failure: an import that is neither stdlib,
          installed nor in the repository; a path expression with a literal component and a base that is not statically
          known whose literal tail does not start at a top-level repository directory, unless its file is in
          `unresolved_ok` (the declaration that `depends` covers what that file reads); a `depends` glob matching nothing.
          A tail that does start at a top-level directory ("... / 'vehicles' / x") is resolved against the repository root.

The closure is deliberately wide: it takes every function of every file, including the --refresh-* paths of an oracle that
refdata.py never runs, so a stale cache can only err towards regenerating. Limits: a path that reaches a file only through a
variable holding a command-line argument other than --dir, through os.environ or through arbitrary computation is not seen;
refdata.py runs a generator with only --dir and --procs. A directory-valued expression adds the directory only as a module
search path, never its contents.
"""

import ast
import glob
import importlib.util
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
UNKNOWN = None
WORK = object()
SKIP_DIRS = {".venv", "build", ".git"}
PATH_FUNCS = {"join", "Path", "joinpath"}
PASS_FUNCS = {"abspath", "normpath", "realpath", "resolve", "str", "expanduser"}


def rel_of(path):
    """Repo-relative posix path of a path inside the repository (not .venv, build, .git), else None."""
    try:
        rel = Path(path).resolve().relative_to(REPO)
    except (ValueError, OSError):
        return None
    if not rel.parts or rel.parts[0] in SKIP_DIRS:
        return None
    return rel.as_posix()


def _func_name(call):
    return call.func.attr if isinstance(call.func, ast.Attribute) else getattr(call.func, "id", "")


def _is_os_path_join(call):
    f = call.func
    return isinstance(f, ast.Attribute) and f.attr == "join" and isinstance(f.value, ast.Attribute) and f.value.attr == "path"


def _div_leaves(n):
    if isinstance(n, ast.BinOp) and isinstance(n.op, ast.Div):
        return _div_leaves(n.left) + _div_leaves(n.right)
    return [n]


def _stringy(leaf):
    """A leaf that makes a `/` chain a path: a string, an f-string, a string concatenation or a Path / os.path.join call."""
    if isinstance(leaf, (ast.JoinedStr, ast.Call)):
        return isinstance(leaf, ast.JoinedStr) or _is_path_expr(leaf)
    if isinstance(leaf, ast.Constant):
        return isinstance(leaf.value, str)
    if isinstance(leaf, ast.BinOp) and isinstance(leaf.op, ast.Add):
        return any(isinstance(c, ast.Constant) and isinstance(c.value, str) for c in (leaf.left, leaf.right))
    return False


def _is_path_expr(n):
    if isinstance(n, ast.BinOp):
        return isinstance(n.op, ast.Div) and any(_stringy(leaf) for leaf in _div_leaves(n))
    if isinstance(n, ast.Call):
        return _func_name(n) in ("Path", "joinpath") or _is_os_path_join(n)
    return False


def _outermost_path_exprs(tree):
    inner = set()
    nodes = [n for n in ast.walk(tree) if isinstance(n, (ast.Call, ast.BinOp)) and _is_path_expr(n)]
    for n in nodes:
        inner.update(id(c) for c in ast.walk(n) if c is not n)
    return [n for n in nodes if id(n) not in inner]


class Evaluator:
    """Evaluates the expressions of one file. A value is a str, an int, a list, WORK (rooted at --dir) or UNKNOWN."""

    def __init__(self, tree, file):
        self.env = {"__file__": str(file)}
        self.local = {}
        for fn in ast.walk(tree):
            if isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
                scope = dict(self.env)
                self._assign(fn.body, scope)
                self.local[id(fn)] = scope
        self._assign(tree.body, self.env)

    def _assign(self, body, scope):
        for st in body:
            if isinstance(st, ast.Assign) and len(st.targets) == 1 and isinstance(st.targets[0], ast.Name):
                scope[st.targets[0].id] = self.eval(st.value, scope)

    def eval(self, node, env=None):
        env = self.env if env is None else env
        if isinstance(node, ast.Constant):
            return node.value if isinstance(node.value, (str, int)) and not isinstance(node.value, bool) else UNKNOWN
        if isinstance(node, ast.Name):
            return env.get(node.id, UNKNOWN)
        if isinstance(node, ast.Attribute):
            if node.attr == "pardir" and isinstance(node.value, ast.Name) and node.value.id == "os":
                return ".."
            if node.attr == "dir" and isinstance(node.value, ast.Name) and node.value.id == "args":
                return WORK
            if node.attr == "parent":
                base = self.eval(node.value, env)
                return os.path.dirname(base) if isinstance(base, str) else base
            return UNKNOWN
        if isinstance(node, ast.Subscript):
            if isinstance(node.value, ast.Attribute) and node.value.attr == "parents" and isinstance(node.slice, ast.Constant):
                base = self.eval(node.value.value, env)
                if isinstance(base, str):
                    for _ in range(node.slice.value + 1):
                        base = os.path.dirname(base)
                    return base
            return UNKNOWN
        if isinstance(node, (ast.List, ast.Tuple)):
            return [self.eval(e, env) for e in node.elts]
        if isinstance(node, ast.BinOp):
            if isinstance(node.op, ast.Mult):
                left, right = self.eval(node.left, env), self.eval(node.right, env)
                return left * right if isinstance(left, list) and isinstance(right, int) else UNKNOWN
            if isinstance(node.op, ast.Add):
                left, right = self.eval(node.left, env), self.eval(node.right, env)
                return left + right if isinstance(left, str) and isinstance(right, str) else UNKNOWN
            if isinstance(node.op, ast.Div):
                return self.joined(self.parts(node, env))
            return UNKNOWN
        if isinstance(node, ast.JoinedStr):
            return "".join(v.value if isinstance(v, ast.Constant) else "*" for v in node.values)
        if isinstance(node, ast.Call):
            name = _func_name(node)
            if name in PATH_FUNCS and (name != "join" or _is_os_path_join(node)):
                return self.joined(self.parts(node, env))
            if name in PASS_FUNCS:
                inner = node.args[0] if node.args else (node.func.value if isinstance(node.func, ast.Attribute) else None)
                base = self.eval(inner, env) if inner is not None else UNKNOWN
                return os.path.normpath(base) if isinstance(base, str) and name != "str" else base
            if name == "dirname" and node.args:
                base = self.eval(node.args[0], env)
                return os.path.dirname(base) if isinstance(base, str) else base
        return UNKNOWN

    def parts(self, node, env=None):
        """The components of a path expression, each a str, WORK or UNKNOWN."""
        env = self.env if env is None else env
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Div):
            return self.parts(node.left, env) + self.parts(node.right, env)
        if isinstance(node, ast.Call):
            name = _func_name(node)
            if name in PATH_FUNCS and (name != "join" or _is_os_path_join(node)):
                out = self.parts(node.func.value, env) if name == "joinpath" and isinstance(node.func, ast.Attribute) else []
                for a in node.args:
                    if isinstance(a, ast.Starred):
                        v = self.eval(a.value, env)
                        out += v if isinstance(v, list) else [UNKNOWN]
                    else:
                        out += self.parts(a, env)
                return out
            if name in PASS_FUNCS and node.args:
                return self.parts(node.args[0], env)
        v = self.eval(node, env)
        return [v if isinstance(v, str) or v is WORK else UNKNOWN]

    @staticmethod
    def joined(parts):
        if any(p is WORK for p in parts):
            return WORK
        if not parts or any(not isinstance(p, str) for p in parts):
            return UNKNOWN
        return os.path.join(*parts)


def _repo_tops():
    return {p.name for p in REPO.iterdir() if p.is_dir() and p.name not in SKIP_DIRS and not p.name.startswith(".")}


def _module_files(parts, level, file_dir, search):
    """Repo files for the dotted module `parts`: each package __init__ on the way and the module itself."""
    bases = [file_dir] if level else list(search)
    for _ in range(max(level - 1, 0)):
        bases = [os.path.dirname(b) for b in bases]
    found = []
    for base in bases:
        for i in range(1, len(parts) + 1):
            stem = os.path.join(base, *parts[:i])
            for cand in (stem + ".py", os.path.join(stem, "__init__.py")):
                if os.path.isfile(cand) and rel_of(cand):
                    found.append(cand)
    return found


def _known_module(top):
    if top in sys.stdlib_module_names or top in sys.builtin_module_names:
        return True
    try:
        return importlib.util.find_spec(top) is not None
    except (ImportError, ValueError):
        return False


def _glob_files(pattern):
    return [h for h in glob.glob(pattern, recursive=True) if os.path.isfile(h) and rel_of(h)]


def closure(generator, depends=(), unresolved_ok=()):
    gen = Path(generator).resolve()
    files = {gen}
    problems = []
    search = [str(gen.parent), str(REPO)]
    parsed = {}
    tops = _repo_tops()
    pending = [gen]

    def add(path):
        path = Path(path).resolve()
        if rel_of(path) and path not in files:
            files.add(path)
            if path.suffix == ".py":
                pending.append(path)

    def scan(f, tree, ev):
        rel_f = rel_of(f)
        for n in ast.walk(tree):
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr in ("insert", "append") \
               and isinstance(n.func.value, ast.Attribute) and n.func.value.attr == "path" and n.args:
                v = ev.eval(n.args[-1])
                if isinstance(v, str) and os.path.isdir(v) and rel_of(v) and v not in search:
                    search.append(v)
        for n in ast.walk(tree):
            if isinstance(n, (ast.Import, ast.ImportFrom)):
                if isinstance(n, ast.Import):
                    targets = [a.name.split(".") for a in n.names]
                    level = 0
                else:
                    mod = n.module.split(".") if n.module else []
                    targets = [mod] + [mod + [a.name] for a in n.names if a.name != "*"]
                    level = n.level
                for parts in targets:
                    for h in (_module_files(parts, level, str(f.parent), search) if parts else []):
                        add(h)
                head = targets[0]
                if head and not level and not _module_files(head, 0, str(f.parent), search) and not _known_module(head[0]):
                    problems.append(f"{rel_f}:{n.lineno}: import '{'.'.join(head)}' is not stdlib, not installed and not in the repository")
        for n in _outermost_path_exprs(tree):
            parts = ev.parts(n)
            if any(p is WORK for p in parts):
                continue
            if all(isinstance(p, str) for p in parts):
                cand = os.path.join(*parts)
                cand = cand if os.path.isabs(cand) else os.path.join(REPO, cand)
                for h in (_glob_files(cand) if "*" in cand else [cand] if os.path.isfile(cand) else []):
                    add(h)
                continue
            tail = []
            for p in reversed(parts):
                if not isinstance(p, str):
                    break
                tail.insert(0, p)
            literal = any(isinstance(c, ast.Constant) and isinstance(c.value, str) for c in ast.walk(n)) or tail
            if not literal:
                continue
            tail_path = os.path.join(*tail) if tail else ""
            if tail_path and not os.path.isabs(tail_path) and tail_path.split(os.sep)[0] in tops:
                cand = os.path.join(REPO, tail_path)
                for h in (_glob_files(cand) if "*" in cand else [cand] if os.path.isfile(cand) else []):
                    add(h)
                continue
            if rel_f in unresolved_ok:
                continue
            problems.append(f"{rel_f}:{n.lineno}: path expression '{ast.unparse(n)}' has a base that is not statically known")
        for n in ast.walk(tree):
            if isinstance(n, ast.Constant) and isinstance(n.value, str) and "/" in n.value and not any(c.isspace() for c in n.value) \
               and not any(c in n.value for c in "*?[") and not os.path.isabs(n.value) and (REPO / n.value).is_file():
                add(REPO / n.value)

    while True:
        state = (len(files), len(search))
        while pending:
            f = pending.pop()
            if f not in parsed:
                tree = ast.parse(f.read_text(encoding="utf-8"), filename=str(f))
                parsed[f] = (tree, Evaluator(tree, f))
                if str(f.parent) not in search:
                    search.append(str(f.parent))
        for f, (tree, ev) in list(parsed.items()):
            scan(f, tree, ev)
        if not pending and state == (len(files), len(search)):
            break
    for pattern in depends:
        hits = _glob_files(str(REPO / pattern))
        if not hits:
            problems.append(f"depends '{pattern}' matches no file")
        files.update(Path(h).resolve() for h in hits)
    return sorted(rel_of(f) for f in files), sorted(set(problems))


if __name__ == "__main__":
    out, issues = closure(sys.argv[1], sys.argv[2:])
    print("\n".join(out))
    for issue in issues:
        print("PROBLEM:", issue, file=sys.stderr)
    sys.exit(1 if issues else 0)
