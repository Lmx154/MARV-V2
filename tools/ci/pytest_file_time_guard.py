"""Per-push time guard for the tools tests (S9, decision 0014 fifth round item 6; closed in quad L6 stage (e), decision 0019).

Loaded with `-p pytest_file_time_guard` (PYTHONPATH=tools/ci) in the core job's per-push tools step. The limit is the
design-budget entry per_push_check_time_max, read from design/budget.yaml as the CTest TIMEOUT reads it
(cmake/marv_test_timeout.cmake). The environment variable MARV_PER_PUSH_CHECK_TIME_MAX_OVERRIDE replaces it and exists
only for the negative control (ci/run_ci.sh, per_push_pytest_control), exactly as the CMake override does.

Each test file is charged the wall time it would take alone in a session:
  - the setup, call and teardown of every one of its tests (the pytest report durations);
  - the collection (import) time of the file;
  - every session- or package-scoped fixture its tests use, in full: such a fixture runs once per session, so its real
    cost is paid by the first file that triggers it, and the plugin removes that actual cost from that file and charges
    the measured cost to every file whose tests use the fixture, since each of them would pay it alone. A parametrised
    shared fixture is charged the sum of its executions.
Not counted: interpreter and pytest start-up, conftest import and the teardown of shared fixtures.
A file over the limit is listed with its time. It fails the session (exit status 1) only when MARV_LOCAL_CI is set to a
non-empty value other than 0 (ci/local_ci.sh sets it in every job container; decision 0019 ruling, S9 (a): enforced in local
CI, the definition of verified, and report-only elsewhere, in particular on GitHub Actions)."""

import os
import time
from collections import defaultdict
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
SHARED_SCOPES = ("session", "package")

_current_file = None
_collect = defaultdict(float)
_phases = defaultdict(float)
_actual_shared = defaultdict(float)
_shared_cost = defaultdict(float)
_uses = defaultdict(set)


def limit_seconds():
    override = os.environ.get("MARV_PER_PUSH_CHECK_TIME_MAX_OVERRIDE", "")
    if override:
        return float(override)
    return float(yaml.safe_load((ROOT / "design" / "budget.yaml").read_text())["per_push_check_time_max"]["value"])


def enforcing():
    return os.environ.get("MARV_LOCAL_CI", "") not in ("", "0")


def _file_of(nodeid):
    return nodeid.split("::", 1)[0]


@pytest.hookimpl(hookwrapper=True)
def pytest_make_collect_report(collector):
    start = time.perf_counter()
    yield
    if isinstance(collector, pytest.Module):
        _collect[_file_of(collector.nodeid)] += time.perf_counter() - start


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_setup(item):
    global _current_file
    _current_file = _file_of(item.nodeid)
    yield


@pytest.hookimpl(hookwrapper=True)
def pytest_fixture_setup(fixturedef, request):
    start = time.perf_counter()
    yield
    if fixturedef.scope in SHARED_SCOPES:
        elapsed = time.perf_counter() - start
        _shared_cost[fixturedef.argname] += elapsed
        _actual_shared[_current_file] += elapsed


def pytest_runtest_logreport(report):
    _phases[_file_of(report.nodeid)] += report.duration


def pytest_itemcollected(item):
    for name, defs in item._fixtureinfo.name2fixturedefs.items():
        if any(d.scope in SHARED_SCOPES for d in defs):
            _uses[_file_of(item.nodeid)].add(name)


def file_times():
    times = {}
    for f in set(_phases) | set(_collect):
        shared = sum(_shared_cost[n] for n in _uses[f])
        times[f] = _collect[f] + _phases[f] - _actual_shared[f] + shared
    return times


def pytest_sessionfinish(session, exitstatus):
    limit = limit_seconds()
    times = file_times()
    over = sorted((f for f, t in times.items() if t > limit), key=lambda f: -times[f])
    reporter = session.config.pluginmanager.get_plugin("terminalreporter")
    write = reporter.write_line if reporter else print
    write("")
    write(f"per-file alone time (limit per_push_check_time_max = {limit:g} s); the five longest:")
    for f in sorted(times, key=lambda f: -times[f])[:5]:
        write(f"  {times[f]:8.1f} s  {f}")
    if over:
        write(f"FILE-TIME-GUARD {'FAILED' if enforcing() else 'REPORT-ONLY (MARV_LOCAL_CI not set, exit status unaffected)'}: files over per_push_check_time_max:")
        for f in over:
            write(f"  {times[f]:8.1f} s > {limit:g} s  {f}")
        if enforcing():
            session.exitstatus = 1
    else:
        write(f"FILE-TIME-GUARD PASSED: {len(times)} files, none over {limit:g} s")
