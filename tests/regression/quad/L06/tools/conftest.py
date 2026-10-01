"""Shared fixtures of the L6 tool tests: rate_lead.design on the committed card runs once per pytest session (about 45 s of
plain Python), however many test files use it."""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[5]
CARD = ROOT / "vehicles" / "uzh_neurobem_5in.yaml"
BUDGET = ROOT / "design" / "budget.yaml"
SCENARIO = ROOT / "design" / "scenario_values.yaml"
PROFILE = ROOT / "sensors" / "profiles" / "marv_v2_board_default.yaml"


@pytest.fixture(scope="session")
def rate_lead_design():
    sys.path.insert(0, str(ROOT / "tools" / "card"))
    import rate_lead

    card, budget, scenario, profile = rate_lead.load(CARD, BUDGET, SCENARIO, PROFILE)
    return rate_lead.design(card, budget, scenario, profile, CARD, PROFILE)
