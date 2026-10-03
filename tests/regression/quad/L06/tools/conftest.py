"""Shared fixtures of the L6 tool tests: rate_lead.design and attitude_lead.design (over the configuration set, on that rate
design) on the committed card run once per pytest session (about 25 s and 35 s on 4 CPUs), however many test files use them."""

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


@pytest.fixture(scope="session")
def attitude_lead_design(rate_lead_design):
    sys.path.insert(0, str(ROOT / "tools" / "card"))
    import attitude_lead
    import rate
    import rate_lead

    card, budget, scenario, _ = rate_lead.load(CARD, BUDGET, SCENARIO, PROFILE)
    l4 = rate.design(card, budget, scenario, CARD)
    return attitude_lead.design(card, budget, scenario, CARD, attitude_lead.lead_inner(rate_lead_design), l4)
