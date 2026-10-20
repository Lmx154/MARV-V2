// The end-to-end specs of the Simulation page (GCS-first unit GX-3): the page lists the vehicle cards and the firmware parameters,
// plans a run with one override and labels it sil-override, shows the helper's refusal of an unknown parameter next to its row, and
// launches and stops a run from the page, the helper's autoconnect connecting the vehicle (the page opens no link of its own).
// Assertions are DOM roles and values.
//
// Two modes, set by tools/gs/e2e.sh. Light: the helper's run is the stand-in command `sleep` and the fake vehicle already holds the
// run's UDP port; run_start and run_stop are the helper's own, only the simulator is replaced. Live (MARV_E2E_MODE=live): the helper
// starts the Gazebo SIL (GUI off) and the spec checks that stop leaves no `gz sim` or `run_quad_fc` process.
import { execFileSync } from 'node:child_process';
import { expect, test, type Locator, type Page } from '@playwright/test';

const url = process.env.MARV_E2E_URL;
const live = process.env.MARV_E2E_MODE === 'live';
const out = process.env.MARV_E2E_OUT;
/** Light: the seconds of the stand-in `sleep` (tools/gs/e2e.sh STAND_IN_SECONDS), which names its process. */
const standIn = process.env.MARV_E2E_STAND_IN ?? '';
/** Live: how long a simulator run may take to bind its socket and answer (a labelled harness value, bounds a hung run only). */
const LIVE_START_MS = 180_000;
/** Light: connected within this long after "Launch" (a labelled harness value). */
const CONNECT_MS = 10_000;
/** A stopped run's processes are gone within this long of the page showing it stopped (a labelled harness value). */
const GONE_MS = 15_000;
const RETRY_MS = 1_000;
/** The label of an override and of a run that has one (tools/sim/sil_run.py LABEL). */
const LABEL = 'sil-override';
const CARDS = ['vehicles/marv_7in_fc3v2.yaml', 'vehicles/uzh_neurobem_5in.yaml'];
/** A parameter of the SIL's parameter set (the stand-in manifest of the light mode has it too) and a value for it (a labelled scenario value). */
const PARAM = 'att_kp';
const VALUE = '2.5';
/** The helper's text for an unknown override name (tools/sim/sil_run.py check_overrides). */
const UNKNOWN = "override 'no_such_parameter': not a parameter of the image's parameter set";

test.beforeAll(() => {
	expect(url, 'MARV_E2E_URL is the address the helper prints: run tools/gs/e2e.sh').toBeTruthy();
});

const bar = (page: Page): Locator => page.getByRole('status', { name: 'Status bar' });
const indicator = (page: Page, name: string): Locator => bar(page).getByRole('group', { name, exact: true });

async function shot(page: Page, name: string): Promise<void> {
	if (out) await page.screenshot({ path: `${out}/simulation-${name}.png`, fullPage: true });
}

/** The Simulation page with no run going and no device open (an earlier spec may have left either). */
async function openPage(page: Page): Promise<void> {
	await page.goto(url as string);
	await expect(indicator(page, 'Helper')).toContainText(/open/);
	const nav = page.getByRole('navigation', { name: 'Pages' });
	await nav.getByRole('link', { name: 'Simulation', exact: true }).click();
	await expect(page.getByRole('heading', { level: 1, name: 'Simulation', exact: true })).toBeVisible();
	// The page learns the helper's status a moment after it connects: wait for a state in which one of the two buttons is enabled.
	const stop = page.getByRole('button', { name: 'Stop' });
	const launch = page.getByRole('button', { name: 'Launch' });
	await expect(stop.or(launch).and(page.locator(':enabled'))).toBeVisible();
	if (await stop.isEnabled()) {
		await stop.click();
		await expect(launch).toBeEnabled({ timeout: GONE_MS });
	}
	await nav.getByRole('link', { name: 'Connection', exact: true }).click();
	const close = page.getByRole('button', { name: 'Close device' });
	await expect(async () => {
		if (await close.isEnabled()) await close.click({ timeout: RETRY_MS });
		await expect(page.getByRole('button', { name: 'Open SIL UDP' })).toBeEnabled({ timeout: RETRY_MS });
	}).toPass();
	await expect(indicator(page, 'Link')).toContainText(/device\s+none/);
	await nav.getByRole('link', { name: 'Simulation', exact: true }).click();
	await expect(page.getByRole('combobox', { name: 'Card' })).toBeEnabled();
}

/** The value of a fact in a definition list of the page (the term and its definition sit in separate boxes: no text runs between them). */
const fact = (list: Locator, name: string): Locator => list.locator(`xpath=./dt[normalize-space()="${name}"]/following-sibling::dd[1]`);

async function addOverride(page: Page, n: number, name: string, value: string): Promise<void> {
	await page.getByRole('button', { name: 'Add override' }).click();
	await page.getByLabel(`Override ${n} parameter`).fill(name);
	await page.getByLabel(`Override ${n} value`).fill(value);
}

const processes = (pattern: string): string => {
	try {
		return execFileSync('pgrep', ['-af', pattern], { encoding: 'utf8' }).trim();
	} catch {
		return ''; // pgrep exits 1 when nothing matches
	}
};

test('lists the vehicle cards and the firmware parameters', async ({ page }) => {
	await openPage(page);
	const options = await page.getByRole('combobox', { name: 'Card' }).locator('option').evaluateAll((os) => os.map((o) => (o as HTMLOptionElement).value));
	for (const card of CARDS) expect(options).toContain(card);
	// The names come from the image library's manifest, read a moment after the page opens.
	const names = (): Promise<string[]> => page.locator('#sim-parameters option').evaluateAll((os) => os.map((o) => (o as HTMLOptionElement).value));
	await expect.poll(names, { timeout: LIVE_START_MS }).toContain(PARAM);
	expect((await names()).length).toBeGreaterThan(1);
});

test('a plan with one override shows the sil-override label on the row and on the plan; without an override it shows none', async ({ page }) => {
	await openPage(page);
	await page.getByRole('combobox', { name: 'Card' }).selectOption(CARDS[1] as string);
	await addOverride(page, 1, PARAM, VALUE);
	const row = page.getByRole('list', { name: 'Parameter overrides' }).getByRole('listitem');
	await expect(row).toContainText(LABEL);
	await expect(page.getByRole('button', { name: 'Launch' })).toBeEnabled();

	await page.getByRole('button', { name: 'Plan' }).click();
	const plan = page.locator('dl[aria-label="Plan"]');
	await expect(fact(plan, 'label')).toHaveText(LABEL);
	await expect(fact(plan, 'allowed')).toHaveText('yes');
	await expect(plan.locator('[data-plan="argv"]')).not.toBeEmpty();
	await expect(page.getByRole('table', { name: 'Planned overrides' })).toContainText(PARAM);
	await expect(page.getByRole('table', { name: 'Planned overrides' })).toContainText(LABEL);
	await shot(page, '0-plan');

	// The control: the same plan without the override carries no label.
	await page.getByRole('button', { name: 'Remove override 1' }).click();
	await page.getByRole('button', { name: 'Plan' }).click();
	await expect(fact(page.locator('dl[aria-label="Plan"]'), 'label')).toHaveText('none: no override');
	await expect(page.getByRole('table', { name: 'Planned overrides' })).toHaveCount(0);
});

test('the helper\'s refusal of an unknown parameter is shown verbatim next to that row', async ({ page }) => {
	await openPage(page);
	await addOverride(page, 1, PARAM, VALUE);
	await addOverride(page, 2, 'no_such_parameter', '1');
	await page.getByRole('button', { name: 'Plan' }).click();
	const rows = page.getByRole('list', { name: 'Parameter overrides' }).getByRole('listitem');
	await expect(rows.nth(1).getByRole('alert')).toHaveText(UNKNOWN);
	await expect(rows.nth(0).getByRole('alert')).toHaveCount(0);
	await expect(page.locator('dl[aria-label="Plan"]')).toHaveCount(0);
	// Launching it is refused the same way and starts nothing.
	await page.getByRole('button', { name: 'Launch' }).click();
	await expect(rows.nth(1).getByRole('alert')).toHaveText(UNKNOWN);
	await expect(page.getByRole('button', { name: 'Stop' })).toBeDisabled();
	await shot(page, '0-refused');

	// The control: the message is about the form as it was; editing the row clears it.
	await page.getByLabel('Override 2 parameter').fill(PARAM + '_x');
	await expect(rows.nth(1).getByRole('alert')).toHaveCount(0);
});

test('launches a run with one override and the GUI off, the vehicle connects by autoconnect, stop leaves no processes', async ({ page }) => {
	await openPage(page);
	// Before: no device, nothing heard, no run.
	await expect(indicator(page, 'Link')).toContainText(/device\s+none/);
	await expect(indicator(page, 'Link')).toHaveAttribute('data-stale', 'true');
	await expect(page.getByLabel(/^GUI/)).not.toBeChecked();
	await addOverride(page, 1, PARAM, VALUE);
	await expect(page.getByRole('button', { name: 'Stop' })).toBeDisabled();
	await shot(page, '1-before');

	await page.getByRole('button', { name: 'Launch' }).click();

	const run = page.locator('dl[aria-label="Run"]');
	await expect(fact(run, 'state')).toContainText('running', { timeout: CONNECT_MS });
	await expect(fact(run, 'state')).toContainText(LABEL);
	await expect(fact(run, 'gui')).toHaveText('off');
	const runRow = page.getByRole('table', { name: 'Run overrides' }).getByRole('row', { name: new RegExp(PARAM) });
	await expect(runRow).toContainText(VALUE);
	await expect(runRow).toContainText(LABEL);
	await expect(page.getByRole('list', { name: 'Parameter overrides' }).getByRole('listitem')).toContainText(LABEL);
	await expect(indicator(page, 'Run')).toContainText(/sil running/);
	await expect(page.getByRole('button', { name: 'Launch' })).toBeDisabled();
	await expect(page.getByRole('button', { name: 'Stop' })).toBeEnabled();

	// The page opened no link: the helper's autoconnect did, and the status bar's link indicator went fresh.
	const timeout = live ? LIVE_START_MS : CONNECT_MS;
	await expect(indicator(page, 'Link')).toContainText(/device\s+udp 127\.0\.0\.1:\d+/, { timeout });
	await expect(indicator(page, 'Link')).toContainText(/alive/, { timeout });
	await expect(indicator(page, 'Link')).toHaveAttribute('data-stale', 'false', { timeout });
	await expect(indicator(page, 'Identity')).not.toContainText('not reported', { timeout });
	if (!live) expect(processes(`^sleep ${standIn}`)).not.toBe('');
	await shot(page, '2-running');

	await page.getByRole('button', { name: 'Stop' }).click();

	await expect(fact(run, 'state')).toContainText('stopped', { timeout: GONE_MS });
	await expect(fact(run, 'state')).toContainText(LABEL);
	await expect(page.getByRole('button', { name: 'Launch' })).toBeEnabled();
	await expect(page.getByRole('button', { name: 'Stop' })).toBeDisabled();
	await expect(indicator(page, 'Run')).toContainText(/sil stopped/);
	await expect(async () => {
		expect(processes(live ? 'gz sim|run_quad_fc' : `^sleep ${standIn}`)).toBe('');
	}).toPass({ timeout: GONE_MS });
	await shot(page, '3-after-stop');
});
