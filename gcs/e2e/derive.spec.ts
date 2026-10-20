// The end-to-end spec of the Derive panel on the Flash page (GCS-first unit GX-13): the panel lists the helper's cards and the editable entries
// of the chosen one (a locked entry has no input), an edit needs its source text before Derive is on, the lint's own findings of a refused copy
// are shown and nothing runs, a derivation can be cancelled and then runs to its result (config hash, the four derivation reports, the values
// that differ from the library image's factory config), the result is validated for an image of the library (the existing G7 validate: a PASS
// report whose id is checked) and its values are staged on the vehicle, after which the Parameters page lists them. The repository's working
// tree is the same before and after (W-1). Assertions are DOM roles and values.
//
// Light mode only (tools/gs/e2e.sh): the helper's steps run the stand-in Python of tools/gs/e2e_fake_derive.py (the real lint and .mcfg encoder, a
// flatten and parameter generator of a second); a real derivation is not an e2e: it is the evidence of tests/regression/quad/L10/results/gcs/gx13/.
import { execFileSync } from 'node:child_process';
import { existsSync, readdirSync, readFileSync } from 'node:fs';
import { dirname, join, relative } from 'node:path';
import { fileURLToPath } from 'node:url';
import { expect, test, type Locator, type Page } from '@playwright/test';

const url = process.env.MARV_E2E_URL;
const live = process.env.MARV_E2E_MODE === 'live';
const out = process.env.MARV_E2E_OUT;
const runsDir = process.env.MARV_E2E_RUNS_DIR ?? '';
const vehiclePid = Number(process.env.MARV_E2E_VEHICLE_PID ?? '');
const root = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
/** A derivation or a cancelled one is over within this long (a labelled harness value: it bounds a hung run, it decides no result). */
const DONE_MS = 30_000;
const CONNECT_MS = 10_000;
const RETRY_MS = 1_000;
const CARDS = ['vehicles/marv_7in_fc3v2.yaml', 'vehicles/uzh_neurobem_5in.yaml'];
const SOURCE = 'Luis 2026-10-21: scale reading 0.800 kg with the pack and props on (e2e)';
const REPORTS = ['marv_params_fc3v2_attitude_report.txt', 'marv_params_fc3v2_gyro_chain_report.txt', 'marv_params_fc3v2_rate_report.txt', 'marv_params_fc3v2_vertical_report.txt'];

test.skip(live, 'the Derive panel is checked against the stand-in derivation of the light mode');
test.setTimeout(120_000);

test.beforeAll(() => {
	expect(url, 'MARV_E2E_URL is the address the helper prints: run tools/gs/e2e.sh').toBeTruthy();
	expect(runsDir, 'MARV_E2E_RUNS_DIR comes from tools/gs/e2e.sh').toBeTruthy();
});

const bar = (page: Page): Locator => page.getByRole('status', { name: 'Status bar' });
const indicator = (page: Page, name: string): Locator => bar(page).getByRole('group', { name, exact: true });
const nav = (page: Page): Locator => page.getByRole('navigation', { name: 'Pages' });

async function shot(page: Page, name: string): Promise<void> {
	if (out) await page.screenshot({ path: `${out}/derive-${name}.png`, fullPage: true });
}

/** The working tree of the repository: `git status` and the diff, as W-1 compares them (the spec's own output directory, when it lies in the repository, is left out). */
function treeState(): string {
	const inside = out ? relative(root, out) : '';
	const exclude = inside !== '' && !inside.startsWith('..') ? [`:(exclude)${inside}`] : [];
	const status = execFileSync('git', ['-C', root, 'status', '--porcelain=v1', '-uall', '--', '.', ...exclude], { encoding: 'utf8' });
	const diff = execFileSync('git', ['-C', root, 'diff', 'HEAD'], { encoding: 'utf8', maxBuffer: 1 << 28 });
	return `${status}\n${diff.length}\n${execFileSync('sha256sum', { input: diff, encoding: 'utf8' })}`;
}

/** The derivation records of the helper's run (<run dir>/derive/<id>/derive.json), newest last. */
function records(): Array<{ id: string; state: string; ok: boolean | null; config_hash: string | null; edits: Array<{ key: string }>; findings: unknown[] }> {
	const found: ReturnType<typeof records> = [];
	for (const run of existsSync(runsDir) ? readdirSync(runsDir) : []) {
		const dir = join(runsDir, run, 'derive');
		for (const id of existsSync(dir) ? readdirSync(dir).sort() : []) {
			const file = join(dir, id, 'derive.json');
			if (existsSync(file)) found.push(JSON.parse(readFileSync(file, 'utf8')));
		}
	}
	return found;
}

async function openPanel(page: Page): Promise<Locator> {
	await page.goto(url as string);
	await expect(indicator(page, 'Helper')).toContainText(/open/);
	await nav(page).getByRole('link', { name: 'Flash', exact: true }).click();
	await expect(page.getByRole('heading', { level: 1, name: 'Flash', exact: true })).toBeVisible();
	const panel = page.locator('.panel', { has: page.getByRole('heading', { level: 3, name: 'Derive a config', exact: true }) });
	await expect(panel.getByRole('table', { name: 'Card entries' })).toBeVisible();
	return panel;
}

const newValue = (panel: Locator, key: string): Locator => panel.getByRole('textbox', { name: `New value of ${key}`, exact: true });
const sourceOf = (panel: Locator, key: string): Locator => panel.getByRole('textbox', { name: `Source of the new value of ${key}`, exact: true });

test('the panel lists the cards and the editable entries, an edit needs its source, a refused copy shows the lint findings, a derivation can be cancelled and then runs to its result, which is validated for an image of the library; the repository is not written', async ({ page }) => {
	const before = treeState();
	const nrecords = records().length;
	const panel = await openPanel(page);
	const card = panel.getByRole('combobox', { name: 'Card', exact: true });
	const derive = panel.getByRole('button', { name: /^Derive|Deriving/ });
	const cancel = panel.getByRole('button', { name: 'Cancel', exact: true });
	const why = panel.locator('[data-derive="why"]');
	const state = panel.locator('[data-derive="state"]');

	// The cards are the helper's: the files of the vehicles directory; opening the page derived nothing.
	await expect(card.locator('option')).toHaveText(CARDS);
	await expect(card).toHaveValue(CARDS[0] as string);
	await expect(derive).toBeEnabled();
	await expect(cancel).toBeDisabled();
	expect(records().length).toBe(nrecords);
	const entries = panel.getByRole('table', { name: 'Card entries' });
	await expect(entries.locator('tr[data-entry="mass"]')).toContainText('0.732');
	// A locked entry has no input: it changes only by the lock editor.
	const locked = entries.locator('tr[data-entry="rotors.speed_range"]');
	await expect(locked).toContainText('locked');
	await expect(locked.getByRole('textbox')).toHaveCount(0);
	await expect(newValue(panel, 'mass')).toBeEnabled();
	await shot(page, '1-before');

	// An edit needs its source text: without it Derive is off, with the reason beside it.
	await newValue(panel, 'mass').fill('0.8');
	await expect(derive).toBeDisabled();
	await expect(why).toHaveText('mass: the new value needs its source or rationale text');
	await sourceOf(panel, 'mass').fill(SOURCE);
	await expect(derive).toBeEnabled();
	await expect(why).toHaveText('');

	// The lint judges the copy: a motor in the wrong quadrant is refused with the lint's own finding, and nothing runs.
	await newValue(panel, 'rotors.m1.position').fill('-0.1, 0.1, -0.02');
	await sourceOf(panel, 'rotors.m1.position').fill('a caliper reading (e2e: a wrong quadrant on purpose)');
	await derive.click();
	const findings = panel.getByRole('alert', { name: 'Lint findings' });
	await expect(findings).toBeVisible();
	await expect(findings.locator('[data-finding="rotors.m1.position"]')).toContainText('wrong quadrant');
	await expect(findings).toContainText('nothing was derived');
	await expect(state).toContainText('refused');
	await expect(panel.locator('[data-derive="config-hash"]')).toHaveCount(0);
	await expect(panel.getByRole('group', { name: 'Derivation reports' })).toHaveCount(0);
	expect(records().length).toBe(nrecords + 1);
	expect(records().at(-1)).toMatchObject({ state: 'refused', ok: false, config_hash: null });
	await shot(page, '2-lint-findings');

	// The control: with that edit taken out, the same click derives. The cancel stops it; the next click runs to the end.
	await newValue(panel, 'rotors.m1.position').fill('');
	await sourceOf(panel, 'rotors.m1.position').fill('');
	await expect(derive).toBeEnabled();
	await derive.click();
	await expect(state).toContainText('running');
	await expect(cancel).toBeEnabled();
	await shot(page, '3-running');
	await cancel.click();
	await expect(state).toContainText('cancelled', { timeout: DONE_MS });
	await expect(panel.getByRole('group', { name: 'Derivation reports' })).toHaveCount(0);
	await expect(findings).toHaveCount(0);
	await derive.click();
	await expect(state).toContainText('ok', { timeout: DONE_MS });
	await expect(panel.locator('[data-derive="config-hash"]')).toHaveText(/^[0-9a-f]{64}$/);
	const hash = (await panel.locator('[data-derive="config-hash"]').textContent()) as string;
	const last = records().at(-1);
	expect(last).toMatchObject({ state: 'done', ok: true, config_hash: hash, edits: [{ key: 'mass' }] });

	// The reports are the derivation's; the values that differ from the library image's factory config are listed.
	const reports = panel.getByRole('group', { name: 'Derivation reports' });
	await expect(reports.getByRole('button')).toHaveText(REPORTS);
	await reports.getByRole('button', { name: REPORTS[2] as string }).click();
	const text = panel.getByLabel(`Report ${REPORTS[2]}`);
	await expect(text).toContainText('stand-in derivation report');
	await expect(text).toContainText('card vehicles/marv_7in_fc3v2.yaml');
	await expect(text).toContainText('card copy differs from the repository card: True');
	await expect(panel.getByRole('table', { name: 'Changed values' }).locator('tbody tr')).toHaveCount(1);
	await shot(page, '4-result');

	// The hand-over: the existing G7 validation of (image, derived config); the report is fetched and its id checked.
	const image = panel.getByRole('combobox', { name: 'Image', exact: true });
	await expect(image.locator('option')).toHaveCount(2);
	await image.selectOption({ index: 1 });
	await panel.getByRole('button', { name: 'Validate', exact: true }).click();
	await expect(panel.locator('[data-derive="verdict"]')).toContainText('PASS', { timeout: DONE_MS });
	await shot(page, '5-validated');

	// W-1: the working tree is what it was.
	expect(treeState()).toBe(before);
});

test.describe('with the vehicle', () => {
	test.skip(!Number.isInteger(vehiclePid) || vehiclePid <= 0, 'needs the fake vehicle of the light mode (MARV_E2E_VEHICLE_PID)');

	/** The Simulation page with no run going and no device open, then a run launched; the vehicle is connected when this returns. */
	async function launch(page: Page): Promise<void> {
		await page.goto(url as string);
		await expect(indicator(page, 'Helper')).toContainText(/open/);
		await nav(page).getByRole('link', { name: 'Simulation', exact: true }).click();
		const stop = page.getByRole('button', { name: 'Stop' });
		const start = page.getByRole('button', { name: 'Launch' });
		await expect(stop.or(start).and(page.locator(':enabled'))).toBeVisible();
		if (await stop.isEnabled()) {
			await stop.click();
			await expect(start).toBeEnabled({ timeout: DONE_MS });
		}
		await nav(page).getByRole('link', { name: 'Connection', exact: true }).click();
		const close = page.getByRole('button', { name: 'Close device' });
		await expect(async () => {
			if (await close.isEnabled()) await close.click({ timeout: RETRY_MS });
			await expect(page.getByRole('button', { name: 'Open SIL UDP' })).toBeEnabled({ timeout: RETRY_MS });
		}).toPass();
		await nav(page).getByRole('link', { name: 'Simulation', exact: true }).click();
		await expect(page.getByRole('combobox', { name: 'Card' })).toBeEnabled();
		await start.click();
		await expect(indicator(page, 'Link')).toContainText(/alive/, { timeout: CONNECT_MS });
		await expect(indicator(page, 'Identity')).not.toContainText('not reported', { timeout: CONNECT_MS });
	}

	test('the values of a derived config are staged on the vehicle with their source texts, and the Parameters page lists them to be validated and activated there', async ({ page }) => {
		await launch(page);
		await nav(page).getByRole('link', { name: 'Parameters', exact: true }).click();
		await page.getByRole('button', { name: 'Read parameters from the vehicle' }).click();
		await expect(page.getByRole('heading', { name: 'Staged changes (0)' })).toBeVisible({ timeout: DONE_MS });
		await nav(page).getByRole('link', { name: 'Flash', exact: true }).click();
		const panel = page.locator('.panel', { has: page.getByRole('heading', { level: 3, name: 'Derive a config', exact: true }) });
		await newValue(panel, 'mass').fill('0.8');
		await sourceOf(panel, 'mass').fill(SOURCE);
		await panel.getByRole('button', { name: /^Derive/ }).click();
		await expect(panel.locator('[data-derive="state"]')).toContainText('ok', { timeout: DONE_MS });
		const stage = panel.getByRole('button', { name: 'Stage on the vehicle' });
		await expect(stage).toBeEnabled();
		await stage.click();
		await expect(panel.locator('[data-derive="staged"]')).toContainText('staged 1', { timeout: CONNECT_MS });
		await shot(page, '6-staged');
		await nav(page).getByRole('link', { name: 'Parameters', exact: true }).click();
		await expect(page.getByRole('heading', { name: 'Staged changes (1)' })).toBeVisible();
		await expect(page.getByRole('button', { name: 'Validate' })).toBeVisible();
		await expect(page.getByRole('button', { name: 'Activate' })).toBeVisible();
		await shot(page, '7-parameters-staged');
		// Put the vehicle back as it was, through the same panel: the card as it is derives the factory values, which differ from the staged one.
		await nav(page).getByRole('link', { name: 'Flash', exact: true }).click();
		await newValue(panel, 'mass').fill('');
		await sourceOf(panel, 'mass').fill('');
		await panel.getByRole('button', { name: /^Derive/ }).click();
		await expect(panel.locator('[data-derive="state"]')).toContainText('ok', { timeout: DONE_MS });
		await expect(stage).toBeEnabled();
		await stage.click();
		await expect(panel.locator('[data-derive="staged"]')).toContainText('staged 1', { timeout: CONNECT_MS });
		await nav(page).getByRole('link', { name: 'Parameters', exact: true }).click();
		await expect(page.getByRole('heading', { name: 'Staged changes (0)' })).toBeVisible({ timeout: CONNECT_MS });
	});
});
