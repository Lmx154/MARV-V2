// The end-to-end spec of the Bench page (GCS-first unit GX-10): the HIL preflight lists each refusal by name, the S4b launch starts only
// when the helper's checks all hold, stop leaves no process, the verdict form saves its mode and text next to the run's report, and the
// SWD flash refuses a motor-capable bench image while the hil kind flashes. Assertions are DOM roles and values.
//
// Light mode only (tools/gs/e2e.sh): the probe, the board running the HIL image, its port and flash.sh are the fakes of
// tools/gs/e2e_fake_bench.py and the HIL run is the stand-in command `sleep`. No port of the host is opened. The live run of S4b
// with Luis flying is not an e2e: it is the runbook of tests/regression/quad/L10/results/gcs/gx10/RUNBOOK.md.
import { execFileSync } from 'node:child_process';
import { existsSync, mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { expect, test, type Locator, type Page } from '@playwright/test';

const url = process.env.MARV_E2E_URL;
const live = process.env.MARV_E2E_MODE === 'live';
const out = process.env.MARV_E2E_OUT;
const runsDir = process.env.MARV_E2E_RUNS_DIR ?? '';
/** The seconds of the stand-in `sleep` of the HIL run (tools/gs/e2e.sh HIL_STAND_IN_SECONDS), which names its process. */
const standIn = process.env.MARV_E2E_HIL_STAND_IN ?? '';
const flashRecord = process.env.MARV_E2E_FLASH_RECORD ?? '';
/** A stopped run's processes are gone within this long of the page showing it stopped (a labelled harness value). */
const GONE_MS = 15_000;
/** The fake bench's names (tools/gs/e2e_fake_bench.py). */
const PORT = /usb-Raspberry_Pi_Pico_E2EBOARD0001-if00/;
/** The helper's texts (tools/gs/hil.py). */
const NO_ACK = 'the ESC battery acknowledgement is not ticked: unplug the ESC battery, then tick it';
const NOT_SUSPENDED = "autoconnect is not suspended: it would open the board's ports again (take the hold)";
const MOTOR = "only after its props-off TB on the exact image and Luis's presence acknowledgement";
const CHECKS = ['probe', 'board', 'hil_image', 'esc_battery', 'host_idle', 'ports'];
const VERDICT_TEXT = 'Took off in ANGLE, held a hover for a minute, landed. Stick feel as expected.';

test.skip(live, 'the Bench page is checked against the fakes of the light mode; the live S4b run is the runbook');

test.beforeAll(() => {
	expect(url, 'MARV_E2E_URL is the address the helper prints: run tools/gs/e2e.sh').toBeTruthy();
});

const bar = (page: Page): Locator => page.getByRole('status', { name: 'Status bar' });
const indicator = (page: Page, name: string): Locator => bar(page).getByRole('group', { name, exact: true });
const check = (page: Page, id: string): Locator => page.getByRole('list', { name: 'Preflight checks' }).locator(`li[data-check="${id}"]`);
const fact = (list: Locator, name: string): Locator => list.locator(`xpath=./dt[normalize-space()="${name}"]/following-sibling::dd[1]`);

async function shot(page: Page, name: string): Promise<void> {
	if (out) await page.screenshot({ path: `${out}/bench-${name}.png`, fullPage: true });
}

const processes = (pattern: string): string => {
	try {
		return execFileSync('pgrep', ['-af', pattern], { encoding: 'utf8' }).trim();
	} catch {
		return ''; // pgrep exits 1 when nothing matches
	}
};

/** The verdict files the helper wrote under its run directory (<run dir>/runs/<run id>/out/verdict.json). */
function verdictFiles(): string[] {
	const found: string[] = [];
	const walk = (dir: string, depth: number): void => {
		for (const e of readdirSync(dir, { withFileTypes: true })) {
			if (e.isDirectory() && depth < 5) walk(join(dir, e.name), depth + 1);
			else if (e.name === 'verdict.json') found.push(join(dir, e.name));
		}
	};
	if (runsDir && existsSync(runsDir)) walk(runsDir, 0);
	return found;
}

test('the preflight names each refusal, the launch starts when every check holds, stop leaves no process, the verdict is saved beside the report', async ({ page }) => {
	await page.goto(url as string);
	await expect(indicator(page, 'Helper')).toContainText(/open/);
	await page.getByRole('navigation', { name: 'Pages' }).getByRole('link', { name: 'Bench', exact: true }).click();
	await expect(page.getByRole('heading', { level: 1, name: 'Bench', exact: true })).toBeVisible();
	const preflight = page.getByRole('button', { name: 'Preflight' });
	const launch = page.getByRole('button', { name: 'Launch S4b' });
	const stop = page.getByRole('button', { name: 'Stop', exact: true });
	const ack = page.getByRole('checkbox', { name: 'ESC battery unplugged' });
	await expect(preflight).toBeEnabled();
	await expect(stop).toBeDisabled();

	// Refused: the acknowledgement is not ticked and autoconnect is not suspended; the other four checks hold. Each reason is named.
	await expect(ack).not.toBeChecked();
	await preflight.click();
	const list = page.getByRole('list', { name: 'Preflight checks' });
	await expect(list.locator('li')).toHaveCount(CHECKS.length);
	for (const id of CHECKS) await expect(check(page, id)).toHaveAttribute('data-state', ['esc_battery', 'ports'].includes(id) ? 'refused' : 'ok');
	await expect(check(page, 'esc_battery')).toContainText(NO_ACK);
	await expect(check(page, 'ports')).toContainText(NOT_SUSPENDED);
	await expect(page.locator('[data-preflight]')).toHaveAttribute('data-preflight', 'refused');
	await shot(page, '1-refused');

	// Launching the same form is refused with the same reasons and starts nothing.
	await launch.click();
	await expect(page.getByRole('alert').filter({ hasText: NO_ACK })).toBeVisible();
	await expect(page.getByRole('alert').filter({ hasText: NOT_SUSPENDED })).toBeVisible();
	await expect(stop).toBeDisabled();
	expect(processes(`^sleep ${standIn}`)).toBe('');

	// The control: the hold and the acknowledgement satisfy the two conditions, and all six checks hold.
	await page.getByRole('button', { name: 'Hold the board' }).click();
	await expect(page.getByRole('button', { name: 'Release the hold' })).toBeVisible();
	await ack.check();
	await preflight.click();
	for (const id of CHECKS) await expect(check(page, id)).toHaveAttribute('data-state', 'ok');
	await expect(page.locator('[data-preflight]')).toHaveAttribute('data-preflight', 'allowed');
	await expect(check(page, 'hil_image')).toContainText(PORT);
	await shot(page, '2-allowed');

	// Launch: a HIL run is going (the stand-in), the launch button is off, the status bar shows the run.
	await launch.click();
	const run = page.locator('dl[aria-label="HIL run"]');
	await expect(fact(run, 'state')).toContainText('running');
	await expect(fact(run, 'port')).toContainText(PORT);
	await expect(fact(run, 'seconds')).toHaveText('300');
	await expect(indicator(page, 'Run')).toContainText(/hil running/);
	await expect(launch).toBeDisabled();
	await expect(stop).toBeEnabled();
	expect(processes(`^sleep ${standIn}`)).not.toBe('');
	// The verdict is for after the flight.
	await expect(page.getByRole('textbox', { name: 'Verdict' })).toBeDisabled();
	await expect(page.getByRole('button', { name: 'Save verdict' })).toBeDisabled();
	await shot(page, '3-running');

	// Stop: no process of the run is left (H-2) and the hold has ended with the run.
	await stop.click();
	await expect(fact(run, 'state')).toContainText('stopped', { timeout: GONE_MS });
	await expect(indicator(page, 'Run')).toContainText(/hil stopped/);
	await expect(async () => {
		expect(processes(`^sleep ${standIn}`)).toBe('');
	}).toPass({ timeout: GONE_MS });
	await expect(page.getByRole('button', { name: 'Hold the board' })).toBeVisible({ timeout: GONE_MS });

	// The verdict form: the mode is required; the text and the mode land in the run directory next to the report.
	const outDir = (await run.locator('[data-run="out-dir"]').innerText()).trim();
	expect(outDir).toContain(runsDir);
	mkdirSync(outDir, { recursive: true });
	writeFileSync(join(outDir, 'marv_e2e_hil_n25.hil_report.txt'), 'the report of the stand-in run\n');
	const save = page.getByRole('button', { name: 'Save verdict' });
	const text = page.getByRole('textbox', { name: 'Verdict' });
	await expect(save).toBeEnabled();
	await text.fill(VERDICT_TEXT);
	await save.click();
	await expect(page.getByRole('alert').filter({ hasText: 'choose the flight mode flown: ANGLE or ACRO' })).toBeVisible();
	expect(verdictFiles()).toEqual([]);
	await page.getByRole('radio', { name: 'ANGLE' }).check();
	await expect(page.getByRole('alert').filter({ hasText: 'choose the flight mode' })).toHaveCount(0);
	await save.click();
	const saved = page.locator('dl[aria-label="Saved verdict"]');
	await expect(fact(saved, 'mode')).toHaveText('ANGLE');
	await expect(fact(saved, 'text')).toHaveText(VERDICT_TEXT);
	await expect(fact(saved, 'report')).toHaveText('marv_e2e_hil_n25.hil_report.txt');
	await expect(save).toBeDisabled(); // written once
	const files = verdictFiles();
	expect(files).toEqual([join(outDir, 'verdict.json')]);
	const record = JSON.parse(readFileSync(files[0] as string, 'utf8')) as { mode: string; text: string; report: string };
	expect(record.mode).toBe('ANGLE');
	expect(record.text).toBe(VERDICT_TEXT);
	expect(record.report).toBe('marv_e2e_hil_n25.hil_report.txt');
	await shot(page, '4-verdict');
});

test('a motor-capable bench image is refused naming the condition, the hil image flashes over SWD', async ({ page }) => {
	await page.goto(url as string);
	await expect(indicator(page, 'Helper')).toContainText(/open/);
	await page.getByRole('navigation', { name: 'Pages' }).getByRole('link', { name: 'Bench', exact: true }).click();
	const kind = page.getByRole('combobox', { name: 'Image kind' });
	const flash = page.getByRole('button', { name: 'Flash over SWD' });
	await expect(flash).toBeEnabled();
	for (const k of ['bench-profile', 'bench-log']) {
		await kind.selectOption(k);
		await flash.click();
		await expect(page.getByRole('alert').filter({ hasText: MOTOR })).toContainText(k);
		await expect(page.locator('dl[aria-label="SWD flash"]')).toHaveCount(0);
	}
	expect(existsSync(flashRecord)).toBe(false); // the script never ran
	await shot(page, '5-motor-refused');

	// The control: the hil kind flashes.
	await kind.selectOption('hil');
	await expect(page.getByRole('alert').filter({ hasText: MOTOR })).toHaveCount(0);
	await flash.click();
	const swd = page.locator('dl[aria-label="SWD flash"]');
	await expect(fact(swd, 'state')).toContainText('done');
	await expect(fact(swd, 'kind')).toHaveText('hil');
	await expect(fact(swd, 'sha256')).toHaveText(/^[0-9a-f]{64}$/);
	await expect(page.getByLabel('Flash output')).toContainText('** Verified OK **');
	expect(readFileSync(flashRecord, 'utf8').trim()).toMatch(/m33-hil\/fw\/hal\/target\/marv_fc3v2\.elf$/);
	await shot(page, '6-flashed');
});
