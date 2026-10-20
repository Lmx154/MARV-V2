// The end-to-end spec of the Build panel on the Flash page (GCS-first unit GX-14): the panel lists the helper's allow-listed presets and their
// status (a dry run), a launch from the Simulation page builds nothing and leaves the image hash as it was (H-3), a held heavy lock keeps the Build
// button off with the helper's reason and the same click works when it is free, a build runs under the lock with a progress tail and ends with the
// describe string in the library entry, and Cancel leaves no process and frees the lock (H-2). Assertions are DOM roles and values.
//
// Light mode only (tools/gs/e2e.sh): cmake and ninja are the fakes of tools/gs/e2e_fake_build.py, put first on the helper's PATH; the build tree is
// the light mode's stand-in library. A real build is not an e2e: it is the evidence of tests/regression/quad/L10/results/gcs/gx14/.
import { execFileSync, spawn, type ChildProcess } from 'node:child_process';
import { existsSync, readFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { expect, test, type Locator, type Page } from '@playwright/test';

const url = process.env.MARV_E2E_URL;
const live = process.env.MARV_E2E_MODE === 'live';
const out = process.env.MARV_E2E_OUT;
const lockFile = process.env.MARV_E2E_HEAVY_LOCK ?? '';
const fakeDir = process.env.MARV_E2E_FAKE_BUILD_DIR ?? '';
const root = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
/** A build or a stopped build is over within this long (a labelled harness value: it bounds a hung run, it decides no result). */
const DONE_MS = 30_000;
const PRESETS = ['host-gz-quadfc', 'm33', 'm33-hil'];
/** The helper's texts (tools/gs/build.py). */
const LOCK_HELD = 'the heavy lock is held';
/** The fake ninja's dry run before the first build (tools/gs/e2e_fake_build.py, MARV_FAKE_BUILD_STEPS in tools/gs/e2e.sh). */
const FAKE_STEPS = '8 steps';

test.skip(live, 'the Build panel is checked against the fake cmake and ninja of the light mode');

test.beforeAll(() => {
	expect(url, 'MARV_E2E_URL is the address the helper prints: run tools/gs/e2e.sh').toBeTruthy();
	expect(lockFile && fakeDir, 'MARV_E2E_HEAVY_LOCK and MARV_E2E_FAKE_BUILD_DIR come from tools/gs/e2e.sh light mode').toBeTruthy();
});

const bar = (page: Page): Locator => page.getByRole('status', { name: 'Status bar' });
const indicator = (page: Page, name: string): Locator => bar(page).getByRole('group', { name, exact: true });
const fact = (list: Locator, name: string): Locator => list.locator(`xpath=./dt[normalize-space()="${name}"]/following-sibling::dd[1]`);

async function shot(page: Page, name: string): Promise<void> {
	if (out) await page.screenshot({ path: `${out}/build-${name}.png`, fullPage: true });
}

const processes = (pattern: string): string => {
	try {
		return execFileSync('pgrep', ['-af', pattern], { encoding: 'utf8' }).trim();
	} catch {
		return ''; // pgrep exits 1 when nothing matches
	}
};

/** What the fake cmake and ninja were asked, one argv per line (tools/gs/e2e_fake_build.py). */
const calls = (): string[][] => {
	const file = join(fakeDir, 'argv.jsonl');
	return existsSync(file) ? readFileSync(file, 'utf8').split('\n').filter(Boolean).map((l) => JSON.parse(l) as string[]) : [];
};
const builds = (): string[][] => calls().filter((c) => c[0] === 'cmake');

const describeOfTree = (): string => execFileSync('git', ['-C', root, 'describe', '--dirty', '--always'], { encoding: 'utf8' }).trim();

/** ci/heavy.sh --status: 0 and "free", or 75 and the holder line. */
function lockStatus(): { code: number; text: string } {
	try {
		return { code: 0, text: execFileSync('bash', [join(root, 'ci', 'heavy.sh'), '--status'], { encoding: 'utf8', env: { ...process.env, MARV_HEAVY_LOCK: lockFile } }).trim() };
	} catch (e) {
		const err = e as { status: number; stdout: string };
		return { code: err.status, text: String(err.stdout).trim() };
	}
}

async function openPanel(page: Page): Promise<Locator> {
	await page.goto(url as string);
	await expect(indicator(page, 'Helper')).toContainText(/open/);
	await page.getByRole('navigation', { name: 'Pages' }).getByRole('link', { name: 'Flash', exact: true }).click();
	await expect(page.getByRole('heading', { level: 1, name: 'Flash', exact: true })).toBeVisible();
	await expect(page.getByRole('heading', { level: 3, name: 'Build', exact: true })).toBeVisible();
	return page.locator('.panel', { has: page.getByRole('heading', { level: 3, name: 'Build', exact: true }) });
}

test('the panel lists the allow-listed presets and their status; a launch builds nothing and the image hash stays; a held heavy lock keeps Build off; a build ends with its describe string in the library; Cancel leaves no process', async ({ page }) => {
	const panel = await openPanel(page);
	const preset = panel.getByRole('combobox', { name: 'Preset' });
	const buildButton = panel.getByRole('button', { name: /^Build|Building/ });
	const cancel = panel.getByRole('button', { name: 'Cancel', exact: true });
	const status = panel.locator('[data-build="status"]');
	const why = panel.locator('[data-build="why"]');
	const images = panel.getByRole('table', { name: 'Images of the build tree' });

	// The allow-list is the helper's; the status is the dry run; opening the page built nothing.
	await expect(preset.locator('option')).toHaveText(PRESETS);
	await expect(preset).toHaveValue('host-gz-quadfc');
	await expect(status).toHaveText(FAKE_STEPS);
	await expect(buildButton).toBeEnabled();
	await expect(cancel).toBeDisabled();
	expect(builds()).toEqual([]);
	const row = images.locator('tr[data-image]');
	await expect(row).toHaveCount(1);
	const before = await row.getAttribute('data-image');
	await expect(row).toContainText('not built by the GCS');
	await shot(page, '1-before');

	// H-3: a launch from the Simulation page spawns the run and nothing builds; the image hash is the one it was.
	await page.getByRole('navigation', { name: 'Pages' }).getByRole('link', { name: 'Simulation', exact: true }).click();
	const launch = page.getByRole('button', { name: 'Launch' });
	const stop = page.getByRole('button', { name: 'Stop', exact: true });
	await expect(stop.or(launch).and(page.locator(':enabled'))).toBeVisible();
	if (await stop.isEnabled()) {
		await stop.click();
		await expect(launch).toBeEnabled({ timeout: DONE_MS });
	}
	await launch.click();
	await expect(stop).toBeEnabled({ timeout: DONE_MS });
	await stop.click();
	await expect(launch).toBeEnabled({ timeout: DONE_MS });
	expect(builds()).toEqual([]);
	await page.getByRole('navigation', { name: 'Pages' }).getByRole('link', { name: 'Flash', exact: true }).click();
	await expect(row).toHaveAttribute('data-image', before as string);
	await expect(status).toHaveText(FAKE_STEPS);

	// A held heavy lock: the button is off with the helper's reason; nothing was started.
	const holder: ChildProcess = spawn('bash', [join(root, 'ci', 'heavy.sh'), '--', 'sleep', '600'], {
		env: { ...process.env, MARV_HEAVY_LOCK: lockFile }, detached: true, stdio: 'ignore'
	});
	try {
		await expect.poll(() => lockStatus().code).toBe(75);
		await panel.getByRole('button', { name: 'Check status' }).click();
		await expect(buildButton).toBeDisabled();
		await expect(why).toContainText(LOCK_HELD);
		await expect(why).toContainText('sleep 600');
		await shot(page, '2-lock-held');
		expect(builds()).toEqual([]);
	} finally {
		if (holder.pid) process.kill(-holder.pid, 'SIGKILL');
	}
	// The control: the lock is free and the same click is allowed.
	await expect.poll(() => lockStatus().code).toBe(0);
	await panel.getByRole('button', { name: 'Check status' }).click();
	await expect(buildButton).toBeEnabled();
	await expect(why).toHaveText('');

	// The build: running under the lock, with a progress tail; one at a time.
	await buildButton.click();
	const run = panel.locator('dl[aria-label="Build"]');
	await expect(fact(run, 'state')).toContainText('running');
	await expect(buildButton).toBeDisabled();
	await expect(cancel).toBeEnabled();
	await expect(preset).toBeDisabled();
	await expect(panel.getByRole('list', { name: 'Build progress' }).getByRole('listitem').filter({ hasText: 'Building CXX object' }).first()).toBeVisible();
	expect(lockStatus().code).toBe(75);
	expect(lockStatus().text).toContain('cmd=GCS build host-gz-quadfc');
	await expect(fact(run, 'describe')).toHaveText(describeOfTree());
	await shot(page, '3-running');
	await expect(fact(run, 'state')).toContainText('ok', { timeout: DONE_MS });
	await expect(fact(run, 'step')).toHaveText('8/8');
	expect(lockStatus()).toEqual({ code: 0, text: 'free' });
	await expect(status).toHaveText('up to date');

	// The library entry: a new image, with the describe string of the tree the build started from.
	await expect(row).toHaveCount(1);
	const after = await row.getAttribute('data-image');
	expect(after).not.toBe(before);
	await expect(row).toContainText(describeOfTree());
	await expect(row).not.toContainText('not built by the GCS');
	await shot(page, '4-done');
	expect(builds().map((c) => c.slice(0, 3))).toEqual([['cmake', '--preset', 'host-gz-quadfc'], ['cmake', '--build', expect.any(String)]]);

	// Cancel: the processes are gone and the lock is free.
	await preset.selectOption('m33');
	await expect(status).toHaveText('not configured');
	await buildButton.click();
	await expect(fact(run, 'preset')).toHaveText('m33');
	await expect(fact(run, 'state')).toContainText('running');
	await expect(panel.getByRole('list', { name: 'Build progress' }).getByRole('listitem').first()).toBeVisible();
	await expect.poll(() => processes('e2e_fake_build.py cmake --build')).not.toBe('');
	await cancel.click();
	await expect(fact(run, 'state')).toContainText('cancelled', { timeout: DONE_MS });
	await expect.poll(() => processes('e2e_fake_build.py cmake --build'), { timeout: DONE_MS }).toBe('');
	expect(lockStatus()).toEqual({ code: 0, text: 'free' });
	await expect(buildButton).toBeEnabled();
	await expect(cancel).toBeDisabled();
	await shot(page, '5-cancelled');
});
