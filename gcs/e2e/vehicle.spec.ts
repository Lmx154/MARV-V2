// The end-to-end specs of the Vehicle page (GCS-first unit GX-5): the page asks the vehicle for its streams, every panel goes live and
// shows a value, leaving the page stops the streams, and stopping the vehicle turns every panel stale (the control) while continuing it
// turns them live again. Assertions are DOM roles and values.
//
// Two modes, set by tools/gs/e2e.sh. Light: the vehicle is the fake of tools/gs/e2e_fake_vehicle.py (it streams on request, a slow
// harness motion); the staleness control stops its process group. Live (MARV_E2E_MODE=live): the helper starts the Gazebo SIL (GUI off)
// and the control stops the processes of that run (those with the helper's run directory in their command line).
import { execFileSync } from 'node:child_process';
import { expect, test, type Locator, type Page } from '@playwright/test';

const url = process.env.MARV_E2E_URL;
const live = process.env.MARV_E2E_MODE === 'live';
const out = process.env.MARV_E2E_OUT;
const vehiclePid = Number(process.env.MARV_E2E_VEHICLE_PID ?? '');
const runsDir = process.env.MARV_E2E_RUNS_DIR ?? '';
/** Live: how long a simulator run may take to bind its socket and answer (a labelled harness value, bounds a hung run only). */
const LIVE_START_MS = 180_000;
/** Light: connected within this long after "Launch" (a labelled harness value). */
const CONNECT_MS = 10_000;
/** A panel goes stale within the shell's vehicle timeout (3 s, shell.ts SHELL_TIMING), one tick and slack (a labelled harness value). */
const STALE_FLIP_MS = 8_000;
/** Every panel is live within this long of the page being opened with the vehicle connected (the first messages of all nine streams; a labelled harness value). */
const LIVE_PANELS_MS = 10_000;
/** A stopped run's processes are gone within this long (a labelled harness value). */
const GONE_MS = 15_000;
const RETRY_MS = 1_000;
/** The window over which the binary frames from the helper (the vehicle's bytes) are counted, and the counts that separate "streaming" from "heartbeats only" (labelled harness values). */
const COUNT_WINDOW_MS = 2_000;
const STREAMING_MIN_FRAMES = 20;
const SILENT_MAX_FRAMES = 6;
/** The panels of the page (gcs/src/lib/state/live_state.ts PANELS): title and the id of its data-panel attribute. */
const PANELS: Array<[string, string]> = [
	['Attitude', 'attitude'],
	['Rates and setpoint', 'rates'],
	['Radio', 'rc'],
	['Motors', 'motors'],
	['Arming', 'arming'],
	['Estimators', 'estimators'],
	['Flags and faults', 'faults'],
	['IMU', 'imu']
];

test.beforeAll(() => {
	expect(url, 'MARV_E2E_URL is the address the helper prints: run tools/gs/e2e.sh').toBeTruthy();
});

const bar = (page: Page): Locator => page.getByRole('status', { name: 'Status bar' });
const indicator = (page: Page, name: string): Locator => bar(page).getByRole('group', { name, exact: true });
/** A panel of the page (the main area: the status bar has an 'Arming' group of its own). */
const panel = (page: Page, id: string): Locator => page.getByRole('main').locator(`[data-panel="${id}"]`);
const row = (page: Page, panelId: string, key: string): Locator => panel(page, panelId).locator(`[data-row="${panelId}.${key}"]`);

async function shot(page: Page, name: string): Promise<void> {
	if (out) await page.screenshot({ path: `${out}/vehicle-${name}.png`, fullPage: true });
}

const number = async (locator: Locator): Promise<number> => Number.parseFloat(((await locator.innerText()) ?? '').trim());

/** Counts the binary frames (link bytes) the helper sends to the page; text frames are the helper's own events. */
function countFrames(page: Page): { count: () => number } {
	let n = 0;
	page.on('websocket', (ws) => ws.on('framereceived', (f) => void (typeof f.payload !== 'string' && n++)));
	return { count: () => n };
}

/** The processes of the helper's run (not the helper): the stand-in of the light mode is not among them, the simulator of the live mode is. */
function runPids(): number[] {
	if (!runsDir) return [];
	try {
		return execFileSync('pgrep', ['-af', runsDir], { encoding: 'utf8' })
			.split('\n')
			.filter((l) => l.trim() !== '' && !l.includes('helper.py') && !l.includes('pgrep'))
			.map((l) => Number.parseInt(l, 10));
	} catch {
		return []; // pgrep exits 1 when nothing matches
	}
}

function signalVehicle(signal: 'SIGSTOP' | 'SIGCONT'): void {
	if (live) {
		for (const pid of runPids()) {
			try {
				process.kill(pid, signal);
			} catch {
				// gone
			}
		}
	} else {
		process.kill(-vehiclePid, signal);
	}
}

/** The Simulation page with no run going and no device open, then a run launched; the vehicle is connected when this returns. */
async function launch(page: Page): Promise<void> {
	await page.goto(url as string);
	await expect(indicator(page, 'Helper')).toContainText(/open/);
	const nav = page.getByRole('navigation', { name: 'Pages' });
	await nav.getByRole('link', { name: 'Simulation', exact: true }).click();
	await expect(page.getByRole('heading', { level: 1, name: 'Simulation', exact: true })).toBeVisible();
	const stop = page.getByRole('button', { name: 'Stop' });
	const start = page.getByRole('button', { name: 'Launch' });
	await expect(stop.or(start).and(page.locator(':enabled'))).toBeVisible();
	if (await stop.isEnabled()) {
		await stop.click();
		await expect(start).toBeEnabled({ timeout: GONE_MS });
	}
	await nav.getByRole('link', { name: 'Connection', exact: true }).click();
	const close = page.getByRole('button', { name: 'Close device' });
	await expect(async () => {
		if (await close.isEnabled()) await close.click();
		await expect(page.getByRole('button', { name: 'Open SIL UDP' })).toBeEnabled({ timeout: RETRY_MS });
	}).toPass();
	await nav.getByRole('link', { name: 'Simulation', exact: true }).click();
	await expect(page.getByRole('combobox', { name: 'Card' })).toBeEnabled();
	await start.click();
	const timeout = live ? LIVE_START_MS : CONNECT_MS;
	await expect(indicator(page, 'Link')).toContainText(/alive/, { timeout });
	await expect(indicator(page, 'Identity')).not.toContainText('not reported', { timeout });
}

async function openVehicle(page: Page): Promise<void> {
	await page.getByRole('navigation', { name: 'Pages' }).getByRole('link', { name: 'Vehicle', exact: true }).click();
	await expect(page.getByRole('heading', { level: 1, name: 'Vehicle', exact: true })).toBeVisible();
}

async function expectAll(page: Page, stale: boolean, timeout: number): Promise<void> {
	for (const [title, id] of PANELS) {
		const p = panel(page, id);
		await expect(p, title).toHaveAttribute('data-stale', String(stale), { timeout });
		await expect(page.getByRole('main').getByRole('group', { name: title, exact: true })).toHaveCount(1);
	}
}

/** Never leave the vehicle stopped: the next test and the cleanup of tools/gs/e2e.sh need it running. */
test.afterEach(() => {
	try {
		signalVehicle('SIGCONT');
	} catch {
		// the group is gone
	}
});

test.describe('the vehicle page', () => {
	test.skip(!live && (!Number.isInteger(vehiclePid) || vehiclePid <= 0), 'needs the fake vehicle of the light mode (MARV_E2E_VEHICLE_PID)');
	test.setTimeout(live ? LIVE_START_MS + 120_000 : 90_000);

	test('every panel goes live with a value, leaving the page stops the streams, stopping the vehicle turns every panel stale', async ({ page }) => {
		const frames = countFrames(page);
		await launch(page);
		const window = async (): Promise<number> => {
			const before = frames.count();
			await page.waitForTimeout(COUNT_WINDOW_MS);
			return frames.count() - before;
		};
		// Before the page is opened the vehicle sends only its heartbeat (and the answers to the identity requests).
		expect(await window()).toBeLessThanOrEqual(SILENT_MAX_FRAMES);

		await openVehicle(page);
		await expectAll(page, false, LIVE_PANELS_MS);
		for (const [, id] of PANELS) await expect(panel(page, id)).toHaveAttribute('data-heard', 'true');
		await expect(page.locator('[data-summary]')).toContainText('every panel is live');
		// Each panel shows a value of its stream, with its unit.
		await expect(row(page, 'attitude', 'use.roll')).toContainText(/^-?\d+\.\d\s*°\s*$/);
		await expect(row(page, 'attitude', 'mahony.roll')).toContainText(/^-?\d+\.\d\s*°\s*$/);
		await expect(row(page, 'rates', 'gyro.x')).toContainText(/^-?\d+\.\d\s*°\/s\s*$/);
		await expect(row(page, 'rates', 'thrust')).toContainText(/\d+\s*% of the largest/);
		await expect(row(page, 'rc', 'quality')).toBeVisible();
		await expect(row(page, 'motors', 'm.1')).toContainText(/^\d+\s*DShot\s*$/);
		await expect(row(page, 'arming', 'state')).toHaveText(/\w+/);
		await expect(row(page, 'estimators', 'mahony.valid')).toHaveText(/^\s*(yes|no)\s*$/);
		await expect(row(page, 'faults', 'deadline')).toContainText(/^\s*\d+\s*$/);
		await expect(row(page, 'imu', 'temp')).toContainText(/^-?\d+\.\d\s*°C\s*$/);
		const az = await number(row(page, 'imu', 'accel.z'));
		expect(az, 'the accelerometer at rest reads about -g on z (FRD)').toBeLessThan(-7);
		expect(az).toBeGreaterThan(-12);
		// The page asked for the streams: the vehicle's bytes arrive at a stream's rate.
		expect(await window()).toBeGreaterThanOrEqual(STREAMING_MIN_FRAMES);
		await shot(page, '1-live');

		// The control: stopping the vehicle turns every panel stale (and nothing else in the page is stale because of it), keeping the last values.
		const last = ((await row(page, 'imu', 'temp').textContent()) ?? '').replace(/\s+/g, ' ').trim();
		signalVehicle('SIGSTOP');
		await expectAll(page, true, STALE_FLIP_MS);
		await expect(page.locator('[data-summary]')).toContainText('8 of 8 panels stale');
		await expect(row(page, 'imu', 'temp')).toHaveText(last);
		await expect(panel(page, 'imu').getByText('stale', { exact: true })).toBeVisible();
		await expect(indicator(page, 'Helper')).toHaveAttribute('data-stale', 'false');
		await shot(page, '2-vehicle-stopped');

		// Continuing it, the panels go live again: the page asks again for streams it no longer hears.
		signalVehicle('SIGCONT');
		await expectAll(page, false, LIVE_PANELS_MS + STALE_FLIP_MS);
		await shot(page, '3-vehicle-continued');

		// Leaving the page stops the streams: the vehicle's bytes are only its heartbeat again.
		await page.getByRole('navigation', { name: 'Pages' }).getByRole('link', { name: 'Connection', exact: true }).click();
		await page.waitForTimeout(RETRY_MS);
		expect(await window()).toBeLessThanOrEqual(SILENT_MAX_FRAMES);
		// And coming back, they start again.
		await openVehicle(page);
		await expectAll(page, false, LIVE_PANELS_MS);
		expect(await window()).toBeGreaterThanOrEqual(STREAMING_MIN_FRAMES);
	});

	test('with the run stopped from the Simulation page every panel is stale and unheard', async ({ page }) => {
		test.skip(!live, 'the light mode keeps its fake vehicle after the stand-in run stops');
		await launch(page);
		await openVehicle(page);
		await expectAll(page, false, LIVE_PANELS_MS);
		const nav = page.getByRole('navigation', { name: 'Pages' });
		await nav.getByRole('link', { name: 'Simulation', exact: true }).click();
		await page.getByRole('button', { name: 'Stop' }).click();
		await expect(page.getByRole('button', { name: 'Launch' })).toBeEnabled({ timeout: GONE_MS });
		await openVehicle(page);
		await expectAll(page, true, STALE_FLIP_MS);
		for (const [, id] of PANELS) await expect(panel(page, id)).toHaveAttribute('data-heard', 'false');
		await shot(page, '4-run-stopped');
	});
});
