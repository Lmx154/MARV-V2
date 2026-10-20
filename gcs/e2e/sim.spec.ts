// The end-to-end specs of the SIM page (GCS-first unit GX-8): the sim channel the simulator publishes beside the link reaches the GCS through
// the helper's /ws/sim, the SIM panels are present while records arrive and gone when they stop, and nothing of it is on the vehicle's
// side (S-1). Assertions are DOM roles and values.
//
// Two modes, set by tools/gs/e2e.sh. Light: the simulator is the fake of tools/gs/e2e_fake_sim.py (a SIL record at 20 Hz); the control
// stops and continues it (MARV_E2E_SIM_PID). Live (MARV_E2E_MODE=live): the helper starts the Gazebo SIL (GUI off), which publishes the
// sim channel itself; the control stops the processes of that run (those with the helper's run directory in their command line) and
// stopping the run from the Simulation page removes the panels for good.
import { execFileSync } from 'node:child_process';
import { expect, test, type Locator, type Page } from '@playwright/test';

const url = process.env.MARV_E2E_URL;
const live = process.env.MARV_E2E_MODE === 'live';
const out = process.env.MARV_E2E_OUT;
const simPid = Number(process.env.MARV_E2E_SIM_PID ?? '');
const runsDir = process.env.MARV_E2E_RUNS_DIR ?? '';
/** Live: how long a simulator run may take to bind its socket and publish (a labelled harness value, bounds a hung run only). */
const LIVE_START_MS = 180_000;
/** Light: the SIM page is listed within this long of the page opening (a labelled harness value). */
const PRESENT_MS = 10_000;
/** The panels vanish within the channel's staleness timeout (2 s, sim/channel.ts SIM_TIMING), a display tick and slack (a labelled harness value). */
const VANISH_MS = 8_000;
/** A stopped run's processes are gone within this long (a labelled harness value). */
const GONE_MS = 15_000;
/** The sim time advances by at least this much between two reads this far apart (a labelled harness value: a loose real-time bound). */
const ADVANCE_WAIT_MS = 2_000;
const ADVANCE_MIN_S = 0.2;
/** The SIL panels of a world with the model IMU. */
const SIL_PANELS = ['run', 'truth', 'injected'];

test.beforeAll(() => {
	expect(url, 'MARV_E2E_URL is the address the helper prints: run tools/gs/e2e.sh').toBeTruthy();
});

const bar = (page: Page): Locator => page.getByRole('status', { name: 'Status bar' });
const indicator = (page: Page, name: string): Locator => bar(page).getByRole('group', { name, exact: true });
const nav = (page: Page): Locator => page.getByRole('navigation', { name: 'Pages' });
const simLink = (page: Page): Locator => nav(page).getByRole('link', { name: 'SIM', exact: true });
const simPanel = (page: Page, id: string): Locator => page.getByRole('main').locator(`[data-sim-panel="${id}"]`);
const row = (page: Page, panelId: string, key: string): Locator => simPanel(page, panelId).locator(`[data-row="sim-${panelId}.${key}"]`);

async function shot(page: Page, name: string): Promise<void> {
	if (out) await page.screenshot({ path: `${out}/sim-${name}.png`, fullPage: true });
}

const number = async (locator: Locator): Promise<number> => Number.parseFloat(((await locator.innerText()) ?? '').trim());

/** The processes of the helper's run (not the helper). */
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

function signalSim(signal: 'SIGSTOP' | 'SIGCONT'): void {
	if (live) {
		for (const pid of runPids()) {
			try {
				process.kill(pid, signal);
			} catch {
				// gone
			}
		}
	} else {
		process.kill(-simPid, signal); // the process group: `uv run` starts the simulator as its child
	}
}

/** The binary frames of the page's two WebSockets, by path: /ws carries the vehicle's bytes, /ws/sim the simulator's records. */
function watchSockets(page: Page): { sim: Buffer[]; link: Buffer[] } {
	const seen = { sim: [] as Buffer[], link: [] as Buffer[] };
	page.on('websocket', (ws) => {
		const path = new URL(ws.url()).pathname;
		ws.on('framereceived', (f) => {
			if (typeof f.payload === 'string') return;
			(path === '/ws/sim' ? seen.sim : seen.link).push(f.payload);
		});
	});
	return seen;
}

/** The Simulation page with no run going and no device open, then a run launched; the vehicle is connected when this returns. */
async function launch(page: Page): Promise<void> {
	await page.goto(url as string);
	await expect(indicator(page, 'Helper')).toContainText(/open/);
	await nav(page).getByRole('link', { name: 'Simulation', exact: true }).click();
	await expect(page.getByRole('heading', { level: 1, name: 'Simulation', exact: true })).toBeVisible();
	const stop = page.getByRole('button', { name: 'Stop' });
	const start = page.getByRole('button', { name: 'Launch' });
	await expect(stop.or(start).and(page.locator(':enabled'))).toBeVisible();
	if (await stop.isEnabled()) {
		await stop.click();
		await expect(start).toBeEnabled({ timeout: GONE_MS });
	}
	await expect(page.getByRole('combobox', { name: 'Card' })).toBeEnabled();
	await start.click();
	await expect(indicator(page, 'Link')).toContainText(/alive/, { timeout: LIVE_START_MS });
}

/** Never leave the simulator stopped: the next test and the cleanup of tools/gs/e2e.sh need it running. */
test.afterEach(() => {
	try {
		signalSim('SIGCONT');
	} catch {
		// the process is gone
	}
});

test.describe('the SIM page', () => {
	test.skip(!live && (!Number.isInteger(simPid) || simPid <= 0), 'needs the fake simulator of the light mode (MARV_E2E_SIM_PID)');
	test.setTimeout(live ? LIVE_START_MS + 180_000 : 90_000);

	test('the SIM panels are present while records arrive, show simulator values, and are gone when they stop; the vehicle side has none', async ({ page }) => {
		const sockets = watchSockets(page);
		if (live) await launch(page);
		else await page.goto(url as string);
		await expect(indicator(page, 'Helper')).toContainText(/open/);
		await expect(simLink(page), 'the SIM page is listed while a simulation publishes').toBeVisible({ timeout: live ? LIVE_START_MS : PRESENT_MS });
		await simLink(page).click();
		await expect(page.getByRole('heading', { level: 1, name: 'SIM', exact: true })).toBeVisible();
		for (const id of SIL_PANELS) {
			const p = simPanel(page, id);
			await expect(p).toBeVisible();
			await expect(p.getByText('SIM', { exact: true }).first()).toBeVisible(); // each panel is labelled
			expect(await p.getAttribute('aria-label')).toMatch(/^SIM/);
		}
		await expect(simPanel(page, 'hil'), 'a SIL run has no HIL board panel').toHaveCount(0);
		// Simulator values, with their units.
		await expect(row(page, 'run', 'kind')).toContainText('SIL');
		await expect(row(page, 'truth', 'height')).toContainText(/^-?\d+\.\d\d\s*m\s*$/);
		await expect(row(page, 'truth', 'rotor.1')).toContainText(/rad\/s/);
		await expect(row(page, 'injected', 'cmd.1')).toContainText(/DShot/);
		await expect(row(page, 'injected', 'accel.z')).toContainText(/m\/s²/);
		const t1 = await number(row(page, 'run', 'time'));
		await page.waitForTimeout(ADVANCE_WAIT_MS);
		const t2 = await number(row(page, 'run', 'time'));
		expect(t2 - t1, 'the sim time advances while the simulator runs').toBeGreaterThan(ADVANCE_MIN_S);
		expect(await number(row(page, 'run', 'rtf'))).toBeGreaterThan(0);
		expect(await number(row(page, 'run', 'rate'))).toBeGreaterThan(0);
		await shot(page, '1-present');

		// The records came on /ws/sim and none of them on /ws (the vehicle's bytes), where only the vehicle's frames are.
		expect(sockets.sim.length).toBeGreaterThan(0);
		expect(sockets.sim.every((b) => b.subarray(0, 4).toString('latin1') === 'MSIM')).toBe(true);
		expect(sockets.link.some((b) => b.subarray(0, 4).toString('latin1') === 'MSIM')).toBe(false);

		// S-1: the Vehicle page has no SIM panel and shows no simulator truth.
		await nav(page).getByRole('link', { name: 'Vehicle', exact: true }).click();
		await expect(page.getByRole('heading', { level: 1, name: 'Vehicle', exact: true })).toBeVisible();
		await expect(page.locator('[data-sim-panel]')).toHaveCount(0);
		await expect(page.getByRole('main')).not.toContainText('SIM ·');
		await simLink(page).click();
		for (const id of SIL_PANELS) await expect(simPanel(page, id)).toBeVisible();

		// The control: the simulator stops, the panels are removed (not marked stale), the page says so and the SIM page leaves the navigation.
		signalSim('SIGSTOP');
		await expect(page.locator('[data-sim-panel]')).toHaveCount(0, { timeout: VANISH_MS });
		await expect(page.getByRole('status').filter({ hasText: 'No simulation is publishing' })).toBeVisible();
		await expect(simLink(page)).toHaveCount(0);
		await shot(page, '2-stopped');

		// Continuing it, they come back.
		signalSim('SIGCONT');
		await expect(simLink(page)).toBeVisible({ timeout: PRESENT_MS });
		await simLink(page).click();
		for (const id of SIL_PANELS) await expect(simPanel(page, id)).toBeVisible({ timeout: PRESENT_MS });
		await shot(page, '3-continued');
	});

	test('stopping the run from the Simulation page removes the SIM page from the navigation for good', async ({ page }) => {
		test.skip(!live, 'the light mode has no run to stop: its fake simulator publishes by itself');
		await launch(page);
		await expect(simLink(page)).toBeVisible({ timeout: LIVE_START_MS });
		await simLink(page).click();
		for (const id of SIL_PANELS) await expect(simPanel(page, id)).toBeVisible();
		await nav(page).getByRole('link', { name: 'Simulation', exact: true }).click();
		await page.getByRole('button', { name: 'Stop' }).click();
		await expect(page.getByRole('button', { name: 'Launch' })).toBeEnabled({ timeout: GONE_MS });
		await expect(simLink(page)).toHaveCount(0, { timeout: VANISH_MS });
		await expect(page.locator('[data-sim-panel]')).toHaveCount(0);
		await shot(page, '4-run-stopped');
	});
});
