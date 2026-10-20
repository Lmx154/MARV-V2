// The end-to-end specs of the GCS shell (GCS-first units GX-0 and GX-2): the app loads from the helper, connects to a vehicle through
// it and shows it in the status bar; the staleness control stops the vehicle and sees the link indicator go stale and come back;
// every page of the navigation opens with the status bar on it. Assertions are DOM roles and values.
//
// Two modes, set by tools/gs/e2e.sh: light (MARV_E2E_MODE unset) talks to the fake vehicle of tools/gs/e2e_fake_vehicle.py;
// live (MARV_E2E_MODE=live) starts the helper's SIL run first and talks to the simulator.
import { createHash } from 'node:crypto';
import { expect, test, type Locator, type Page } from '@playwright/test';

const url = process.env.MARV_E2E_URL;
const live = process.env.MARV_E2E_MODE === 'live';
const out = process.env.MARV_E2E_OUT;
/** Light mode: the process group of the fake vehicle (tools/gs/e2e.sh); the staleness control stops and continues it. */
const vehiclePid = Number(process.env.MARV_E2E_VEHICLE_PID ?? '');
/** The fake vehicle's image hash is the SHA-256 of this text (tools/gs/e2e_fake_vehicle.py IMAGE_TEXT). */
const FAKE_IMAGE_TEXT = 'marv-e2e-fake-image';
/** The status bar shows the first 12 hex digits of the image hash (shell.ts). */
const SHOWN_HEX = 12;
/** Live: how long a simulator run may take to bind its socket and answer (a labelled harness value, bounds a hung run only). */
const LIVE_START_MS = 180_000;
/** Light: connected (the link indicator fresh and alive) within this long after "Open SIL UDP" (a labelled harness value, N of the unit's done-when). */
const CONNECT_MS = 10_000;
/** The link indicator must flip to stale within this long of the vehicle being stopped: the stale timeout of shell.ts (3 s), one tick and slack (a labelled harness value). */
const STALE_FLIP_MS = 8_000;
/** One look at a control while a state settles (a labelled harness value). */
const RETRY_MS = 1_000;
/** The pages of the navigation (gcs/src/lib/state/nav.ts) and the heading each shows. */
const PAGES: Array<[string, string]> = [
	['Connection', 'Connection'],
	['Vehicle', 'Vehicle'],
	['Simulation', 'Simulation'],
	['Radio', 'Radio'],
	['Parameters', 'Parameters'],
	['Flash', 'Flash'],
	['Bench', 'Bench'],
	['Logs', 'Logs'],
	['Flight prep', 'Flight prep']
];

test.beforeAll(() => {
	expect(url, 'MARV_E2E_URL is the address the helper prints: run tools/gs/e2e.sh').toBeTruthy();
});

const bar = (page: Page): Locator => page.getByRole('status', { name: 'Status bar' });
const indicator = (page: Page, name: string): Locator => bar(page).getByRole('group', { name, exact: true });

async function shot(page: Page, name: string): Promise<void> {
	if (out) await page.screenshot({ path: `${out}/shell-${name}.png` });
}

async function open(page: Page) {
	await page.goto(url as string);
	// The helper link is up once the page has its WebSocket.
	await expect(indicator(page, 'Helper')).toContainText(/open/);
	// The helper keeps its device across page loads and across tests: every test starts with no device open. The page learns the
	// helper's status a moment after it connects, so close until the form can open a device.
	const close = page.getByRole('button', { name: 'Close device' });
	await expect(async () => {
		if (await close.isEnabled()) await close.click({ timeout: RETRY_MS });
		await expect(page.getByRole('button', { name: 'Open SIL UDP' })).toBeEnabled({ timeout: RETRY_MS });
	}).toPass();
	await expect(indicator(page, 'Link')).toContainText(/device\s+none/);
}

async function connect(page: Page) {
	if (live) {
		await page.getByRole('button', { name: 'Start SIL run' }).click();
		await expect(page.getByRole('button', { name: 'Stop SIL run' })).toBeEnabled({ timeout: LIVE_START_MS });
	}
	const openUdp = page.getByRole('button', { name: 'Open SIL UDP' });
	await expect(openUdp).toBeEnabled();
	await openUdp.click();
}

test('connects to the vehicle and shows it in the status bar', async ({ page }) => {
	await open(page);
	// Before the connection: no device, no heartbeat, and the link indicator says nothing is being heard (the indicator is not constant).
	await expect(indicator(page, 'Link')).toContainText(/no heartbeat/);
	await expect(indicator(page, 'Link')).toHaveAttribute('data-stale', 'true');
	await expect(indicator(page, 'Arming')).toHaveAttribute('data-stale', 'true');
	await shot(page, '1-before');

	await connect(page);

	const timeout = live ? LIVE_START_MS : CONNECT_MS;
	await expect(indicator(page, 'Link')).toContainText(/device\s+udp 127\.0\.0\.1:\d+/, { timeout });
	await expect(indicator(page, 'Link')).toContainText(/alive/, { timeout });
	await expect(indicator(page, 'Link')).toHaveAttribute('data-stale', 'false', { timeout });
	await expect(indicator(page, 'Arming')).toContainText(/disarmed/);
	await expect(indicator(page, 'Arming')).toContainText(/reason\s+vehicle \w+/);
	await expect(indicator(page, 'Arming')).toHaveAttribute('data-stale', 'false');
	await expect(indicator(page, 'Helper')).toHaveAttribute('data-stale', 'false');
	await expect(indicator(page, 'Host')).toContainText(/heavy lock/);

	const id = indicator(page, 'Identity');
	await expect(id).not.toContainText('not reported');
	await expect(id).toContainText(/source\s+\w+/);
	await expect(id).toContainText(/state\s+ok/);
	await expect(id).toHaveAttribute('data-stale', 'false');
	if (!live) {
		const shown = createHash('sha256').update(FAKE_IMAGE_TEXT).digest('hex').slice(0, SHOWN_HEX);
		await expect(id).toContainText(shown);
	}
	await shot(page, '2-connected');
});

test('closing the device returns the status bar to no device and no heartbeat', async ({ page }) => {
	await open(page);
	await connect(page);
	await expect(indicator(page, 'Link')).toContainText(/alive/, { timeout: live ? LIVE_START_MS : CONNECT_MS });

	await page.getByRole('button', { name: 'Close device' }).click();

	await expect(indicator(page, 'Link')).toContainText(/device\s+none/);
	await expect(indicator(page, 'Link')).toContainText(/no heartbeat/);
	await expect(indicator(page, 'Link')).toHaveAttribute('data-stale', 'true');
	await expect(indicator(page, 'Identity')).toContainText(/not reported/);
});

test.describe('the staleness control', () => {
	test.skip(!Number.isInteger(vehiclePid) || vehiclePid <= 0, 'needs the fake vehicle of the light mode (MARV_E2E_VEHICLE_PID)');
	// Never leave the fake vehicle stopped: the next test, and the cleanup of tools/gs/e2e.sh, need it running.
	test.afterEach(() => {
		try {
			process.kill(-vehiclePid, 'SIGCONT');
		} catch {
			// the group is gone
		}
	});

	test('stopping the vehicle turns the link indicator stale; continuing it turns it fresh again', async ({ page }) => {
		await open(page);
		await connect(page);
		const link = indicator(page, 'Link');
		await expect(link).toHaveAttribute('data-stale', 'false', { timeout: CONNECT_MS });

		process.kill(-vehiclePid, 'SIGSTOP');
		await expect(link).toHaveAttribute('data-stale', 'true', { timeout: STALE_FLIP_MS });
		await expect(indicator(page, 'Arming')).toHaveAttribute('data-stale', 'true', { timeout: STALE_FLIP_MS });
		await expect(indicator(page, 'Identity')).toHaveAttribute('data-stale', 'true');
		// The control: what is sourced from the helper is not stale because the vehicle is silent.
		await expect(indicator(page, 'Helper')).toHaveAttribute('data-stale', 'false');
		await expect(indicator(page, 'Host')).toHaveAttribute('data-stale', 'false');
		await shot(page, '3-vehicle-stopped');

		process.kill(-vehiclePid, 'SIGCONT');
		await expect(link).toHaveAttribute('data-stale', 'false', { timeout: STALE_FLIP_MS });
		await expect(indicator(page, 'Arming')).toHaveAttribute('data-stale', 'false', { timeout: STALE_FLIP_MS });
		await expect(link).toContainText(/alive/);
		await shot(page, '4-vehicle-continued');
	});
});

test('every page of the navigation opens with the status bar on it', async ({ page }) => {
	await open(page);
	const nav = page.getByRole('navigation', { name: 'Pages' });
	for (const [link, heading] of PAGES) {
		await nav.getByRole('link', { name: link, exact: true }).click();
		await expect(nav.getByRole('link', { name: link, exact: true })).toHaveAttribute('aria-current', 'page');
		await expect(page.getByRole('heading', { level: 1, name: heading, exact: true })).toBeVisible();
		await expect(bar(page).getByRole('group')).toHaveCount(6);
	}
	await nav.getByRole('link', { name: 'Flight prep', exact: true }).click();
	await expect(page.getByText(/Not built yet: GX-17 builds this page/)).toBeVisible();
	await shot(page, '5-placeholder');
	// A page that is loaded directly (not by the navigation) is served by the helper too.
	await page.goto(new URL('/vehicle', url as string).toString() + new URL(url as string).search);
	await expect(page.getByRole('heading', { level: 1, name: 'Vehicle', exact: true })).toBeVisible();
});
