// The first end-to-end spec (GCS-first unit GX-0): the app loads from the helper, connects to a vehicle through it and shows the
// connection indicator (the status region of the header). Assertions are DOM roles and values.
//
// Two modes, set by tools/gs/e2e.sh: light (MARV_E2E_MODE unset) talks to the fake vehicle of tools/gs/e2e_fake_vehicle.py;
// live (MARV_E2E_MODE=live) starts the helper's SIL run first and talks to the simulator.
import { createHash } from 'node:crypto';
import { expect, test, type Page } from '@playwright/test';

const url = process.env.MARV_E2E_URL;
const live = process.env.MARV_E2E_MODE === 'live';
/** The fake vehicle's image hash is the SHA-256 of this text (tools/gs/e2e_fake_vehicle.py IMAGE_TEXT). */
const FAKE_IMAGE_TEXT = 'marv-e2e-fake-image';
/** The identity strip shows the first 12 hex digits of the image hash (IdentityStrip.svelte). */
const SHOWN_HEX = 12;
/** Live: how long a simulator run may take to bind its socket and answer (a labelled harness value, bounds a hung run only). */
const LIVE_START_MS = 180_000;
/** One look at a control while a state settles (a labelled harness value). */
const RETRY_MS = 1_000;

test.beforeAll(() => {
	expect(url, 'MARV_E2E_URL is the address the helper prints: run tools/gs/e2e.sh').toBeTruthy();
});

async function open(page: Page) {
	await page.goto(url as string);
	const state = page.getByRole('status', { name: 'Connection state' });
	// The helper link is up once the page has its WebSocket.
	await expect(state).toContainText(/helper\s+open/);
	// The helper keeps its device across page loads and across tests: every test starts with no device open. The page learns the
	// helper's status a moment after it connects, so close until the form can open a device.
	const close = page.getByRole('button', { name: 'Close device' });
	await expect(async () => {
		if (await close.isEnabled()) await close.click();
		await expect(page.getByRole('button', { name: 'Open SIL UDP' })).toBeEnabled({ timeout: RETRY_MS });
	}).toPass();
	await expect(state).toContainText(/device\s+none/);
	return state;
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

test('connects to the vehicle and shows it in the connection indicator', async ({ page }) => {
	const state = await open(page);
	// Before the connection: no device and no heartbeat (the indicator is not constant).
	await expect(state).toContainText(/device\s+none/);
	await expect(state).toContainText(/vehicle\s+no heartbeat/);

	await connect(page);

	await expect(state).toContainText(/device\s+udp 127\.0\.0\.1:\d+/, { timeout: live ? LIVE_START_MS : undefined });
	await expect(state).toContainText(/vehicle\s+alive/, { timeout: live ? LIVE_START_MS : undefined });
	await expect(state).toContainText(/arming\s+disarmed/);

	const identity = page.getByLabel('Vehicle identity');
	await expect(identity).not.toContainText('not reported by the vehicle');
	if (!live) {
		const shown = createHash('sha256').update(FAKE_IMAGE_TEXT).digest('hex').slice(0, SHOWN_HEX);
		await expect(identity).toContainText(shown);
	}
});

test('closing the device returns the indicator to no device and no heartbeat', async ({ page }) => {
	const state = await open(page);
	await connect(page);
	await expect(state).toContainText(/vehicle\s+alive/, { timeout: live ? LIVE_START_MS : undefined });

	await page.getByRole('button', { name: 'Close device' }).click();

	await expect(state).toContainText(/device\s+none/);
	await expect(state).toContainText(/vehicle\s+no heartbeat/);
});
