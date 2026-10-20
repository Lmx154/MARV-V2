// The end-to-end spec of the pilot bridge control (ground-segment unit GX-9, light mode only): on the Simulation page the joystick is
// opened and closed through the helper, and the Pilot panel shows its state and the 16 channels being sent. The joystick is the fake of
// tools/gs/e2e_fake_joystick.py (a FIFO of js_event records: channel 5 high, channel 1 sweeping); the run is the stand-in `sleep`, so
// what the spec checks is the control and the helper's state, not the simulator's response (that is the gz test, tests/regression/quad/
// L10/gz/test_pilot_bridge.py). Assertions are DOM roles and values.
//
// Controls: with no run, and with a run started without the Pilot option, "Open joystick" is disabled; after "Close joystick" the frame
// count stops (PB-2).
import { expect, test, type Locator, type Page } from '@playwright/test';

const url = process.env.MARV_E2E_URL;
const out = process.env.MARV_E2E_OUT;
const enabled = process.env.MARV_E2E_PILOT === '1';
/** The CRSF +100 % tick the fake joystick's full-scale axis maps to (tools/gs/pilot.py axis_to_ticks) and the centre tick. */
const TICK_HIGH = '1811';
const TICK_CENTRE = '992';
/** A stopped run's processes are gone within this long (a labelled harness value). */
const GONE_MS = 15_000;
/** The frame count stays put for this long after Close (a labelled harness value: 12 frame periods of 20 ms plus slack). */
const QUIET_MS = 500;

test.beforeAll(() => {
	expect(url, 'MARV_E2E_URL is the address the helper prints: run tools/gs/e2e.sh').toBeTruthy();
});
test.skip(!enabled, 'needs the fake joystick of the light mode (MARV_E2E_PILOT)');

const bar = (page: Page): Locator => page.getByRole('status', { name: 'Status bar' });
const indicator = (page: Page, name: string): Locator => bar(page).getByRole('group', { name, exact: true });
const state = (page: Page): Locator => page.locator('[data-pilot="state"]');
const frames = async (page: Page): Promise<number> => Number.parseInt((await page.locator('[data-pilot="frames"]').innerText()).trim(), 10);
const channel = (page: Page, n: number): Locator => page.getByRole('table', { name: 'Pilot channels' }).locator(`tr[data-channel="${n}"] td`).nth(1);
const openButton = (page: Page): Locator => page.getByRole('button', { name: 'Open joystick' });
const closeButton = (page: Page): Locator => page.getByRole('button', { name: 'Close joystick' });

async function shot(page: Page, name: string): Promise<void> {
	if (out) await page.screenshot({ path: `${out}/pilot-${name}.png`, fullPage: true });
}

/** The Simulation page with no run going. */
async function openPage(page: Page): Promise<void> {
	await page.goto(url as string);
	await expect(indicator(page, 'Helper')).toContainText(/open/);
	await page.getByRole('navigation', { name: 'Pages' }).getByRole('link', { name: 'Simulation', exact: true }).click();
	await expect(page.getByRole('heading', { level: 1, name: 'Simulation', exact: true })).toBeVisible();
	const stop = page.getByRole('button', { name: 'Stop' });
	const launch = page.getByRole('button', { name: 'Launch' });
	await expect(stop.or(launch).and(page.locator(':enabled'))).toBeVisible();
	if (await stop.isEnabled()) {
		await stop.click();
		await expect(launch).toBeEnabled({ timeout: GONE_MS });
	}
}

test('the joystick opens only for a run with the Pilot option, shows its state and channels, and stops sending when closed', async ({ page }) => {
	await openPage(page);
	const launch = page.getByRole('button', { name: 'Launch' });
	await expect(page.getByRole('combobox', { name: 'Joystick' }).locator('option')).toHaveCount(2); // "choose a device" and the fake js0
	await page.getByRole('combobox', { name: 'Joystick' }).selectOption({ index: 1 });

	// Control 1: no run.
	await expect(openButton(page)).toBeDisabled();
	await expect(state(page)).toContainText('closed');

	// Control 2: a run without the Pilot option.
	const pilotBox = page.getByRole('checkbox', { name: /^Pilot:/ });
	await expect(pilotBox).not.toBeChecked();
	await launch.click();
	await expect(page.getByRole('button', { name: 'Stop' })).toBeEnabled();
	await expect(openButton(page)).toBeDisabled();
	await expect(openButton(page)).toHaveAttribute('title', /Pilot option/);
	await page.getByRole('button', { name: 'Stop' }).click();
	await expect(launch).toBeEnabled({ timeout: GONE_MS });

	// A run with the Pilot option.
	await pilotBox.check();
	await launch.click();
	await expect(page.getByRole('button', { name: 'Stop' })).toBeEnabled();
	await expect(page.getByRole('definition').filter({ hasText: 'joystick' }).first()).toBeVisible(); // the Run panel says so
	await expect(openButton(page)).toBeEnabled();
	await shot(page, '1-run-with-pilot');

	await openButton(page).click();
	await expect(state(page)).toContainText('open');
	await expect(page.locator('[data-pilot="device"]')).toContainText('js0');
	await expect(closeButton(page)).toBeEnabled();
	await expect(openButton(page)).toBeDisabled();
	await expect.poll(() => frames(page)).toBeGreaterThan(5);
	// The fake joystick holds channel 5 (the arm switch) at +100 % and sweeps channel 1; the other channels are centred.
	await expect(channel(page, 5)).toHaveText(TICK_HIGH);
	await expect(channel(page, 3)).toHaveText(TICK_CENTRE);
	const seen = new Set<string>();
	await expect.poll(async () => (seen.add(await channel(page, 1).innerText()), seen.size)).toBeGreaterThan(3); // channel 1 moves
	await expect(page.getByText('never on the vehicle link', { exact: false })).toBeVisible();
	await shot(page, '2-open');

	// PB-2: after Close the frames stop.
	await closeButton(page).click();
	await expect(state(page)).toContainText('closed');
	await expect(page.getByRole('table', { name: 'Pilot channels' })).toHaveCount(0);
	const after = await frames(page);
	await page.waitForTimeout(QUIET_MS);
	expect(await frames(page), 'no frame is sent after Close').toBe(after);
	await shot(page, '3-closed');

	// Opening again works, and stopping the run closes the joystick.
	await openButton(page).click();
	await expect(state(page)).toContainText('open');
	await page.getByRole('button', { name: 'Stop' }).click();
	await expect(state(page)).toContainText('closed', { timeout: GONE_MS });
	await expect(page.locator('[data-pilot="reason"]')).toContainText('run');
	await shot(page, '4-run-stopped');
});
