// The end-to-end specs of the Radio page (GCS-first unit GX-7), against the fake vehicle of tools/gs/e2e_fake_vehicle.py (light mode):
// the bars move with the vehicle's RC stream, the mapping comes from the vehicle's parameters, a switch that sends on a channel nothing
// uses is pointed out, "move the switch" finds it, a collision is warned about before staging and the vehicle's refusal is shown, a
// staged remap goes to the staged panel, and stopping the vehicle turns the page stale (the control). The live mode has no radio that
// moves (the SIL's sticks are held by its script), so these specs are light only.
import { expect, test, type Locator, type Page } from '@playwright/test';

const url = process.env.MARV_E2E_URL;
const live = process.env.MARV_E2E_MODE === 'live';
const out = process.env.MARV_E2E_OUT;
const vehiclePid = Number(process.env.MARV_E2E_VEHICLE_PID ?? '');
/** Connected within this long after "Launch" (a labelled harness value). */
const CONNECT_MS = 10_000;
/** The fake radio's switch changes every 4 s (RC_SWITCH_PERIOD_S / 2); a page finds it within a full period and slack (a labelled harness value). */
const SWITCH_MS = 12_000;
/** A panel goes stale within the shell's vehicle timeout (3 s, shell.ts SHELL_TIMING), one tick and slack (a labelled harness value). */
const STALE_FLIP_MS = 8_000;
const GONE_MS = 15_000;
const RETRY_MS = 1_000;
/** The channel the fake radio's switch sends on, and the channels the parameters name for the angle and the arm switch and for roll. */
const SWITCH_CHANNEL = 14;
const SOURCE = 'Luis 2026-10-19: the RadioMaster sends the angle switch on channel 14 (e2e)';

test.beforeAll(() => {
	expect(url, 'MARV_E2E_URL is the address the helper prints: run tools/gs/e2e.sh').toBeTruthy();
});
test.skip(live || !Number.isInteger(vehiclePid) || vehiclePid <= 0, 'needs the fake vehicle of the light mode (MARV_E2E_VEHICLE_PID)');
test.setTimeout(90_000);

const bar = (page: Page): Locator => page.getByRole('status', { name: 'Status bar' });
const indicator = (page: Page, name: string): Locator => bar(page).getByRole('group', { name, exact: true });
const fn = (page: Page, id: string): Locator => page.locator(`tr[data-fn="${id}"]`);
const channelRow = (page: Page, n: number): Locator => page.locator(`tr[data-channel="${n}"]`);

async function shot(page: Page, name: string): Promise<void> {
	if (out) await page.screenshot({ path: `${out}/radio-${name}.png`, fullPage: true });
}

function signalVehicle(signal: 'SIGSTOP' | 'SIGCONT'): void {
	process.kill(-vehiclePid, signal);
}

test.afterEach(async ({ page }) => {
	try {
		signalVehicle('SIGCONT');
	} catch {
		// the group is gone
	}
	// The fake vehicle keeps a stream until it is stopped, and the page stops them when it is left: leave it, so that the next spec starts
	// from a quiet vehicle (a closed browser tab stops them only on a best-effort basis).
	try {
		await page.getByRole('navigation', { name: 'Pages' }).getByRole('link', { name: 'Connection', exact: true }).click({ timeout: RETRY_MS * 5 });
		await page.waitForTimeout(RETRY_MS);
	} catch {
		// the page never got as far as the navigation
	}
});

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
	await expect(indicator(page, 'Link')).toContainText(/alive/, { timeout: CONNECT_MS });
	await expect(indicator(page, 'Identity')).not.toContainText('not reported', { timeout: CONNECT_MS });
}

/** The Vehicle page first (it asks for the streams and stops them when it is left), then the Radio page. */
async function openRadio(page: Page): Promise<void> {
	const nav = page.getByRole('navigation', { name: 'Pages' });
	await nav.getByRole('link', { name: 'Vehicle', exact: true }).click();
	await expect(page.getByRole('heading', { level: 1, name: 'Vehicle', exact: true })).toBeVisible();
	await nav.getByRole('link', { name: 'Radio', exact: true }).click();
	await expect(page.getByRole('heading', { level: 1, name: 'Radio', exact: true })).toBeVisible();
	await expect(page.locator('[data-summary]')).toContainText(/live/);
}

/** The distinct values a locator's text took over `ms` (sampled every 100 ms, a labelled harness value). */
async function samples(page: Page, locator: Locator, ms: number): Promise<Set<string>> {
	const seen = new Set<string>();
	for (let t = 0; t < ms; t += 100) {
		seen.add(((await locator.textContent()) ?? '').trim());
		await page.waitForTimeout(100);
	}
	return seen;
}

test('the bars move with the vehicle, the mapping comes from its parameters, and the switch on an unmapped channel is pointed out and found', async ({ page }) => {
	await launch(page);
	await openRadio(page);
	await expect(page.getByRole('table', { name: 'Channels', exact: true }).locator('tbody tr')).toHaveCount(16);
	await expect(page.locator('[data-summary]')).toContainText(/link quality \d+ %/);
	await expect(page.locator('[data-summary]')).toContainText(/16 channels/);

	// The bars move: channel 1 swings, so it takes several values; channel 3 (the throttle) rests at the low end.
	const roll = await samples(page, channelRow(page, 1).locator('[data-cell="us"]'), 3000);
	expect(roll.size, 'channel 1 takes several values in 3 s').toBeGreaterThan(2);
	await expect(channelRow(page, 3).locator('[data-cell="us"]')).toHaveText('988');

	// The mapping is the vehicle's: AETR and the register's arm 5 (locked) and angle 8.
	await expect(fn(page, 'roll').locator('[data-cell="active"]')).toHaveText('1', { timeout: CONNECT_MS });
	await expect(fn(page, 'throttle').locator('[data-cell="active"]')).toHaveText('3');
	await expect(fn(page, 'arm').locator('[data-cell="active"]')).toHaveText('5');
	await expect(fn(page, 'angle').locator('[data-cell="active"]')).toHaveText('8');
	await expect(fn(page, 'arm')).toContainText('locked');
	await expect(fn(page, 'angle')).not.toContainText('locked');
	await expect(channelRow(page, 5)).toContainText('Arm switch');
	await expect(channelRow(page, 8)).toContainText('Angle switch');

	// The thresholds against the live channels: both switches rest low, below 1700 us.
	await expect(page.locator('[data-switch="arm"]')).toHaveAttribute('data-position', 'below');
	await expect(page.locator('[data-switch="angle"]')).toHaveAttribute('data-position', 'below');
	await expect(page.locator('[data-switch="arm"]')).toContainText('threshold 0.391 (1700 µs)');
	await expect(page.locator('[data-check="throttle.low"]')).toHaveAttribute('data-status', 'ok');
	// The locks are shown read only, with their source.
	await expect(page.locator('[data-lock="rc_channel_arm"]')).toContainText('locked');
	await expect(page.locator('[data-lock="rc_channel_angle"]')).toContainText('not locked');

	// The switch that moves is on channel 14, which nothing uses: the page says so while the angle switch (channel 8) stays still.
	await expect(page.locator('[data-hint]')).toContainText(`Channel ${SWITCH_CHANNEL} moved`, { timeout: SWITCH_MS });
	await expect(page.locator('[data-hint]')).toContainText('angle switch (channel 8)');
	await expect(channelRow(page, SWITCH_CHANNEL)).toHaveAttribute('data-moved', 'true');
	await expect(channelRow(page, 8)).toHaveAttribute('data-moved', 'false');
	await shot(page, '1-live-hint');

	// "Move the switch you want for the angle switch": the channel that moved is found and highlighted.
	await page.getByRole('button', { name: 'Detect Angle switch' }).click();
	await expect(page.locator('[data-found-channel]')).toHaveText(String(SWITCH_CHANNEL), { timeout: SWITCH_MS });
	await expect(channelRow(page, SWITCH_CHANNEL)).toHaveAttribute('data-detected', 'true');
	await expect(page.locator('tr[data-detected="true"]')).toHaveCount(1);
	await shot(page, '2-detected');
	// The control: nothing is highlighted once the detection is cancelled.
	await page.getByRole('button', { name: 'Cancel' }).click();
	await expect(page.locator('tr[data-detected="true"]')).toHaveCount(0);
});

test('a collision is warned about before staging and refused by the vehicle; a staged remap goes to the staged panel', async ({ page }) => {
	await launch(page);
	await openRadio(page);
	await expect(fn(page, 'angle').locator('[data-cell="active"]')).toHaveText('8', { timeout: CONNECT_MS });
	const staged = page.locator('[data-fn="angle"] [data-cell="staged"]');
	await expect(staged).toHaveText('8');

	// Pick channel 14 by moving the switch, then stage it with its source text.
	await page.getByRole('button', { name: 'Detect Angle switch' }).click();
	await page.getByRole('button', { name: 'Use channel 14 for Angle switch' }).click({ timeout: SWITCH_MS });
	await expect(page.getByLabel('Angle switch channel')).toHaveValue(String(SWITCH_CHANNEL));
	await expect(page.locator('[data-r1]')).toHaveCount(0);
	const stage = page.getByRole('button', { name: 'Stage Angle switch' });
	await expect(stage).toBeDisabled();
	await page.getByLabel('Source text').fill(SOURCE);
	await expect(stage).toBeEnabled();
	await stage.click();
	await expect(page.locator('[data-fn-note="angle"]')).toContainText(`staged ${SWITCH_CHANNEL}`, { timeout: CONNECT_MS });
	await expect(staged).toHaveText(String(SWITCH_CHANNEL));
	// It goes through the one flow: the staged panel of the Parameters page lists it, to be validated and activated there.
	await expect(page.getByRole('heading', { name: 'Staged changes (1)' })).toBeVisible();
	await expect(page.getByRole('button', { name: 'Validate' })).toBeVisible();
	await expect(page.getByRole('button', { name: 'Activate' })).toBeVisible();
	await expect(channelRow(page, SWITCH_CHANNEL)).toContainText('→ Angle switch');
	// The unmapped-switch hint is gone: the staged config reads channel 14.
	await expect(page.locator('[data-hint]')).toHaveCount(0);
	await shot(page, '3-staged');

	// A channel another control uses: the warning shows before anything is staged, and the vehicle refuses it anyway.
	await page.getByLabel('Roll channel').fill('5');
	await expect(page.locator('[data-r1]')).toContainText('Channel 5 is used by Roll and Arm switch');
	await page.getByRole('button', { name: 'Stage Roll' }).click();
	await expect(page.locator('[data-fn-note="roll"]')).toContainText('refused by the vehicle', { timeout: CONNECT_MS });
	await expect(page.locator('[data-fn-note="roll"]')).toContainText('channel in use (R-1)');
	await expect(page.locator('[data-fn="roll"] [data-cell="staged"]')).toHaveText('1');
	await shot(page, '4-collision-refused');
	// The control: a free channel for roll raises no warning.
	await page.getByLabel('Roll channel').fill('9');
	await expect(page.locator('[data-r1]')).toHaveCount(0);

	// Put the vehicle back as it was: stage channel 8 again, so that nothing is staged.
	await page.getByLabel('Angle switch channel').fill('8');
	await page.getByRole('button', { name: 'Stage Angle switch' }).click();
	await expect(staged).toHaveText('8');
	await expect(page.getByRole('heading', { name: 'Staged changes (0)' })).toBeVisible();
});

test('stopping the vehicle turns the page stale, keeping the last values; continuing it turns the page live again', async ({ page }) => {
	await launch(page);
	await openRadio(page);
	const summary = page.locator('[data-summary]');
	const last = ((await channelRow(page, 3).locator('[data-cell="us"]').textContent()) ?? '').trim();
	signalVehicle('SIGSTOP');
	await expect(summary).toContainText('stale', { timeout: STALE_FLIP_MS });
	await expect(channelRow(page, 3).locator('[data-cell="us"]')).toHaveText(last);
	await expect(indicator(page, 'Helper')).toHaveAttribute('data-stale', 'false');
	await shot(page, '5-vehicle-stopped');
	signalVehicle('SIGCONT');
	await expect(summary).toContainText('live', { timeout: STALE_FLIP_MS + CONNECT_MS });
});
