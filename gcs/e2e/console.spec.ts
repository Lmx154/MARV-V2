// The end-to-end spec of the Console page (ground-segment unit GX-16, decision 0038): the page opens the board's console through the helper,
// shows its output, sends what is typed unchanged, and stops when closed. It is a terminal: the spec types lines and reads what the fake
// board received; it knows no command of the board. Assertions are DOM roles and values (and the helper's console.log).
//
// Light mode only (tools/gs/e2e.sh): the board's console is the fake of tools/gs/e2e_fake_console.py, a pty that prints a banner and a
// line a second and answers each line it receives with the bytes it got. No port of the host is opened.
//
// Controls: the input is off and Open is on while the console is closed; the output stops after Close (the same wait that sees ticks while
// open); with the board held for a HIL run Open is off and the helper's reason is shown, and with the hold released it is on.
import { existsSync, readdirSync, readFileSync } from 'node:fs';
import { join } from 'node:path';
import { expect, test, type Locator, type Page } from '@playwright/test';

const url = process.env.MARV_E2E_URL;
const live = process.env.MARV_E2E_MODE === 'live';
const out = process.env.MARV_E2E_OUT;
const runsDir = process.env.MARV_E2E_RUNS_DIR ?? '';
/** The fake console's texts (tools/gs/e2e_fake_console.py). */
const BANNER = 'marv_fc3v2 fake console (e2e)';
const DEVICE = /usb-MARV_E2E_console-if00/;
/** How long "nothing more arrives" is watched for: two periods of the fake's one-second line (a labelled harness value). */
const QUIET_MS = 2500;

test.skip(live, 'the Console page is checked against the fake console of the light mode');

test.beforeAll(() => {
	expect(url, 'MARV_E2E_URL is the address the helper prints: run tools/gs/e2e.sh').toBeTruthy();
});

const bar = (page: Page): Locator => page.getByRole('status', { name: 'Status bar' });
const indicator = (page: Page, name: string): Locator => bar(page).getByRole('group', { name, exact: true });
const output = (page: Page): Locator => page.getByRole('log', { name: 'Console output' });
const input = (page: Page): Locator => page.getByRole('textbox', { name: 'Console input' });
const state = (page: Page): Locator => page.locator('[data-console="state"]');
const openButton = (page: Page): Locator => page.getByRole('button', { name: 'Open', exact: true });
const closeButton = (page: Page): Locator => page.getByRole('button', { name: 'Close', exact: true });

async function shot(page: Page, name: string): Promise<void> {
	if (out) await page.screenshot({ path: `${out}/console-${name}.png`, fullPage: true });
}

/** The raw text of the output (white space as the board sent it). */
const raw = async (page: Page): Promise<string> => (await output(page).textContent()) ?? '';

async function openPage(page: Page): Promise<void> {
	await page.goto(url as string);
	await expect(indicator(page, 'Helper')).toContainText(/open/);
	await page.getByRole('navigation', { name: 'Pages' }).getByRole('link', { name: 'Console', exact: true }).click();
	await expect(page.getByRole('heading', { level: 1, name: 'Console', exact: true })).toBeVisible();
}

async function chooseFake(page: Page): Promise<string> {
	const option = page.getByRole('option', { name: DEVICE });
	await expect(option).toHaveCount(1);
	const value = (await option.getAttribute('value')) as string;
	await page.getByRole('combobox', { name: 'Device' }).selectOption(value);
	return value;
}

/** The helper's console logs under its run directory. */
function consoleLogs(): string[] {
	if (!runsDir || !existsSync(runsDir)) return [];
	return readdirSync(runsDir).flatMap((d) => readdirSync(join(runsDir, d)).filter((f) => /^console.*\.log$/.test(f)).map((f) => join(runsDir, d, f)));
}

test('the console opens, shows the board output, sends typed lines unchanged and stops when closed', async ({ page }) => {
	await openPage(page);
	const device = await chooseFake(page);

	// Control: closed, the input is off and Open is on.
	await expect(state(page)).toContainText('closed');
	await expect(input(page)).toBeDisabled();
	await expect(openButton(page)).toBeEnabled();
	await expect(closeButton(page)).toBeDisabled();
	await shot(page, '1-closed');

	await openButton(page).click();
	await expect(state(page)).toContainText('open');
	await expect(output(page)).toContainText(`console opened: ${device}`);
	await expect(output(page)).toContainText(BANNER);
	await expect(output(page)).toContainText('tick 2'); // the board's own line, a second apart: output keeps arriving
	await expect(input(page)).toBeEnabled();
	await expect(openButton(page)).toBeDisabled();

	// What is typed reaches the board as it is: spaces, case and a line end.
	const typed = '  Hello   World 123 !? ';
	await input(page).fill(typed);
	await input(page).press('Enter');
	await expect(input(page)).toHaveValue('');
	await expect.poll(() => raw(page)).toContain(`received ${typed.length} bytes: ${JSON.stringify(typed)}`);
	await expect(output(page).locator('[data-kind="tx"]').first()).toHaveText(`> ${typed}`);
	await input(page).fill('stop');
	await page.getByRole('button', { name: 'Send', exact: true }).click();
	await expect.poll(() => raw(page)).toContain('received 4 bytes: "stop"');
	await input(page).press('Enter'); // an empty line is a line: only its line end is sent
	await expect.poll(() => raw(page)).toContain('received 0 bytes: ""');
	await shot(page, '2-open');

	// Close stops the traffic: nothing arrives in the time that showed a new line while open.
	await closeButton(page).click();
	await expect(state(page)).toContainText('closed');
	await expect(input(page)).toBeDisabled();
	await expect(output(page)).toContainText('console closed: closed by request');
	const before = await raw(page);
	await page.waitForTimeout(QUIET_MS);
	expect(await raw(page)).toBe(before);
	await shot(page, '3-closed');

	// The helper's log holds what crossed while it was open: the banner the board printed and the lines the operator sent.
	const logs = consoleLogs();
	expect(logs.length, `console logs under ${runsDir}`).toBeGreaterThan(0);
	const bytes = readFileSync(logs[logs.length - 1] as string).toString('latin1');
	expect(bytes).toContain(BANNER);
	expect(bytes).toContain(typed + '\n');
	expect(bytes).toContain('stop\n');
});

test('the console is refused while the board is held for a HIL run, and opens once the hold is released', async ({ page }) => {
	await openPage(page);
	await chooseFake(page);
	await expect(openButton(page)).toBeEnabled();

	const nav = page.getByRole('navigation', { name: 'Pages' });
	await nav.getByRole('link', { name: 'Bench', exact: true }).click();
	await page.getByRole('button', { name: 'Hold the board' }).click();
	await expect(page.getByRole('button', { name: 'Release the hold' })).toBeVisible();
	try {
		await nav.getByRole('link', { name: 'Console', exact: true }).click();
		await expect(openButton(page)).toBeDisabled(); // the helper's status carries the reason within a poll
		await expect(page.locator('[data-console="blocked"]').first()).toContainText('held for a HIL run');
		await shot(page, '4-held');
	} finally {
		await nav.getByRole('link', { name: 'Bench', exact: true }).click();
		await page.getByRole('button', { name: 'Release the hold' }).click();
		await expect(page.getByRole('button', { name: 'Hold the board' })).toBeVisible();
	}
	await nav.getByRole('link', { name: 'Console', exact: true }).click();
	await expect(openButton(page)).toBeEnabled(); // control: the same page, the hold released
	await expect(page.locator('[data-console="blocked"]')).toHaveCount(0);
});
