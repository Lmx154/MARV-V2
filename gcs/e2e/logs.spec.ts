// The end-to-end spec of the Logs page (GCS-first unit GX-15, light mode only): the .mlog sample of logger unit U1 is on a mounted card
// of a directory of the run (the helper's --log-card-dir); the page lists it, imports it, shows the decoder's report and the SHA-256, and
// plots one field from the helper's min/max series. The expected values come from the decoder itself (tools/gs/e2e.sh runs
// tools/log/mlog.py on the sample and passes them in); the SHA-256 is computed here from the sample's bytes. Assertions are DOM roles and values.
//
// Controls: a narrower time range gives a narrower min and max (the plot is not constant), and a path inside the repository is refused with the
// helper's reason (W-1) and imports nothing.
import { createHash } from 'node:crypto';
import { readFileSync } from 'node:fs';
import { expect, test, type Locator, type Page } from '@playwright/test';

const url = process.env.MARV_E2E_URL;
const out = process.env.MARV_E2E_OUT;
const sample = process.env.MARV_E2E_LOG_SAMPLE ?? '';
const expectedGyro = (process.env.MARV_E2E_LOG_GYRO_X ?? '').split(' ');
const repoFile = process.env.MARV_E2E_LOG_REPO_FILE ?? '';
const enabled = sample !== '';
/** The sample's name on the card (logs.py CARD_LOG_DIR: <mount>/marv/log/NNNNNNNN.mlog). */
const CARD_NAME = '00000001.mlog';
/** The sample has this many sectors and IMU records (tests/tools/data/u1_sample.mlog = good_log(600)); the decoder's report says so. */
const SECTORS = 41;
const IMU_RECORDS = 600;

test.beforeAll(() => {
	expect(url, 'MARV_E2E_URL is the address the helper prints: run tools/gs/e2e.sh').toBeTruthy();
});
test.skip(!enabled, 'needs the sample card of the light mode (MARV_E2E_LOG_SAMPLE)');

const bar = (page: Page): Locator => page.getByRole('status', { name: 'Status bar' });
const indicator = (page: Page, name: string): Locator => bar(page).getByRole('group', { name, exact: true });
const report = (page: Page, key: string): Locator => page.locator(`[aria-label="Decoder report"] [data-report="${key}"]`);
const plot = (page: Page, field: string, what: 'min' | 'max' | 'filled'): Locator => page.locator(`[data-plot-field="${field}"] [data-plot="${what}"]`);

async function shot(page: Page, name: string): Promise<void> {
	if (out) await page.screenshot({ path: `${out}/logs-${name}.png`, fullPage: true });
}

async function openPage(page: Page): Promise<void> {
	await page.goto(url as string);
	await expect(indicator(page, 'Helper')).toContainText(/open/);
	await page.getByRole('navigation', { name: 'Pages' }).getByRole('link', { name: 'Logs', exact: true }).click();
	await expect(page.getByRole('heading', { level: 1, name: 'Logs', exact: true })).toBeVisible();
}

test('the card file is listed, imported with its SHA-256 and decoder report, and one field is plotted from the min/max series', async ({ page }) => {
	const bytes = readFileSync(sample);
	const sha = createHash('sha256').update(bytes).digest('hex');
	await openPage(page);

	// The card: listed before the import, the library empty.
	await expect(page.getByRole('list', { name: 'Card files' }).getByRole('listitem')).toHaveCount(1);
	await expect(page.locator('[data-logs="empty"]')).toBeVisible();

	await page.getByRole('button', { name: `Import ${CARD_NAME}` }).click();
	await expect(page.getByRole('status').filter({ hasText: sha.slice(0, 12) })).toBeVisible();
	const table = page.getByRole('table', { name: 'Imported logs' });
	await expect(table.getByRole('row')).toHaveCount(2); // the header row and the log
	await expect(table.locator(`tr[data-sha="${sha}"]`)).toContainText('clean');

	// The decoder's report equals the decoder's (tools/gs/e2e.sh gives the expected values; the SHA-256 is of the bytes).
	await expect(report(page, 'sha256')).toHaveText(sha);
	await expect(report(page, 'size')).toHaveText(`${bytes.length} B`);
	await expect(report(page, 'sectors')).toHaveText(`${SECTORS} valid of ${SECTORS}`);
	await expect(report(page, 'records')).toHaveText(`end 1, imu ${IMU_RECORDS}, status 2`);
	await expect(report(page, 'end')).toHaveText('yes');
	await expect(report(page, 'corrupt')).toHaveText('none');
	await shot(page, '1-imported');

	// One field: the IMU's gyro_x is chosen by default; plot it.
	await expect(page.getByRole('combobox', { name: 'Record kind' })).toHaveValue('imu');
	await expect(page.getByRole('checkbox', { name: 'gyro_x' })).toBeChecked();
	await page.getByRole('button', { name: 'Plot', exact: true }).click();
	await expect(page.getByRole('img', { name: /^gyro_x: minimum to maximum/ })).toBeVisible();
	await expect(plot(page, 'gyro_x', 'min')).toHaveText(expectedGyro[0] as string);
	await expect(plot(page, 'gyro_x', 'max')).toHaveText(expectedGyro[1] as string);
	await expect(plot(page, 'gyro_x', 'filled')).toHaveText(`${IMU_RECORDS} of 600`);
	await expect(page.locator('[data-series="summary"]')).toContainText(`${IMU_RECORDS} records in 600 buckets`);
	await shot(page, '2-plot');

	// Control: a narrower range of the same field gives a narrower min and max.
	await page.getByRole('textbox', { name: 'From (s)' }).fill('0.02');
	await page.getByRole('textbox', { name: 'To (s)' }).fill('0.04');
	await page.getByRole('button', { name: 'Plot', exact: true }).click();
	await expect(plot(page, 'gyro_x', 'filled')).not.toHaveText(`${IMU_RECORDS} of 600`);
	const narrowMin = Number.parseFloat(await plot(page, 'gyro_x', 'min').innerText());
	const narrowMax = Number.parseFloat(await plot(page, 'gyro_x', 'max').innerText());
	expect(narrowMin).toBeGreaterThan(Number.parseFloat(expectedGyro[0] as string));
	expect(narrowMax).toBeLessThan(Number.parseFloat(expectedGyro[1] as string));
	expect(narrowMax).toBeGreaterThan(narrowMin);
	await shot(page, '3-narrow');
});

test('a path inside the repository is refused with the helper’s reason and nothing is imported (W-1)', async ({ page }) => {
	expect(repoFile, 'MARV_E2E_LOG_REPO_FILE').toBeTruthy();
	await openPage(page);
	const before = await page.getByRole('table', { name: 'Imported logs' }).getByRole('row').count();
	await page.getByRole('textbox', { name: 'Absolute path of an .mlog file' }).fill(repoFile);
	await page.getByRole('button', { name: 'Import path' }).click();
	await expect(page.getByRole('alert').filter({ hasText: 'inside the repository' })).toBeVisible();
	await expect(page.getByRole('table', { name: 'Imported logs' }).getByRole('row')).toHaveCount(before);
	await shot(page, '4-refused');
});
