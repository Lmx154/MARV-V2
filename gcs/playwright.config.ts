import { defineConfig } from '@playwright/test';

// The end-to-end run (GCS-first unit GX-0). tools/gs/e2e.sh starts the vehicle (a fake one, or a live SIL), the helper and the
// built GCS, and sets MARV_E2E_URL to the address the helper prints (it carries the session token). Assertions are DOM roles and
// values, never screenshot goldens. The timeouts are labelled harness values: they bound a hung run, they decide no result.
export default defineConfig({
	testDir: './e2e',
	// One helper takes one browser client at a time (decision 0053 Q5), so the tests run one after the other, in one worker.
	workers: 1,
	fullyParallel: false,
	retries: 0,
	timeout: 60_000,
	expect: { timeout: 15_000 },
	outputDir: process.env.MARV_E2E_OUT ? `${process.env.MARV_E2E_OUT}/artifacts` : 'test-results',
	reporter: [
		['list'],
		['json', { outputFile: process.env.MARV_E2E_OUT ? `${process.env.MARV_E2E_OUT}/report.json` : 'test-results/report.json' }]
	],
	use: { screenshot: 'only-on-failure', trace: 'off' }
});
