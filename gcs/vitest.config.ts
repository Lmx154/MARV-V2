import { fileURLToPath } from 'node:url';
import { defineConfig } from 'vitest/config';

export default defineConfig({
	resolve: { alias: { $link: fileURLToPath(new URL('./src/lib/link', import.meta.url)) } },
	test: {
		include: ['src/**/*.test.ts', '../tests/regression/quad/L00/link/ts/**/*.test.ts'],
		environment: 'node'
	},
	server: { fs: { allow: ['..'] } }
});
