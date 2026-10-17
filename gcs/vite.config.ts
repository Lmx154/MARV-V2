import { sveltekit } from '@sveltejs/kit/vite';
import { defineConfig } from 'vitest/config';

export default defineConfig({
	plugins: [sveltekit()],
	test: {
		// src/ holds the GUI's tests; the L00 suite holds the codec conformance tests (ctest link_ts_conformance runs this vitest).
		include: ['src/**/*.test.ts', '../tests/regression/quad/L00/link/ts/**/*.test.ts'],
		environment: 'node'
	},
	server: { fs: { allow: ['..'] } }
});
