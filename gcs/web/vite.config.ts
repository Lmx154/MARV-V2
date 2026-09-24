import adapter from '@sveltejs/adapter-static';
import { sveltekit } from '@sveltejs/kit/vite';
import { defineConfig } from 'vite';

// The GCS backend (gcs/): serves /api and /ws, and this app's build/ in production.
const backend = 'http://127.0.0.1:8765';
const proxy = {
	'/api': backend,
	'/ws': { target: backend, ws: true }
};

export default defineConfig({
	plugins: [
		sveltekit({
			compilerOptions: {
				runes: ({ filename }) => (filename.split(/[/\\]/).includes('node_modules') ? undefined : true)
			},
			adapter: adapter({ fallback: 'index.html' })
		})
	],
	server: { proxy },
	preview: { proxy }
});
