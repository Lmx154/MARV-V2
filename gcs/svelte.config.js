import adapter from '@sveltejs/adapter-static';

/** @type {import('@sveltejs/kit').Config} */
export default {
	kit: {
		// A client-rendered single-page app: every route is prerendered to an empty shell, `npm run build` writes gcs/build, and
		// the helper (tools/gs/helper.py) serves that directory at / (it resolves /params to params.html).
		adapter: adapter({ pages: 'build', assets: 'build', fallback: undefined, strict: true }),
		alias: { $link: 'src/lib/link' }
	},
	compilerOptions: {
		// Force runes mode for the project, except for libraries (as the toolbox does).
		runes: ({ filename }) => (filename.split(/[/\\]/).includes('node_modules') ? undefined : true)
	}
};
