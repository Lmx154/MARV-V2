// Records block-level golden vectors of the toolbox's WGS 84 normal gravity (core section 9).
//
// Usage: node tools/golden/gravity_vectors.mjs > tests/regression/quad/L01/unit/prim/gravity_golden.csv
// Environment: TOOLBOX_DIR = avionics-toolbox checkout (default ~/Projects/Web/avionics-toolbox).
//
// The toolbox source (src/lib/physics/gravity.ts, TypeScript, extensionless imports) is bundled in memory with the
// toolbox's own esbuild and imported unchanged; nothing in the toolbox is modified or copied. The output header
// records the toolbox commit and the node version. Latitude is in degrees at the toolbox interface; the column
// phi_rad is exactly the radian value the toolbox computes internally, (deg * Math.PI) / 180, printed with 17
// significant digits (round-trips a double).
import { execFileSync } from 'node:child_process';
import { createRequire } from 'node:module';
import { homedir } from 'node:os';
import { join } from 'node:path';

const toolbox = process.env.TOOLBOX_DIR ?? join(homedir(), 'Projects/Web/avionics-toolbox');
const { buildSync } = createRequire(join(toolbox, 'package.json'))('esbuild');

// Stated grid. Latitude: both poles, equator and every 15 degrees between; height: sea level to 50 km.
const latDeg = [-90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90];
const heightM = [0, 100, 1000, 5000, 10000, 50000];

const out = buildSync({
	stdin: {
		contents: "export { somigliana, normalGravity } from './gravity';",
		resolveDir: join(toolbox, 'src/lib/physics'),
		loader: 'ts'
	},
	bundle: true,
	format: 'esm',
	write: false
});
const source = out.outputFiles[0].text;
const { somigliana, normalGravity } = await import(
	'data:text/javascript;base64,' + Buffer.from(source).toString('base64')
);

const commit = execFileSync('git', ['-C', toolbox, 'rev-parse', 'HEAD']).toString().trim();
const lines = [
	'# Golden vectors: toolbox src/lib/physics/gravity.ts, somigliana() and normalGravity(). Produced by tools/golden/gravity_vectors.mjs.',
	`# toolbox commit ${commit}; node ${process.version}`,
	`# grid: lat_deg in {${latDeg.join(',')}}; h_m in {${heightM.join(',')}}`,
	'lat_deg,phi_rad,h_m,gamma_ellipsoid,gamma_h'
];
for (const lat of latDeg) {
	for (const h of heightM) {
		const phi = (lat * Math.PI) / 180;
		lines.push([lat, phi, h, somigliana(lat), normalGravity(lat, h)].map((v) => v.toPrecision(17)).join(','));
	}
}
process.stdout.write(lines.join('\n') + '\n');
