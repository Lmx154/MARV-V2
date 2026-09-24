#!/usr/bin/env node
// Writes tests/fixtures/rocket-airbrakes.json: the avionics toolbox's rocket-airbrakes preset (src/lib/sim/lab/presets.ts)
// run by the toolbox's own lab engine, with the controller swapped for apogee-pid at its defaults (the kind this flight
// software ports; the preset flies apogee-mpc). The toolbox's TypeScript is bundled in memory with its own esbuild and
// imported from a data: URL: nothing is written inside the toolbox.
//
//   node scripts/rocket-fixture.mjs [toolbox dir]        default ~/Projects/Web/avionics-toolbox
//
// One row per FSW tick (20 ms): the truth, the navigation state the toolbox's ESKF handed guidance and control, the
// sensor samples its estimator fused on that tick, the mission's fly and coast flags, guid/apogee-pred, fcs/brake-cmd and
// the brake state; and, on the truth instead of the estimate, the toolbox's own predictApogee and apogee-pid outputs.
import { execFileSync } from 'node:child_process';
import { mkdirSync, writeFileSync } from 'node:fs';
import { createRequire } from 'node:module';
import { homedir } from 'node:os';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const toolbox = resolve(process.argv[2] ?? join(homedir(), 'Projects/Web/avionics-toolbox'));
const out = join(dirname(fileURLToPath(import.meta.url)), '..', 'tests', 'fixtures', 'rocket-airbrakes.json');
const git = (...a) => execFileSync('git', ['-C', toolbox, ...a]).toString().trim();
const commit = git('rev-parse', '--short', 'HEAD');
const dirty = git('status', '--porcelain', '--', 'src') !== '';

const esbuild = createRequire(join(toolbox, 'package.json'))('esbuild');
const { outputFiles } = await esbuild.build({
	stdin: {
		contents: `export * from './index'; export { predictApogee } from './blocks/guidance'; export { RHO0 } from '../sixdof/plant';`,
		resolveDir: join(toolbox, 'src/lib/sim/lab'),
		loader: 'ts'
	},
	bundle: true,
	write: false,
	format: 'esm',
	platform: 'node',
	logLevel: 'error'
});
const tb = await import('data:text/javascript;base64,' + Buffer.from(outputFiles[0].contents).toString('base64'));

const config = tb.cloneConfig(tb.PRESETS['rocket-airbrakes'].config);
const pidDef = tb.blockDef('controller', 'apogee-pid');
config.blocks.controller = { kind: 'apogee-pid', params: tb.defaultParams(pidDef) };

// Taps, in this process only: what the controller was handed and commanded, and the bus the estimator fused, per tick.
const ticks = [];
const pidCreate = pidDef.create;
pidDef.create = (p, d) => {
	const c = pidCreate(p, d);
	return {
		...c,
		run: (t, ref, nav, bus, mode, dt) => {
			const req = c.run(t, ref, nav, bus, mode, dt);
			ticks.push({ t, mode, coast: ref.scalars.coast === 1 ? 1 : 0, nav: [...nav.p, ...nav.v, ...nav.q, ...nav.w], u: req.channels.brake });
			return req;
		}
	};
};
const samples = [];
const eskfDef = tb.blockDef('estimator', 'eskf');
const eskfCreate = eskfDef.create;
eskfDef.create = (p, d) => {
	const e = eskfCreate(p, d);
	return {
		...e,
		run: (n, t, bus) => {
			samples.push({ t, imu: [...bus.imu.am, ...bus.imu.wm], gnss: bus.gnss ? [...bus.gnss.p, ...bus.gnss.v] : null, baro: bus.baro });
			e.run(n, t, bus);
		}
	};
};

const run = tb.simulateLab(config);
const frames = run.frames;
if (ticks.length !== frames.length || samples.length < frames.length) throw new Error(`taps ${ticks.length}/${samples.length} vs ${frames.length} frames`);

// The toolbox's predictor and PI on the truth: the same model, parameters, coast flag and mode.
const gp = config.blocks.guidance.params;
const g = config.blocks.environment.params.g;
const model = { mass: gp.mass, cdA: gp.cdBase * gp.refArea, brakeCdA: gp.brakeCdArea, g, rho0: tb.RHO0, scaleHeight: 8500 };
const pidTruth = pidCreate(config.blocks.controller.params, { dt: config.dt, g, authority: {} });
const target = config.blocks.mission.params.targetApogee;

const columns = [
	't', 'fly', 'coast',
	'p_n', 'p_e', 'p_d', 'v_n', 'v_e', 'v_d', 'q_w', 'q_x', 'q_y', 'q_z', 'w_x', 'w_y', 'w_z',
	'nav_p_n', 'nav_p_e', 'nav_p_d', 'nav_v_n', 'nav_v_e', 'nav_v_d', 'nav_q_w', 'nav_q_x', 'nav_q_y', 'nav_q_z', 'nav_w_x', 'nav_w_y', 'nav_w_z',
	'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z',
	'gnss_p_n', 'gnss_p_e', 'gnss_p_d', 'gnss_v_n', 'gnss_v_e', 'gnss_v_d', 'baro_alt',
	'apogee_pred', 'brake_cmd', 'brake', 'apogee_pred_truth', 'brake_cmd_truth'
];
const rows = frames.map((f, k) => {
	const tk = ticks[k];
	const s = samples.find((x) => Math.abs(x.t - f.t) < 1e-9);
	if (Math.abs(tk.t - f.t) > 1e-9 || !s) throw new Error(`frame ${k} at ${f.t}: no tick or sample`);
	const a0t = tk.coast ? tb.predictApogee(-f.p[2], -f.v[2], Math.hypot(f.v[0], f.v[1]), 0, model, gp.predictDt) : NaN;
	const ut = pidTruth.run(f.t, { scalars: { coast: tk.coast, targetApogee: target, apogeeNoBrake: a0t } }, null, null, tk.mode, config.dt).channels.brake;
	return [
		f.t, tk.mode === 'fly' ? 1 : 0, tk.coast,
		...f.p, ...f.v, ...f.q, ...f.w,
		...tk.nav,
		...s.imu,
		...(s.gnss ?? [NaN, NaN, NaN, NaN, NaN, NaN]), s.baro ?? NaN,
		f.signals['guid/apogee-pred'], f.signals['fcs/brake-cmd'], f.brake, a0t, ut
	];
});
if (rows.some((r) => r.length !== columns.length)) throw new Error('row width');

const num = (x) => (Number.isFinite(x) ? String(Number(x.toPrecision(10))) : 'null');
const pick = (o, keys) => Object.fromEntries(keys.map((k) => [k, o[k]]));
const head = {
	source: `avionics-toolbox ${commit}${dirty ? ' (src modified)' : ''}: src/lib/sim/lab/presets.ts rocket-airbrakes, controller apogee-pid at its defaults; written by scripts/rocket-fixture.mjs`,
	toolbox_commit: commit,
	dt: config.dt,
	seed: config.seed,
	sensors: 'IMU every row: mean specific force and body rate (FRD) over the 20 ms up to t, with bias and noise; GNSS NED position and velocity (5 Hz) and barometric altitude (m up, 10 Hz) on the rows they were sampled, null otherwise; no magnetometer',
	model: { ...pick(gp, ['predictDt', 'cdBase', 'refArea', 'brakeCdArea', 'mass']), g, rho0: model.rho0, scaleHeight: model.scaleHeight, targetApogee: target, ...pick(config.blocks.controller.params, ['kp', 'ki', 'iLimit']) },
	sensor_model: config.blocks.sensors.params,
	summary: run.summary,
	columns
};
const text = JSON.stringify(head, null, 1).replace(/\n}$/, `,\n "frames": [\n${rows.map((r) => `  [${r.map(num).join(',')}]`).join(',\n')}\n ]\n}\n`);
mkdirSync(dirname(out), { recursive: true });
writeFileSync(out, text);
console.log(`${out}: ${rows.length} rows, ${(text.length / 1024).toFixed(0)} KiB, toolbox ${commit}${dirty ? ' (dirty)' : ''}; apogee ${run.summary.apogee.toFixed(2)} m`);
