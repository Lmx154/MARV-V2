#!/usr/bin/env node
// SITL end to end of the GCS mission executor (ADR-0010 (e)): Gazebo and marv_bridge (scripts/sim.sh --sitl --ground
// --seconds 400 --log), marv_gcs on the bridge's UDP link (scripts/gcs.sh), driven over the GCS WebSocket as the web
// Mission view drives it. Truth comes from the bridge's --log, read while it is written. Not in ctest (needs Gazebo).
//
//   node scripts/gcs-mission-e2e.mjs [--out DIR] [--http PORT]     takes /tmp/marv-rig.lock itself
//
//   1. arm: for 5 s telemetry.armed, every motor == spin_arm +- 1e-6, truth_d_m > -0.05 (no liftoff).
//   2. climb 5: hold within 30 s; truth_d over 2..5 s of hold at -5 +- 0.3.
//   3. a 3-waypoint mission, 12 m legs at 6..8 m above home: at each wp_index advance, truth within 2.0 m
//      horizontal / 0.5 m vertical of the waypoint; then the automatic rth to hold within 2.0 m of home.
//   4. a second mission, rth after its first waypoint: hold within 2.0 m of home.
//   5. land: disarmed ("landed") within 60 s, truth_d_m > -0.1, every motor 0.
// Waypoints are placed in the truth frame (NED about the vehicle's spawn point, the world origin of
// sitl/gazebo/world.sdf.in) and sent as lat/lon about that origin. Exit 0 when every check passes.
import { spawn, spawnSync } from 'node:child_process';
import { closeSync, existsSync, mkdirSync, openSync, readFileSync, readSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const LOCK = '/tmp/marv-rig.lock';
const args = process.argv.slice(2);
const opt = (name, def) => {
	const i = args.indexOf(name);
	return i >= 0 && i + 1 < args.length ? args[i + 1] : def;
};
if (process.env.MARV_E2E_LOCKED !== '1') {
	const r = spawnSync('flock', [LOCK, process.execPath, fileURLToPath(import.meta.url), ...args], {
		stdio: 'inherit',
		env: { ...process.env, MARV_E2E_LOCKED: '1' }
	});
	process.exit(r.status ?? 1);
}

const out = resolve(opt('--out', join(root, 'build', 'e2e-mission')));
const port = Number(opt('--http', '8779'));
mkdirSync(out, { recursive: true });
const logPath = join(out, 'bridge.csv');

// ---- the world origin and a flat earth about it (fsw/include/marv/fsw/geo.hpp, in double) -----------------------------
const sdf = readFileSync(join(root, 'sitl/gazebo/world.sdf.in'), 'utf8');
const tag = (t) => Number(new RegExp(`<${t}>([^<]+)</${t}>`).exec(sdf)[1]);
const LAT0 = tag('latitude_deg'), LON0 = tag('longitude_deg'), ALT0 = tag('elevation');
const toGeo = (() => {
	const A = 6378137, E2 = 6.69437999014e-3, rad = Math.PI / 180;
	const s = Math.sin(LAT0 * rad), d = 1 - E2 * s * s;
	const mN = (A * (1 - E2) / (d * Math.sqrt(d)) + ALT0) * rad; // m per degree
	const mE = (A / Math.sqrt(d) + ALT0) * Math.cos(LAT0 * rad) * rad;
	return (n, e) => ({ lat: LAT0 + n / mN, lon: LON0 + e / mE });
})();

// ---- results -------------------------------------------------------------------------------------------------------
const results = [];
const f = (x, n = 3) => (x === undefined || x === null || Number.isNaN(x) ? String(x) : x.toFixed(n));
function check(name, ok, detail) {
	results.push({ name, ok });
	console.log(`${ok ? 'PASS' : 'FAIL'} ${name}: ${detail}`);
	return ok;
}
class Abort extends Error {}

// ---- processes ---------------------------------------------------------------------------------------------------------
const procs = [];
function start(cmd, argv, name, env = process.env) {
	const fd = openSync(join(out, `${name}.out`), 'w');
	const p = spawn(cmd, argv, { cwd: root, detached: true, stdio: ['ignore', fd, fd], env });
	closeSync(fd);
	p.exited = new Promise((r) => p.on('exit', (code, sig) => r({ code, sig })));
	p.name = name;
	procs.push(p);
	return p;
}
async function stopAll() {
	for (const p of procs.reverse()) {
		if (p.exitCode !== null || p.signalCode !== null) continue;
		try {
			process.kill(-p.pid, 'SIGINT');
		} catch {}
		const done = await Promise.race([p.exited, sleep(15000).then(() => null)]);
		if (!done) {
			try {
				process.kill(-p.pid, 'SIGKILL');
			} catch {}
			await p.exited;
		}
	}
}
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// ---- the bridge log, read while it grows ----------------------------------------------------------------------------
const truth = { t: [], n: [], e: [], d: [], m: [[], [], [], []] };
let csvFd = -1, csvRest = '', cols = null;
function readCsv() {
	if (csvFd < 0) {
		if (!existsSync(logPath)) return;
		csvFd = openSync(logPath, 'r');
	}
	const buf = Buffer.alloc(1 << 20);
	for (;;) {
		const n = readSync(csvFd, buf, 0, buf.length, null);
		if (n <= 0) break;
		const lines = (csvRest + buf.toString('utf8', 0, n)).split('\n');
		csvRest = lines.pop();
		for (const line of lines) {
			const c = line.split(',');
			if (!cols) {
				cols = Object.fromEntries(c.map((k, i) => [k, i]));
				continue;
			}
			truth.t.push(Number(c[cols.t_us]));
			truth.n.push(Number(c[cols.truth_n_m]));
			truth.e.push(Number(c[cols.truth_e_m]));
			truth.d.push(Number(c[cols.truth_d_m]));
			for (let k = 0; k < 4; ++k) truth.m[k].push(Number(c[cols[`motor${k}`]]));
		}
	}
}
// Index of the last row at or before t_us.
function at(t_us) {
	let lo = 0, hi = truth.t.length - 1;
	if (hi < 0 || truth.t[0] > t_us) return -1;
	while (lo < hi) {
		const mid = (lo + hi + 1) >> 1;
		if (truth.t[mid] <= t_us) lo = mid;
		else hi = mid - 1;
	}
	return lo;
}
async function truthUntil(t_us, timeout = 5000) {
	const end = Date.now() + timeout;
	for (;;) {
		readCsv();
		const n = truth.t.length;
		if (n && truth.t[n - 1] >= t_us) return;
		if (Date.now() > end) throw new Abort(`bridge log did not reach t_us ${t_us}`);
		await sleep(20);
	}
}
// Rows with t in [t0, t1].
function rows(t0, t1) {
	const r = [];
	for (let i = Math.max(0, at(t0)); i < truth.t.length && truth.t[i] <= t1; ++i) if (truth.t[i] >= t0) r.push(i);
	return r;
}

// ---- the GCS WebSocket -----------------------------------------------------------------------------------------------
let ws = null, tlm = null, mission = null, setup = null;
const states = []; // every mission_state, with the telemetry t_us current when it arrived
const errors = [];
async function connect() {
	const end = Date.now() + 300000; // gcs.sh may build the web first
	for (;;) {
		try {
			ws = await new Promise((res, rej) => {
				const s = new WebSocket(`ws://127.0.0.1:${port}/ws`);
				s.onopen = () => res(s);
				s.onerror = () => rej(new Error('ws'));
			});
			break;
		} catch {
			if (Date.now() > end) throw new Abort('no GCS WebSocket within 300 s');
			await sleep(500);
		}
	}
	ws.onmessage = (ev) => {
		const m = JSON.parse(ev.data);
		if (m.type === 'telemetry') tlm = m;
		else if (m.type === 'mission_state') {
			const prev = mission;
			mission = m;
			if (!prev || prev.state !== m.state || prev.wp_index !== m.wp_index || prev.reason !== m.reason)
				states.push({ ...m, t_us: tlm ? tlm.t_us : 0, at: Date.now() });
		} else if (m.type === 'setup' && m.values) setup = m;
		else if (m.type === 'error') errors.push(m);
	};
}
const send = (m) => ws.send(JSON.stringify(m));
async function until(pred, timeout, what) {
	const end = Date.now() + timeout;
	for (;;) {
		readCsv();
		const v = pred();
		if (v) return v;
		const gone = procs.find((p) => p.exitCode !== null || p.signalCode !== null);
		if (gone) throw new Abort(`${gone.name} exited (${gone.exitCode ?? gone.signalCode}) while waiting for ${what}; see ${out}`);
		if (Date.now() > end) throw new Abort(`timeout (${timeout / 1000} s) waiting for ${what}`);
		await sleep(10);
	}
}
// Sends a mission command; a refusal aborts.
async function command(m) {
	const n = errors.length;
	send(m);
	await sleep(150);
	if (errors.length > n) throw new Abort(`${m.type} refused: ${errors[errors.length - 1].error}`);
}
const nextState = (from, pred, timeout, what) => until(() => states.slice(from).find(pred), timeout, what);

// ---- the run -------------------------------------------------------------------------------------------------------
const WPS = [
	{ n: 12, e: 0, alt: 6 },
	{ n: 12, e: 12, alt: 8 },
	{ n: 0, e: 12, alt: 7 }
];
const wire = (w) => ({ ...toGeo(w.n, w.e), alt_m: w.alt });

async function run() {
	// The GCS first: gcs.sh may build the web before it serves, and the simulated seconds run in wall time.
	// Its own single-instance lock (MARV_GCS_LOCK, for tests), so it runs beside the user's marv_gcs.
	start(join(root, 'scripts/gcs.sh'), ['--udp', '127.0.0.1:14650', '--http', String(port)], 'gcs', {
		...process.env,
		MARV_GCS_LOCK: join(out, 'marv-gcs.lock')
	});
	await connect();
	start(join(root, 'scripts/sim.sh'), ['--sitl', '--ground', '--seconds', '400', '--log', logPath], 'sim');

	// The setup: factory 0 stored, and spin_arm of the running actuators kind.
	const schema = await (await fetch(`http://127.0.0.1:${port}/api/schema`)).json();
	await until(() => tlm, 60000, 'telemetry');
	send({ type: 'request_setup' });
	await until(() => setup, 10000, 'setup');
	const fac = schema.factory[0];
	const same = setup.values.every((v, i) => v === fac.values[i]) && fac.kinds.every((k, i) => k === setup.header.kind[i]);
	check('setup is factory 0', same && setup.header.running_crc === setup.header.staged_crc,
		`values and kinds ${same ? '==' : '!='} factory 0 (${fac.label}); running_crc ${setup.header.running_crc} staged_crc ${setup.header.staged_crc}`);
	const fa = schema.families.findIndex((x) => x.id === 'actuators');
	const spinIdx = schema.families[fa].kinds[setup.header.kind[fa]].params.find((p) => p.id === 'spin_arm').index;
	const spin = setup.values[spinIdx];
	console.log(`spin_arm = ${spin} (param ${spinIdx})`);

	await until(() => tlm.est.valid && tlm.home_valid && tlm.geo, 90000, 'a valid estimate and home');
	await sleep(2000);

	// 1. arm.
	let k = states.length;
	await command({ type: 'arm' });
	await nextState(k, (s) => s.state === 'armed', 2000, 'armed');
	const spinning = () => tlm.armed && tlm.motor.every((m) => Math.abs(m - spin) <= 1e-6);
	await until(spinning, 2000, 'motors at spin_arm');
	const tA = tlm.t_us;
	let tlmBad = 0, tlmN = 0, lastT = 0;
	await until(() => {
		if (tlm.t_us !== lastT) {
			lastT = tlm.t_us;
			++tlmN;
			if (!spinning()) ++tlmBad;
		}
		return tlm.t_us >= tA + 5e6;
	}, 10000, 'armed 5 s');
	await truthUntil(tA + 5e6);
	const armRows = rows(tA, tA + 5e6);
	let dMin = Infinity, mErr = 0;
	for (const i of armRows) {
		dMin = Math.min(dMin, truth.d[i]);
		for (let m = 0; m < 4; ++m) mErr = Math.max(mErr, Math.abs(truth.m[m][i] - spin));
	}
	check('arm 5 s', tlmBad === 0 && mErr <= 1e-6 && dMin > -0.05 && armRows.length > 4900,
		`${tlmN} telemetry messages, ${tlmBad} not armed at spin_arm; ${armRows.length} log rows, max |motor - spin_arm| ${mErr.toExponential(2)}, min truth_d ${f(dMin, 4)} m`);

	// 2. climb 5 -> hold.
	k = states.length;
	const tC = Date.now();
	await command({ type: 'climb', alt_m: 5 });
	const hold = await nextState(k, (s) => s.state === 'hold', 30000, 'hold after climb 5');
	const climbS = (hold.at - tC) / 1000;
	await until(() => tlm.t_us >= hold.t_us + 5e6, 10000, 'hold 5 s');
	await truthUntil(hold.t_us + 5e6);
	const hr = rows(hold.t_us + 2e6, hold.t_us + 5e6);
	const hd = hr.map((i) => truth.d[i]);
	const hMin = Math.min(...hd), hMax = Math.max(...hd);
	check('climb 5 -> hold', climbS <= 30 && hMin >= -5.3 && hMax <= -4.7 && hold.reason === 'altitude reached',
		`hold after ${f(climbS, 1)} s ("${hold.reason}"), truth_d at entry ${f(truth.d[at(hold.t_us)])}, over 2..5 s of hold ${f(hMin)}..${f(hMax)} m`);

	// 3. the mission, then the automatic rth to hold over home.
	const home = { n: truth.n[at(tA)], e: truth.e[at(tA)] };
	console.log(`home (truth at arm) n ${f(home.n)} e ${f(home.e)}; waypoints ${JSON.stringify(WPS.map(wire))}`);
	k = states.length;
	await command({ type: 'mission_start', waypoints: WPS.map(wire) });
	for (let w = 0; w < WPS.length; ++w) {
		const adv = await nextState(k, (s) => (w + 1 < WPS.length ? s.state === 'mission' && s.wp_index === w + 1 : s.state === 'rth'),
			90000, `advance past waypoint ${w}`);
		await truthUntil(adv.t_us);
		const i = at(adv.t_us);
		const dh = Math.hypot(truth.n[i] - WPS[w].n, truth.e[i] - WPS[w].e), dv = Math.abs(truth.d[i] + WPS[w].alt);
		check(`waypoint ${w}`, dh <= 2.0 && dv <= 0.5,
			`advance at t ${f(adv.t_us / 1e6, 2)} s: truth miss ${f(dh)} m horizontal, ${f(dv)} m vertical (${adv.state}${adv.reason ? ' "' + adv.reason + '"' : ''})`);
	}
	const rthHold = await nextState(k, (s) => s.state === 'hold', 90000, 'hold after the automatic rth');
	await until(() => tlm.t_us >= rthHold.t_us + 2e6, 10000, 'hold 2 s');
	await truthUntil(rthHold.t_us + 2e6);
	let i = at(rthHold.t_us + 2e6);
	let dHome = Math.hypot(truth.n[i] - home.n, truth.e[i] - home.e);
	check('automatic rth -> hold over home', rthHold.reason === 'home reached' && dHome <= 2.0,
		`hold ("${rthHold.reason}"), truth 2 s later ${f(dHome)} m from home, truth_d ${f(truth.d[i])} m`);

	// 4. a second mission, rth after its first waypoint.
	k = states.length;
	await command({ type: 'mission_start', waypoints: WPS.map(wire) });
	await nextState(k, (s) => s.state === 'mission' && s.wp_index === 1, 90000, 'second mission past waypoint 0');
	await sleep(1500);
	const altNow = -tlm.est.p_ned[2];
	k = states.length;
	await command({ type: 'rth' });
	const rth2 = await nextState(k, (s) => s.state === 'rth', 2000, 'rth');
	const rthAlt = rth2.target ? rth2.target.alt_m : NaN;
	const hold2 = await nextState(k, (s) => s.state === 'hold', 90000, 'hold after the rth mid-mission');
	await until(() => tlm.t_us >= hold2.t_us + 2e6, 10000, 'hold 2 s');
	await truthUntil(hold2.t_us + 2e6);
	i = at(hold2.t_us + 2e6);
	dHome = Math.hypot(truth.n[i] - home.n, truth.e[i] - home.e);
	check('rth mid-mission -> hold over home', hold2.reason === 'home reached' && dHome <= 2.0 &&
		Math.abs(rthAlt - Math.max(altNow, 5)) < 0.3,
		`rth_alt ${f(rthAlt, 2)} m (alt at the command ${f(altNow, 2)}, climb 5), hold ("${hold2.reason}"), truth 2 s later ${f(dHome)} m from home`);

	// 5. land.
	k = states.length;
	const tL = Date.now();
	await command({ type: 'land' });
	const landed = await nextState(k, (s) => s.state === 'disarmed', 60000, 'disarmed after land');
	const landS = (landed.at - tL) / 1000;
	await until(() => !tlm.armed && tlm.motor.every((m) => m === 0), 3000, 'motors 0');
	const tZ = tlm.t_us;
	await truthUntil(tZ + 1e6);
	i = at(landed.t_us);
	const zr = rows(tZ, tZ + 1e6);
	const mMax = Math.max(...zr.map((j) => Math.max(truth.m[0][j], truth.m[1][j], truth.m[2][j], truth.m[3][j])));
	check('land', landed.reason === 'landed' && landS <= 60 && truth.d[i] > -0.1 && mMax === 0,
		`disarmed ("${landed.reason}") ${f(landS, 1)} s after land, truth_d ${f(truth.d[i], 4)} m, telemetry motors 0 at t ${f(tZ / 1e6, 2)} s, max log motor over the next 1 s ${mMax}`);
	await sleep(1500);
	check('disarmed 1.5 s after landing', mission.state === 'disarmed' && !tlm.armed, `mission_state ${mission.state}, telemetry.armed ${tlm.armed}, refusals ${errors.length}`);
}

let rc = 1;
try {
	await run();
	rc = results.every((r) => r.ok) ? 0 : 1;
} catch (e) {
	check('run', false, e instanceof Abort ? e.message : e.stack);
} finally {
	if (ws) ws.close();
	await stopAll();
}
console.log(`${results.filter((r) => r.ok).length}/${results.length} PASS; logs in ${out}`);
process.exit(rc);
