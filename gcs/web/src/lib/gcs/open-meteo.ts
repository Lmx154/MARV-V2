/**
 * Current weather at a real place from Open-Meteo (https://open-meteo.com, CC BY 4.0), as the avionics toolbox fetches it
 * (avionics-toolbox src/lib/weather/open-meteo.ts): geocoding, the current surface readings with the hourly wind aloft,
 * and the sim environment derived from them.
 */
import type { SimEnv } from './types';

export type Fetch = (url: string) => Promise<Response>;

const defaultFetch: Fetch = (url) => globalThis.fetch(url);

export interface Place {
	name: string;
	admin1?: string;
	country: string;
	latitude: number;
	longitude: number;
	elevation: number;
}

export interface Weather {
	/** Local ISO time of the current reading, e.g. "2026-09-23T14:30". */
	time: string;
	timezone: string;
	/** m/s */
	windSpeed10: number;
	/** Direction the wind blows from, degrees clockwise from north. */
	windDir10: number;
	/** m/s */
	gust10: number;
	wind80?: number;
	wind120?: number;
	wind180?: number;
	/** °C */
	temperature2m: number;
	/** hPa */
	surfacePressure: number;
	/** m */
	elevation: number;
}

async function getJson(url: string, fetchFn: Fetch): Promise<unknown> {
	const res = await fetchFn(url);
	if (!res.ok) throw new Error(`Open-Meteo request failed: HTTP ${res.status}`);
	return res.json();
}

const num = (v: unknown): number | undefined => (typeof v === 'number' && Number.isFinite(v) ? v : undefined);

/** Up to five places matching `query`. */
export async function geocode(query: string, fetchFn: Fetch = defaultFetch): Promise<Place[]> {
	const q = query.trim();
	if (!q) return [];
	const url = `https://geocoding-api.open-meteo.com/v1/search?name=${encodeURIComponent(q)}&count=5&language=en&format=json`;
	const data = (await getJson(url, fetchFn)) as { results?: Record<string, unknown>[] };
	return (data.results ?? []).slice(0, 5).map((r) => ({
		name: String(r.name ?? ''),
		...(typeof r.admin1 === 'string' && r.admin1 ? { admin1: r.admin1 } : {}),
		country: String(r.country ?? r.country_code ?? ''),
		latitude: Number(r.latitude),
		longitude: Number(r.longitude),
		elevation: num(r.elevation) ?? NaN
	}));
}

/** Current surface weather at (lat, lon), with the hourly wind at 80, 120 and 180 m for the hour of the reading. */
export async function fetchWeather(lat: number, lon: number, fetchFn: Fetch = defaultFetch): Promise<Weather> {
	const url =
		`https://api.open-meteo.com/v1/forecast?latitude=${lat}&longitude=${lon}` +
		'&current=temperature_2m,surface_pressure,wind_speed_10m,wind_direction_10m,wind_gusts_10m' +
		'&hourly=wind_speed_80m,wind_speed_120m,wind_speed_180m&wind_speed_unit=ms&forecast_days=1&timezone=auto';
	const data = (await getJson(url, fetchFn)) as {
		timezone?: string;
		elevation?: number;
		current?: Record<string, unknown>;
		hourly?: { time?: string[] } & Record<string, unknown>;
	};
	const c = data.current;
	if (!c || typeof c.time !== 'string') throw new Error('Open-Meteo response has no current weather');
	const hour = `${c.time.slice(0, 13)}:00`;
	const i = data.hourly?.time?.indexOf(hour) ?? -1;
	const aloft = (key: string): number | undefined => {
		const series = data.hourly?.[key];
		return i >= 0 && Array.isArray(series) ? num(series[i]) : undefined;
	};
	const w: Weather = {
		time: c.time,
		timezone: data.timezone ?? '',
		windSpeed10: num(c.wind_speed_10m) ?? NaN,
		windDir10: num(c.wind_direction_10m) ?? NaN,
		gust10: num(c.wind_gusts_10m) ?? NaN,
		temperature2m: num(c.temperature_2m) ?? NaN,
		surfacePressure: num(c.surface_pressure) ?? NaN,
		elevation: num(data.elevation) ?? NaN
	};
	const w80 = aloft('wind_speed_80m');
	const w120 = aloft('wind_speed_120m');
	const w180 = aloft('wind_speed_180m');
	if (w80 !== undefined) w.wind80 = w80;
	if (w120 !== undefined) w.wind120 = w120;
	if (w180 !== undefined) w.wind180 = w180;
	return w;
}

export const placeLabel = (p: Place): string => [p.name, p.admin1, p.country].filter(Boolean).join(', ');

const round = (v: number, step: number): number => Number((Math.round(v / step) * step).toFixed(String(step).split('.')[1]?.length ?? 0));

/**
 * The sim environment from a reading at a place: the 10 m wind and the direction it blows FROM (Open-Meteo's
 * convention, kept); the gust sigma from the gust factor, a 10-min peak gust being about mean + 3 sigma; the place's
 * coordinates, Open-Meteo's elevation (else the place's); temperature, and pressure in Pa. Missing readings leave the
 * field out.
 */
export function simEnvFromWeather(w: Weather, place: Place): { env: Partial<SimEnv>; provenance: string } {
	const env: Partial<SimEnv> = { lat: place.latitude, lon: place.longitude };
	if (Number.isFinite(w.windSpeed10)) env.wind_speed_ms = round(w.windSpeed10, 0.1);
	if (Number.isFinite(w.windDir10)) env.wind_dir_deg = round(((w.windDir10 % 360) + 360) % 360, 1) % 360;
	if (Number.isFinite(w.windSpeed10)) env.gust_sigma_ms = round(Math.max(0, (Number.isFinite(w.gust10) ? w.gust10 : w.windSpeed10) - w.windSpeed10) / 3, 0.01);
	const elevation = Number.isFinite(w.elevation) ? w.elevation : place.elevation;
	if (Number.isFinite(elevation)) env.elevation_m = elevation;
	if (Number.isFinite(w.temperature2m)) env.temperature_c = w.temperature2m;
	if (Number.isFinite(w.surfacePressure)) env.pressure_pa = Math.round(w.surfacePressure * 100);
	const time = `${w.time.replace('T', ' ')}${w.timezone ? ` ${w.timezone}` : ''}`;
	return { env, provenance: `Open-Meteo, ${placeLabel(place)}, ${time}` };
}
