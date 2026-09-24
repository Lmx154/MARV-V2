import { describe, expect, it } from 'vitest';
import { fetchWeather, geocode, simEnvFromWeather, type Fetch, type Place, type Weather } from './open-meteo';
import { windToward } from './sim';

const reply =
	(body: unknown, status = 200): Fetch =>
	async () =>
		new Response(JSON.stringify(body), { status });

const zurich: Place = { name: 'Zürich', admin1: 'Zurich', country: 'Switzerland', latitude: 47.36667, longitude: 8.55, elevation: 408 };

describe('open-meteo', () => {
	it('asks the toolbox URLs and maps the geocoding results', async () => {
		const urls: string[] = [];
		const places = await geocode(' Zürich ', async (url) => {
			urls.push(url);
			return new Response(JSON.stringify({ results: [{ name: 'Zürich', admin1: 'Zurich', country: 'Switzerland', latitude: 47.36667, longitude: 8.55, elevation: 408 }] }));
		});
		expect(urls[0]).toBe('https://geocoding-api.open-meteo.com/v1/search?name=Z%C3%BCrich&count=5&language=en&format=json');
		expect(places).toEqual([zurich]);
		expect(await geocode('   ', reply({}))).toEqual([]);
	});

	it('reads the current reading and the wind aloft for its hour', async () => {
		const urls: string[] = [];
		const body = {
			timezone: 'Europe/Zurich',
			elevation: 411,
			current: { time: '2026-09-24T14:45', temperature_2m: 18.4, surface_pressure: 963.2, wind_speed_10m: 4.2, wind_direction_10m: 250, wind_gusts_10m: 7.8 },
			hourly: { time: ['2026-09-24T13:00', '2026-09-24T14:00'], wind_speed_80m: [5, 6.1], wind_speed_120m: [6, 7], wind_speed_180m: [null, null] }
		};
		const w = await fetchWeather(47.37, 8.55, async (url) => {
			urls.push(url);
			return new Response(JSON.stringify(body));
		});
		expect(urls[0]).toBe(
			'https://api.open-meteo.com/v1/forecast?latitude=47.37&longitude=8.55&current=temperature_2m,surface_pressure,wind_speed_10m,wind_direction_10m,wind_gusts_10m&hourly=wind_speed_80m,wind_speed_120m,wind_speed_180m&wind_speed_unit=ms&forecast_days=1&timezone=auto'
		);
		expect(w).toMatchObject({ time: '2026-09-24T14:45', timezone: 'Europe/Zurich', windSpeed10: 4.2, windDir10: 250, gust10: 7.8, temperature2m: 18.4, surfacePressure: 963.2, elevation: 411, wind80: 6.1, wind120: 7 });
		expect(w.wind180).toBeUndefined();
	});

	it('fails on an HTTP error, a reply without current weather, and a network error', async () => {
		await expect(fetchWeather(0, 0, reply({}, 502))).rejects.toThrow('HTTP 502');
		await expect(fetchWeather(0, 0, reply({ hourly: {} }))).rejects.toThrow('no current weather');
		await expect(
			geocode('x', async () => {
				throw new TypeError('Failed to fetch');
			})
		).rejects.toThrow('Failed to fetch');
	});

	const weather = (over: Partial<Weather> = {}): Weather => ({
		time: '2026-09-24T14:45',
		timezone: 'Europe/Zurich',
		windSpeed10: 4.2,
		windDir10: 250,
		gust10: 7.8,
		temperature2m: 18.4,
		surfacePressure: 963.2,
		elevation: 411,
		...over
	});

	it('maps a reading onto the sim environment with its provenance', () => {
		const { env, provenance } = simEnvFromWeather(weather(), zurich);
		expect(env).toEqual({ lat: 47.36667, lon: 8.55, wind_speed_ms: 4.2, wind_dir_deg: 250, gust_sigma_ms: 1.2, elevation_m: 411, temperature_c: 18.4, pressure_pa: 96320 });
		expect(provenance).toBe('Open-Meteo, Zürich, Zurich, Switzerland, 2026-09-24 14:45 Europe/Zurich');
	});

	it('keeps the direction the wind blows FROM, within [0, 360)', () => {
		expect(simEnvFromWeather(weather({ windDir10: 270 }), zurich).env.wind_dir_deg).toBe(270);
		expect(simEnvFromWeather(weather({ windDir10: 360 }), zurich).env.wind_dir_deg).toBe(0);
		expect(simEnvFromWeather(weather({ windDir10: -90 }), zurich).env.wind_dir_deg).toBe(270);
		// A westerly (from 270) moves the air east; a northerly (from 0) moves it south.
		const w = windToward(5, 270);
		expect(w.n).toBeCloseTo(0, 9);
		expect(w.e).toBeCloseTo(5, 9);
		expect(windToward(5, 0).n).toBeCloseTo(-5, 9);
	});

	it('gusts below the mean give no sigma; missing readings leave fields out', () => {
		expect(simEnvFromWeather(weather({ gust10: 3 }), zurich).env.gust_sigma_ms).toBe(0);
		const { env } = simEnvFromWeather(weather({ temperature2m: NaN, surfacePressure: NaN, elevation: NaN }), zurich);
		expect(env.temperature_c).toBeUndefined();
		expect(env.pressure_pa).toBeUndefined();
		expect(env.elevation_m).toBe(408);
	});
});
