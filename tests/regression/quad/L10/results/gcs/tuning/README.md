# Tuning page: light e2e evidence

`light/` is the Playwright report and screenshots of `tools/gs/e2e.sh -- radio.spec.ts` against the fake vehicle (the Tuning flow is the
last test of `gcs/e2e/radio.spec.ts`: edit `rate_kp_roll`, write, read the staged value back). `light-connection/` is
`-- connection.spec.ts` (the navigation lists the Tuning page). The fake vehicle answers config commands "unavailable", so activation (the
new current value) is not exercised here; the page shows the vehicle's staged value read back.
