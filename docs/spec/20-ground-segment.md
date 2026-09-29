# MARV V2 — Ground segment

**Status: PARKED.** Read for context; do not implement from it until Luis marks it ACTIVE in the core's table.
Depends on `00-core-contracts.md`, which owns the functional interface (core §8) and the parameter interface (core
§4). Split 1, 2026-09-28.

**Activation triggers:**

- **G0–G1** (interface schema and codec) activate when the quad spine reaches L9, because quad L10 (HIL) carries IF-9
  over the codec.
- **G2 onward** activate when quad branch B2 (mission) needs missions, telemetry and the GCS.

**Until G3 passes,** parameters reach the firmware through the bootstrap struct generated from the card (core §4, the
declared exception to G2).

---

## 0. What the ground segment is

**The pipeline in one line:** vehicle ⇄ serial link ⇄ **ground control application (GCS)** ⇄ vehicle.

- **The link** is USB on the bench, or a UART to a radio module that bridges to a matching module at the ground
  station. The flight controller sees a byte stream either way and knows nothing about the radio.
- **The GCS** is the toolbox's Lab UI, extended. In it the operator:
  - chooses the GNC architecture (one kind per family, parameters generated from physics);
  - builds the same C++ for the PC and the RP2354B;
  - proves it (SIL, RocketPy, Gazebo, HIL), producing a report bound to the image and config hashes;
  - flashes a UF2 only when the structure changes. Everything else is **one universal config file**, edited as text
    and uploaded to or downloaded from the FC (SD card or internal flash) without reflashing;
  - commands missions, watches telemetry, and configures hardware, firmware and software;
  - flies Freestyle from a radio handset plugged into the desktop (controller passthrough).
- **The same GCS drives the simulators** exactly as it drives the vehicle.
- **Drone or rocket is a flash, not a new config.** Both images read the same config file. Flash the rocket image to
  fly, flash the drone image back, and the drone's settings are where they were.

```
  MARV V2 vehicle ⇄ serial link (USB, or UART ⇄ radio module ⇄ radio module) ⇄ GCS
        ▲                                                          │  architecture · flash · config files · missions ·
        └──────────────────────────────────────────────────────────┘  telemetry · configuration · controller passthrough

  SIL / Gazebo / HIL: the same GCS and the same codec, over a socket (SIL, Gazebo) or USB (HIL)
```

The radio modules bridge bytes to each other; whatever encoding they use between themselves is theirs.

---

## 1. From the Lab to the device: configure → prove → flash

The Flight Lab is the architecture reference. It has:

- **nine block families**, in a fixed order inside one flight-software tick: mission, vehicle, environment, sensors,
  estimator, guidance, controller, allocation, actuators;
- **replaceable kinds per family** (e.g. estimator = `eskf` / `ekf` / `ukf` / `complementary` / `mahony`);
- **numeric `ParamSpec`s that carry a `source`**;
- a **`compatible()` check** across families;
- **one `LabConfig`** that fully defines a run (`src/lib/sim/lab/types.ts`, `registry.ts`).

MARV V2 is that, in C++, with the flight-software side flashed to the RP2354B.

**Which families end up where:**

| Lab family | On the device (flashed) | In simulation only |
| --- | --- | --- |
| mission | the mode and phase state machine | scenario scripts |
| estimator, guidance, controller, allocation | the chosen kinds, compiled in | the same C++ build, in SIL |
| sensors | drivers for the parts present, and their profiles | `marv_plant`'s generic sensor models, driven by the same profiles |
| actuators | output drivers (DShot, servo) | `marv_plant`'s motor / servo dynamics |
| vehicle | what the flight software *knows* about the airframe, from the card and self-calibration | the plant's *truth*: the same card plus Monte Carlo dispersions (core §6) |
| environment | the site model | the true atmosphere, wind and turbulence |

**The pipeline:**

1. **Sources.** Vehicle cards, sensor profiles, site data, the budget register. Nothing else feeds a number in.
2. **Configure** in the GCS configurator, or a config file with the same schema. Pick one kind per family, the mission
   and the rates. **Parameters are generated, not typed:** gains by loop shaping on the card's plant; noise and priors
   from the sensor profile; gates from χ² quantiles; limits from the requirement rules; later, values from
   self-calibration. The output has two parts:
   - the **structure**: the vehicle definition, its localization stack, its controller and guidance kinds. All
     compiled in;
   - the **config file**: parameters with provenance, rates, missions and settings.
3. **Build.** A generator turns the structure into a composition header; the same sources build for the host (SIL)
   and for the RP2354B as a **UF2**. The UF2 carries no tunable numbers; the firmware checks the config file against
   its compiled schema when it loads it.
4. **Prove.** T1–T6 as the product specs define them. The result is a **validation report** bound to (image hash,
   config hash, firmware commit, card hashes), listing every requirement with its derived threshold, measured value
   and margin. The config hash covers only the sections the image reads.
5. **Load.** Two separate paths:
   - **Flash** a UF2 from the GCS over USB, **only when the structure changes**. The GCS keeps a library of built,
     validated images.
   - **Config file:** saved in the GCS, edited as text, uploaded or downloaded over USB, over the radio (drone image
     only, disarmed), or by moving the SD card. Activation copies it into a region of internal flash no image covers,
     so flashing never touches it, and the FC boots from that copy (GS-11).

   The GCS refuses to flash or activate anything whose (image, config) pair has no passing report. The FC reports both
   hashes at boot and in every log.
6. **Fly, log, replay.** Logs replay through the same plant; the card and requirements update from them.

**The config file is universal.** One schema, generated from the whole registry, covers every vehicle and kind:

- **`common`**: board-level items the same on any airframe (link ports, logging, telemetry rates, board sensor
  calibration);
- **`quad`**: mounting and outputs, the parameters of its estimator, controller and guidance, failsafes, missions;
- **`rocket`**: mounting and the airbrake output, the parameters of its estimator, predictor and MPC, the target
  apogee, and the recovery events.

Each image checks and reads its sections and carries the others through byte for byte, including when it writes a
parameter back. Editing the drone's settings never invalidates the rocket's report, and the other way round.

**Decided:** controller and guidance are compiled into the UF2, never selected by the config file.

---

## 2. The link

The flight controller knows its **ports** (USB, the radio UART) but not what is behind them.

```
                      ┌── USB CDC (bench: flash, configure, calibrate, full-rate telemetry, logs) ──┐
  MARV V2 vehicle  ⇄  │                                                                              │  ⇄  GCS
                      └── UART ⇄ radio module · · · radio module ⇄ serial (field) ───────────────────┘
          SIL / Gazebo: the same GCS and the same codec over a socket
```

- **The functional interface, its layering and the per-image handler table are in core §8.** This spec implements the
  codec, the transports and the GCS side.
- **Replacing MAVLink later** means writing a new codec and passing the same codec tests. Nothing above the codec
  changes.
- **The flight controller assumes nothing about link capacity.** The drone's stream rates are set by the GCS per
  connection; the rocket's come from its config file. The drone detects link loss from missing traffic, with a timeout
  derived from the configured heartbeat rate and a false-alarm budget.

---

## 3. The GCS: the toolbox's Lab UI, extended

It runs locally, as its own build of the toolbox's UI, not as the public site.

**Keep and reuse:** `BlockChain`, `BlockCard`, `ParamSlider` (the configurator, now fed by the **registry manifest
exported by the C++ build**: families, kinds, `ParamSpec`s with sources, `compatible()`, qualification levels; the
GCS keeps no block list of its own); `CompareTable`, `Diagnosis`; `ViewTriptych`, `VehicleView`, `LayerToggles`,
`SignalPlot`, `Scrubber` (live telemetry and log replay); the `ui` primitives.

**Add:**

- a connection manager: serial port or socket, link health (sequence gaps, round-trip time);
- stream-rate configuration per connection;
- a mission planner (drone): map; waypoints, routes, RTL, geofence; checks against the vehicle's derived limits (QA-1,
  QO-2, QR-1) before a transactional upload;
- a flash panel: build the UF2 when the structure changes; the image library; validation-report gate; hash read-back;
- a config manager: library of saved config files, a text editor with schema checking, upload, download, activate;
  the rocket's target apogee set here and checked against RK-2 before upload;
- configuration pages: hardware, firmware parameters with provenance and locks, the GCS's own settings;
- a calibration wizard (S0–S2, each result with σ and acceptance);
- controller passthrough: a radio plugged in over USB as a joystick (e.g. a RadioMaster Pocket in USB-joystick mode,
  through the browser's Gamepad API), forwarded to the vehicle in Freestyle only (IF-8). It replaces the quad spine's
  joystick bridge behind the same manual-control struct;
- log download and replay into SIL;
- a validation-report viewer.

**Remove from the GCS build:** the site-only parts (ads, affiliate disclosure, support button, SEO) and teaching pages
the operator does not need.

**Device access from the browser.** Web Serial works for the USB port and the ground radio module in Chromium-based
browsers. Flashing the RP2354B needs USB access to its bootloader or a small local helper (GD-2).

---

## 4. Requirements (GS)

| ID | Requirement | Rule or test |
| --- | --- | --- |
| GS-1 | One functional interface end to end. | The same typed messages at the vehicle, the GCS and the simulators; transport adapters and one codec only. The GCS cannot tell SIL from the vehicle; it is tested against the simulators first. |
| GS-2 | Functions before encoding. | The IF table (core §8) is signed off before the codec mapping is finalized. |
| GS-3 | Codec isolation. | CI fails if any module outside the codec includes a MAVLink header or refers to a MAVLink message. A second, trivial codec (for tests) passes the same conformance suite. |
| GS-4 | Flash only on the USB port, only disarmed; config activation only disarmed. | Enforced on the vehicle by port and arming state, not merely hidden in the GCS. Writing flash stalls the other core, so internal-flash writes happen only disarmed. |
| GS-5 | Authority (drone). | Manual control (IF-8) accepted only in Freestyle. Mode changes are explicit, acknowledged commands. While armed, parameter writes outside an explicit in-flight allow-list are rejected. |
| GS-6 | Link loss (drone). | Detected from missing traffic; timeout from the configured heartbeat rate and a false-alarm budget; triggers a QX failsafe. |
| GS-7 | Telemetry fits the connection. | The GCS measures each connection's throughput and refuses a stream configuration that exceeds it. |
| GS-8 | The ground log is replayable. | Everything sent and received is logged with timestamps and can be replayed against a SIL run. |
| GS-9 | No remote control of the rocket. | The rocket image compiles no IF-3, IF-4, IF-5 or IF-8 handler, and its radio port is downlink only. No image has a fire command on any port. CI checks the image's link map for those handlers. A SIL test sends every uplink message to the rocket's radio port and finds its state, outputs and configuration unchanged. |
| GS-10 | One config file for every image. | Both images load the same file; each writes back only its own sections, preserves the others byte for byte, and reports a config hash over its own sections. Test: flash the rocket image, change the target apogee, flash the drone image: the drone's sections and hash are unchanged. |
| GS-11 | The config file cannot fail silently. | Each section carries a schema version; an image refuses a version it does not know; the GCS migrates files forward with migrations generated from registry changes (a new parameter takes its value from its rule, with provenance). Activation writes into one of two internal-flash slots, verifies by SHA-256 read-back (the RP2350's hardware SHA-256, datasheet §12.13), then switches; the other slot stays as the last good file. The FC always boots from internal flash. If the active slot fails its hash at boot, the FC refuses to arm and says why; returning to the last good file is an explicit, disarmed activation. |

---

## 5. Plan

| Phase | Work | Exit test |
| --- | --- | --- |
| G0 Functional interface | Complete and sign off the IF table (core §8): every GCS function, its data, its semantics. The schema that generates the C++ and TypeScript types. | Every GCS function maps to interface messages; Luis signs off. No encoding decisions yet. |
| G1 Codec and transports | MAVLink v2 codec plus the MARV dialect; transports for USB CDC, the radio UART and the SIL socket. | Round-trip conformance tests pass; GS-3 passes; the trivial test codec passes the same suite. Quad L10 can carry IF-9. |
| G2 GCS over SIL | Connection manager, telemetry, mission planner and controller passthrough, driving Gazebo SIL through the socket. | A waypoint mission is planned, checked, uploaded, read back and flown in SIL entirely from the GCS. |
| G3 Configurator, flash and config files | Configurator rendered from the registry manifest; UF2 build and flash over USB; config-file library, editor, upload, download and activation (IF-11). | An unvalidated pair is refused. A validated UF2 flashes and reports its hash. A config file round-trips through SD and internal flash unchanged and survives a flash. GS-10 passes. **Ends the core §4 bootstrap exception.** |
| G4 Link robustness | Stream rates per connection; link degradation emulated in SIL (throughput limit, loss, latency, values measured on the real link); link-loss failsafe. | GS-5 to GS-7 and GS-9 pass in SIL, then on the bench over the radio modules. |
| G5 Configuration and bench calibration | Hardware, firmware and parameter pages; motor tests; S1–S2 self-calibration from the GCS. | Every parameter shows its provenance; locked parameters cannot be overwritten; GS-4 and GS-5 rejections verified on the vehicle. |

---

## 6. Decisions needed (ground)

| ID | Decision |
| --- | --- |
| GD-1 | Sign-off of the functional interface (core §8) before the codec mapping is finalized. |
| GD-2 | Flashing from the GCS: browser USB access to the RP2354B bootloader, or a small local helper. |
| GD-3 | Config file format. JSON with a schema generated from the registry is proposed (small parser on the RP2354B, native to the Lab UI); TOML is friendlier to hand-edit. |
