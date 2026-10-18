# 0053: Ground segment params-first design (architect ruling, 2026-10-17)

## What changed

Architect ruling for decision 0051's scope (full text: Serena memory `ground-segment/architect-params-first-2026-10-17`;
lead summary here). Items marked (Luis) were signed by him on 2026-10-17 (Approval).

1. **MAVLink parameters:** standard PARAM_REQUEST_LIST/READ/SET/VALUE, byte-wise encoding (REAL32 / INT32 bits);
   param_index = ParamId; param_id = a generated link id (the name if ≤ 16 chars, else name[:11] + "_" + 4 hex of
   SHA-256(name), optional `mav_id:` override; collisions refused). Provenance, lock and active/staged values in MARV
   dialect messages (MARV_PARAM_RECORD, MARV_PARAM_SOURCE); identity message with image, config and schema hashes.
2. **Store:** writes change a staged copy, only while disarmed; nothing is live (every parameter is read at init);
   activation (disarmed, USB) writes the staged config to SD, reboots, and the boot step verifies it and copies it into
   an internal-flash A/B slot outside every image before the IMU, receiver and motors start. The FC boots from flash and
   never needs the SD card to boot or arm. A table-parametric validator equals init's panics. Locked parameters keep
   their compiled values (list: Luis). Outside the validator by nature: imu_start's check of the tick period against the IMU's
   FREQ_FINE trim (lsm6dsv.cpp ~:351) depends on the chip, so it stays a boot-time panic.
   `tests/regression/quad/L06/tools/test_rate_lead_checks.py` adds `fw/params/include` to the include list of its
   firmware-law tool (the module headers now include param_types.hpp); nothing else changes. New arming refusals ConfigRefused and Maintenance; reboot / bootloader reset only
   with a disarmed maintenance grant (Luis). G7: arming refuses a stored config not bound to this image's validation
   report (check list: Luis).
   Unit 4a frozen tests extended: tests/regression/quad/L08/unit/arming_test.cpp gains the ArmState codes (ConfigRefused 16, Maintenance 17),
   ConfigRefused for every status with every other condition good, and the maintenance request armed (not granted) and disarmed
   (granted, persists until reset); tests/regression/quad/L09/unit/quad_fc_gate_test.cpp gains the same through the composition
   (request_maintenance, maintenance_granted, set_config_status). Negative controls run and restored: removing the armed check
   of the grant fails both request-while-armed tests, removing the ConfigRefused refusal fails both ConfigRefused tests.
   Frozen tests extended (unit 3, VALIDATE): tests/regression/quad/L09/unit/quad_fc_gate_test.cpp gains the validate cases
   (the default table accepted, one rejecting table per parameter-dependent panic site, init panicking with validate's reason).
   Unit 4b (STORE), the parameter-table link: the modules (attitude, mixer, rate, rate_group, arming, store) take their set's
   header compile-only and link no set's archives, so the m33 image and each SIL library link exactly one set (linking the
   store into the image had made the link resolve the product set's 104-record kParamDefaults under the FC3v2 runtime);
   a post-link check (cmake/param_table.cmake, tools/ci/check_param_table.py; planted control
   tests/regression/quad/L09/controls) fails the build unless the linked table is the set's. Frozen-test change, link line
   only: tests/regression/quad/L05/unit/angle/CMakeLists.txt and tests/regression/quad/L05/unit/attitude/CMakeLists.txt link
   marv_params_runtime themselves (they had it only through marv_attitude); their tests and the code they link are unchanged.
3. **Storage format:** binary `.mcfg` on the device, JSON as the host view (GD-3, Luis); FatFs (BSD-style) over SPI0 for
   the SD card; pins from the madflight page, UNVERIFIED until the mount test.
   Unit 2 (GEN) frozen-test changes: `tests/regression/quad/L00/tools/test_params_gen.py` (extended: the manifest
   entry assertion checks id, type and unit; new link-id, lock and manifest-field tests) and
   `tests/regression/quad/L01/tools/test_params_gen_l1.py` (three manifest comparisons now compare id, type, unit and
   link id, or ignore `source_file`, because `params_manifest.json` entries gain link_id, lock, default_bits, method,
   sigma_kind, sigma, sigma_bits, origin, source and source_file). Their requirements are unchanged: the schema hash and
   the header do not depend on value, sigma, lock or shape.
4. **Transport:** USB composite: CDC0 the existing console unchanged, CDC1 MAVLink; link serviced in the background loop
   below every interrupt; the tick publishes one telemetry snapshot through a seqlock. SIL speaks the same link over UDP
   127.0.0.1 via a new `marv_sil_link_service()`; runs without a GCS stay byte-identical.
5. **GCS:** `gcs/` in this repo (SvelteKit 2 / Svelte 5 / TS, toolbox components copied with their commit), a local
   helper `tools/gs/helper.py` (serial, SIL socket, ground log, picotool flashing, generators) behind a localhost
   WebSocket with a session token (GD-2, Luis); C++ codec generated by pymavlink mavgen (MIT output), TS codec by our
   own generator.
6. **Units (ordered):** 1 G0-IF (Luis signs GD-1), 2 GEN, 3 VALIDATE, 4 STORE, 5 CODEC, 6 LINK-SIL, 7 HELPER,
   8 GCS-PARAMS, 9 REPORT, 10 DERIVE, 11 PLOTS, 12 USB (bench), 13 SD (bench), 14 FLASH-SLOTS (bench), 15 FLASH-PANEL,
   16 END-BOOTSTRAP (after first flight or Luis's go). The M4 flight board stays on its compiled table until 9 and 14 pass.

## Why

Luis's requirement (decision 0051): MAVLink parameters stored on the SD card, "you may work it out as you see fit". The
SD card is kept out of the boot and arming path (a contact fault must not ground or silently reconfigure the vehicle);
runtime flash writes would trip the IMU deadline (single core, interrupts off during erase).

## Evidence

The architect's file:line index (memory above): quad_fc.cpp init :282-379; lsm6dsv.cpp deadline :14-19; params_manifest
(136 parameters, 62 names > 16 chars); hil_s1 throughput; m4-bench-card EMB-3.

## Approval

Design: architect (2026-10-17), accepted by the lead. Luis, 2026-10-17, all "Approve (Recommended)": the storage model;
the arming refusals and the disarmed-only maintenance grant; the locked list plus airmode_activate_throttle and the rotor
speed limits ("Approve, lock both too"); the G7 check list including the Gazebo run; GD-1, GD-2, GD-3, the two-port USB,
`gcs/` in this repo, and the drafted spec sentences (applied: core §4, §8, §10; ground §1, §3, §4, §5, §6).
