# MARV

Quadrotor flight software for a Pico 2 / RP2350 (target board: madflight FC3, RP2350B), developed
against Gazebo. Modelled on the flight lab of `~/Projects/Web/avionics-toolbox`: swappable blocks
behind fixed contracts.

## Who owns what
- **Gazebo** (`sitl/gazebo/world.sdf.in`, `sitl/airframes/<id>/`): vehicle, environment, sensors, motor physics.
- **Firmware** (`fsw/`, run by `firmware/` on the Pico or in-process by the bridge): estimator,
  guidance, controller, allocation, actuator commands. It *knows of* the vehicle and environment
  (the setup: `fsw/include/marv/fsw/params.def` is the one source of every parameter) but never simulates them.
- **Mission software** (`ground/`): what to do — manual sticks, GPS setpoints, presets.
- **Bridge** (`bridge/`): steps Gazebo 4 ms at a time, runs the flight software on every 1 ms step, and moves frames between it, the
  flight controller (`--sitl` in-process, or `--port` to the Pico) and the ground software.

## Contracts
`fsw/include/marv/fsw/contracts.hpp` (types, NED/FRD, Hamilton body→NED) and
`link/include/marv/link/protocol.hpp` (the wire) are the only interfaces between blocks. Use the
existing type for a concept; do not add a second one. Change them deliberately, never as a side effect.

## Rules
- Change only what the task asks. Note anything else you find; do not fix it.
- `fsw/` is MCU code: float only, no heap, no exceptions, no virtuals.
- Every module gets a truth-referenced test. A new estimator runs in shadow against truth before it
  flies; the handover needs a stated metric.
- Numbers come from a source (datasheet in `docs/datasheets/`, the SDF, a measurement), cited in a comment.
- No flight state machine until it is asked for.

## Verify
```
cmake -S . -B build/native -G Ninja -DCMAKE_BUILD_TYPE=Debug && cmake --build build/native   # 0 warnings (-Werror)
ctest --test-dir build/native --output-on-failure
PICO_SDK_PATH=$HOME/pico/pico-sdk cmake -S firmware -B build/fw -G Ninja -DCMAKE_BUILD_TYPE=Release && cmake --build build/fw
scripts/sim.sh --sitl --mission missions/square.txt --seconds 33 --log out.csv               # SITL
scripts/sim.sh --world x500 --sitl --mission ...   # PX4's x500 (sitl/airframes/x500); its setup: setups/x500.json
scripts/sim.sh --port /dev/serial/by-id/usb-MARV_MARV_flight_controller_* --mission ...      # on the Pico
```
Fly by hand or by GPS point (a second terminal, while `scripts/sim.sh --sitl --ground --seconds 600` runs):
`build/native/ground/marv_ground manual` (RadioMaster if plugged in, else the Xbox pad), `goto LAT LON ALT`,
`waypoints FILE`, `preset N`, `reboot`. Presets are listed in `fsw/include/marv/fsw/presets.hpp`.
Ground control GUI (http://127.0.0.1:8765/): `scripts/gcs.sh --udp 127.0.0.1:14650` (not under flock); its Development tab
launches the sim (an airframe of `sitl/airframes`, wind/gusts/location, SITL or the Pico, the Gazebo window) holding the rig lock.
On the Pico with no sim: `scripts/gcs.sh --serial /dev/serial/by-id/usb-MARV_MARV_flight_controller_*` (its Flash button takes the lock).
Flash over SWD (debug probe): `~/pico/openocd-install/bin/openocd -s ~/pico/openocd-install/share/openocd/scripts -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" -c "program build/fw/marv_fw.elf verify reset exit"`.
One Gazebo server and one Pico: when agents run in parallel, wrap sim/Pico use in `flock /tmp/marv-rig.lock`.
