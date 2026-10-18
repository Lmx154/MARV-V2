# Composite USB device, ground-segment unit 12a (decision 0053)

Bench evidence, not a frozen test. Board: FC3v2 Rev-C, SD card in, ESC unpowered, no motor command (the only console lines sent
were refused ones). Date 2026-10-18. Code: branch `gs-usb`, commit `2fff26a` (firmware). Flashed with `tools/target/flash.sh` through the
Debug Probe; every capture waited 12 s after a reboot (ModemManager).

## Files

- `boot_cycles.py` (N probe reboots, first console seconds of each), `echo_bench.py` + `analyze_echo.py` (link-port echo and the IMU
  spacing windows against it), `reset_refusal.py` (picotool and 1200-baud resets against the gate), `fields_diff.py` (console line layout
  against a reference), `usb_common.py`. The older `../sd/capture.py` is the console capture and was not changed.
- `raw/`: `lsusb_before.txt` / `lsusb_after.txt` (`lsusb -v`), `sysfs_after.txt` (interface strings and drivers), `before_m4.log` (the m4
  flight image on this board), `flight_final.log` (the gs-usb flight image), `cycles_*.txt` and `base_m4_*.log` / `usb_flight_*.log`
  (reboot cycles), `echo2.*` (echo run), `refusal2.*` (reset run), `console_input.txt` (console commands sent to if00).

## Results

| Done-when | Result |
| --- | --- |
| (a) two ACM ports | `lsusb -v -d 2e8a:0009` and sysfs after the change: bNumInterfaces 5; interface 0 `MARV console` (CDC comm, cdc_acm), 1 data, 2 `MARV link` (CDC comm, cdc_acm), 3 data, 4 `Reset` (vendor, class 0xff). Host paths `/dev/serial/by-id/usb-Raspberry_Pi_Pico_B2189353BDF6DD1E-if00` (console) and `-if02` (link). VID:PID 2e8a:0009, serial B2189353BDF6DD1E, bcdUSB 2.10: unchanged. Before: 3 interfaces, strings `Board CDC`, `Reset` (`lsusb_before.txt`, sysfs). |
| (b) console unchanged | `fields_diff.py` of `raw/flight_final.log` against `../sd/flight_boot1.log`: stats line 45 fields before, 46 after; added `usb_reset_refused` (appended at the end of the line, as the task asked for the refusal counter on the stats line); nothing removed; common fields in the same order; no other line template differs. `est_valid=1`, attitude `timing_faults=0`, boot line present (`IMU start at 9852..10120 us` after probe reboots). Console input unchanged: `bogus` and `motor_test 1 100 500` (no unlock token) are refused with the usual texts, banner reprinted on close/reopen (`console_input.txt`). Rate `timing_faults`: see the note below. |
| (c) echo | Bench image (`MARV_FC3V2_USB_BENCH=ON`, `MARV_FC3V2_BENCH_STATS=ON`, `build/m33-usb`): 1 MiB random through if02, echoed intact, sha256 equal (`205caa07...`), 2.62 s = 400.1 kB/s each way; 8 MiB sustained, sha256 equal, 21.0 s = 399.5 kB/s (`raw/echo2.txt`). 0 stale bytes before, 0 after. |
| (c) IMU spacing | Interrupt-entry spacing window max (`analyze_echo.py`, `raw/echo2.analysis.txt`): idle before 20123 cycles = 134.15 us (15 windows); during the echo 20127 cycles = 134.18 us (20 whole windows of the 8 MiB stream; the 1 MiB check fell in one); idle after 20118 = 134.12 us (9 windows). Window min 132.64..132.69 us in all. USBCTRL handler-chain window max: 3.95 us idle, 82.6 us during the traffic. IMU counters ovr/spi/partial/clamp/order/empty all 0; attitude timing_faults 0. |
| (d) resets refused | Flight image, `reset_refusal.py`: the `usb_reset_refused` counter went 2 -> 3 -> 4 on a 1200-baud line coding on if00 and on if02 (one each); the IMU time `t` rose monotonically across the run and the console port never went away (`raw/refusal2.json`). **picotool could not be exercised**: the device node `/dev/bus/usb/001/NNN` is `root:root 0664` (no udev rule for 2e8a:0009), so `picotool info/reboot` stop at "unable to connect ... check your permissions" before any request is sent; the vendor-interface refusal is therefore NOT bench-verified. It needs one run as root or with a udev rule, e.g. `sudo picotool reboot` then read `usb_reset_refused` (+1 expected, board keeps running). The vendor gate is the same `reset_granted()` test as the verified 1200-baud path. |
| (d) negative control | A throwaway build with the gate returning true (not committed; main.cpp restored): a 1200-baud open of if02 reset the board to BOOTSEL (`2e8a:000f Raspberry Pi RP2350 Boot` on the bus, the link port gone). The flight image was then flashed back through the probe. So the refusal above is the gate, not a dead path. |
| (e) HIL image | `cmake --preset m33 -B build/m33-hil -DMARV_FC3V2_HIL=ON -DMARV_FC3V2_HIL_RECEIVER=ON` builds; its ELF has the SDK's `tud_descriptor_string_cb`, no `link_usb_*`, no gated reset driver. Not flashed. |

## Notes

- Rate `timing_faults`: 1 appears in the first console line after a probe reboot in 5 of 8 cycles of the m4 flight image (`cycles_base.txt`) and
  5 of 8 of the gs-usb flight image (`cycles_new.txt`); attitude `timing_faults` is 0 in all 16. It is a boot-time count that does not grow
  (echo run: 1 at the first line, 1 at the last, 190 s apart). The m4 image has it as often, so it is not from this change; the power-on path
  (IMU start 306 us, README of `../sd`) was not re-run here (no way to power-cycle from the bench scripts).
- The echo image (`MARV_FC3V2_USB_BENCH`) does not reprint the banner when a console host connects to a running board; the flight image,
  the BENCH_STATS-only image and the m4 BENCH_STATS image do. Bench-only effect of the echo step running every loop pass; not
  investigated further. `analyze_echo.py` then takes clk_sys = 150 MHz (the flight banner's).

## Result: picotool vendor-reset refusal, 2026-10-18 (closes the open item)

Luis installed `/etc/udev/rules.d/60-marv-picotool.rules` (`SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", MODE="0660",
GROUP="plugdev"`), soldered the FC3v2 BAR_INT jumper (GPIO 23) and replugged the board; flight image from 3db706c.

| Command (picotool 2.x at ~/pico/picotool-install) | picotool says | Board afterwards | Log |
| --- | --- | --- | --- |
| before | | `usb_reset_refused=0`, est_valid=1 | `raw/picotool_before.log` |
| `picotool reboot` / `reboot -u` (no -f) | rc 249: no BOOTSEL device; suggests -f (no request sent) | unchanged | `raw/picotool_reboot*.txt` |
| `picotool reboot -f` | rc 0 "asked to reboot into application mode" | still running (same USB address 38, t = 35.06 s continuing), `usb_reset_refused=1` | `raw/picotool_reboot_f.txt`, `raw/picotool_after_f.log` |
| `picotool reboot -f -u` | rc 0 "asked to reboot into BOOTSEL mode" | still running (2e8a:0009, not BOOTSEL 000f; t = 46.06 s), `usb_reset_refused=2`, timing_faults 0 | `raw/picotool_reboot_fu.txt`, `raw/picotool_after_fu.log` |

picotool reports rc 0 because it only sends the request; the board stalls it without the maintenance grant and counts it.
The board also booted normally after the jumper was soldered (est_valid=1, timing_faults=0).
