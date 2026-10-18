# Flash panel, ground-segment unit 15 (decision 0053 Q6, G7)

Bench evidence, not a frozen test. Board: FC3v2 Rev-C, SD in, Debug Probe on SWD (unused here), ESC unpowered, never armed, no
motor command (`arming=no_link`). Date 2026-10-18. Branch `gs-flash`.

The GUI's flow (gcs/src/lib/state/flash_state.ts) is driven from Python by `flash_common.py` `Gcs.gui_flash` against the real helper
(tools/gs/helper.py on the link CDC, tools/gs/flash.py), the real validator, the real picotool (through `picotool_logged.sh`, which
logs each invocation to `raw/picotool_invocations.log`) and the real board. The playwright tools were not available to this run, so
the page itself was not clicked; its gate and flow are held by the vitest files `gcs/src/lib/flash/flash.test.ts` and
`gcs/src/lib/state/flash_state.test.ts`.

Images (ELFs not committed; hashes are, `tools/config/image_hash.py` = the board's boot hash):
- A: banner `quad-L2-pass-419-g4495b9c`, image hash `be23dcd2...a87f`, 214132 B.
- B: banner `quad-L2-pass-423-g67e1544` (the same code at a later commit: the git description differs), image hash `9dc7389e...9f12`.
Parameter schema `0x1648260490893e6e`; the board runs its factory table, config hash `5961b933...52dd`.

| Step (raw/) | Result |
| --- | --- |
| `look0` | before: image `e9e48456...b105`, factory, ok, disarmed |
| (a) `a1_validate_A` | Flash-page Validate for A with the config the board will run (factory): PASS, report `57212a23...49b7` for (A, `5961b933...`) |
| (a) `a2x_padding_refused` | first flash of A: the helper's UF2 check refused the picotool-converted UF2 (it holds the image plus zero padding to the 256-byte flash page); picotool `load` never ran; the board was left holding the maintenance grant (`arming=maintenance`). Fixed in tools/gs/flash.py (less than a flash page of zeros is allowed) with a unit test |
| (a) `a2_flash_A` | gate passes; grant ACCEPTED; `picotool uf2 convert` + `load -f -v -x`, status 0; port reopened; link identity read back: image `be23dcd2...` = A, config `5961b933...`, source factory (all equal to the plan); boot line image hash = host hash; est_valid=1. PASS |
| (b) `b1_refuse_B` | no report for B: `flash_plan` gate closed ("no validation report for image 9dc7389e1f6f with config 5961b93350a3"); a direct `flash` to the helper (no grant) refused "picotool was not started"; no flash state, device still open; `picotool_invocations.log` has no line for it; console uptime 47.121605 s -> 51.121705 s over 4.0 s of host time (no reboot); USB device number 106 and serial nodes (inode, ctime) unchanged. PASS |
| (c) `c1_validate_B` | PASS, report `a9062dc1...3711` for (B, `5961b933...`) |
| (c) `c2_flash_B` | as (a): grant ACCEPTED, picotool status 0, read-back image `9dc7389e...` = B, config and source as planned; boot line `image: sha256 9dc7389e...`, `config: source factory, status ok`. PASS |
| `final_look` | B (a normal flight image of the branch) on the board: factory, ok, disarmed, `arming=no_link`, est_valid=1, rate and attitude timing_faults 0, IMU faults 0, usb_reset_refused 0 |

The first picotool lines in `picotool_invocations.log` belong to the refused-padding run (convert only), then A (convert, load), then B
(convert, load); none for step (b).

Files: `bench_flash.py` (steps look, validate, flash, refuse), `flash_common.py`, `picotool_logged.sh`, `raw/` (JSON per step, console
and ground logs, `reports/<image>/<config>/{config.mcfg,report.json}`, the helper run directories).
