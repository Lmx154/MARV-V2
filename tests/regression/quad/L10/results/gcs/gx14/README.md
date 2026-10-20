# GX-14 BUILD: evidence

- `light/`: the Playwright JSON report and screenshots of the light e2e (`tools/gs/e2e.sh`, all specs; `gcs/e2e/build.spec.ts` is the Build panel's, with the fake cmake and ninja of `tools/gs/e2e_fake_build.py`).
- `real_build.py`: drives the real helper with the commands the Build panel sends, for one real host-gz-quadfc build under the heavy lock; its outputs (status before and after, events, the library entry with its describe string, the helper's build log) are in this directory when the run has been committed.
