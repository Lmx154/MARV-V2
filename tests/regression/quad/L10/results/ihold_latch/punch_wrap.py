import sys
sys.path.insert(0, "tools/sim")
import run_quad_fc as r
PUNCH_STICK = 0.30  # scenario value, above airmode_activate_throttle 0.29296875
orig = r.build_phases_ihold
def build(hover, dip_stick):
    ph = orig(hover, dip_stick)
    out = []
    for p in ph:
        if p.name == "hold":
            half = r.us(r.IHOLD_HOLD_S) // 2
            out.append(r.dataclasses.replace(p, duration_us=half))
            out.append(r.dataclasses.replace(p, name="punch", start_us=p.start_us + half, duration_us=r.us(0.05), throttle=PUNCH_STICK) if hasattr(p, "throttle") else None)
            out.append(r.dataclasses.replace(p, name="hold_b", start_us=p.start_us + half + r.us(0.05), duration_us=p.duration_us - half - r.us(0.05)))
        else:
            out.append(p)
    return out
r.build_phases_ihold = build
sys.exit(r.main(sys.argv[1:]))
