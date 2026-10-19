"""The display-only additions of a Gazebo world and the gz GUI client, shared by the runs that can show the quad: the HIL S4b run
(tools/sim/run_hil_s4.py --gui) and the interactive SIL (tools/sim/run_quad_fc.py --interactive --gui, ground-segment unit GX-3).
Moved out of run_hil_s4.py unchanged (the HIL run's behaviour is the same); nothing here changes the plant.
"""

from __future__ import annotations

import math
import re
import subprocess
import threading
from pathlib import Path

import run_scenario

# --gui (S4b, Luis flying, 2026-10-18: "it's tough for me to do anything when i can't see the gazebo 3d env"): display-only
# additions to the world and a gz GUI client on the server's partition. Nothing here has mass, inertia or collision, so the
# plant is unchanged; every size below is a labelled display value, not vehicle data. The rotor discs sit at the card's rotor
# positions (FRD in the plugin block, drawn in the link's FLU frame: y and z negated).
DISPLAY_PROP_RADIUS_M = 0.0889  # display only: a 7 inch propeller disc (Luis, 2026-10-18: "it's a 7 inch frame")
DISPLAY_BODY_M = (0.10, 0.05, 0.03)  # display only: the body box
DISPLAY_MARKER_SPACING_M = 2.0  # display only: reference posts every 2 m so motion is visible against the ground
DISPLAY_MARKER_COUNT = 5  # display only: posts per side of the grid
DISPLAY_GUI_START_S = 15.0  # harness value: the GUI loads its rendering engine before the real-time run starts
GUI_STOP_WAIT_S = 10.0  # harness value: how long the GUI client gets after SIGTERM before SIGKILL (run_hil_s4 PORT_WAIT_S, the value this replaces)
DISPLAY_SUN = "<light type='directional' name='sun'><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>" \
    "<diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.5 0.1 -0.9</direction></light>"


def display_world(text):
    """The world with display-only visuals (quad body, arms, rotor discs, a red nose, ground plane and reference posts),
    the scene broadcaster a GUI client needs, and a light."""
    rotors = re.findall(r'<rotor motor="(\d)">\s*<position_frd_m unit="m">([-\d.e]+) ([-\d.e]+) ([-\d.e]+)</position_frd_m>', text)
    assert len(rotors) == 4, rotors
    bx, by, bz = DISPLAY_BODY_M
    # The ground visual sits at the ground height (<ground/>: the body's start height, the model pose's z) less half the body box,
    # so the quad at rest there visibly stands on it (derived display value).
    start_z = float(re.search(r'<model name="[^"]+">\s*<pose[^>]*>\s*\S+\s+\S+\s+(\S+)', text).group(1))
    ground_z = start_z - bz / 2
    vis = [f"<visual name='body'><geometry><box><size>{bx} {by} {bz}</size></box></geometry>"
           "<material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>",
           f"<visual name='nose'><pose>{bx / 2} 0 0 0 0 0</pose><geometry><box><size>{bz} {bz} {bz}</size></box></geometry>"
           "<material><ambient>0.8 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual>"]
    for motor, x, y, z in rotors:
        fx, fy, fz = float(x), -float(y), -float(z)
        arm = math.hypot(fx, fy)
        yaw = math.atan2(fy, fx)
        colour = "0.1 0.4 1 0.6" if float(x) > 0 else "0.9 0.9 0.9 0.6"  # front rotors blue, rear white
        vis.append(f"<visual name='arm{motor}'><pose>{fx / 2} {fy / 2} 0 0 0 {yaw}</pose><geometry><box><size>{arm} 0.012 0.008"
                   "</size></box></geometry><material><diffuse>0.3 0.3 0.3 1</diffuse></material></visual>")
        vis.append(f"<visual name='rotor{motor}'><pose>{fx} {fy} {fz} 0 0 0</pose><geometry><cylinder><radius>"
                   f"{DISPLAY_PROP_RADIUS_M}</radius><length>0.004</length></cylinder></geometry><material><ambient>{colour}"
                   f"</ambient><diffuse>{colour}</diffuse></material><transparency>0.4</transparency></visual>")
    text = text.replace("</inertial>", "</inertial>\n" + "\n".join(vis), 1)
    posts = []
    half = DISPLAY_MARKER_COUNT // 2
    for i in range(-half, half + 1):
        for j in range(-half, half + 1):
            if i == 0 and j == 0:
                continue
            px, py = i * DISPLAY_MARKER_SPACING_M, j * DISPLAY_MARKER_SPACING_M
            posts.append(f"<visual name='post_{i}_{j}'><pose>{px} {py} 0.5 0 0 0</pose><geometry><cylinder><radius>0.03</radius>"
                         "<length>1.0</length></cylinder></geometry><material><diffuse>1 0.6 0 1</diffuse></material></visual>")
    ground = ("<model name='display_ground'><static>true</static><pose>0 0 " f"{ground_z}" " 0 0 0</pose>"
              "<link name='ground_link'><visual name='ground'><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane>"
              "</geometry><material><ambient>0.3 0.45 0.3 1</ambient><diffuse>0.35 0.55 0.35 1</diffuse></material></visual>"
              + "".join(posts) + "</link></model>")
    extra = ("<plugin filename='gz-sim-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>\n"
             + DISPLAY_SUN + "\n" + ground + "\n")
    i = text.index("<model name=")
    return text[:i] + extra + text[i:]


def unpause(world_name, env, who):
    """Start the paused world (the --gui run): gz-sim's world control service, pause false."""
    req = ["gz", "service", "--force-version", "13", "-s", f"/world/{world_name}/control", "--reqtype", "gz.msgs.WorldControl",
           "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", "pause: false"]
    r = subprocess.run(req, env=env, capture_output=True, text=True)
    print(f"{who}: world unpaused ({r.stdout.strip() or r.stderr.strip()})", flush=True)


def start_gui(world_path, log_path, env, who):
    """The --gui run's GUI client and the timer that unpauses the paused world once the GUI holds the scene; returns (client
    process, timer). The caller starts the server paused (no -r) and calls stop_gui when the server has ended."""
    world_name = re.search(r'<world name="([^"]+)"', Path(world_path).read_text(encoding="utf-8")).group(1)
    gui_log = open(log_path, "w", encoding="utf-8")
    gui_p = subprocess.Popen(["nice", "-n", "19", *run_scenario.GZ_ARGS, "-g"], env=env, stdout=gui_log, stderr=subprocess.STDOUT)
    print(f"{who}: gz GUI starting; the world is paused and starts in {DISPLAY_GUI_START_S:.0f} s", flush=True)
    timer = threading.Timer(DISPLAY_GUI_START_S, unpause, args=(world_name, env, who))
    timer.start()
    return gui_p, timer


def stop_gui(gui_p, timer):
    if timer is not None:
        timer.cancel()
    if gui_p is not None:
        gui_p.terminate()
        try:
            gui_p.wait(timeout=GUI_STOP_WAIT_S)
        except subprocess.TimeoutExpired:
            gui_p.kill()
