"""Headless check of egg_scene.xml: grip force must settle at the actuator force cap.
Usage: python check_scene.py egg_scene.xml"""
import sys
import numpy as np
import mujoco


def grip_force(m, d, egg, pads, f6):
    """Largest normal force (N) that either finger pad puts on the egg."""
    per_pad = dict.fromkeys(pads, 0.0)
    for i in range(d.ncon):
        c = d.contact[i]
        pair = {int(c.geom1), int(c.geom2)}
        if egg in pair:
            other = (pair - {egg}).pop()
            if other in per_pad:
                mujoco.mj_contactForce(m, d, i, f6)
                per_pad[other] += f6[0]
    return max(per_pad.values())


def run(path, cap):
    m = mujoco.MjModel.from_xml_path(path)
    d = mujoco.MjData(m)
    m.actuator_forcerange[:] = [[-cap, cap]] * m.nu
    gid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
    egg, pads = gid("egg"), [gid("pad_l"), gid("pad_r")]
    egg_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "egg")
    f6, peak = np.zeros(6), 0.0

    def phase(seconds, z=None, ctrl=None):
        nonlocal peak
        n = int(seconds / m.opt.timestep)
        z0, c0 = d.mocap_pos[0, 2], d.ctrl[0]
        for k in range(n):
            a = (k + 1) / n
            if z is not None:
                d.mocap_pos[0, 2] = z0 + a * (z - z0)
            if ctrl is not None:
                d.ctrl[:] = c0 + a * (ctrl - c0)
            mujoco.mj_step(m, d)
            peak = max(peak, grip_force(m, d, egg, pads, f6))

    phase(0.5)                    # settle
    z_rest = d.xpos[egg_body][2]
    phase(1.5, z=0.475)           # lower the gripper over the egg
    phase(0.5, ctrl=0.04)         # full pinch
    phase(1.0)                    # hold
    phase(1.0, z=0.600)           # lift 125 mm
    phase(0.5)
    lifted_mm = (d.xpos[egg_body][2] - z_rest) * 1000
    print(f"cap {cap:5.1f} N  peak grip {peak:5.1f} N  egg lifted {lifted_mm:6.1f} mm")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("scene", nargs="?", default="egg_scene.xml", help="MJCF scene file (default egg_scene.xml)")
    for cap in (60.0, 30.0, 5.0, 2.0):
        run(ap.parse_args().scene, cap)
