#!/usr/bin/env python3
import sys
import os
import time
import numpy as np
import mujoco
from mujoco.viewer import launch
from mujoco import mj_name2id, mjtObj

def main():
    """
    Run a control loop that applies a sinusoidal
    position command via a single position actuator.
    Prints the commanded and actual joint angles.
    """
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} path/to/model.xml [actuator_name] [joint_name]")
        sys.exit(1)

    xml_path = sys.argv[1]
    if not os.path.isfile(xml_path):
        print(f"Error: file '{xml_path}' not found")
        sys.exit(1)

    # Optionally set actuator/joint name from command-line
    actuator_name = sys.argv[2] if len(sys.argv) > 2 else "Revolute_6_act"
    joint_name    = sys.argv[3] if len(sys.argv) > 3 else "Revolute_6"

    # Load model and data
    model = mujoco.MjModel.from_xml_path(xml_path)
    data  = mujoco.MjData(model)

    # Find actuator
    act_id = mj_name2id(model, mjtObj.mjOBJ_ACTUATOR, actuator_name)
    if act_id < 0:
        print(f"[WARN] Actuator '{actuator_name}' not found in model. Disabling position control.")
        act_id = None
    else:
        lo, hi = model.actuator_ctrlrange[act_id]
        print(f"Found actuator '{actuator_name}' (id={act_id}), ctrl range: [{lo:.3f}, {hi:.3f}]")

    # Find joint
    j_idx = mj_name2id(model, mjtObj.mjOBJ_JOINT, joint_name)
    if j_idx < 0:
        print(f"[WARN] Joint '{joint_name}' not found in model. Disabling angle printing.")
        qpos_index = None
    else:
        qpos_index = model.jnt_qposadr[j_idx]
        print(f"Monitoring joint '{joint_name}' (id={j_idx}) at qpos index {qpos_index}")

    # Launch the viewer
    viewer = launch(model, data)

    # Sinusoidal control parameters
    amplitude = np.pi / 2   # peak (rad)
    frequency = 0.5         # Hz
    t0 = time.time()
    step = 0

    print(f"Running sinusoidal actuator test on model: {xml_path}")

    # Main control loop
    while viewer.is_running():
        t = time.time() - t0
        # Compute command
        if act_id is not None:
            desired_pos = amplitude * np.sin(2 * np.pi * frequency * t)
            data.ctrl[act_id] = desired_pos

        mujoco.mj_step(model, data)

        # Print debug info
        if act_id is not None and qpos_index is not None:
            actual_pos = data.qpos[qpos_index]
            print(f"Step {step}: cmd={desired_pos:.4f} rad, actual={actual_pos:.4f} rad")
        elif act_id is not None:
            print(f"Step {step}: cmd={desired_pos:.4f} rad")

        viewer.sync()
        step += 1

    viewer.close()

if __name__ == '__main__':
    main()
