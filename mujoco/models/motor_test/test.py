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
    motor_test_sine.py

    Load the MJCF model and run a control loop that applies a sinusoidal
    position command via a single position actuator, prints debug info,
    and prints the angle of 'revolute_2'.
    """
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} path/to/model.xml")
        sys.exit(1)

    xml_path = sys.argv[1]
    if not os.path.isfile(xml_path):
        print(f"Error: file '{xml_path}' not found")
        sys.exit(1)

    # Load model and data
    model = mujoco.MjModel.from_xml_path(xml_path)
    data  = mujoco.MjData(model)

    # Identify the position actuator
    act_name = "revolute_2_act"
    act_id = mj_name2id(model, mjtObj.mjOBJ_ACTUATOR, act_name)
    if act_id < 0:
        print(f"Actuator '{act_name}' not found. Position control disabled.")
        act_id = None
    else:
        lo, hi = model.actuator_ctrlrange[act_id]
        print(f"Found actuator '{act_name}' (id={act_id}), ctrl range: [{lo:.3f}, {hi:.3f}]")

    # Identify joint for angle printing
    j_idx = mj_name2id(model, mjtObj.mjOBJ_JOINT, "revolute_2")
    if j_idx < 0:
        print("Joint 'revolute_2' not found. Angle printing disabled.")
        qpos_index = None
    else:
        qpos_index = model.jnt_qposadr[j_idx]
        print(f"Monitoring joint 'revolute_2' (id={j_idx}) at qpos index {qpos_index}")

    # Launch the viewer
    viewer = launch(model, data)

    # Sinusoidal command parameters
    amplitude = np.pi / 2   # peak position (rad)
    frequency = 0.5         # Hz
    t0 = time.time()
    step = 0
    print(f"Running sinusoidal test on model: {xml_path}")

    # Main loop
    while viewer.is_running():
        # Compute time
        t = time.time() - t0

        # Apply desired position command
        if act_id is not None:
            desired_pos = amplitude * np.sin(2 * np.pi * frequency * t)
            data.ctrl[act_id] = desired_pos

        # Step simulation (integrates data.qpos)
        mujoco.mj_step(model, data)

        # Read back actual position and print
        if act_id is not None and qpos_index is not None:
            actual_pos = data.qpos[qpos_index]
            print(f"Step {step}: cmd={desired_pos:.4f} rad, actual={actual_pos:.4f} rad")

        # Render
        viewer.sync()
        step += 1

    viewer.close()


if __name__ == '__main__':
    main()
