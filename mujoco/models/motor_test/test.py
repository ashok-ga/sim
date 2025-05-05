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
    run_model.py

    Load the MJCF model and run a control loop that applies PD via actuators,
    prints contact sensor reading, and prints the angle of 'revolute_2'.
    Allows full rotation by expanding actuator ctrlrange.
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
    data = mujoco.MjData(model)

    # Identify actuators for PD control
    try:
        iP = mj_name2id(model, mjtObj.mjOBJ_ACTUATOR, "rev2_P")
        iD = mj_name2id(model, mjtObj.mjOBJ_ACTUATOR, "rev2_D")
    except Exception:
        print("Actuators 'rev2_P' or 'rev2_D' not found. PD control disabled.")
        iP = iD = None

    # Expand actuator range to full rotation (±2π)
    if iP is not None:
        model.actuator_ctrlrange[iP, :] = np.array([-2*np.pi, 2*np.pi])
    if iD is not None:
        model.actuator_ctrlrange[iD, :] = np.array([-2*np.pi, 2*np.pi])

    # Identify contact sensor
    try:
        s_idx = mj_name2id(model, mjtObj.mjOBJ_SENSOR, "shaft_holder_contact")
    except Exception:
        print("Sensor 'shaft_holder_contact' not found. Contact printing disabled.")
        s_idx = None

    # Identify joint for angle printing
    try:
        j_idx = mj_name2id(model, mjtObj.mjOBJ_JOINT, "revolute_2")
        qpos_index = model.jnt_qposadr[j_idx]
    except Exception:
        print("Joint 'revolute_2' not found. Angle printing disabled.")
        qpos_index = None

    # Launch viewer
    viewer = launch(model, data)

    # PD control parameters
    amplitude = 3.14  # rad
    frequency = 1.0   # Hz
    t0 = time.time()
    step = 0
    print(f"Running simulation for model: {xml_path}")
    
    while viewer.is_running():
        # Compute time
        t = time.time() - t0

        # Apply PD via actuators
        if iP is not None and iD is not None:
            pos_des = amplitude * np.sin(2 * np.pi * frequency * t)
            vel_des = amplitude * 2 * np.pi * frequency * np.cos(2 * np.pi * frequency * t)
            data.ctrl[iP] = pos_des
            data.ctrl[iD] = vel_des

        # Step simulation
        mujoco.mj_step(model, data)

        # Print contact sensor force
        if s_idx is not None:
            contact_force = data.sensordata[s_idx]
            print(f"Step {step}: Contact force = {contact_force:.3f}")

        # Print joint angle
        if qpos_index is not None:
            angle = data.qpos[qpos_index]
            print(f"Step {step}: revolute_2 angle = {angle:.4f} rad")

        # Render
        viewer.sync()
        step += 1

    viewer.close()


if __name__ == '__main__':
    main()

