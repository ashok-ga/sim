#!/usr/bin/env python3
"""
motor_viewer.py

Loads test.xml and opens a MuJoCo viewer, running the simulation with no control commands.
"""

import os
import time
import mujoco
import mujoco.viewer

def main():
    # Locate files
    here     = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(here, 'test.xml')

    # Load model & data
    model = mujoco.MjModel.from_xml_path(xml_path)
    data  = mujoco.MjData(model)

    print("Loaded model. Number of joints:", model.njnt)
    print("Joints:")
    for i in range(model.njnt):
        print(f"  {i}: {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)} (type={model.jnt_type[i]})")
    print("Actuators:")
    for i in range(model.nu):
        print(f"  {i}: {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)} (joint_id={model.actuator_trnid[i, 0]})")

    # Run with viewer, no control
    with mujoco.viewer.launch_passive(model, data) as viewer:
        t0 = time.time()
        while viewer.is_running():
            # No control: don't touch data.ctrl
            mujoco.mj_step(model, data)
            viewer.sync()
            if time.time() - t0 > 130.0:
                break

    print("Done.")

if __name__ == '__main__':
    main()
