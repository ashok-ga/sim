import mujoco
import mujoco.viewer
import sys
import os
import numpy as np

# Set your MuJoCo XML file path here
MJCF_PATH = "leader_left.xml"  # change if your file name differs

# PD gains (tuned to reduce oscillation)
KP = 20.0
KD = 1.0


def main():
    if not os.path.isfile(MJCF_PATH):
        print(f"ERROR: Cannot find file '{MJCF_PATH}'")
        sys.exit(1)

    # Load the MJCF model
    try:
        model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    except Exception as e:
        print(f"Failed to load MJCF: {e}")
        sys.exit(1)

    # Create data object
    data = mujoco.MjData(model)

    # Get actuator joint ids
    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
    joint_ids = [model.joint(name).id for name in joint_names]

    print("Press ESC in the viewer window to quit.")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Capture initial joint positions after viewer launches (user can move joints before holding)
        initial_qpos = np.copy(data.qpos[joint_ids])
        print("Captured initial joint positions:", initial_qpos)
        while viewer.is_running():
            # PD control to hold captured initial position
            q = data.qpos[joint_ids]
            qd = data.qvel[joint_ids]
            data.ctrl[:] = KP * (initial_qpos - q) - KD * qd
            mujoco.mj_step(model, data)
            viewer.sync()
    print("Viewer closed.")

if __name__ == "__main__":
    main()
