import numpy as np
import os
import time
import mujoco
import mujoco.viewer

def main():
    here     = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(here, 'robot.xml')

    model = mujoco.MjModel.from_xml_path(xml_path)
    data  = mujoco.MjData(model)

    print("Loaded model. Number of joints:", model.njnt)
    print("Joints:")
    for i in range(model.njnt):
        print(f"  {i}: {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)} (type={model.jnt_type[i]})")
    print("Actuators:")
    for i in range(model.nu):
        print(f"  {i}: {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)} (joint_id={model.actuator_trnid[i, 0]})")

    # Find qpos indices for joints (skip free joints: they have 7 dof!)
    joint_ids = []
    qpos_ids  = []
    for i in range(model.njnt):
        addr = model.jnt_qposadr[i]
        size = 1 if model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE else 7
        if model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE:
            joint_ids.append(i)
            qpos_ids.append(addr)

    # Run with viewer, randomizing joints in-range
    with mujoco.viewer.launch_passive(model, data) as viewer:
        t0 = time.time()
        next_change = t0
        while viewer.is_running():
            now = time.time()
            # Every 0.5s, pick a new random value for each joint (within limits)
            if now > next_change:
                for j, qid in zip(joint_ids, qpos_ids):
                    low, high = model.jnt_range[j]
                    data.qpos[qid] = np.random.uniform(low, high)
                next_change = now + 0.5

            mujoco.mj_step(model, data)
            viewer.sync()
            if now - t0 > 130.0:
                break

    print("Done.")

if __name__ == '__main__':
    main()
