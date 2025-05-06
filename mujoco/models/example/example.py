import os
import time
import numpy as np
import mujoco
import mujoco.viewer

# Determine path to the XML model relative to this script
here = os.path.dirname(__file__)
model_path = os.path.join(here, 'example.xml')

# Load model and data
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Launch a passive (non-blocking) viewer and drive the joint
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    while viewer.is_running():
        # Sinusoidal control: amplitude 0.8, frequency 0.5 Hz
        t = time.time() - start_time
        data.ctrl[0] = 0.8 * np.sin(2 * np.pi * 0.5 * t)

        mujoco.mj_step(model, data)
        viewer.sync()