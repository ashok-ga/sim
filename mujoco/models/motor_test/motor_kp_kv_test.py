#!/usr/bin/env python3
"""
motor_param_sweep.py

Performs a sequential parameter sweep over Kp and Kv values for the `revolute_2_act` position actuator in `motor_setup.xml`.
For each (kp, kv) combination, waits 3 seconds, launches a non-blocking MuJoCo viewer,
runs a 5-second sinusoidal test at 1 Hz, logs time, command, and qpos to a CSV, computes MSE, then closes the viewer
and waits 1 second before proceeding to the next pair. Generates individual CSVs like `motor_log_kp10_kv1.csv` and a summary `sweep_summary.csv`.
"""
import os
import time
import numpy as np
import csv
import mujoco
from mujoco import MjModel, MjData, mjtObj, mj_step, mj_name2id, viewer

# Run a single simulation for given kp, kv
def run_simulation(xml_path, kp, kv, duration=5.0, ctrl_freq=50.0):
    # Load model & data
    model = MjModel.from_xml_path(xml_path)
    data  = MjData(model)

    # Set actuator gains
    act_id = mj_name2id(model, mjtObj.mjOBJ_ACTUATOR, 'revolute_2_act')
    model.actuator_gainprm[act_id, 0] = kp
    model.actuator_gainprm[act_id, 1] = kv

    # Prepare to collect log entries
    log_entries = []

    # Control parameters
    sim_dt         = model.opt.timestep
    steps_per_ctrl = max(1, int(round(1.0 / (ctrl_freq * sim_dt))))
    freq           = 1.0    # Hz for sine
    amplitude      = np.pi  # rad
    errors         = []     # accumulate squared errors

    # Wait before start
    print(f"Starting simulation for kp={kp}, kv={kv} in 3 seconds...")
    time.sleep(3)

    # Launch non-blocking viewer
    with viewer.launch_passive(model, data) as sim_viewer:
        t0 = time.time()
        while sim_viewer.is_running():
            t = time.time() - t0
            if t >= duration:
                break

            # compute control command
            cmd = amplitude * np.sin(2 * np.pi * freq * t)
            data.ctrl[act_id] = cmd

            # advance simulation
            for _ in range(steps_per_ctrl):
                mj_step(model, data)

            # measure and record
            qpos = data.qpos[0]
            errors.append((qpos - cmd)**2)
            log_entries.append((f"{t:.4f}", f"{cmd:.6f}", f"{qpos:.6f}"))

            # render
            sim_viewer.sync()

    # Write CSV log
    here     = os.path.dirname(os.path.abspath(__file__))
    csv_name = f"motor_log_kp{kp}_kv{kv}.csv"
    csv_path = os.path.join(here, csv_name)
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'cmd', 'qpos'])
        writer.writerows(log_entries)

    # compute MSE
    mse = float('nan') if not errors else np.mean(errors)
    print(f"Finished kp={kp}, kv={kv}, MSE={mse:.6f}")

    # Wait before next
    print("Waiting 1 second before next simulation...\n")
    time.sleep(1)

    return mse

# Main sweep sequentially
if __name__ == '__main__':
    here     = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(here, 'motor_setup.xml')

    kp_values = [10,20,30,40,50,60,70,80,90,100,110]
    kv_values = [1,2,3,4,5,6,7,8,9,10,11]

    summary = []  # to store (kp, kv, mse)

    for kp in kp_values:
        for kv in kv_values:
            mse = run_simulation(xml_path, kp, kv)
            summary.append((kp, kv, mse))

    # Write summary CSV
    summary_path = os.path.join(here, 'sweep_summary.csv')
    with open(summary_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['kp', 'kv', 'mse'])
        writer.writerows(summary)

    print(f"Sweep complete. Summary written to {summary_path}")
