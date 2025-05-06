#!/usr/bin/env python3
"""
motor.py

Runs the revolute_2 hinge defined in `motor_setup.xml` for 10 seconds at 100 Hz,
driven by a smooth sinusoid (±π at 1 Hz), and logs timestamped commands and
measured positions to CSV via multiprocessing.
"""
import os
import time
import numpy as np
import csv
import multiprocessing as mp
import mujoco
import mujoco.viewer

def logger_proc(log_queue, csv_path):
    """Logger process: writes (time, cmd, qpos) entries to CSV."""
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'cmd', 'qpos'])
        for entry in iter(log_queue.get, None):
            writer.writerow(entry)

def main():
    # Locate files
    here     = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(here, 'motor_setup.xml')
    csv_path = os.path.join(here, 'motor_log.csv')

    # Load model & data
    model = mujoco.MjModel.from_xml_path(xml_path)
    data  = mujoco.MjData(model)

    # Start logger
    log_queue = mp.Queue()
    logger    = mp.Process(target=logger_proc, args=(log_queue, csv_path))
    logger.start()

    # Control loop parameters
    ctrl_freq = 100.0               # Hz
    sim_dt    = model.opt.timestep  # e.g. 0.002
    steps_per_ctrl = max(1, int(round(1.0 / (ctrl_freq * sim_dt))))

    # Sinusoid parameters
    freq      = 1.0      # Hz (how many full ±π cycles per second)
    amplitude = np.pi    # radians

    # Run with non-blocking viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        t0 = time.time()
        while viewer.is_running():
            t = time.time() - t0
            # stop after 10 seconds
            if t >= 5.0:
                break

            # compute smooth sine command
            cmd = amplitude * np.sin(2 * np.pi * freq * t)
            data.ctrl[0] = cmd

            # step simulation to hold 100 Hz control
            for _ in range(steps_per_ctrl):
                mujoco.mj_step(model, data)

            # log time, command, actual qpos
            log_queue.put((f"{t:.4f}", f"{cmd:.6f}", f"{data.qpos[0]:.6f}"))

            # render
            viewer.sync()

    # finish logging
    log_queue.put(None)
    logger.join()
    print(f"Logged data to {csv_path}")

if __name__ == '__main__':
    mp.set_start_method('spawn')
    main()
