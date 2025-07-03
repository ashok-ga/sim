#!/usr/bin/env python3

import time
import os
import math
import multiprocessing as mp
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np

import mujoco
import mujoco.viewer
import can

from commands import (
    Joint,
    set_joint_hybrid_control,
    read_motor_angle,
    BionicMotorCommands as BMC
)

# CONFIG
CHANNEL = "can0"
JOINT = Joint(name="joint_1", motor_type="X4_24", motor_id=0x01, min_pos=-3.14, max_pos=3.14)
DEBUG = True

def logger_proc(log_queue, csv_path):
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'target_deg', 'real_deg', 'sim_deg', 'sim_qvel'])
        for entry in iter(log_queue.get, None):
            writer.writerow(entry)

def hardware_reader(shared_state):
    bus = can.interface.Bus(channel=CHANNEL, interface="socketcan")
    while shared_state['running']:
        pos = read_motor_angle(bus, JOINT)
        if pos is not None:
            shared_state['real_position'] = pos
        time.sleep(0.005)

def hardware_controller(shared_state, amplitude_deg, frequency_hz):
    bus = can.interface.Bus(channel=CHANNEL, interface="socketcan")
    while shared_state['real_position'] is None and shared_state['running']:
        time.sleep(0.01)
    initial_setpoint = shared_state['real_position']
    start_time = time.time()
    try:
        while shared_state['running']:
            elapsed = time.time() - start_time
            target_setpoint = initial_setpoint + amplitude_deg * math.sin(2 * math.pi * frequency_hz * elapsed)
            shared_state['target_position'] = target_setpoint
            current = shared_state['real_position']
            if current is None:
                continue
            cmd = set_joint_hybrid_control(
                JOINT,
                position=target_setpoint,
                speed=0.0,
                torque_ff=0.0
            )
            bus.send(can.Message(arbitration_id=JOINT.motor_id,
                                 data=bytearray(cmd),
                                 is_extended_id=False))
            time.sleep(0.02)
    except KeyboardInterrupt:
        shared_state['running'] = False

def mujoco_worker(shared_state, xml_path, sim_freq, amplitude_rad, frequency_hz, log_queue):
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    sim_dt = model.opt.timestep
    steps_per_ctrl = max(1, int(round(1.0 / (sim_freq * sim_dt))))
    with mujoco.viewer.launch_passive(model, data) as viewer:
        t0 = time.time()
        while viewer.is_running() and shared_state['running']:
            t = time.time() - t0
            if t >= shared_state['duration']:
                shared_state['running'] = False
                break
            # Target (deg) is in shared_state (from main process)
            if 'target_position' in shared_state and shared_state['target_position'] is not None:
                cmd = np.deg2rad(shared_state['target_position'])
            else:
                # Compute in simulation directly if not yet set
                cmd = amplitude_rad * np.sin(2 * np.pi * frequency_hz * t)
            data.ctrl[0] = cmd
            for _ in range(steps_per_ctrl):
                mujoco.mj_step(model, data)
            # Update sim position to shared state (in deg for plotting)
            shared_state['sim_position'] = np.rad2deg(data.qpos[0])
            shared_state['sim_velocity'] = np.rad2deg(data.qvel[0])
            # Logging
            log_queue.put((f"{t:.4f}",
                           f"{shared_state['target_position']:.2f}",
                           f"{shared_state['real_position']:.2f}" if shared_state['real_position'] is not None else "",
                           f"{shared_state['sim_position']:.2f}",
                           f"{shared_state['sim_velocity']:.2f}"))
            viewer.sync()
            time.sleep(1.0 / sim_freq)

def plotter_thread(shared_state):
    plt.ion()
    fig, ax = plt.subplots()
    target_positions, real_positions, sim_positions, timestamps = [], [], [], []
    start_time = time.time()

    def on_key(event):
        if event.key == 'q':
            print("🛑 'q' pressed — stopping...")
            shared_state['running'] = False

    fig.canvas.mpl_connect('key_press_event', on_key)

    while shared_state['running']:
        target = shared_state.get('target_position')
        real = shared_state.get('real_position')
        sim = shared_state.get('sim_position')
        t = time.time() - start_time
        if target is not None:
            target_positions.append(target)
            timestamps.append(t)
        else:
            target_positions.append(np.nan)
            timestamps.append(t)
        if real is not None:
            real_positions.append(real)
        else:
            real_positions.append(np.nan)
        if sim is not None:
            sim_positions.append(sim)
        else:
            sim_positions.append(np.nan)

        ax.clear()
        ax.plot(timestamps, target_positions, label="Target (deg)", linewidth=2)
        ax.plot(timestamps, real_positions, label="Physical (deg)", linewidth=2)
        ax.plot(timestamps, sim_positions, label="Simulation (deg)", linewidth=2)
        ax.set_xlabel('Time (s)', fontsize=18)
        ax.set_ylabel('Degrees', fontsize=18)
        ax.legend()
        fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(0.05)

    plt.ioff()
    plt.close(fig)

if __name__ == "__main__":
    mp.set_start_method('spawn')
    with mp.Manager() as manager:
        shared_state = manager.dict()
        shared_state['real_position'] = None
        shared_state['sim_position'] = None
        shared_state['target_position'] = None
        shared_state['sim_velocity'] = None
        shared_state['running'] = True
        shared_state['duration'] = 30.0

        here = os.path.dirname(os.path.abspath(__file__))
        xml_path = os.path.join(here, 'test.xml')
        csv_path = os.path.join(here, 'motor_sim_log.csv')
        amplitude_deg = 30.0
        amplitude_rad = np.deg2rad(amplitude_deg)
        frequency_hz = 0.5
        sim_freq = 50.0

        log_queue = mp.Queue()
        logger = mp.Process(target=logger_proc, args=(log_queue, csv_path))
        hw_reader = mp.Process(target=hardware_reader, args=(shared_state,))
        hw_controller = mp.Process(target=hardware_controller, args=(shared_state, amplitude_deg, frequency_hz))
        mujoco_sim = mp.Process(target=mujoco_worker, args=(shared_state, xml_path, sim_freq, amplitude_rad, frequency_hz, log_queue))

        logger.start()
        hw_reader.start()
        hw_controller.start()
        mujoco_sim.start()

        # Plot in main thread
        plotter_thread(shared_state)

        logger.join()
        hw_reader.join()
        hw_controller.join()
        mujoco_sim.join()

    print("🏁 Experiment complete.")
