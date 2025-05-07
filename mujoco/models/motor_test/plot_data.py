import os
import csv
import matplotlib
# Use TkAgg backend for interactive plotting
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

def load_csv(path):
    """Load time, command, and qpos data from CSV."""
    times, cmds, qpos = [], [], []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row['time']))
            cmds.append(float(row['cmd']))
            qpos.append(float(row['qpos']))
    return times, cmds, qpos


def main():
    # Locate CSV in same directory as this script
    here = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(here, 'motor_log.csv')

    # Load data
    times, cmds, qpos = load_csv(csv_path)

    # Compute Mean Squared Error
    errors = [(c - p)**2 for c, p in zip(cmds, qpos)]
    mse = sum(errors) / len(errors) if errors else float('nan')
    print(f"Mean Squared Error: {mse:.6f} rad^2")

    # Create figure and axes
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(times, cmds, label='Command (rad)')
    ax.plot(times, qpos, label='Position (rad)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle (rad)')
    ax.set_title('Motor Command vs. Measured Position')
    ax.legend()
    ax.grid(True)

    # Annotate MSE on plot
    ax.text(
        0.98, 0.02,
        f'MSE = {mse:.6f} rad^2',
        transform=ax.transAxes,
        ha='right', va='bottom',
        fontsize=10,
        bbox=dict(facecolor='white', alpha=0.5, edgecolor='none')
    )

    plt.tight_layout()

    # Close on 'q' key press
    def on_key(event):
        if event.key == 'q':
            plt.close(fig)
    fig.canvas.mpl_connect('key_press_event', on_key)

    # Show the plot
    plt.show()

if __name__ == '__main__':
    main()
