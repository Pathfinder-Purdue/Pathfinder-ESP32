import serial
import re
import numpy as np
import threading
import time
import matplotlib.pyplot as plt


def parse_matrix_line(line):
    # Extract integers, accept any 16 numbers in the line
    nums = re.findall(r"-?\d+", line)
    if len(nums) != 16:
        return None
    vals = [int(x) for x in nums]
    return np.array(vals, dtype=float).reshape((4, 4))


def live_3d_from_com(port="COM4", baudrate=115200, timeout=1):
    lock = threading.Lock()
    latest = {"matrix": np.zeros((4, 4), dtype=float)}

    def serial_thread():
        try:
            with serial.Serial(port, baudrate, timeout=timeout) as ser:
                print(f"Listening on {port} at {baudrate} baud. Press Ctrl+C to stop.")
                while True:
                    if ser.in_waiting:
                        line = ser.readline().decode("utf-8", errors="replace").strip()
                        # Always print raw serial line
                        print(f"Raw reading: {line}")
                        mat = parse_matrix_line(line)
                        if mat is not None:
                            with lock:
                                latest["matrix"] = mat.copy()
        except serial.SerialException as e:
            print(f"Serial error: {e}")

    t = threading.Thread(target=serial_thread, daemon=True)
    t.start()

    # Setup 3D plot
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # Coordinates for 4x4 grid
    x = np.arange(4)
    y = np.arange(4)
    xpos, ypos = np.meshgrid(x, y)
    xpos = xpos.flatten()
    ypos = ypos.flatten()
    dx = dy = 0.6

    try:
        while True:
            with lock:
                mat = latest["matrix"].copy()

            ax.cla()
            ax.set_zlim(0, 4000)
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Distance mm")
            ax.set_title("VL53L5CX 4x4 live 3D")

            z = mat.flatten()
            # For bar3d, zpos is base (0) and dz is height
            zpos = np.zeros_like(z)
            dz = 4000 - z

            # color mapping by absolute range 0..4000
            cmap = plt.get_cmap("viridis")
            norm = plt.Normalize(vmin=0, vmax=4000)
            colors = cmap(norm(np.clip(dz, 0, 4000)))

            ax.bar3d(xpos, ypos, zpos, dx, dy, dz, color=colors, shade=True)

            plt.draw()
            plt.pause(0.05)
    except KeyboardInterrupt:
        print("Stopped by user.")


if __name__ == "__main__":
    live_3d_from_com(port="COM4", baudrate=115200)
