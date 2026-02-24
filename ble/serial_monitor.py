# live_serial_plot.py
import re
import time
from collections import deque

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

SERIAL_PORT = "COM8"
BAUD_RATE = 115200

WINDOW_SEC = 5  # visible time window (seconds)

# ---------- Serial ----------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)
ser.reset_input_buffer()

# ---------- Data buffers ----------
t_buf = deque()
ir_buf = deque()
red_buf = deque()

ax_buf = deque()
ay_buf = deque()
az_buf = deque()
asvm_buf = deque()

gx_buf = deque()
gy_buf = deque()
gz_buf = deque()
gsvm_buf = deque()

t0 = None  # first MCU time


# ---------- Parser ----------
pattern = re.compile(
    r"IR_RAW:\s*(\d+);\s*RED_RAW:\s*(\d+);\s*AX:\s*([-0-9.]+);\s*AY:\s*([-0-9.]+);\s*AZ:\s*([-0-9.]+);\s*"
    r"GX:\s*([-0-9.]+);\s*GY:\s*([-0-9.]+);\s*GZ:\s*([-0-9.]+);\s*"
    r"ASVM:\s*([-0-9.]+);\s*GSVM:\s*([-0-9.]+);\s*MCU_TIME:\s*(\d+)"
)


def parse_line(line):
    m = pattern.search(line)
    if not m:
        return None
    vals = list(m.groups())
    return (
        int(vals[0]),
        int(vals[1]),
        float(vals[2]),
        float(vals[3]),
        float(vals[4]),
        float(vals[5]),
        float(vals[6]),
        float(vals[7]),
        float(vals[8]),
        float(vals[9]),
        int(vals[10]),
    )


# ---------- Plot setup ----------
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

# IR / RED
l_ir, = axes[0].plot([], [], label="IR_RAW",color='blue')
l_red, = axes[0].plot([], [], label="RED_RAW", color='red')
axes[0].legend()
axes[0].set_ylabel("Raw IR, Raw RED")
axes[0].set_title("PPG: Raw Sensor IR and Red Values")

# Accel
l_ax, = axes[1].plot([], [], label="AX")
l_ay, = axes[1].plot([], [], label="AY")
l_az, = axes[1].plot([], [], label="AZ")
l_asvm, = axes[1].plot([], [], label="ASVM", linewidth=2)
axes[1].legend()
axes[1].set_ylabel("Acceleration (G)")
axes[1].set_title("Accelerometer: 3-D Values + Signal Vector Magnitude")

# Gyro
l_gx, = axes[2].plot([], [], label="GX")
l_gy, = axes[2].plot([], [], label="GY")
l_gz, = axes[2].plot([], [], label="GZ")
l_gsvm, = axes[2].plot([], [], label="GSVM", linewidth=2)
axes[2].legend()
axes[2].set_ylabel("Gyroscope (rad/s)")
axes[2].set_title("Gyroscope: 3-D Values + Signal Vector Magnitude")
axes[2].set_xlabel("Time (s)")


# ---------- Update ----------
def update(frame):
    global t0

    while ser.in_waiting:
        line = ser.readline().decode(errors="ignore")
        parsed = parse_line(line)
        if parsed is None:
            continue

        ir, red, ax, ay, az, gx, gy, gz, asvm, gsvm, mcu = parsed

        if t0 is None:
            t0 = mcu

        t = (mcu - t0) / 1e6  # convert µs → s (adjust if needed)

        t_buf.append(t)
        ir_buf.append(ir)
        red_buf.append(red)

        ax_buf.append(ax)
        ay_buf.append(ay)
        az_buf.append(az)
        asvm_buf.append(asvm)

        gx_buf.append(gx)
        gy_buf.append(gy)
        gz_buf.append(gz)
        gsvm_buf.append(gsvm)

    # remove old samples outside window
    while t_buf and (t_buf[-1] - t_buf[0] > WINDOW_SEC):
        t_buf.popleft()
        ir_buf.popleft()
        red_buf.popleft()
        ax_buf.popleft()
        ay_buf.popleft()
        az_buf.popleft()
        asvm_buf.popleft()
        gx_buf.popleft()
        gy_buf.popleft()
        gz_buf.popleft()
        gsvm_buf.popleft()

    # update lines
    l_ir.set_data(t_buf, ir_buf)
    l_red.set_data(t_buf, red_buf)

    l_ax.set_data(t_buf, ax_buf)
    l_ay.set_data(t_buf, ay_buf)
    l_az.set_data(t_buf, az_buf)
    l_asvm.set_data(t_buf, asvm_buf)

    l_gx.set_data(t_buf, gx_buf)
    l_gy.set_data(t_buf, gy_buf)
    l_gz.set_data(t_buf, gz_buf)
    l_gsvm.set_data(t_buf, gsvm_buf)

    # autoscale
    for ax in axes:
        ax.relim()
        ax.autoscale_view()

    return (
        l_ir, l_red,
        l_ax, l_ay, l_az, l_asvm,
        l_gx, l_gy, l_gz, l_gsvm
    )


ani = FuncAnimation(fig, update, interval=50)
plt.tight_layout()
plt.show()