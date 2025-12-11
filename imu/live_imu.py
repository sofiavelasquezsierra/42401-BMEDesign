# live_imu.py
import asyncio
import numpy as np
import joblib
from collections import deque
from bleak import BleakClient, BleakScanner

UART_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
RX_UUID   = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

WINDOW = 25  # must match training
buffer = deque(maxlen=WINDOW)

recv_buffer = ""
latest_vals = [0,0,0,0,0,0]

def handle_notification(sender, data):
    global recv_buffer, latest_vals

    text = data.decode(errors="ignore")
    recv_buffer += text

    while True:
        parts = [p for p in recv_buffer.strip().split(",") if p != ""]

        if len(parts) < 6:
            return

        try:
            ax, ay, az, gx, gy, gz = map(float, parts[:6])
            latest_vals = [ax, ay, az, gx, gy, gz]
            buffer.append(latest_vals)
        except:
            pass

        remaining = parts[6:]
        recv_buffer = ",".join(remaining)
        return

def compute_features(win):
    win = np.array(win)
    ax, ay, az, gx, gy, gz = win.T

    acc_mag = np.sqrt(ax**2 + ay**2 + az**2)
    gyro_mag = np.sqrt(gx**2 + gy**2 + gz**2)

    return np.array([
        ax.mean(),
        ay.mean(),
        az.mean(),
        acc_mag.max(),
        acc_mag.std(),
        gyro_mag.max(),
        gyro_mag.std(),
    ]).reshape(1, -1)

async def main():
    print("Loading model…")
    model = joblib.load("imu_model.pkl")
    scaler = joblib.load("imu_scaler.pkl")
    le = joblib.load("imu_label_encoder.pkl")

    print("Scanning for XIAO-IMU…")
    device = await BleakScanner.find_device_by_name("XIAO-IMU")
    if device is None:
        print("❌ Device not found")
        return

    async with BleakClient(device) as client:
        print("Connected.\n")
        await client.start_notify(RX_UUID, handle_notification)

        try:
            while True:
                if len(buffer) == WINDOW:
                    feats = compute_features(buffer)
                    feats_scaled = scaler.transform(feats)
                    pred = model.predict(feats_scaled)[0]
                    label = le.inverse_transform([pred])[0]
                    print(f"Activity: {label}")

                await asyncio.sleep(0.02)
        except KeyboardInterrupt:
            print("Stopping…")
            await client.stop_notify(RX_UUID)

if __name__ == "__main__":
    asyncio.run(main())
