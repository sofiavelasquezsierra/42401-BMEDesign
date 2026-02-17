# hr_spo2_data_collect_ble.py
import asyncio
import csv
import time
from datetime import datetime
from typing import Optional, Tuple
from collections import deque

from bleak import BleakClient, BleakScanner

TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
DEVICE_NAME = "XIAO-PPG"

OUTPUT_FILE = "ppg_6d_7.5l_center.csv"

# OUTPUT_FILE = "ppg_sample.csv"

CALIBRATION_SETS = 30
WINDOW_SECONDS = 5

recv_buffer = ""

sample_queue = deque(maxlen=5000)

def get_clinical_values() -> Tuple[float, float]:
    print("\nEnter fingertip oximeter readings (HR SpO2):")
    while True:
        try:
            hr_str, spo2_str = input("> ").split()
            return float(hr_str), float(spo2_str)
        except Exception:
            print("Invalid. Example: 72 98")


def parse_line_ir_red(line: str) -> Optional[Tuple[int, int]]:
    line = line.strip()
    if not line:
        return None
    parts = [p for p in line.split(",") if p != ""]
    if len(parts) < 2:
        return None
    try:
        return int(float(parts[0])), int(float(parts[1]))
    except ValueError:
        return None


def handle_notification(sender: int, data: bytearray):
    global recv_buffer
    text = data.decode(errors="ignore")
    recv_buffer += text

    while "\n" in recv_buffer:
        line, recv_buffer = recv_buffer.split("\n", 1)
        parsed = parse_line_ir_red(line)
        if parsed is not None:
            sample_queue.append((time.time(), parsed[0], parsed[1]))


async def main():
    print(f"Scanning for BLE device '{DEVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15.0)
    if device is None:
        print("Device not found.")
        return

    print("Connecting...")
    async with BleakClient(device) as client:
        print("Connected.")
        await client.start_notify(TX_CHAR_UUID, handle_notification)
        print("Receiving notifications.\n")

        with open(OUTPUT_FILE, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "window", "sample", "ir_raw", "red_raw", "true_hr", "true_spo2"])

            for window in range(CALIBRATION_SETS):
                print(f"\nCalibration window {window+1}/{CALIBRATION_SETS}")
                true_hr, true_spo2 = get_clinical_values()

                print("Recording raw samples…")

                sample_queue.clear()

                start_time = time.time()
                sample_index = 0

                while time.time() - start_time < WINDOW_SECONDS:
                    # Let notifications come in
                    await asyncio.sleep(0.01)

                    # Drain queue
                    while sample_queue:
                        t, ir, red = sample_queue.popleft()
                        writer.writerow([
                            datetime.now().isoformat(),
                            window,
                            sample_index,
                            ir,
                            red,
                            true_hr,
                            true_spo2
                        ])
                        sample_index += 1

                print(f"Done. Collected {sample_index} samples.")

        await client.stop_notify(TX_CHAR_UUID)

    print(f"\nSaved to {OUTPUT_FILE}")


if __name__ == "__main__":
    asyncio.run(main())
