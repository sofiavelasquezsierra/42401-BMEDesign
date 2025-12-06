# hr_spo2_data_collect.py
# Collect raw IR + RED PPG for later calibration in Jupyter Notebook

import serial
import csv
import time
from datetime import datetime

SERIAL_PORT = "COM5"
BAUD_RATE = 115200
OUTPUT_FILE = "hr_spo2_calibration_data.csv"

CALIBRATION_SETS = 170
WINDOW_SECONDS = 2

def get_clinical_values():
    print("\nEnter fingertip oximeter readings (HR SpO2):")
    while True:
        try:
            hr_str, spo2_str = input("> ").split()
            return float(hr_str), float(spo2_str)
        except:
            print("Invalid. Example: 72 98")

def parse_line(line):
    try:
        d = {}
        for p in line.strip().split(","):
            k, v = p.split("=")
            d[k] = float(v)
        return d
    except:
        return None

def main():
    print("Opening serial port...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("Connected.")

    with open(OUTPUT_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "window", "sample", "ir_raw", "red_raw", "true_hr", "true_spo2"])

        for window in range(CALIBRATION_SETS):
            print(f"\nCalibration window {window+1}/{CALIBRATION_SETS}")
            true_hr, true_spo2 = get_clinical_values()

            print("Recording raw samples…")

            start_time = time.time()
            sample_index = 0

            while time.time() - start_time < WINDOW_SECONDS:
                line = ser.readline().decode(errors="ignore")

                if not line.startswith("IR="):
                    continue

                parsed = parse_line(line)
                if parsed is None:
                    continue

                writer.writerow([
                    datetime.now().isoformat(),
                    window,
                    sample_index,
                    parsed["IR"],
                    parsed["RED"],
                    true_hr,
                    true_spo2
                ])

                sample_index += 1

            print("Done.")

    print(f"\nSaved to {OUTPUT_FILE}")

if __name__ == "__main__":
    main()
