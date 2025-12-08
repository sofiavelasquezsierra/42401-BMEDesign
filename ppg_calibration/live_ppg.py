# live_inference.py
# Reads raw MAX30102 PPG data over serial
# Computes HR and SpO2 in real time

import serial
import time
import json
import numpy as np
from datetime import datetime
from scipy.signal import butter, filtfilt, find_peaks

SERIAL_PORT = "COM5"
BAUD_RATE = 115200

SAVE_TO_CSV = False
CSV_FILE = "ppg_live_predictions.csv"

# Load calibration constants
with open("ppg_calibration_constants.json", "r") as f:
    cal = json.load(f)

HR_A = cal["hr"]["a"]
HR_B = cal["hr"]["b"]
SPO2_A = cal["spo2"]["A"]
SPO2_B = cal["spo2"]["B"]

# Bandpass filter (0.5–5 Hz)
fs = 100  # sampling rate of Arduino output

def butter_bandpass(low, high, fs, order=3):
    nyq = fs / 2
    b, a = butter(order, [low/nyq, high/nyq], btype="band")
    return b, a

def apply_filter(sig):
    b, a = butter_bandpass(0.5, 5, fs)
    return filtfilt(b, a, sig)


# Parse raw Arduino messages
def parse_line(line):
    try:
        parts = line.strip().split(",")
        data = {}
        for p in parts:
            if "=" in p:
                key, val = p.split("=")
                data[key.strip()] = float(val)
        return data
    except:
        return None


# Real-time inference loop
def main():
    print("Opening serial port...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("Connected.\n")

    # Buffers for 2-second windows (200 samples)
    ir_buffer = []
    red_buffer = []

    # Optional CSV logging
    if SAVE_TO_CSV:
        import csv
        csv_file = open(CSV_FILE, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow([
            "timestamp", "HR", "SpO2", "IR_DC", "RED_DC", "R"
        ])

    print("Streaming... CTRL+C to stop.\n")

    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if not line.startswith("IR="):
                continue

            parsed = parse_line(line)
            if parsed is None:
                continue

            ir_raw = parsed["IR"]
            red_raw = parsed["RED"]

            # Append new sample
            ir_buffer.append(ir_raw)
            red_buffer.append(red_raw)

            # Keep only last 200 samples (~2 seconds)
            if len(ir_buffer) > 200:
                ir_buffer.pop(0)
                red_buffer.pop(0)

            # Run inference only if buffer full
            if len(ir_buffer) < 200:
                continue

            # Apply filtering
            ir_f = apply_filter(np.array(ir_buffer))
            red_f = apply_filter(np.array(red_buffer))

            # Compute AC/DC and R ratio
            ir_dc = np.mean(ir_buffer)
            red_dc = np.mean(red_buffer)
            ir_ac = np.ptp(ir_f)
            red_ac = np.ptp(red_f)

            R = (red_ac / red_dc) / (ir_ac / ir_dc)

            # HR estimation (peak count)
            peaks, _ = find_peaks(ir_f, distance=0.4 * fs)
            hr_est = len(peaks) * 60 / 2  # peaks per 2 sec -> bpm

            # Apply HR calibration
            hr_true = HR_A * hr_est + HR_B
            hr_true = max(0, min(hr_true, 200))

            # SpO2 estimation
            spo2 = SPO2_A - SPO2_B * R
            spo2 = max(70, min(spo2, 100))

            # Output
            print(f"HR={hr_true:5.1f} bpm   SpO2={spo2:5.1f}%   IR={ir_raw:.0f}")

            # Optional logging
            if SAVE_TO_CSV:
                writer.writerow([
                    datetime.now().isoformat(),
                    hr_true, spo2, ir_dc, red_dc, R
                ])

        except KeyboardInterrupt:
            print("\nStopping...")
            break

        except Exception as e:
            print("Error:", e)
            continue

    if SAVE_TO_CSV:
        csv_file.close()
        print("Saved to", CSV_FILE)


if __name__ == "__main__":
    main()
