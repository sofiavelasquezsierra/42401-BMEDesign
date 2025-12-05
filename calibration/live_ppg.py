# live_inference.py
# Reads raw PPG features from MAX30102 (PPG) over serial
# Uses trained ML models to estimate heart rate and SpO2 in real time

import serial
import time
import joblib
import numpy as np
from datetime import datetime

SERIAL_PORT = "COM5"
BAUD_RATE = 115200

SAVE_TO_CSV = False
CSV_FILE = "ppg_live_predictions.csv"

def parse_line(line):
    try:
        parts = line.strip().split(",")
        data = {}
        for p in parts:
            if "=" not in p:
                continue
            key, val = p.split("=")
            data[key.strip()] = float(val)
        return data
    except Exception as e:
        print("Parse error:", e)
        return None

def main():
    print("Loading models...")

    hr_model = joblib.load("hr_model.pkl")
    spo2_model = joblib.load("spo2_model.pkl")

    print("Models loaded.\nOpening serial port...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("Connected.\n")

    # Optional CSV logging
    if SAVE_TO_CSV:
        import csv
        csv_file = open(CSV_FILE, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow([
            "timestamp", "IR", "RED",
            "IR_AC", "IR_DC", "RED_AC", "RED_DC", "R",
            "EST_HR", "EST_SpO2"
        ])

    print("Streaming data... press CTRL+C to stop.\n")

    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            # print("RAW:", line)
            if "IR=" not in line:
                continue

            parsed = parse_line(line)
            if parsed is None:
                continue

            # Arrange the features in the same order used for training
            X = np.array([[
                parsed["IR"],
                parsed["RED"],
                parsed["IR_AC"],
                parsed["IR_DC"],
                parsed["RED_AC"],
                parsed["RED_DC"],
                parsed["R"]
            ]])

            # Predict HR and SpO2
            est_hr = float(hr_model.predict(X)[0])
            est_spo2 = float(spo2_model.predict(X)[0])

            # Clamp final values to reasonable ranges
            est_hr = max(0, min(est_hr, 200))
            est_spo2 = max(70, min(est_spo2, 100))

            print(
                f"HR={est_hr:5.1f} bpm   "
                f"SpO2={est_spo2:5.1f}%   "
                f"(IR={parsed['IR']:.0f}, RED={parsed['RED']:.0f})"
            )

            # Optional CSV logging
            if SAVE_TO_CSV:
                writer.writerow([
                    datetime.now().isoformat(),
                    parsed["IR"], parsed["RED"],
                    parsed["IR_AC"], parsed["IR_DC"],
                    parsed["RED_AC"], parsed["RED_DC"],
                    parsed["r"],
                    est_hr, est_spo2
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
