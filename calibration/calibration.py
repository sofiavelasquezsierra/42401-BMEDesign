# calibration.py
# Collect calibration data from MAX30102 (PPG) over serial
# Match to fingertip oximeter HR/SpO₂ readings
# Output CSV used for model training

import serial
import csv
import time
from datetime import datetime

# Serial connection settings
SERIAL_PORT = "COM5"
BAUD_RATE = 115200

# Output CSV path
OUTPUT_FILE = "ppg_calibration_data.csv"

# Number of calibration repetitions and duration per set
CALIBRATION_SETS = 5
SECONDS_PER_SET = 2

# Ask user to input HR and SpO₂ from clinical pulse oximeter
# User enters: "72 98"
def get_clinical_values():
    print("\nEnter clinical readings from the fingertip oximeter:")

    while True:
        try:
            combined = input("Format: HR SpO2   (example: 72 98) > ")

            hr_str, spo2_str = combined.strip().split()
            hr = float(hr_str)
            spo2 = float(spo2_str)

            return hr, spo2

        except Exception:
            print("Invalid input. Use format:  HR SpO2   (example: 72 98)")

# Parse one line of Arduino output
# e.g. "IR=50000,RED=45000,IR_AC=123,..."
# Returns a dictionary or None if malformed
def parse_line(line):
    try:
        parts = line.strip().split(",")

        data = {}
        for p in parts:
            key, val = p.split("=")
            data[key] = float(val)

        return data

    except Exception:
        return None

def main():
    print("Opening serial port...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # allow Arduino to reset
    print("Connected.\n")

    # Open CSV and write header
    with open(OUTPUT_FILE, "w", newline="") as f:
        writer = csv.writer(f)

        writer.writerow([
            "timestamp",
            "ir", "red",
            "ir_ac", "ir_dc",
            "red_ac", "red_dc",
            "r_value",
            "true_hr", "true_spo2"
        ])

        print("Starting calibration.\n")

        # Repeat calibration sets
        for i in range(CALIBRATION_SETS):
            print(f"Calibration Set {i+1}/{CALIBRATION_SETS}")

            # User provides reference HR and SpO₂ once per set
            hr, spo2 = get_clinical_values()
            print("Recording… hold still.")

            start = time.time()

            # Read PPG samples for the duration of this set
            while time.time() - start < SECONDS_PER_SET:
                line = ser.readline().decode(errors="ignore")

                if line.startswith("IR="):
                    parsed = parse_line(line)
                    if parsed is None:
                        continue

                    timestamp = datetime.now().isoformat()

                    # Write raw sensor values + reference HR/SpO₂
                    writer.writerow([
                        timestamp,
                        parsed["IR"],
                        parsed["RED"],
                        parsed["IR_AC"],
                        parsed["IR_DC"],
                        parsed["RED_AC"],
                        parsed["RED_DC"],
                        parsed["R"],
                        hr,
                        spo2
                    ])

                    # Live preview
                    print(
                        f"IR={parsed['IR']:.0f}  "
                        f"RED={parsed['RED']:.0f}  "
                        f"R={parsed['R']:.4f}  "
                        f"HR={hr} SpO2={spo2}"
                    )

            print("Set complete.\n")

    print(f"Calibration finished. Saved to {OUTPUT_FILE}")

if __name__ == "__main__":
    main()
