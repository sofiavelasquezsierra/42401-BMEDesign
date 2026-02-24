# hr_spo2_data_collect_serial.py
import csv
import time
from datetime import datetime
from typing import Optional, Tuple

import serial

SERIAL_PORT = "COM8"
BAUD_RATE = 115200

OUTPUT_FILE = "harry_fall_forward_serial.csv"

SETS = 84
WINDOW = 5

def parse_line(line: str) -> Optional[Tuple[float, float, float, float, float, float, float, float, int, float]]:
    line = line.strip()
    if not line:
        return None
    parts = [p for p in line.split(",") if p != ""]
    if len(parts) < 10:
        return None
    try:
        return float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7]), float(parts[8]), float(parts[9])  
    except ValueError:
        return None

def main():
    print(f"Opening serial port {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()
    print("Connected.\n")

    with open(OUTPUT_FILE, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "AX", "AY", "AZ", "GX", "GY", "GZ", "ASVM", "GSVM", "MCU_TIME", "FALL_EVENT", "label"])

        for window in range(SETS):
            print("Recording raw samples…")
            start_time = time.time()
            sample_index = 0

            while time.time() - start_time < WINDOW:
                line = ser.readline().decode(errors="ignore")
                parsed = parse_line_ir_red(line)
                if parsed is None:
                    continue

                ir, red = parsed
                writer.writerow([
                    datetime.now().isoformat(timespec="milliseconds"),
                    AX,
                    sample_index,
                    ir,
                    red,
                    true_hr,
                    true_spo2
                ])
                sample_index += 1

            print(f"Done. Collected {sample_index} samples.")

    ser.close()
    print(f"\nSaved to {OUTPUT_FILE}")

if __name__ == "__main__":
    main()