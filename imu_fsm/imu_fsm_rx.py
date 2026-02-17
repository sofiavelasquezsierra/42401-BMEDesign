import csv
import asyncio
from datetime import datetime
from bleak import BleakClient, BleakScanner

UART_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
RX_UUID   = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

OUTPUT_FILE = "misc.csv"

CONTINUOUS_LABELS = ["idle", "walk", "run", "jump", "lying", "sitting", "falling", "misc"]
EVENT_LABELS = ["fall"]
LABELS = CONTINUOUS_LABELS + EVENT_LABELS
WINDOW_DURATION = 5.0


async def collect(label, mode):
    print("Scanning for BLE device…")
    device = await BleakScanner.find_device_by_name("XIAO-IMU")

    async with BleakClient(device) as client:
        print("Connected.")

        buffer = {"mcu_time": 0, "FALL_EVENT":0, "AX":0, "AY":0, "AZ":0, "GX":0, "GY":0, "GZ":0, "ASVM":0, "GSVM":0}
        recv_buffer = ""

        def handle(sender, data):
            nonlocal recv_buffer, buffer

            text = data.decode(errors="ignore")
            recv_buffer += text

            while True:
                parts = recv_buffer.split(",")

                # need 10 full fields
                if len(parts) < 11:   # 10 values + trailing remainder
                    break

                fields = parts[:10]
                recv_buffer = ",".join(parts[10:])  # keep exact remainder

                try:
                    buffer["mcu_time"] = int(float(fields[0]))
                    buffer["FALL_EVENT"] = int(float(fields[1]))
                    buffer["AX"] = float(fields[2])
                    buffer["AY"] = float(fields[3])
                    buffer["AZ"] = float(fields[4])
                    buffer["GX"] = float(fields[5])
                    buffer["GY"] = float(fields[6])
                    buffer["GZ"] = float(fields[7])
                    buffer["ASVM"] = float(fields[8])
                    buffer["GSVM"] = float(fields[9])
                except Exception as e:
                    print("Parse error:", e, fields)
            
            # # Try parsing as long as at least some commas are present
            # while True:
            #     parts = [p for p in recv_buffer.strip().split(",") if p != ""]

            #     # Not enough numbers yet → wait for more BLE fragments
            #     if len(parts) < 10:
            #         break
            #     try:
            #         mcu_time, fall_event, ax, ay, az, gx, gy, gz, asvm, gsvm = map(float, parts[:10])
            #         buffer["mcu_time"] = mcu_time
            #         buffer["FALL_EVENT"] = fall_event
            #         buffer["AX"] = ax
            #         buffer["AY"] = ay
            #         buffer["AZ"] = az
            #         buffer["GX"] = gx
            #         buffer["GY"] = gy
            #         buffer["GZ"] = gz
            #         buffer["ASVM"] = asvm
            #         buffer["GSVM"] = gsvm
            #     except Exception as e:
            #         print("Parse error:", e)

                # Remove first 9 values from the buffer and keep the rest
                remaining = parts[10:]
                recv_buffer = ",".join(remaining)
                break

        # Enable BLE notifications
        await client.start_notify(RX_UUID, handle)

        with open(OUTPUT_FILE, "a", newline="") as f:
            writer = csv.writer(f)
            if f.tell() == 0:
                writer.writerow(["mcu_time", "FALL_EVENT", "AX","AY","AZ","GX","GY","GZ","ASVM","GSVM","label"])

            if mode == "continuous":
                input("Move into position + press ENTER to start…\n")
                print("Recording (CTRL+C to stop)…\n")
                try:
                    while True:
                        row = [
                            # datetime.now().isoformat(),
                            buffer["mcu_time"],
                            buffer["FALL_EVENT"],
                            buffer["AX"], buffer["AY"], buffer["AZ"],
                            buffer["GX"], buffer["GY"], buffer["GZ"],
                            buffer["ASVM"], buffer["GSVM"],
                            label
                        ]
                        writer.writerow(row)
                        print(label, row)
                        await asyncio.sleep(0.02)
                except KeyboardInterrupt:
                    print("Stopped.\n")

            else:  # event window
                input("Press ENTER, then perform fall…\n")
                print("Recording fall window…")

                start = datetime.now().timestamp()
                while datetime.now().timestamp() - start < WINDOW_DURATION:
                    row = [
                        buffer["mcu_time"],
                        buffer["FALL_EVENT"],
                        buffer["AX"], buffer["AY"], buffer["AZ"],
                        buffer["GX"], buffer["GY"], buffer["GZ"],
                        buffer["ASVM"], buffer["GSVM"],
                        label
                    ]
                    writer.writerow(row)
                    await asyncio.sleep(0.02)

                print("Fall event recorded.\n")

async def main():
    print("Available labels:")
    for i, lbl in enumerate(LABELS):
        print(f"  {i}: {lbl}")

    idx = int(input("Select label index: "))
    label = LABELS[idx]
    mode = "continuous" if label in CONTINUOUS_LABELS else "event"

    await collect(label, mode)

asyncio.run(main())
