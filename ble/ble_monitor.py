# ble_monitor.py
import asyncio
import struct
from bleak import BleakClient, BleakScanner

TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
DEVICE_NAME = "XIAO-SENSE"

PACKET_LEN = 11
recv_buffer = bytearray()

def handle_notification(sender: int, data: bytearray):
    global recv_buffer

    # Append new chunk of raw bytes
    recv_buffer.extend(data)

    # Parse as many full packets as we can
    while len(recv_buffer) >= PACKET_LEN:
        pkt = recv_buffer[:PACKET_LEN]
        del recv_buffer[:PACKET_LEN]

        ptype = chr(pkt[0])

        if ptype == 'P':
            ts, ir, red = struct.unpack_from("<IHH", pkt, 1)
            # ts in microseconds since t0
            print(f"[BLE RX] P ts={ts} ir={ir} red={red}")

        elif ptype == 'A':
            ts, ax, ay, az = struct.unpack_from("<Ihhh", pkt, 1)
            # back to floats: accel was scaled by 100 => 0.01g
            print(f"[BLE RX] A ts={ts} ax={ax/100:.2f} ay={ay/100:.2f} az={az/100:.2f}")

        elif ptype == 'G':
            ts, gx, gy, gz = struct.unpack_from("<Ihhh", pkt, 1)
            # back to floats: gyro was scaled by 10 => 0.1 dps
            print(f"[BLE RX] G ts={ts} gx={gx/10:.2f} gy={gy/10:.2f} gz={gz/10:.2f}")

        else:
            # If you ever get out of sync, this helps diagnose it
            print(f"[BLE RX] Unknown packet type: {pkt[0]} (raw={pkt.hex()})")


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
        print("Receiving BLE binary packets... (Ctrl+C to stop)\n")

        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("\nStopping...")
            await client.stop_notify(TX_CHAR_UUID)

if __name__ == "__main__":
    asyncio.run(main())
