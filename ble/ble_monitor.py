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

    recv_buffer.extend(data)

    while len(recv_buffer) >= PACKET_LEN:
        pkt = recv_buffer[:PACKET_LEN]
        del recv_buffer[:PACKET_LEN]

        ptype = chr(pkt[0])

        if ptype == 'P':
            # 'P' + ts(uint32) + ir(3 bytes LE, low 18 bits) + red(3 bytes LE, low 18 bits)
            ts = struct.unpack_from("<I", pkt, 1)[0]

            ir = pkt[5] | (pkt[6] << 8) | (pkt[7] << 16)
            red = pkt[8] | (pkt[9] << 8) | (pkt[10] << 16)

            ir &= 0x3FFFF
            red &= 0x3FFFF

            print(f"[BLE RX] P ts={ts} ir={ir} red={red}")

        elif ptype == 'A':
            ts, ax, ay, az = struct.unpack_from("<Ihhh", pkt, 1)
            print(f"[BLE RX] A ts={ts} ax={ax/100:.2f} ay={ay/100:.2f} az={az/100:.2f}")

        elif ptype == 'G':
            ts, gx, gy, gz = struct.unpack_from("<Ihhh", pkt, 1)
            print(f"[BLE RX] G ts={ts} gx={gx/10:.2f} gy={gy/10:.2f} gz={gz/10:.2f}")

        else:
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
