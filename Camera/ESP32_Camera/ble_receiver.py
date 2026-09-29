#!/usr/bin/env python3
"""BLE receiver for ESP32S3_CAM_BLE.ino. Saves each JPEG into ./photos.
Setup:  pip install bleak      Run:  python ble_receiver.py      Stop: Ctrl+C"""
import asyncio, os, struct, time
from bleak import BleakScanner, BleakClient

DEVICE_NAME    = "ESP32S3-CAM"
CTRL_CHAR_UUID = "a1b2c3d4-0002-4a5b-8c6d-1234567890ab"
DATA_CHAR_UUID = "a1b2c3d4-0003-4a5b-8c6d-1234567890ab"
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "photos")

buffer = bytearray(); expected = 0; receiving = False

def ctrl_handler(_s, data):
    global expected, buffer, receiving
    if len(data) != 4: return
    expected = struct.unpack("<I", data)[0]; buffer = bytearray(); receiving = True
    print(f"-> Incoming image: {expected} bytes")

def data_handler(_s, data):
    global buffer, receiving
    if not receiving: return
    buffer.extend(data)
    if len(buffer) >= expected:
        save_image(bytes(buffer[:expected])); receiving = False

def save_image(data):
    os.makedirs(OUT_DIR, exist_ok=True)
    fname = os.path.join(OUT_DIR, f"photo_{time.strftime('%Y%m%d_%H%M%S')}.jpg")
    if data[:2] == b"\xff\xd8" and data[-2:] == b"\xff\xd9":
        with open(fname, "wb") as f: f.write(data)
        print(f"   Saved {os.path.basename(fname)} ({len(data)} bytes)")
    else:
        print(f"   ! Dropped corrupt frame ({len(data)} bytes) - lower CHUNK_SIZE / raise CHUNK_DELAY")

async def main():
    print(f"Scanning for '{DEVICE_NAME}' ...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=20.0)
    if device is None:
        print("Board not found. Is it powered and advertising?"); return
    print(f"Found {device.address}. Connecting...")
    async with BleakClient(device) as client:
        print("Connected. Subscribing...")
        await client.start_notify(DATA_CHAR_UUID, data_handler)
        await client.start_notify(CTRL_CHAR_UUID, ctrl_handler)
        print(f"Ready. Photos -> {OUT_DIR}\nWaiting (one every ~5s). Ctrl+C to stop.\n")
        while client.is_connected:
            await asyncio.sleep(1.0)

if __name__ == "__main__":
    try: asyncio.run(main())
    except KeyboardInterrupt: print("\nStopped.")