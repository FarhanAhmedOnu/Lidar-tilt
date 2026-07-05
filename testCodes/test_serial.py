import serial
import struct
import time

# Adjust port to your Arduino (e.g., 'COM3' on Windows, '/dev/ttyACM0' on Linux)
PORT = '/dev/ttyACM0' 
BAUD = 250000

ser = serial.Serial(PORT, BAUD, timeout=1)
print(f"Listening on {PORT} at {BAUD} baud...")
print("Sending servo to 1200us (tilted left)...")
ser.write(b"S1200\n") # Command servo to 1200us

try:
    while True:
        # 1. Sync to our binary header (0xAA 0x55)
        while ser.read(1) != b'\xaa':
            pass
        if ser.read(1) != b'\x55':
            continue
            
        # 2. Read the remaining 28 bytes
        data = ser.read(28)
        if len(data) == 28:
            # Unpack: Little-endian (<), 1 uint32 (I), 6 floats (f)
            values = struct.unpack('<I6f', data)
            t, ax, ay, az, gx, gy, gz = values
            
            # Print at roughly 10Hz so it doesn't flood the terminal
            if t % 50000 < 5000: 
                print(f"Time: {t}us | Accel: [{ax:6.2f}, {ay:6.2f}, {az:6.2f}] | Gyro: [{gx:6.2f}, {gy:6.2f}, {gz:6.2f}]")
                
except KeyboardInterrupt:
    ser.write(b"S1500\n") # Center servo before exiting
    ser.close()
    print("\nStopped and centered servo.")