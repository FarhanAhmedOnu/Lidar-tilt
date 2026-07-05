import serial
import struct
import numpy as np
import json
import time

# Adjust this to your Arduino port
PORT = '/dev/ttyACM0' 
BAUD = 250000

def read_imu_average(ser, duration=3.0):
    """Reads IMU data for a given duration and returns the average accel and gyro."""
    accels = []
    gyros = []
    start_time = time.time()
    
    # Clear buffer to ensure fresh data
    ser.reset_input_buffer()
    
    while time.time() - start_time < duration:
        # Sync to our binary header (0xAA 0x55)
        while ser.read(1) != b'\xaa':
            pass
        if ser.read(1) != b'\x55':
            continue
            
        data = ser.read(28)
        if len(data) == 28:
            values = struct.unpack('<I6f', data)
            t, ax, ay, az, gx, gy, gz = values
            accels.append([ax, ay, az])
            gyros.append([gx, gy, gz])
            
    if not accels:
        raise RuntimeError("No IMU data received. Check connection.")
        
    return np.mean(accels, axis=0), np.mean(gyros, axis=0)

def main():
    print("Connecting to Arduino...")
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2) # Wait for Arduino to reset after serial connection
    
    print("\n=== IMU Spatial Calibration Tool ===")
    print("This script will determine the exact orientation of your MPU6050")
    print("relative to the tilt axis by measuring gravity at two positions.")
    
    # --- Step 1: 0 Degrees ---
    input("\n[Step 1] Physically move the arm to exactly 0 degrees (horizontal).")
    input("Use a physical level if possible. Press Enter when ready to record...")
    print("Recording for 3 seconds. Keep it perfectly still...")
    avg_accel_0, avg_gyro_0 = read_imu_average(ser, 3.0)
    print(f"Recorded Accel: {avg_accel_0} | Gyro: {avg_gyro_0}")
    
    # --- Step 2: 90 Degrees ---
    input("\n[Step 2] Physically move the arm to exactly 90 degrees (vertical).")
    input("Use a physical square if possible. Press Enter when ready to record...")
    print("Recording for 3 seconds. Keep it perfectly still...")
    avg_accel_90, avg_gyro_90 = read_imu_average(ser, 3.0)
    print(f"Recorded Accel: {avg_accel_90} | Gyro: {avg_gyro_90}")
    
    ser.close()
    
    # --- Step 3: Calculate the Math ---
    print("\nCalculating calibration matrix...")
    v1 = avg_accel_0 / np.linalg.norm(avg_accel_0) # Normalized gravity at 0 deg
    v2 = avg_accel_90 / np.linalg.norm(avg_accel_90) # Normalized gravity at 90 deg
    
    # The tilt axis is perpendicular to both gravity vectors
    y_axis = np.cross(v1, v2)
    y_axis = y_axis / np.linalg.norm(y_axis)
    
    # Z axis is "Up" at 0 degrees (opposite to gravity)
    z_axis = -v1 
    
    # X axis completes the right-handed coordinate system (direction of tilt)
    x_axis = np.cross(y_axis, z_axis)
    
    # The rotation matrix transforms IMU coordinates to Canonical coordinates
    R = np.vstack((x_axis, y_axis, z_axis))
    
    # --- Step 4: Save to JSON ---
    calibration_data = {
        "rotation_matrix": R.tolist(),
        "gyro_bias": avg_gyro_0.tolist(),
        "description": "Multiply raw IMU vectors by this matrix to align with the tilt axis (Y) and Up (Z)."
    }
    
    filename = "imu_calibration.json"
    with open(filename, "w") as f:
        json.dump(calibration_data, f, indent=4)
        
    print(f"\n✅ Calibration complete! Saved to {filename}")
    print("Calculated Rotation Matrix (IMU -> Canonical):")
    print(np.round(R, 3))
    print("\nNext step: Your ROS 2 node will load this JSON and apply R to the raw data.")

if __name__ == "__main__":
    main()