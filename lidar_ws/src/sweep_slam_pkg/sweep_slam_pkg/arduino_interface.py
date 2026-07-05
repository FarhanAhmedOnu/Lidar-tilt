import serial
import struct
import threading
import queue
import numpy as np
import json
import os

class ArduinoInterface:
    def __init__(self, port, baudrate, calib_file_path):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        
        # Load calibration data
        if not os.path.exists(calib_file_path):
            raise FileNotFoundError(f"Calibration file not found at {calib_file_path}")
            
        with open(calib_file_path, 'r') as f:
            calib = json.load(f)
            
        self.R_calib = np.array(calib['rotation_matrix'])
        self.gyro_bias = np.array(calib['gyro_bias'])
        
        # Thread-safe queue to hold the latest IMU data (maxsize=1 prevents lag)
        self.data_queue = queue.Queue(maxsize=1)
        self.running = True
        
        # Connect to Arduino
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"[ArduinoInterface] Connected to {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to connect to Arduino on {self.port}: {e}")
            
        # Start background reading thread
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _read_loop(self):
        """Background thread that continuously reads and parses binary data."""
        while self.running:
            try:
                # 1. Sync to header (0xAA 0x55)
                if self.ser.read(1) != b'\xaa':
                    continue
                if self.ser.read(1) != b'\x55':
                    continue
                    
                # 2. Read payload
                data = self.ser.read(28)
                if len(data) != 28:
                    continue
                    
                # 3. Unpack binary data
                t, ax, ay, az, gx, gy, gz = struct.unpack('<I6f', data)
                
                # 4. Apply Calibration
                raw_accel = np.array([ax, ay, az])
                raw_gyro = np.array([gx, gy, gz])
                
                # Note: Adafruit library outputs m/s^2 and rad/s, so no unit conversion needed!
                calib_accel = self.R_calib @ raw_accel
                calib_gyro = self.R_calib @ (raw_gyro - self.gyro_bias)
                
                # 5. Push to queue (drop oldest if full to ensure real-time data)
                if self.data_queue.full():
                    self.data_queue.get_nowait()
                self.data_queue.put((t, calib_accel, calib_gyro))
                
            except serial.SerialException:
                if self.running:
                    print("[ArduinoInterface] Serial port disconnected.")
                break

    def get_latest_data(self):
        """Returns the latest (timestamp, accel, gyro) tuple, or None if empty."""
        try:
            return self.data_queue.get_nowait()
        except queue.Empty:
            return None
        
    def send_servo_command(self, pwm_microseconds):
        """Sends a PWM command to the servo. Thread-safe."""
        if self.ser and self.ser.is_open:
            # Constrain to safe physical limits of the MG996R
            us = max(1000, min(2000, int(pwm_microseconds)))
            cmd = f"S{us}\n"
            try:
                self.ser.write(cmd.encode('utf-8'))
            except serial.SerialException:
                pass # Port might be closing

    def close(self):
        """Safely shuts down the thread and closes the serial port."""
        print("[ArduinoInterface] Shutting down...")
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[ArduinoInterface] Serial port closed safely.")