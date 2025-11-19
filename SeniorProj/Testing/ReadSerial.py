import serial
import time
import serial
import time

PORT = 'COM8'      # Change if needed
BAUD = 115200      # Match your device

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Opened {PORT} at {BAUD} baud.")
except Exception as e:
    print("Error opening port:", e)
    exit()

print("Listening...  (Ctrl+C to stop)")

try:
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode(errors='ignore').strip()
            print(f"> {data}")
        time.sleep(0.01)
except KeyboardInterrupt:
    print("\nExiting.")
finally:
    ser.close()

PORT = 'COM8'      # Change if needed
BAUD = 115200      # Match your device

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Opened {PORT} at {BAUD} baud.")
except Exception as e:
    print("Error opening port:", e)
    exit()

print("Listening...  (Ctrl+C to stop)")

try:
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode(errors='ignore').strip()
            print(f"> {data}")
        time.sleep(0.01)
except KeyboardInterrupt:
    print("\nExiting.")
finally:
    ser.close()
