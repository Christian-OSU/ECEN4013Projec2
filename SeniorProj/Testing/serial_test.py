import serial
import time

try:
    ser = serial.Serial(
        port='COM9',  # Change to your serial port
        baudrate=9600,
        timeout=1
    )
    time.sleep(2)  # Give some time for the connection to establish

    if ser.is_open:
        print(f"Serial port {ser.port} opened successfully.")
        while True:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line:  # Only print if data was received
                    print(f"Received: {line}")
            except UnicodeDecodeError:
                print("Could not decode received bytes.")
            except serial.SerialException as e:
                print(f"Serial communication error: {e}")
                break
            except KeyboardInterrupt:
                print("\nExiting program.")
                break
    else:
        print(f"Could not open serial port {ser.port}.")

except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")