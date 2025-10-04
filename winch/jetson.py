import serial
import time

SERIAL_PORT = 'COM4'
BAUD_RATE = 115200

def send_message(message):
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            ser.write(message.encode('utf-8'))
            print(f"Sent: {message}")
    except serial.SerialException as e:
        print(f"Error: {e}")

# Optional: Quick test
if __name__ == "__main__":
    send_message("Hello world!")
    time.sleep(1)
