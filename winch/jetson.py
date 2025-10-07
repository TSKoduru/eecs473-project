import serial

# Change this to whatever port the NUCLEO is connected to
ser = serial.Serial('COM9', 115200, timeout=1)

while True:
    cmd = input("Enter (BLINK or DBLINK): ")
    ser.write((cmd + '\n').encode())
