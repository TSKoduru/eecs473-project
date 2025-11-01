import serial
import serial.tools.list_ports
for port in serial.tools.list_ports.comports():
    print(port.device, port.description)
# Change this to whatever port the NUCLEO is connected to
ser = serial.Serial('COM5', 115200, timeout=1)

ser.write("G".encode())

input("Press Enter to exit...\n")

ser.write("S".encode())

ser.close()