from __future__ import print_function
import serial
import binascii
import subprocess


bluetoothSerial = serial.Serial( "/dev/cu.HC-06-DevB", baudrate=9600 )

while True:
	leftX = int(binascii.hexlify(bluetoothSerial.read()), 16)
	rightX = int(binascii.hexlify(bluetoothSerial.read()), 16)

	leftY = int(binascii.hexlify(bluetoothSerial.read()), 16)
	rightY = int(binascii.hexlify(bluetoothSerial.read()), 16)


	xVal = (leftX << 8) | (rightX)
	outputX = float(xVal) / 100.00

	yVal = (leftY << 8) | (rightY)
	outputY = float(yVal) / 100.00

	mult = 2

	command="./click -x "
	argx = int(outputX)
	commy = " -y "
	argy = int(outputY)


	output = command + str(argx) + commy + str(argy)
	print(output)

	result = subprocess.check_output(output, shell=True)
	#print(result)






















