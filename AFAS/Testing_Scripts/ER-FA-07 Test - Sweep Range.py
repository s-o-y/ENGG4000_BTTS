# Author: Zachary Demerson
# Last Updated: April 2, 2024
# Description: 
#	Sweeps from the maximum possible position to the minimum possible position
# 	Determines the angular range of the system.
# 	Repeats multiple times to ensure accuracy.

import serial
import numpy as np
from picamera2 import Picamera2, Preview
from time import sleep

def controlMotor(position,interval,direction):
    max_motor_position = 3648							# Max far position - focused at infinity
    min_motor_position = 2409							# Furthest near position without hitting bracket
    serial_port = serial.Serial("/dev/ttyS0", 115200)	# Using RPi 4 default serial port (Mini-UART on Pin 8 (TX) and Pin 10 (RX))
    
    if(position + (interval*direction) > max_motor_position):		# Motor position is clamped between min and max
        newPosition = max_motor_position				# Set position to maximum
        direction = direction * -1						# Reverse direction for next movement
    elif(position + (interval*direction) < min_motor_position):
        newPosition = min_motor_position				# Set position to minimum
        direction = direction * -1						# Reverse direction for next movement
    else:
        newPosition = (position + (interval*direction))				# increment the position of the motor by interval/4096 * 360 degrees
    
    message = int(newPosition)							# Message to be sent (ONLY SUPPORTS UP TO 2 BYTE INTEGERS)
    message = message.to_bytes(2,signed=False)			# Convert message to bytes for transmission
    bytesWritten = serial_port.write(message)			# Transmit the message body
    return direction

def readPosition(serial_port):
    # Force the while loop
    serialReady = False
    while(not serialReady):											# Loop until the position is sent (Waiting for motor to finish moving)
        if(serial_port.in_waiting > 0):								# Once the message is sent, flip our loop flag
            serialReady = True
        if(serialReady):
            while(serial_port.in_waiting > 0):						# If the RB-150 has sent its position, read until port is clear
                received_message = serial_port.read(size=2)			# Read in the position bytes
                position = int.from_bytes(received_message,"big")	# Convert bytes to int
                
                '''
                # For Debugging:
                print(received_message)
                print(received_int)
                '''
    sleep(0.15)			# This delay is because immediately after movement, the measured MRA is not true (worse than expected). 0.15 determined experimentally
    return position

serial_port = serial.Serial("/dev/ttyS0", 115200)	# Using RPi 4 default serial port (Mini-UART on Pin 8 (TX) and Pin 10 (RX))
max_motor_position = 3648							# Max far position - focused at infinity
min_motor_position = 2409							# Furthest near position without hitting bracket
interval = max_motor_position - min_motor_position	# How many positions to move the motor at a time (motor has range 0 to 4095)
count = 0
while(count < 3):
    direction = 1
    controlMotor(min_motor_position,interval,direction)
    position1 = readPosition(serial_port)
    print("Position: "+str(position1))
    sleep(3)
    direction = -1
    controlMotor(max_motor_position,interval,direction)
    position2 = readPosition(serial_port)
    print("Position: "+str(position2))
    print("Degrees Swept: "+str(360*abs(position1-position2)/4096))
    sleep(3)
    count+= 1