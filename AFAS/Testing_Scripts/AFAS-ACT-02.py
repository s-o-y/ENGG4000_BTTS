# Author: Zachary Demerson
# Last Updated: March 15 2024
# Description:
#	Code to implement test AFAS-ACT-02.
#	Tests the response time of the motor from sending the control signal to its settled new position.
#	Sends the motor to a known position first, then measures the time to move one standard interval.
#	Time is measured using tic toc functions. 
#
#	Code for automatic focus adjustment system for BBST.
#	Implemented on Raspberry Pi 4. Ensure enable_UART = 1 in boot/config.txt
#	Uses fourier analysis to determine the level of focus of a lens.
#	Interfaces with a motor controller to adjust lens focal length.
#	Communicates with servo motor controller (OpenRB-150) via UART serial comms.
#	Sends an integer between 0 and 4095 indicating the angular position of the motor.
#	Sweeps range of angular positions then moves to position of best focus.
#	If a global max is found, remaining positions are skipped.
#	Receives serial comms from RB-150 containing the real position of the motor.
#	The worst motor position accuracy observed is 15 positions off (1.3 degrees)

import serial
import time
import numpy as np
from picamera2 import Picamera2, Preview
from time import sleep

# Description:
#	Adjusts the position of the Dynamixel motor by a specified interval.
#	Calculates new position and uses UART to send the new position to the OpenRB-150
# Inputs:
#	position - Type int contains the position of the motor prior to adjustment (0 to 4095 positions)
#	interval - Type int contains the amount to adjust the position by.
#			   Note that positive interval = CCW, negative interval = CW
# Outputs:
#	newPosition - Type int contains the position of the motor after adjustment (0 to 4095 positions)
#	interval    - As above. Polarity may change if a boundary is reached.
def controlMotor(position,interval):
    max_motor_position = 3396							# Max far position - focused at infinity
    min_motor_position = 2553							# Furthest near position without hitting bracket
    serial_port = serial.Serial("/dev/ttyS0", 115200)	# Using RPi 4 default serial port (Mini-UART on Pin 8 (TX) and Pin 10 (RX))
    
    if(position + interval > max_motor_position):		# Motor position is clamped between min and max
        newPosition = max_motor_position				# Set position to maximum
        interval = interval * -1						# Reverse direction for next movement
    elif(position + interval < min_motor_position):
        newPosition = min_motor_position				# Set position to minimum
        interval = interval * -1						# Reverse direction for next movement
    else:
        newPosition = (position + interval)				# increment the position of the motor by interval/4096 * 360 degrees
    
    message = int(newPosition)							# Message to be sent (ONLY SUPPORTS UP TO 2 BYTE INTEGERS)
    message = message.to_bytes(2,signed=False)			# Convert message to bytes for transmission
    bytesWritten = serial_port.write(message)			# Transmit the message body
    return interval

# Description:
#	Takes the serial port to read from and reads the message sent on the port.
#	In the context of this program, the message will be a 2 byte integer representing the position of the motor
# Inputs:
#	serial_port - the port that is being read from
# Outputs:
#	position - type int containing the current position of the motor
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

# Functions to measure time
def TicTocGenerator():
    ti = 0
    tf = time.time()
    while True:
        ti = tf
        tf = time.time()
        yield tf-ti

TicToc = TicTocGenerator()

def toc(tempBool=True):
    tempTimeInterval = next(TicToc)
    if tempBool:
        print("Total Response Time of the Motor: %f seconds." %tempTimeInterval)
        
def tic():
    toc(False)

TicToc2 = TicTocGenerator()

def toc2(tempBool=True):
    tempTimeInterval = next(TicToc2)
    if tempBool:
        print("Time to Send Serial Control to Motor: %f seconds." %tempTimeInterval)
        
def tic2():
    toc2(False)

# Main Script:

# Init Block:
# Initialize the Pi Camera
camera = Picamera2()
config = camera.create_preview_configuration()
camera.configure(config)
camera.start_preview(Preview.QTGL)
camera.start()

serial_port = serial.Serial("/dev/ttyS0", 115200)	# Using RPi 4 default serial port (Mini-UART on Pin 8 (TX) and Pin 10 (RX))
max_motor_position = 3396							# Max far position - focused at infinity
min_motor_position = 2553							# Furthest near position without hitting bracket
interval = 30										# How many positions to move the motor at a time (motor has range 0 to 4095)

position = max_motor_position						# Start motor position at max

interval = controlMotor(position,interval)		# Send motor to max position
position = readPosition(serial_port)			# Update the position with the real position (read from serial)
sleep(5)										# wait for the motor to settle

# Enter main focusing loop
tic()											# Start timing right before we send the motor control signal
tic2()											# Start timing right before we send the motor control signal
interval = controlMotor(position,interval)		# Update position of the motor
toc2()											# Stop timing after Serial motor control is sent
position = readPosition(serial_port)			# Update the position with the real position (read from serial)
toc()											# Stop timing after the motor has reached its position
