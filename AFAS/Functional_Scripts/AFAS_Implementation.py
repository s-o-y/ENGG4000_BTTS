# Author: Zachary Demerson
# Last Updated: April 2, 2024
# Description:
#	Code for automatic focus adjustment system for BBST.
#	Implemented on Raspberry Pi 4. Ensure enable_UART = 1 in boot/config.txt
#	Uses fourier analysis to determine the level of focus of a lens.
#	Interfaces with a motor controller to adjust lens focal length.
#	Communicates with servo motor controller (OpenRB-150) via UART serial comms on GPIO ports 14 and 16.
#	Sends an integer between 0 and 4095 indicating the angular position of the motor.
#	Nudges motor position in a direction and determines if focus is getting better or worse.
#	If a global max is found, the motor is sent there and the normal loop is reinitiated.
#	Receives serial comms from RB-150 containing the real position of the motor.
#	Accuracy of positional commands on RB-150 is related to motor speed. Motor speed is set on RB-150

import serial
import numpy as np
from picamera2 import Picamera2, Preview
import time
from time import sleep

# Class used to store data about any particular motor position
class iteration:
    def setEvaluation(self,evaluation):
        self.evaluation = evaluation
        
    def setPosition(self,position):
        self.position = position
        
    def setGettingBetter(self,gettingBetter):
        self.gettingBetter = gettingBetter

# Functions to measure time
def TicTocGenerator():
    ti = 0
    tf = time.time()
    while True:
        ti = tf
        tf = time.time()
        yield tf-ti

# Initialize Tic Toc Generators to measure different time metrics.
#=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
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
        print("Total Focusing Time: %f seconds." %tempTimeInterval)
        
def tic2():
    toc2(False)
    
TicToc3 = TicTocGenerator()

def toc3(tempBool=True):
    tempTimeInterval = next(TicToc3)
    if tempBool:
        print("Total Image Processing Time: %f seconds." %tempTimeInterval)
        
def tic3():
    toc3(False)

TicToc4 = TicTocGenerator()

def toc4(tempBool=True):
    tempTimeInterval = next(TicToc4)
    if tempBool:
        print("Time To Send Signal to the Motor: %f seconds." %tempTimeInterval)
        
def tic4():
    toc4(False)
#=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

# Description:
#	Adjusts the position of the Dynamixel motor by a specified interval.
#	Calculates new position and uses UART to send the new position to the OpenRB-150
# Inputs:
#	position - Type int contains the position of the motor prior to adjustment (0 to 4095 positions)
#	interval - Type int contains the amount to adjust the position by.
#	direction - Type int contains the direction to turn the motor: -1 = clockwise, +1 = counterclockwise
# Outputs:
#	direction - suggested direction for next adjustment: direction will change if a positional boundary is reached
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

# Description:
#	Takes a camera object, uses it to take a photo and the determine the fft of it
# Inputs:
#	camera - PiCamera2() object initialized used to take an image
# Outputs:
#	fft - Type 2D numpy array containing the fft of the image taken 'photo'
def getFFT(camera):
    photo = camera.capture_image("main").convert('L')
    photoNP = np.array(photo)
    photoFFT = np.fft.fft2(photoNP)
    fft = np.log(np.abs(np.fft.fftshift(photoFFT))**2)
    return fft

# Description:
#	Takes a camera object, then takes a photo, calculates the fft, then returns the fft's radial average array
# Inputs:
#	camera - PiCamera2() object initialized used to take an image
# Outputs:
#	radialAvg - 1D numpy array containing the radial average of fft about the center of the image
def getRadialAverage(camera):
    fft = getFFT(camera)
    # Create a matrix containing the radius from the center of the matrix
    x,y = np.meshgrid(np.arange(fft.shape[1]),np.arange(fft.shape[0]))
    x0 = fft.shape[1]/2						# x0 is the center x coordinate
    y0 = fft.shape[0]/2						# y0 is the center y coordinate
    radii = np.sqrt((x-x0)**2+(y-y0)**2)	# radii is a matrix with the same size as fft. Each element is the distance from the origin
    
    # mean_fun: Take the mean of all the indices in the FFT where our radii matrix = lambda variable rad
    mean_fun = lambda rad : fft[(radii >= rad-0.5) & (radii < rad+0.5)].mean()
    rad = np.linspace(1,(fft.shape[0]-1)/2,num=int((fft.shape[0]-1)/2))			# lambda variable - shortest distance from (0,0) to edge of fft
    radialAvg = np.vectorize(mean_fun)(rad)										# Calculate radial average over rad
    return radialAvg

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

# Runs on startup.
# Sweeps the entire range of positional values to find the point of best focus.
# This is done to ensure that the system begins with a known position value to utilize in the faster algorithm.
def startupFocus(max_motor_position,min_motor_position,serial_port,camera,interval,direction):
    print("Entering Startup Focusing Procedure - Sweeping All Positions")
    position = max_motor_position						# Start motor position at max
    finished = False									# Flag to end focusing loop
    
    loopIndex = 0
    gettingWorseCounter = 0
    maxFound = False
    signChangeOccurred = False

    positionRange = []				# initialize list to store position values
    meanRadialAverageRange = []		# initialize list to store comparison metric
    compareMRA = []					# initialize list to store if focus is getting better (1) or worse (-1) (NULL first value)

    # Enter main focusing loop
    while(not finished):
        direction = controlMotor(position,interval,direction)		# Update position of the motor
        
        '''
        # For Debugging:
        print("bytes written: "+str(bytesWritten))
        print("Desired Position: " + str(position))
        print("Desired Position (Bytes): " + str(message))
        '''
        
        position = readPosition(serial_port)			# Update the position with the real position (read from serial)
        
        radialAverageArray = getRadialAverage(camera)	# Determine radial average array at new position
        meanRadialAvg = radialAverageArray.mean()		# Condense array down to one number
        
        # For Debugging:
        print("Position: "+str(position)+" Mean Radial Average: "+str(meanRadialAvg))
        
        positionRange.append(position)					# Store current position
        meanRadialAverageRange.append(meanRadialAvg)	# Store current mean radial average
        
        # If Block: State analysis to determine if a global max has been found
        # Store None for the first element of the array (Can't compare with only one value)
        if(loopIndex-1<0):
            compareMRA.append(None)
        else:
            # Determine if focus got better or worse after motor adjustment (1 = better, -1 = worse)
            differenceMRA = meanRadialAverageRange[loopIndex]-meanRadialAverageRange[loopIndex-1]
            compareMRA.append((differenceMRA)/abs(differenceMRA))
            thisCompare = compareMRA[loopIndex]		# Better or worse this increment
            lastCompare = compareMRA[loopIndex-1]	# Better or worse last increment
            if(lastCompare is not None and thisCompare != lastCompare):		# If direction of focus changed
                if(thisCompare == -1):										# If focus is getting worse now
                    gettingWorseCounter = 1									# Start counting to 3 in a row to make sure its a global max
                    maxFound = True											# Flag indicating stability countdown is started
                elif(maxFound):												# If it changed during countdown -> local maximum so keep going
                    maxFound = False										# Reset countdown flags
                    gettingWorseCounter = 0
            elif(maxFound):						# If it keeps getting worse, increment countdown
                gettingWorseCounter += 1
                
        loopIndex += 1
        
        # Once global max found or the whole range has been swept, exit loop
        if(gettingWorseCounter == 3 or position == min_motor_position):
            finished = True

    bestFocusIndex = meanRadialAverageRange.index(max(meanRadialAverageRange))	# Get the index with the highest mean radial average
    bestFocusPosition = positionRange[bestFocusIndex]							# get the position value of best focus
    interval = abs(bestFocusPosition - position)										# Calculate how much to move to get to the position of best focus
    direction = (bestFocusPosition - position)/abs(bestFocusPosition - position)
    direction = controlMotor(position,interval,direction)									# Move motor to best spot
    position = readPosition(serial_port)										# Update position variable
    lastFocusedMRA = getRadialAverage(camera).mean()							# Update our benchmark MRA
    testingCount = 0															# Reset testing counter
    
    print("Best Focus Position: "+str(bestFocusPosition)+" Final Real Position: "+str(position))
    print("Best Radial Average: "+str(meanRadialAverageRange[bestFocusIndex])+" Final Radial Average: "+str(lastFocusedMRA))
    return position

# Main Script:

# Init Block:
# Initialize the Pi Camera
camera = Picamera2()
config = camera.create_preview_configuration(main = {"size": (512,512)},lores = {"size": (512,512)})
camera.configure(config)
camera.start_preview(Preview.QTGL)
camera.start()

serial_port = serial.Serial("/dev/ttyS0", 115200)	# Using RPi 4 default serial port (Mini-UART on Pin 8 (TX) and Pin 10 (RX))
focusingDelay = 10									# Time in seconds to wait before checking if focus needs to be adjusted
bufferMRA = 0.2										# Allowable difference between the MRA at focusing (lastFocusedMRA) and the current MRA (testMetric)
max_motor_position = 3648							# Max far position - focused at infinity
min_motor_position = 2409							# Furthest near position without hitting bracket
interval = 50									# How many positions to move the motor at a time (motor has range 0 to 4095)
direction = 1

testingCount = 0									# Counter for iterations without significatn change in focus (Force focusing after 3 for testing purposes) 

position = startupFocus(max_motor_position,min_motor_position,serial_port,camera,interval,direction)

# Determine the mean radial average of the focused image
lastFocusedMRA = getRadialAverage(camera).mean()

# For Debugging:
print("Mean Radial Average After Startup Focus: "+str(lastFocusedMRA))

while(True):
    interval = 50									# How many positions to move the motor at a time (motor has range 0 to 4095)
    direction = 1
    sleep(focusingDelay)							# Only check if lens needs focusing at some interval focusingDelay seconds
    testMetric = getRadialAverage(camera).mean()	# Determine the radial average at this instant
    
    '''
    # Testing Purposes - After a few loops, change the focus to start the focusing loop
    if(testingCount >= 3):
        controlMotor(position,interval,direction)				# Send motor to end
        position = readPosition(serial_port)	# update motor position/wait for motor to finish moving
        testMetric = getRadialAverage(camera).mean()	# Determine the radial average at this instant
    '''
    
    # For Debugging:
    print("Current Radial Average "+str(testMetric)+" Last Focused Radial Average: "+str(lastFocusedMRA))
    
    # If the focus is better than when we last focused it, update the benchmark metric
    if(lastFocusedMRA < testMetric):
        lastFocusedMRA = testMetric
        testingCount += 1
    # If the focus has deteriorated more than our bufferMRA, Enter focusing loop
    elif(lastFocusedMRA-bufferMRA > testMetric):
        finished = False
        print("Entering Focus Loop - Benchmark MRA Exceeded")
        currentIteration = iteration()
        lastIteration = iteration()
        
        currentIteration.setPosition(position)
        currentIteration.setEvaluation(testMetric)
        currentIteration.setGettingBetter(None)
        camera.capture_file("/home/zdemerso/Desktop/Demo Code/StartImage.jpg")
        tic2()	# Start timing total focus time before entering focusing loop
        
        while(not finished):
            lastIteration.setPosition(currentIteration.position)
            lastIteration.setEvaluation(currentIteration.evaluation)
            lastIteration.setGettingBetter(currentIteration.gettingBetter)
            
            direction = controlMotor(lastIteration.position,interval,direction)
            currentIteration.setPosition(readPosition(serial_port))
            
            tic3()	# Start measuring total image capture and processing time before grabbing image
            currentIteration.setEvaluation(getRadialAverage(camera).mean())
            toc3()	# Stop measuring total image capture and processing time after mean radial average of the image is measured
            
            # For Debugging:
            print("Position: "+str(currentIteration.position)+" Mean Radial Average: "+str(currentIteration.evaluation)+" Getting Better: "+str(currentIteration.gettingBetter))
            print("Last Position: "+str(lastIteration.position)+" Last Mean Radial Average: "+str(lastIteration.evaluation)+" Last Getting Better: "+str(lastIteration.gettingBetter))
            
            # Set Getting Better
            if(currentIteration.evaluation > lastIteration.evaluation):
                currentIteration.setGettingBetter(True)
            else:
                currentIteration.setGettingBetter(False)
            
            # State Analysis
            if(currentIteration.gettingBetter):
                pass
            elif(lastIteration.gettingBetter):
                finished = True
            else:
                direction = direction * -1
            
            # Handle Extremes
            if(currentIteration.position + (direction * interval) < min_motor_position or currentIteration.position + (direction * interval) > max_motor_position):
                direction = direction * -1
            
        direction = direction * -1
        tic4()
        tic()	# Start measuring total motor response time before calling control motor
        direction = controlMotor(currentIteration.position,interval,direction)
        toc4()
        currentIteration.setPosition(readPosition(serial_port))
        toc()	# Stop measureing total motor response time after the motor has reached its final position
        # Note: readPosition polls serial line from RB-150 until the position is sent back
        # 		The RB-150 only sends the position once position has stabilized (not moving anymore)
        
        currentIteration.setEvaluation(getRadialAverage(camera).mean())
        
        print("Best Focus Position: "+str(lastIteration.position)+" Final Real Position: "+str(currentIteration.position))
        print("Best Radial Average: "+str(lastIteration.evaluation)+" Final Radial Average: "+str(currentIteration.evaluation))
        
        toc2()	# Stop timing total focusing time after final position is reached
        position = currentIteration.position
        testingCount = 0
        lastFocusedMRA = getRadialAverage(camera).mean()
        camera.capture_file("/home/zdemerso/Desktop/Demo Code/EndImage.jpg")
    # Increment Testing counter
    else:
        testingCount += 1

