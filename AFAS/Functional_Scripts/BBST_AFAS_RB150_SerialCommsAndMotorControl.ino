// Author: Zachary Demerson
// Last Updated: March 11 2024
// Description:
//  Code for automatic focus adjustment system. Implemented on OpenRB-150
//  Receives serial data from Raspberry Pi 4 and uses it to control
//  a Dynamixel servo motor (XL330-M288-T)
//  Relays positional data back to Raspberry Pi after adjustment.

#include <Dynamixel2Arduino.h>
                                          // Port uses are specific to RB-150
#define DXL_SERIAL Serial1                // Serial 1 = Servo comms port
#define DEBUG_SERIAL Serial               // Serial = USB-C comms port (for debugging)
#define RPI_SERIAL Serial2                // Serial 2 = Board serial port. Connected to RPi for this purpose
const int DXL_DIR_PIN = -1;               // Direction pin is unused
const uint8_t DXL_ID = 1;                 // Assign and ID for the motor
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN); // Instantiate the servo motor object

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 0);

  DEBUG_SERIAL.begin(115200);
  RPI_SERIAL.begin(115200);
  return;
}

// new_position = Desired position of the motor (Range 0 - 4095, but limited by movement range of lens ring). 
// Positive = counter clockwise (focusing far), Negative = clockwise (focusing near)
int setMotorPosition(int new_position){
  int max_position = 3396; // Focused at infinity
  int min_position = 2553; // Furthest near position without hitting bracket
  int present_position;
  
  if(new_position > max_position){
    dxl.setGoalPosition(DXL_ID, max_position);              // If over max, only go to max
    present_position = waitForMotorToStop(max_position);
  }
  else if(new_position < min_position){                     // If under min, only go to min
    dxl.setGoalPosition(DXL_ID, min_position);
    present_position = waitForMotorToStop(min_position);
  }
  else{
    dxl.setGoalPosition(DXL_ID, new_position);              //Change the position of the motor to the received position
    present_position = waitForMotorToStop(new_position);
  }
  return present_position;
}

int waitForMotorToStop(int currentPosition){
  delay(20);
  int lastPosition = currentPosition;
  currentPosition = dxl.getPresentPosition(DXL_ID);
  
  while(currentPosition != lastPosition){
    delay(5);
    lastPosition = currentPosition;
    currentPosition = dxl.getPresentPosition(DXL_ID);
  }
  
  return currentPosition;
}

void loop() {  
  byte incomingByte;            // Singular byte read from the serial buffer
  byte receivedMessage[2];      // Array to store the full message (two bytes)
  byte endMarker = 'A';         // Marker to signify end of messages (\n = 0bA)
  bool newData = false;         // flag for if a new message has been fully received
  int i = 0;                    // iterator for receivedMessage
  uint16_t receivedInt = 9999;  // Received message converted from hex to int (9999 as a flag)

  /*
  // For Debugging:
  int present_position = dxl.getPresentPosition(DXL_ID);
  DEBUG_SERIAL.print("Current Motor Position: ");
  DEBUG_SERIAL.println(present_position); //Far: 2854, Near: 2553
  */
  
  // If theres a message in the buffer and we havent finished reading the data yet
  while(RPI_SERIAL.available() > 0){
    incomingByte = RPI_SERIAL.read();               // Read the message as a singular byte      
    receivedMessage[i] = incomingByte;              // Store byte in array

    
    // For debugging:
    DEBUG_SERIAL.print("Array Contents: ");
    DEBUG_SERIAL.println(receivedMessage[i],HEX);
    DEBUG_SERIAL.print("Raw Serial Read Byte: ");
    DEBUG_SERIAL.println(incomingByte,HEX);
    
    
    i=i+1;                                          // Increment array index

    if(RPI_SERIAL.available() == 0){                // If the end of the message, throw flag   
      newData = true;                               // Flag indicates there is a new instruction
      
      // For Debugging:
      DEBUG_SERIAL.println("END OF MESSAGE");     
      
    }
  }
  
  // NOTE: Serial comms from Raspberry Pi are Big-Endian
  // Assemble byte array into an int
  receivedInt = (receivedMessage[0] << 8) + (receivedMessage[1]);
  
  // When a new instruction is received:
  if(newData){
    int present_position = setMotorPosition(receivedInt);
    byte transmit[2];                               // Initialize byte array for transmission
    transmit[0] = (present_position >> 8) & 0xFF;   // First element = most significant byte of int present position
    transmit[1] = present_position & 0xFF;          // Second element = least significant byte of int present position
    if(RPI_SERIAL.availableForWrite() > 0){         // Send present position to RPI4 over serial bus
      RPI_SERIAL.write(transmit,2);
    
    //For debugging:
    DEBUG_SERIAL.print("Decoded Message: ");
    DEBUG_SERIAL.println(receivedInt);
    DEBUG_SERIAL.print("Current Motor Position: ");
    DEBUG_SERIAL.println(present_position); //Far: 2854, Near: 2553
    DEBUG_SERIAL.println();
    }
  }
}
