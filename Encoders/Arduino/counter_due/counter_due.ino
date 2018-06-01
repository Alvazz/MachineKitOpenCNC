#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <DueTimer.h>
#include "counter.h"

/*** Usage Notes ***/
//
//
//
//


//#define SS6 8
//#define setCmdTimeout 1


//#define numAxes 6


const uint32_t timerFreq = 1000;

//Structure for result of command read
typedef struct {
  char cmd;
  int cmdResult;
  
  //Received data
  int bytesRead;

  //Returned data
  uint32_t encoderCounts[numAxes];
} commandData_t;

typedef struct {
  uint32_t encoder1pos;
  uint32_t encoder2pos;
  uint32_t encoder3pos;
  uint32_t encoder5pos;
} stateData_t;

void waitForByte(&byteValue) {
  while (Serial.available() == 0) {
    delayMicroseconds(1);
  }
  *byteValue = Serial.read() - '0';
}

commandData_t readCommand(void) {
  //Check for command string
  
  char incomingByte;
  char incomingBytes[64];
  //int byteCount = 0;
  //int retVal[numAxes];
  
  commandData_t commandData;
  commandData.cmd = 'E';
  commandData.cmdResult = 0;
  
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    
    //int axis = 0;
    uint32_t byteCount = 0;
    uint32_t bytesToRead = 0;
    uint32_t targetIC = 0;
    //uint32_t setCount = 0;
    
    //Read command character
    switch (incomingByte) {
      
      case 'S':
        //"SET" encoder counts for all axes: syntax "SXYYYY", X is numberof digits in desired counter setting YYYY
        //"SET" encoder counts for all axes: syntax "SYXZZZZ", X is numberof digits in desired counter setting ZZZZ for axis Y
        //Serial.println("Setting encoder count");
        commandData.cmd = 'S';
        
        //wait for the rest of the command
//        while (Serial.available() == 0) {
//          delayMicroseconds(1);
//        }
        waitForByte(&targetIC);
        waitForByte(&bytesToRead);
        
        
//        targetIC = Serial.read() - '0';
//        bytesToRead = Serial.read() - '0';
        
        Serial.println(targetIC);
        Serial.println(bytesToRead);
        
        while (Serial.available() < bytesToRead) {
          delayMicroseconds(1);
        }
        
        //uint32_t byteCount = 0;
        while (Serial.available() > 0) {
          incomingByte = Serial.read();
          incomingBytes[byteCount] = incomingByte;
          Serial.print("received ");
          Serial.println(incomingByte);
          byteCount += 1;
        }
        commandData.bytesRead = byteCount;

        if (commandData.bytesRead > 0) {
          //Have bytes to write to CNTRs
          uint32_t setCount = strtoul(incomingBytes, NULL, 10);
          Serial.print("setcount is ");
          Serial.println(setCount,DEC);
          //byte4 readCount;
          uint32_t readCount;
          commandData.cmdResult = 1;

          //Set DTR, load CNTR for each IC
          for (int axis = 0; axis < numAxes; axis++) {
            setCounter(axes[axis], setCount);
            readCount = readFourBytes(axes[axis], READ_CNTR);
            //Check for successful write
            if (readCount == setCount) {
              //Successful set for IC axes[axis]
              commandData.cmdResult &= 1;
            } else {
              //Write failure
              commandData.cmdResult &= 0;
            }
          }
          if (commandData.cmdResult) {
            Serial.print("SUCCESS: wrote ");
            Serial.print(setCount,DEC);
            Serial.println(" S&");
          }
          else {
            Serial.print("FAILURE: did not write ");
            Serial.print(setCount,DEC);
            Serial.println(" F&");
          }
        }
        break;
        
      case 'G':
        //"GET" encoder counts for all axes
        commandData.cmd = 'G';
        //int axisCounts[numAxes];
        if (!Serial.available()) {
          //No more bytes to read, return encoder counts
          for (int axis = 0; axis < numAxes; axis++) {
            //axisCounts[axis] = (int) readFourBytes(axes[axis], READ_CNTR);
            commandData.encoderCounts[axis] = readFourBytes(axes[axis], READ_CNTR);
          }
          //retVal = axisCounts;
          //Assume success
          commandData.cmdResult = 0;
          //commandData.encoderCounts = axisCounts;
        } else {
          //Error, wrong command string
          commandData.cmdResult = -1;
        }
        break;
    }
  } else {
    //No bytes to read
    commandData.cmd = 'N';
    commandData.cmdResult = 0;
  }
  return commandData;
}

/* Timerx ISR */
void timerCallBack(){

}// samplingCallBack


void setup() {
  
  //initialize spi and serial port
  initSpi(axes);
  
  Serial.begin(115200);
  //Serial.begin(250000);
  delay(100);

  // initialize counter by setting control registers and clearing flag & counter registers
  initCounter(axes, 10000);

  //Force CNTR to 1e6
  for (int axis = 0; axis < numAxes; axis++) {
    setCounter(axes[axis], INI_CNTR);
  }
 
  // initialize timer
  //noInterrupts();
  //initTimer(timerCallBack, timerFreq);
  //interrupts();
}

void loop() {

  //int command;
  commandData_t commandData;
  commandData = readCommand();
  if (commandData.cmdResult == 0) {
    //Success on read/process command
    switch (commandData.cmd) {
      case 'G':
        //Command request from master
        //int countOutput;
        for (int axis = 0; axis < numAxes-1; axis++) {
          Serial.print(commandData.encoderCounts[axis], DEC);
          Serial.print(' ');
        }
        Serial.print(commandData.encoderCounts[numAxes-1]);
        //Serial.println("done");
        Serial.println(" C&");
        break;
    }
  } else {
    //Serial.print("Command error with ");
    //Serial.println(commandData.cmd);
  }
  //command = readCommand();
   
//  if (command >= 0) {
//    Serial.println("resetting counter");
//    setCounter(SS1, command);
//    setCounter(SS2, command);
//    setCounter(SS3, command);
//  }
  
  //forceEncoderCount = readEncoderResetCommand();
  //Serial.println(count);
  
  //delay(1);
  //delayMicroseconds(50);
}
