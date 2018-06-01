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

const uint32_t timerFreq = 1000;

//Structure for result of command read
typedef struct {
  char cmd;
  bool readResult;
  bool cmdResult;
  bool cmdError;
  int bytesRead;
  int bytesToRead;
  char commandString[BUFFER_SIZE];

  //SET command specific items
  uint32_t targetIC;
  uint32_t setCount;

  //BANDWIDTH command specific items
  uint32_t newBandwidth;
} commandData_t;

typedef struct {
  uint32_t encoderCounts[numAxes];
  bool encoderError[numAxes];
} stateData_t;

stateData_t encoderStateData;
commandData_t commandData;
static char inBuf[64];

void initializeStateData(stateData_t *stateData) {
  for (int axis = 0; axis < numAxes; axis++) {
    stateData->encoderCounts[axis] = 0;
    stateData->encoderError[axis] = false;
  }
}

void initializeCommandData(commandData_t *commandData) {
  commandData->cmd = 'N';
  commandData->readResult = false;
  commandData->cmdResult = false;
  commandData->cmdError = false;
  commandData->bytesRead = 0;

  commandData->targetIC = 0;
  commandData->newBandwidth = 0;
  
  memset(&commandData->commandString, '\0', BUFFER_SIZE*sizeof(char));
}

void waitForByte(uint32_t* byteValue) {
  while (Serial.available() == 0) {
    //delayMicroseconds(1);
  }
  *byteValue = Serial.read() - '0';
}

int waitForBytes(char* bufPosition, int bytesToRead) {
  int bytesRead = 0;
  while (bytesRead < bytesToRead) {
    if (Serial.available()) {
      *bufPosition = Serial.read();
      //Serial.println(*bufPosition);
      bufPosition++;
      bytesRead++;
    }
  }
  return bytesRead;
}

void resetCommandData(commandData_t *commandData) {
  commandData->cmd = 'N';
  commandData->readResult = false;
  commandData->cmdResult = false;
  commandData->cmdError = false;
  commandData->bytesRead = 0;
}

void resetEncoderError(stateData_t *stateData) {
  for (int axis = 0; axis < numAxes; axis++) {
    stateData->encoderError[axis] = false;
  }
}

void updateEncoderCounts(stateData_t *stateData) {
  for (int axis = 0; axis < numAxes; axis++) {
    stateData->encoderCounts[axis] = readFourBytes(axes[axis], READ_CNTR);
  }
}

void readCommand(stateData_t *stateData, commandData_t *commandData) {
  char incomingByte;
  
  //Check for command string
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    
    int command_string_index = 0;
    uint32_t bytesToRead = 0;
    uint32_t targetIC = 0;
    char* inBufPosition = inBuf;
    
    //Read command character
    commandData->commandString[command_string_index] = incomingByte;
    command_string_index++;
    
    switch (incomingByte) {
      case 'S':
        //"SET" encoder counts for all axes: syntax "SYXZZZZ", X is number of digits in desired counter setting ZZZZ for axis Y
        commandData->cmd = 'S';
        
        waitForByte(&targetIC);
        commandData->commandString[command_string_index] = targetIC;
        commandData->targetIC = targetIC;
        command_string_index++;
        waitForByte(&bytesToRead);
        commandData->commandString[command_string_index] = bytesToRead;
        command_string_index++;
        
        if (bytesToRead == waitForBytes(inBufPosition, bytesToRead)) {
//          Serial.print("Successful read of ");
//          Serial.print(bytesToRead, DEC);
//          Serial.println(" bytes");
          memcpy(&commandData->commandString[command_string_index],&inBuf,bytesToRead*sizeof(char));
          commandData->commandString[command_string_index+bytesToRead] = '&';
          commandData->bytesRead = bytesToRead;
          commandData->readResult = true;
          commandData->cmdError = false;
        } else {
          Serial.println("Read failure");
          commandData->readResult = false;
          commandData->cmdError = true;
        }

        if (commandData->bytesRead > 0) {
          //Have bytes to write to CNTRs
          long setCount = strtol(&commandData->commandString[command_string_index], NULL, 10);
          commandData->setCount = (uint32_t) setCount;
          commandData->cmdResult = true;

          switch (targetIC) {
            case 0:
              //Set DTR, load CNTR for each IC
              for (int axis = 0; axis < numAxes; axis++) {
                if (setCounter(axes[axis], setCount)) {
                  //Successful set for IC axes[axis]
                  commandData->cmdResult &= true;
                  stateData->encoderCounts[axis] = (uint32_t) setCount;
                  stateData->encoderError[axis] = false;
                } else {
                  //Write failure
                  commandData->cmdResult &= false;
                  stateData->encoderError[axis] = true;
                }
              }
              break;
              
            default:
              //Set DTR, load CNTR for only targetIC
              int targetICAddress = targetIC-1;
              if (setCounter(axes[targetICAddress], setCount)) {
                commandData->cmdResult = 1;
                stateData->encoderCounts[targetICAddress] = (uint32_t) setCount;
                stateData->encoderError[targetICAddress] = false;
              } else {
                commandData->cmdResult = 0;
                stateData->encoderError[targetICAddress] = true;
              }
          }
        }
        break;
        
      case 'G':
        //"GET" stored encoder counts for all axes
        commandData->cmd = 'G';
        commandData->cmdResult = true;
        break;
        
      case 'R':
        //"READ" encoder counts for all axes
        commandData->cmd = 'R';
        commandData->cmdResult = true;
        updateEncoderCounts(&encoderStateData);
        break;
        
      case 'B':
        //"BANDWIDTH" sets the speed of the UART link
        commandData->cmd = 'B';
        uint32_t newBandwidth;
        
        waitForByte(&bytesToRead);
        commandData->commandString[command_string_index] = bytesToRead;
        command_string_index++;
        
        if (bytesToRead == waitForBytes(inBufPosition, bytesToRead)) {
          memcpy(&commandData->commandString[command_string_index],&inBuf,bytesToRead*sizeof(char));
          commandData->commandString[command_string_index+bytesToRead] = '&';
          commandData->bytesRead = bytesToRead;
          commandData->readResult = true;
          commandData->cmdError = false;
        } else {
          Serial.println("Read failure");
          commandData->readResult = false;
          commandData->cmdError = true;
        }
        
        if (commandData->bytesRead > 0) {
          long newBandwidth = strtol(&commandData->commandString[command_string_index], NULL, 10);
          commandData->newBandwidth = (uint32_t) newBandwidth;
          //Serial.print("new bandwidth command is ");
          //Serial.println(newBandwidth,DEC);
          commandData->cmdResult = true;
        }
        break;
    }
  } else {
    //No bytes to read
    commandData->cmd = 'N';
  }
}

/* Timerx ISR */
void timerCallBack(){

}// samplingCallBack

void setup() {
  //initialize SPI and serial port
  initSpi(axes);
  Serial.begin(DEFAULT_BAUD);
  delay(100);

  // initialize counter by setting control registers and clearing flag & counter registers
  initCounters(axes);
  initializeStateData(&encoderStateData);
  initializeCommandData(&commandData);
  
  for (int axis = 0; axis < numAxes; axis++) {
    setCounter(axes[axis], INI_CNTR);
    encoderStateData.encoderCounts[axis] = (uint32_t) INI_CNTR;
  }
  
  // initialize timer
  //noInterrupts();
  //initTimer(timerCallBack, timerFreq);
  //interrupts();
}

void loop() {
  readCommand(&encoderStateData, &commandData);
  //Success on read/process command
  switch (commandData.cmd) {
    case 'G':
    case 'R':
      //Count request from master
      if (commandData.cmdResult) {
        Serial.print(commandData.cmd);
        for (int axis = 0; axis < numAxes; axis++) {
          Serial.print(' ');
          Serial.print(encoderStateData.encoderCounts[axis], DEC);
        }
        Serial.println(" &");
      } else {
        Serial.println("F&");
      }
      //Serial.print(stateData.encoderCounts[numAxes-1]);
      //Serial.println("done");
      //Serial.println("C&");
      
      break;
    case 'S':
      if (commandData.cmdResult) {
        Serial.println("S &");
      } else {
        Serial.println("F&");
      }
      break;
    case 'B':
      if (commandData.cmdResult) {
        Serial.print("B ");
        Serial.print(commandData.newBandwidth,DEC);
        Serial.println(" &");
        delay(100);
        Serial.begin(commandData.newBandwidth);
      }
    default:
    //NULL command
    break;
  }

  resetCommandData(&commandData);
  resetEncoderError(&encoderStateData);
}
