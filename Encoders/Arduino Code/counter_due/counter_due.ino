#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>

/***   MDR0   ***/
//Count modes
#define NQUAD         0x00        //non-quadrature mode 
#define QUADRX1       0x01        //X1 quadrature mode 
#define QUADRX2       0x02        //X2 quadrature mode 
#define QUADRX4       0x03        //X4 quadrature mode 

//Running modes
#define FREE_RUN      0x00
#define SINGE_CYCLE   0x04
#define RANGE_LIMIT   0x08
#define MODULO_N      0x0C

//Index modes
#define DISABLE_INDX  0x00        //index_disabled 
#define INDX_LOADC    0x10        //index_load_CNTR 
#define INDX_RESETC   0x20        //index_rest_CNTR 
#define INDX_LOADO    0x30        //index_load_OL 
#define ASYNCH_INDX   0x00        //asynchronous index 
#define SYNCH_INDX    0x80        //synchronous index 

//Clock filter modes
#define FILTER_1      0x00        //filter clock frequncy division factor 1 
#define FILTER_2      0x80        //filter clock frequncy division factor 2 


/***   MDR1   ***/
//Flag modes
#define NO_FLAGS      0x00        //all flags disabled           
#define IDX_FLAG      0x10        //IDX flag 
#define CMP_FLAG      0x20        //CMP flag 
#define BW_FLAG       0x40        //BW flag 
#define CY_FLAG       0x80        //CY flag 

//1 to 4 bytes data-width
#define BYTE_4        0x00        //four byte mode           
#define BYTE_3        0x01        //three byte mode           
#define BYTE_2        0x02        //two byte mode           
#define BYTE_1        0x03        //one byte mode  

//Enable/disable counter
#define EN_CNTR       0x00        //counting enabled 
#define DIS_CNTR      0x04        //counting disabled 


/***   LS7366R op-code list   ***/
#define CLR_MDR0      0x08
#define CLR_MDR1      0x10
#define CLR_CNTR      0x20
#define CLR_STR       0x30
#define READ_MDR0     0x48
#define READ_MDR1     0x50
#define READ_CNTR     0x60
#define READ_OTR      0x68
#define READ_STR      0x70

#define WRITE_MDR1    0x90
#define WRITE_MDR0    0x88
#define WRITE_DTR     0x98
#define LOAD_CNTR     0xE0
#define LOAD_OTR      0xE4

#define NON           0x00
#define INI_CNTR      1000000

// CS pin
#define SS1 4
#define SS2 5
#define SS3 6

//Number of ICs to read
#define numAxes 3
//#define axes[numAxes] = {SS1, SS2, SS3}

static int axes[numAxes] = {SS1, SS2, SS3};

// 4 byte counter
union byte4{
   unsigned long comb;
   uint8_t bytes[4];
};

//Structure for result of command read
typedef struct {
  char cmd;
  int cmdResult;
  
  //Received data
  int bytesRead;

  //Returned data
  int encoderCounts[numAxes];
} commandData_t;

void ini_spi(uint8_t ss_pin) {
  pinMode(ss_pin, OUTPUT);
  SPI.begin(ss_pin);
  digitalWrite(ss_pin, HIGH);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(ss_pin, SPI_MODE0);
  SPI.setClockDivider(ss_pin, 10);
  delay(5);
}

// write 1 byte to spi
void clr_load_reg(uint8_t ss_pin, uint8_t op_code) {
  digitalWrite(ss_pin, LOW);
  SPI.transfer(ss_pin, op_code, SPI_LAST);
  digitalWrite(ss_pin, HIGH);
}

// write 2 bytes to spi
void write_1_byte(uint8_t ss_pin, uint8_t op_code, uint8_t op_data) {
  digitalWrite(ss_pin, LOW);
  SPI.transfer(ss_pin, op_code, SPI_CONTINUE);
  SPI.transfer(ss_pin, op_data, SPI_LAST);
  digitalWrite(ss_pin, HIGH);
}

// write 5 bytes to spi
void write_4_byte(uint8_t ss_pin, uint8_t op_code, byte4 op_data) {
  digitalWrite(ss_pin, LOW);
  SPI.transfer(ss_pin, op_code, SPI_CONTINUE);
  SPI.transfer(ss_pin, op_data.bytes[3], SPI_CONTINUE);
  SPI.transfer(ss_pin, op_data.bytes[2], SPI_CONTINUE);
  SPI.transfer(ss_pin, op_data.bytes[1], SPI_CONTINUE);
  SPI.transfer(ss_pin, op_data.bytes[0], SPI_LAST);
  digitalWrite(ss_pin, HIGH);
}

// write 1 byte then read 1 byte through spi
uint8_t read_1_byte(uint8_t ss_pin, uint8_t op_code) {
  uint8_t res;
  digitalWrite(ss_pin, LOW);  
  SPI.transfer(ss_pin, op_code, SPI_CONTINUE);
  res = SPI.transfer(ss_pin, NON, SPI_LAST);
  digitalWrite(ss_pin, HIGH);
  return res;
}

// write 1 byte then read 4 bytes through spi
unsigned long read_4_byte(uint8_t ss_pin, uint8_t op_code) {
  byte4 res;
  digitalWrite(ss_pin, LOW);
  SPI.transfer(ss_pin, op_code, SPI_CONTINUE);
  res.bytes[3] = SPI.transfer(ss_pin, NON, SPI_CONTINUE);
  res.bytes[2] = SPI.transfer(ss_pin, NON, SPI_CONTINUE);
  res.bytes[1] = SPI.transfer(ss_pin, NON, SPI_CONTINUE);
  res.bytes[0] = SPI.transfer(ss_pin, NON, SPI_LAST);
  digitalWrite(ss_pin, HIGH);
  return res.comb;
}

// Counter initialization
void initCounter(uint8_t ss_pin, int count) {
  byte4 binCount;
  binCount.comb = count;
  write_4_byte(ss_pin, WRITE_DTR, binCount);
  //Move written data from DTR to CNTR register
  clr_load_reg(ss_pin, LOAD_CNTR);
}

// responses for 1 byte and 4 bytes.
uint8_t rx_data1;
uint8_t rx_data2;
uint8_t rx_data3;
unsigned long rx_data41;
unsigned long rx_data42;
unsigned long rx_data43;

void setup() {
  /*  setup PWM on pin: DAC1
   *  freq = 500K
   *  dutycycle = 50%
   */
  REG_PMC_PCER1 |= PMC_PCER1_PID36;  // Enable PWM
  REG_PIOB_ABSR |= PIO_ABSR_P16;    // Set PWM pin perhipheral type A or B, in this case B
  REG_PIOB_PDR |= PIO_PDR_P16;    // Set PWM pin to an output
  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(1);  // Set the PWM clock rate to 84MHz (84MHz/1)
  REG_PWM_CMR0 = PWM_CMR_CALG | PWM_CMR_CPRE_CLKA;  // Enable dual slope PWM and set the clock source as CLKA
  REG_PWM_CPRD0 = 84;  // Set the PWM frequency 84MHz/(2 * 84) = 500KHz
  REG_PWM_CDTY0 = 42;  // duty cycle
  REG_PWM_ENA = PWM_ENA_CHID0;  // Enable the PWM channel 
  
  //initialize spi and serial port
  ini_spi(SS1);
  ini_spi(SS2);
  ini_spi(SS3);
  //Serial.begin(115200);
  Serial.begin(250000);
  delay(100);

  // set MRD0 and MRD1
  write_1_byte(SS1, WRITE_MDR0, QUADRX4 | FREE_RUN | DISABLE_INDX);
  write_1_byte(SS1, WRITE_MDR1, BYTE_4 | EN_CNTR | NO_FLAGS);

  write_1_byte(SS2, WRITE_MDR0, QUADRX4 | FREE_RUN | DISABLE_INDX);
  write_1_byte(SS2, WRITE_MDR1, BYTE_4 | EN_CNTR | NO_FLAGS);

  write_1_byte(SS3, WRITE_MDR0, QUADRX4 | FREE_RUN | DISABLE_INDX);
  write_1_byte(SS3, WRITE_MDR1, BYTE_4 | EN_CNTR | NO_FLAGS);

  // clear STR and CNTR
  clr_load_reg(SS1, CLR_STR);
  clr_load_reg(SS1, CLR_CNTR);

  clr_load_reg(SS2, CLR_STR);
  clr_load_reg(SS2, CLR_CNTR);

  clr_load_reg(SS3, CLR_STR);
  clr_load_reg(SS3, CLR_CNTR);

  //Force CNTR to 1e6
  initCounter(SS1, INI_CNTR);
  initCounter(SS2, INI_CNTR);
  initCounter(SS3, INI_CNTR);

  // check CNTR and STR
//  rx_data41 = read_4_byte(SS1, READ_CNTR);
//  rx_data42 = read_4_byte(SS2, READ_CNTR);
//  rx_data43 = read_4_byte(SS3, READ_CNTR);
//  Serial.print("CNTR check 1: ");
//  Serial.println(rx_data41, DEC);
//  Serial.print("CNTR check 2: ");
//  Serial.println(rx_data42, DEC);
//  Serial.print("CNTR check 3: ");
//  Serial.println(rx_data43, DEC);
//  delay(5);
  
//  rx_data1 = read_1_byte(SS1, READ_STR);
//  rx_data2 = read_1_byte(SS2, READ_STR);
//  rx_data3 = read_1_byte(SS3, READ_STR);
//  Serial.print("STR check 1: ");
//  Serial.println(rx_data1, BIN);
//  Serial.print("STR check 2: ");
//  Serial.println(rx_data2, BIN);
//  Serial.print("STR check 3: ");
//  Serial.println(rx_data3, BIN);
//  delay(10);
}

//int incomingBytes = 0;

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
    int byteCount = 0;
    int bytesToRead = 0;
    //int setCount = 0;
    
    //Read command character
    switch (incomingByte) {
      
      case 'S':
        //"SET" encoder counts for all axes
        Serial.println("Setting encoder count");
        commandData.cmd = 'S';
        
        //wait for the rest of the command
        while (Serial.available() == 0) {
          delayMicroseconds(1);
        }
        bytesToRead = Serial.read() - '0';
        while (Serial.available() < bytesToRead) {
          delayMicroseconds(1);
        }
        
        //int byteCount = 0;
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
          int setCount = atoi(incomingBytes);
          //byte4 readCount;
          unsigned long readCount;
          commandData.cmdResult = 1;

          //Set DTR, load CNTR for each IC
          for (int axis = 0; axis < numAxes+1; axis++) {
            initCounter(axes[axis], setCount);
            readCount = read_4_byte(axes[axis], READ_CNTR);
            //Check for successful write
            if ((int) readCount == setCount) {
              //Successful set for IC axes[axis]
              commandData.cmdResult &= 1;
            } else {
              //Write failure
              commandData.cmdResult = -1;
            }
          }
        }
        break;
        
      case 'G':
        //"GET" encoder counts for all axes
        commandData.cmd = 'G';
        //int axisCounts[numAxes];
        if (!Serial.available()) {
          //No more bytes to read, return encoder counts
          for (int axis = 0; axis < numAxes+1; axis++) {
            //axisCounts[axis] = (int) read_4_byte(axes[axis], READ_CNTR);
            commandData.encoderCounts[axis] = (int) read_4_byte(axes[axis], READ_CNTR);
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
        

//    }
//    if (incomingByte == 'S') {
//      
//    }
//    else {
//      incomingBytes[byteCount] = incomingByte;
//      //Serial.print("I received: ");
//      //Serial.println(incomingByte);
//      byteCount += 1;
//    }
//  }
//  return atoi(incomingBytes);
//}

void loop() {
//  rx_data41 = read_4_byte(SS1, READ_CNTR);
//  rx_data42 = read_4_byte(SS2, READ_CNTR);
//  rx_data43 = read_4_byte(SS3, READ_CNTR);
//  //Serial.print("CNTR is ");
//  Serial.print(rx_data41, DEC);
//  Serial.print(' ');
//  Serial.print(rx_data42, DEC);
//  Serial.print(' ');
//  Serial.println(rx_data43, DEC);

//  int count;
//  if (readCommand()) {
//    
//  }

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
        Serial.println(commandData.encoderCounts[numAxes-1]);
        break;
    }
  } else {
    //Serial.print("Command error with ");
    //Serial.println(commandData.cmd);
  }
  //command = readCommand();
   
//  if (command >= 0) {
//    Serial.println("resetting counter");
//    initCounter(SS1, command);
//    initCounter(SS2, command);
//    initCounter(SS3, command);
//  }
  
  //forceEncoderCount = readEncoderResetCommand();
  //Serial.println(count);
  
  //delay(1);
  //delayMicroseconds(50);
}
