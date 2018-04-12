#ifndef COUNTER
#define COUNTER

//#define numAxes 6

// CS pin
#define SS1 4
#define SS2 5
#define SS3 6
#define SS4 8
#define SS5 7
#define SS6 9

//Number of ICs to read
const int numAxes = 5;
const uint8_t axes[numAxes] = {SS1, SS2, SS3, SS4, SS5};

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
#define INI_CNTR      10000

/*** Code Parameters ***/
#define setCmdTimeout 1

// 4 byte counter
union byte4{
   uint32_t comb;
   uint8_t bytes[4];
};

void initSpi(const uint8_t ss[]) {
  for (int iss=0; iss<numAxes; iss++){
    pinMode(ss[iss], OUTPUT);
    SPI.begin(ss[iss]);
    digitalWrite(ss[iss], HIGH);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(ss[iss], SPI_MODE0);
    SPI.setClockDivider(ss[iss], 32);
    delay(5);
  }
}

// write 1 byte to SPI
void clrLoadReg(const uint8_t ss_pin, const uint8_t op_code) {
  digitalWrite(ss_pin, LOW);
  SPI.transfer(ss_pin, op_code, SPI_LAST);
  digitalWrite(ss_pin, HIGH);
}

// write 2 bytes to SPI
void writeOneByte(const uint8_t ss_pin, const uint8_t op_code, const uint8_t op_data) {
  digitalWrite(ss_pin, LOW);
  SPI.transfer(ss_pin, op_code, SPI_CONTINUE);
  SPI.transfer(ss_pin, op_data, SPI_LAST);
  digitalWrite(ss_pin, HIGH);
}

// write 5 bytes to SPI
void writeFourBytes(const uint8_t ss_pin, const uint8_t op_code, const byte4 op_data) {
  digitalWrite(ss_pin, LOW);
  SPI.transfer(ss_pin, op_code, SPI_CONTINUE);
  SPI.transfer(ss_pin, op_data.bytes[3], SPI_CONTINUE);
  SPI.transfer(ss_pin, op_data.bytes[2], SPI_CONTINUE);
  SPI.transfer(ss_pin, op_data.bytes[1], SPI_CONTINUE);
  SPI.transfer(ss_pin, op_data.bytes[0], SPI_LAST);
  digitalWrite(ss_pin, HIGH);
}

// write 1 byte then read 1 byte through SPI
uint8_t readOneByte(const uint8_t ss_pin, const uint8_t op_code) {
  uint8_t res;
  digitalWrite(ss_pin, LOW);  
  SPI.transfer(ss_pin, op_code, SPI_CONTINUE);
  res = SPI.transfer(ss_pin, NON, SPI_LAST);
  digitalWrite(ss_pin, HIGH);
  return res;
}

// write 1 byte then read 4 bytes through SPI
uint32_t readFourBytes(const uint8_t ss_pin, const uint8_t op_code) {
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

void initCounter(const uint8_t ss[], const int count) {
  // set PWM for CLK
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
  
  for (int iss=0; iss<numAxes; iss++){
    // set MRD0 and MRD1
    writeOneByte(ss[iss], WRITE_MDR0, QUADRX4 | FREE_RUN | DISABLE_INDX);
    writeOneByte(ss[iss], WRITE_MDR1, BYTE_4 | EN_CNTR | NO_FLAGS);
    
    // clear STR and CNTR
    clrLoadReg(ss[iss], CLR_STR);
    clrLoadReg(ss[iss], CLR_CNTR);
  }
}

// set counter register to a desired number
void setCounter(const uint8_t ss_pin, const int count) {
  byte4 binCount;
  binCount.comb = count;
  writeFourBytes(ss_pin, WRITE_DTR, binCount);
  //Move written data from DTR to CNTR register
  clrLoadReg(ss_pin, LOAD_CNTR);
}

void initTimer(void (*isr)(), uint32_t timerFreq){
  Timer0.attachInterrupt(isr);
  Timer0.setFrequency(timerFreq);
  Timer0.start();
}

#endif
