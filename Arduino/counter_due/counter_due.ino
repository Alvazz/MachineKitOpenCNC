#include <SPI.h>


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


// CS pin
#define SS 4
#define CLK DAC1

// 4 byte counter
union byte4{
   unsigned long comb;
   uint8_t bytes[4];
};

void ini_spi(uint8_t ss_pin) {
  pinMode(CLK, OUTPUT);
  pinMode(ss_pin, OUTPUT);
  SPI.begin(ss_pin);
  digitalWrite(ss_pin, HIGH);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SS, SPI_MODE0);
  SPI.setClockDivider(SS, 200);
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
  SPI.transfer(ss_pin, op_code, SPI_CONTINUE);
  res = SPI.transfer(ss_pin, NON, SPI_LAST);
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

// responses for 1 byte and 4 bytes.
uint8_t rx_data1;
unsigned long rx_data4;

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
  ini_spi(SS);
  Serial.begin(9600);
  delay(100);

  // set MRD0 and MRD1
  write_1_byte(SS, WRITE_MDR0, QUADRX4 | FREE_RUN | DISABLE_INDX);
  write_1_byte(SS, WRITE_MDR1, BYTE_4 | EN_CNTR | NO_FLAGS);

  // clear STR and CNTR
  clr_load_reg(SS, CLR_STR);
  clr_load_reg(SS, CLR_CNTR);

  // check CNTR and STR
  rx_data4 = read_4_byte(SS, READ_CNTR);
  Serial.print("CNTR check: ");
  Serial.println(rx_data4, DEC);
  delay(5);
  rx_data1 = read_1_byte(SS, READ_STR);
  Serial.print("STR check: ");
  Serial.println(rx_data1, BIN);
  delay(10);
}

void loop() {
  rx_data4 = read_4_byte(SS, READ_CNTR);
  Serial.print("CNTR is ");
  Serial.println(rx_data4, DEC);
  delay(200);
}
