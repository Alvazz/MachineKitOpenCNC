// 1/T = 1/t0+1/b*ln(R/R0);
#include <math.h>
#include <PID_v1.h>

// tempperature calculation parameters
const double KELVIN = 273.15; // Kelvin to Celcuis
const double BETA = 3950;
const double T0 = 25 + KELVIN;
const double R0 = 100e3;

// ADC parameters
const int adcBits = 12;
const double fullADC = 4096;

// system parameters
double setTemp = 230;
static double temperature;
const double equilibriumDutyCycle = 55;
static double dutyCycle = 255-equilibriumDutyCycle; // -70 to 185
const int pin = 3;
static bool isReached = false;

// pid parameters
double KP = 3;
double KI = 0;
double KD = 1;

PID tempPID(&temperature, &dutyCycle, &setTemp,KP,KI,KD, P_ON_M, DIRECT);

double thermistor(int rawADC) {
  double temperature;
  double R1= R0*(fullADC/rawADC-1);

  temperature = 1 / (1/T0+1/BETA*log(R1/R0)) - KELVIN;// Celcius
  //temperature = (temperature * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
  return temperature;
}

double getTemp(){
  uint8_t numReadings = 100;
  double total = 0;
  for (uint16_t index = 0; index<numReadings; index++){
    total += thermistor(analogRead(A0));
    delay(1);
  }
  total /= numReadings;
  return total;
}

void setup() {
  pinMode(pin,OUTPUT);
  Serial.begin(115200);
  analogReadResolution(12);
  tempPID.SetOutputLimits(-equilibriumDutyCycle, 255-equilibriumDutyCycle);
  tempPID.SetMode(AUTOMATIC);
}

void loop() {
  //
  temperature = getTemp();
  if (setTemp - temperature > 30){
    KP = 20;
    KI = 0;
    KP = 0.1;
    tempPID.SetTunings(KP, KI, KP);
    tempPID.Compute();
    Serial.print(temperature);  // display Fahrenheit
    Serial.print("\t");
    Serial.println(dutyCycle+equilibriumDutyCycle);
    analogWrite(pin,(int)(dutyCycle+equilibriumDutyCycle));
  } else {
    KP = 50;
    KI = 0.5;
    KP = 0.1;
    tempPID.SetTunings(KP, KI, KP);
    tempPID.Compute();
    Serial.print(temperature);  // display Fahrenheit
    Serial.print("\t\t");
    Serial.println(dutyCycle+equilibriumDutyCycle);
    analogWrite(pin,(int)(dutyCycle+equilibriumDutyCycle));
  }
  delay(20);
}
