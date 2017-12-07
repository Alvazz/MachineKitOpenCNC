// 1/T = 1/t0+1/b*ln(R/R0);
#include <math.h>
#include <PID_v1.h>

const double KELVIN = 273.15; // Kelvin to Celcuis
const double BETA = 3950;
const double T0 = 25 + KELVIN;
const double R0 = 100e3;
const int adcBits = 12;
const double fullADC = 4096;

static double temperature;
static double dutyCycle = 70; // 0 to 255
const int pin = 3;

double thermistor(int rawADC) {
  double temperature;
  double R1= R0*(fullADC/rawADC-1);

  temperature = 1 / (1/T0+1/BETA*log(R1/R0)) - KELVIN;// Celcius
  //temperature = (temperature * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
  return temperature;
}

double getTemp(){
  uint8_t numReadings = 30;
  double total = 0;
  for (uint16_t index = 0; index<numReadings; index++){
    total += thermistor(analogRead(A0));
    delay(10);
  }
  total /= numReadings;
  return total;
}

void setup() {
  pinMode(pin,OUTPUT);
  Serial.begin(115200);
  analogReadResolution(12);
  analogWrite(pin,(int)dutyCycle);
}

void loop() {
  temperature = getTemp();
  Serial.println(temperature);  // display Fahrenheit
  delay(100);
}
