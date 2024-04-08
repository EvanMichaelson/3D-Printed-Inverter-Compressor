#include <SPI.h>
#include <Wire.h>
#include <ArduPID.h>

// Define lookup table rows and columns
#define tempEnthROWS 8
#define tempEnthCOLUMNS 2
#define tempDenROWS 7
#define tempDenCOLUMNS 2
// Define massflow sensor physical conversion values
#define OFFSET_FLOW 32768
#define SCALE_FACTOR_FLOW 120
// Define pressure sensor physical conversion values
#define SCALE_FACTOR_PRESSURE 0x3FFF
#define RANGE_PRESSURE 689476 // 100 PSI = 689476 pascals
// Define temperature sensor physical conversion values
#define MAX_TEMPERATURE 1023.75
#define MIN_TEMPERATURE 0
#define MAX_RAW_TEMPERATURE 0b111111111111
#define MIN_RAW_TEMPERATURE 0b0

// Ideal gas properties of air TEMP-ENTHALPY (COL 1 = temperature COL 2 = enthalpy)
const double tempEnthTable[tempEnthROWS][tempEnthCOLUMNS] = { 
  {280, 280.13},
  {285, 285.14},
  {290, 290.16},
  {295, 295.17},
  {298, 298.18},
  {300, 300.19},
  {305, 305.22},
  {310, 310.24}
}; 

// Ideal gas properties of air TEMP-DENSITY (COL 1 = temperature COL 2 = density)
// From https://www.engineeringtoolbox.com/air-density-specific-weight-d_600.html
const double tempDenTable[tempDenROWS][tempDenCOLUMNS] = { 
  {278.15, 1.268},
  {283.15, 1.246},
  {288.15, 1.225},
  {293.15, 1.204},
  {298.15, 1.184},
  {303.15, 1.164},
  {313.15, 1.127},
}; 

// Defining sensor readings as doubles
double initialTemp;
double finalTemp;
double volumetricFlow;
double massFlow;
double finalPressure;
// Defining final calculation as double
double initialEnth;
double finalEnth;
double finalWork; 
// Defining spi communication pins
const int spiTempPin1 = 8;
const int spiTempPin2 = 7;
// Defining i2c communication addresses
const int i2cPressureAdrs = 0x28; 
const int i2cMassAdrs = 0x40;
// Defining i2c identifiers
const int pressureID = 1;
const int massID = 2;
// Defining motor sensor / speed control pins
const int hallEffectPin1 = 4;
const int hallEffectPin2 = 3;
const int hallEffectPin3 = 2;
// Defining speed reading variables
volatile unsigned long pulseCount = 0;
unsigned long lastPulseTime = 0;
unsigned long rpm = 0;
// Defining PWM pin
const int pwmPin = 3;
int pwmVal = 0;

// Defining timer values
unsigned long lastPrintTime = 0;
unsigned long lastReadTime1 = 0;
unsigned long lastReadtime2 = 0;

// Defining user inputs
char userInputVal = 0;

// PID Controller Parameters
double setPoint = 0;
double input;
double output;
double p = 0;
double i = 8.1;
double d = 0;

// Defining functions
double spiCommuniction(int spiPin);
double i2cCommunication(int i2cAdrs);
double convertRaw(int MSB, int LSB, int ID);
double volume2Mass(double volumetricFlow, double temperature);
unsigned long getSpeed();
double getEnthalpy(double knownTemp);
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max);
void userInput();
void printResults();

void setup() {
  // Initializing I/O pins
  pinMode(pwmPin, OUTPUT); 
  
  // Initialize spi communication
  SPI.begin();
  pinMode(spiTempPin1, OUTPUT);
  digitalWrite(spiTempPin1, HIGH);
  pinMode(spiTempPin2, OUTPUT);
  digitalWrite(spiTempPin2, HIGH);

  // Initialize i2c communication
  Wire.begin();

  // Initialize serial communication
  Serial.begin(9600);

  myController.begin(&input, &output, &setpoint, p, i, d);
}

void loop() {
  
  // Read temp every .5 seconds
  if (millis() - lastReadTime1 >= 1000){
    // Read temperatures from spi communication
    initialTemp = spiCommunication(spiTempPin1);
    finalTemp = spiCommunication(spiTempPin2);
    lastReadTime1 = 0;
    delay(10);
  }

  // Read pressure and massflow from I2C communication
  finalPressure = i2cCommunication(i2cPressureAdrs, pressureID);
  volumetricFlow = i2cCommunication(i2cMassAdrs, massID);
  massFlow = volume2Mass(volumetricFlow, finalTemp);

  // Calculate work
  initialEnth = getEnthalpy(initialTemp);
  finalEnth = getEnthalpy(finalTemp);
  finalWork = (finalEnth - initialEnth)*massFlow;

  // Display results every second
  if (millis() - lastPrintTime >= 1000){
    userInput();
    printResults();
    lastPrintTime = millis();
  }

  // Write the PWM value to the PWM module 
  analogWrite(pwmPin, pwmVal);

  // Update the PWM value using a PID controller
  input = finalWork
  myController.compute();
  pwmVal = min((output - 0.1953) / 9.8117, 255);
  pwmVal = max(pwmVal, 0);

}

// ************************************************************************ SPI COMMUNICATION FUNCTION ***************************************************************
double spiCommunication(int spiPin){
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(spiPin, LOW);
  byte temperatureMSB = SPI.transfer(0x00);
  byte temperatureLSB = SPI.transfer(0x00);
  digitalWrite(spiPin, HIGH);

  SPI.endTransaction();


  bool thermocoupleOpen = temperatureLSB & 0b00000100;

  if (thermocoupleOpen){
    Serial.println("WARNING: Thermocouple is disconnected");
    return 0;
  }

  int rawTemperatureData = (temperatureMSB << 8) | temperatureLSB;
  rawTemperatureData = (rawTemperatureData & 0b0111111111111000) >> 3;
  double convertedTemperatureData = mapfloat(rawTemperatureData, MIN_RAW_TEMPERATURE, MAX_RAW_TEMPERATURE, MIN_TEMPERATURE, MAX_TEMPERATURE);

  //convert degrees celsius to Kelvin
  convertedTemperatureData = convertedTemperatureData+273.15;
  return convertedTemperatureData;
}

// ************************************************************************ I2C COMMUNICATION FUNCTION ***************************************************************
double i2cCommunication(int i2cAdrs, int ID){

  switch (ID) {
    case 1:
      Wire.requestFrom(i2cAdrs, 2);
      if(Wire.available() >= 2) {
        byte dataMSB = Wire.read();
        byte dataLSB = Wire.read();
        double data = convertRaw(dataMSB, dataLSB, ID);
        return data;
      }
      break;
    case 2:
      Wire.beginTransmission(i2cAdrs);
      Wire.write(0x1000 >> 8);
      Wire.write(0x1000 & 0b11111111);
      Wire.endTransmission();
      Wire.requestFrom(i2cAdrs, 3);
      if(Wire.available() >= 3) {
        byte dataMSB = Wire.read();
        byte dataLSB = Wire.read();
        byte garbageCollection = Wire.read();
        double data = convertRaw(dataMSB, dataLSB, ID);
        delay(10);
        if (data < 120);
        return data;
      }
  }

}

// ************************************************************************ I2C DATA CONVERSION FUNCTION ***************************************************************
double convertRaw(byte MSB, byte LSB, int ID){
  double convertedData;

  if (ID == 1){
    unsigned long rawData = ((uint16_t)(MSB << 8) | LSB);
    switch(rawData >> 16){
      case 0:
        convertedData = mapfloat((rawData & 0b0011111111111111), 0, SCALE_FACTOR_PRESSURE, 0, RANGE_PRESSURE);
        break;
      case 1:
        Serial.print("\nWARNING: Device is in command mode!");
        break;
      case 2:
        Serial.print("\nWARING: Data is stale!");
        break;
      case 3:
        Serial.print("\nWARNING: Diagnostic condition!");
        break;
    }
    
  } else if (ID == 2){
    unsigned long rawData = (uint16_t)(MSB << 8) | (LSB & 0b11111111);
    convertedData = (rawData - 32768) / 120;
    Serial.println(convertedData);
  }

  return convertedData;
}

// *************************************************************** CONVERT VOLUMETRIC FLOW RATE TO MASS FLOW RATE ******************************************************
double volume2Mass(double volumetricFlow, double temperature){
  double density;
  double massFlow;

  
  if (temperature <= tempDenTable[0][0]){ // low extreme
    Serial.print("\nWarning: Volume to mass conversion is negatively out of bounds at temp value of: ");
    Serial.println(temperature);
    density = tempDenTable[0][1];
  } else if (temperature >= tempDenTable[tempDenROWS-1][0]){ // high extreme
    Serial.print("\nWarning: Volume to mass conversion is positively out of bounds at temp value of: ");
    Serial.println(temperature);
    density = tempDenTable[tempDenROWS-1][1];
  } else {
    for (int i = 1; i <tempDenROWS; ++i){
      if (temperature < tempDenTable[i][0]){ // if known temperature is below the table value
        float lowerTemp = tempDenTable[i-1][0];
        float upperTemp = tempDenTable[i][0];
        float lowerDens = tempDenTable[i-1][1];
        float upperDens = tempDenTable[i][1];
        float slope = (temperature - lowerTemp) / (upperTemp - lowerTemp); // find linear slope
        density = lowerDens + slope * (upperDens - lowerDens); // interpolate
      }}  
  }  

  massFlow = volumetricFlow*density*0.000016666666666666667; // [Liters / Minute] * 60 [] * [Kg / Meter^3]

  return massFlow;
}

// ************************************************************************ ENTHALPY CALCULATION FUNCTION ***************************************************************
double getEnthalpy(double knownTemp){

  float calcEnth;

  if (knownTemp <= tempEnthTable[0][0]){ // low extreme
    Serial.print("\nWarning: Enthalpy conversion is negatively out of bounds at temp value of: ");
    Serial.println(knownTemp);
    calcEnth = tempEnthTable[0][1];
  } else if (knownTemp >= tempEnthTable[tempEnthROWS-1][0]){ // high extreme
    Serial.print("\nWarning: Enthalpy is positively out of bounds at temp value of: ");
    Serial.println(knownTemp);
    calcEnth = tempEnthTable[tempEnthROWS-1][1];
  } else {

    for (int i = 1; i <tempEnthROWS; ++i){
      if (knownTemp < tempEnthTable[i][0]){ // if known temperature is below the table value
        float lowerTemp = tempEnthTable[i-1][0];
        float upperTemp = tempEnthTable[i][0];
        float lowerEnth = tempEnthTable[i-1][1];
        float upperEnth = tempEnthTable[i][1];
        float slope = (knownTemp - lowerTemp) / (upperTemp - lowerTemp); // find linear slope
        calcEnth = lowerEnth + slope * (upperEnth - lowerEnth); // interpolate
      }}  
  }

  return calcEnth;
}

// *********************************************************************** FLOAT MAP ****************************************************************************
float mapfloat(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

// ********************************************************************** USER INPUT AND PRINTING ******************************************************************
void userInput(){
  if (Serial.available() > 0) {
    userInputVal = Serial.read();
      if (!(userInputVal == 'r' || userInputVal == 'l')) {
        pwmVal = userInputVal - '0';
        Serial.print("\n\n Trying to reach ");
        Serial.print(pwmVal);
        Serial.println(" W output");
        pwmVal = map(pwmVal, 0, 9, 0, 255);
        setPoint = userInputVal - '0';
      }
  }
  while (Serial.available() > 0) {
    char collectGarbage = Serial.read();
  }

}

void printResults(){
  switch(userInputVal){
    case 'r':
      Serial.print("Displaying sensor readings!\n");
      userInputVal = 0;
    case 'l':
      Serial.print("\n\nInitial temperature -> ");
      Serial.print(initialTemp - 273.15);
      Serial.print(" degC");
      Serial.print("\nFinal temperature -> ");
      Serial.print(finalTemp - 273.15);
      Serial.print(" degC");
      Serial.print("\nFinal pressure -> ");
      Serial.print(finalPressure/1000 - 46);
      Serial.print(" kPa");
      Serial.print("\nMass flow -> ");
      Serial.print(massFlow*1000);
      Serial.print(" g/s");

      Serial.print("\n\nHeat pump heating / cooling capacity -> ");
      Serial.print(abs(finalWork*1000)*3.98);
      Serial.print(" W");
      break;
  }

}
