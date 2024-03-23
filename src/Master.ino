#include <Arduino.h>
#include <Wire.h>
#include "Waveshare_LCD1602_RGB.h"
#include <SD.h>
#include <FS.h>
#include <SPI.h>
#include <string>
#include <lut_ADC.hpp> // for ADC LUT
#include <ESP32Time.h>
#include <cstdio>

// would be great to use FreeRTOS for ESP32 but issues with SD card prevent this from happening

// pin definitions
const uint8_t dac_reference = D2; // reference voltage for TI compensation network

const uint8_t SPI_CS = D3; // other spi ports mapped automatically
const uint8_t SPI_SD_DET = A3; // use digital internal pullup

const uint8_t opt1 = A0; // optional button inputs (SW1,3)
const uint8_t opt2 = A1; // all butons have pulldown resistors on PCB

const uint8_t int1 = D6; // interrupt ports for distance tracking
const uint8_t int2 = D7; // use either ADC2_CH4/6 or digital to detect
const uint8_t simulatedInterrupt = D4; // need to simulate interrupts for distance calculation

const uint8_t currentOCD = D10; // overcurrent detection for current sensor
const uint8_t currentStandBy = D11; // standby (or EN) for current sensor
const uint8_t currentMeasure = D12; // current measurement transformed as voltage

const uint8_t voltageMeasure = A4; // voltage measurement

// distance calculation
const float wheelRadius=0.3; // Radius of rear wheel in meters (put real value here)
unsigned long prevWheelRevolutions=0, wheelRevolutions=0; // wheel revolutions from interrupt
float totalDistance=0; // total distance travelled

// circuit parameters
const float dac_voltage = 1.5; // reference voltage for TI compensation network

const float voltageR1=21.5,voltageR2=7.87,voltageR3=6.19,voltageR4=9.53; // R1,R2,R3,R4 -> R5,R7,R12,C2 on PCB
const float voltageR5=160, voltageR6=10; // R5,R6 -> R3,R4 on PCB

const float currentR1=28.7,currentR2=6.8,currentR3=7.32,currentR4=16.9; // R1,R2,R3,R4 -> R6,R8,R11,C9 on PCB
const double current2VoltageStep = 37.5/1.15; // Current sensor voltage step per Ampere of current

// energy and efficiency metric
float cumulativeEnergy = 0;

// timing parameters
const uint16_t sampFreq = 10000; // data sampling frequency in hertz
const float Ts = 1000/sampFreq; // data sampling period in milliseconds
const float writeFreqScaler = 200; // multiplier to change sd card write frequency, change accordingly
const float writePeriod_sd = Ts*200; // time interval between consecutive writes in milliseconds 

// display config
uint8_t g,b,t=0;
uint8_t r = 255;

// LCD object
Waveshare_LCD1602_RGB lcd(16,2);  

// use of internal ESP32 RTC
ESP32Time rtc(0); // for GMT leave as 0, for GMT+1 (BST) change to 3600

// file object
File dataFile; // must format in either FAT16 or FAT32 otherwise no data can be written on empty card

// baseline Filename
const char baseFileName[] = "/ datalog";
char* fullFileName;
const uint8_t maxFiles = 50; // maximum number of files stored + 1

uint32_t writeIterations = 0;

float loggedCurrent, loggedVoltage, loggedPower, loggedEnergy=0, loggedEfficiency = 0;
float inputCurrent, calibratedCurrentMeasure;

//======================== variable conversion for display ====================================//
const char* boolToString(bool value) {
    return value ? "true" : "false";
} 

char* intToCharArray(int number) {
    // Determine the length of the resulting string
    int length = snprintf(NULL, 0, "%d", number) + 1; // +1 for null terminator

    // Allocate memory for the character array
    char* charArray = new char[length];

    // Convert the integer to a string
    snprintf(charArray, length, "%d", number);

    return charArray;
}

char* concatenateStrings(const char* str1, const char* str2) {
    // Calculate the length of the concatenated string
    size_t len1 = strlen(str1);
    size_t len2 = strlen(str2);
    size_t lenTotal = len1 + len2 + 1; // +1 for the null terminator

    // Allocate memory for the concatenated string
    char* result = new char[lenTotal];

    // Copy the first string to the result
    strcpy(result, str1);

    // Concatenate the second string to the result
    strcat(result, str2);

    return result;
}

char* floatToChar(float value) {
    // Buffer size for the resulting string
    const int bufferSize = 10; // Sufficient for most float values with 1 decimal place

    // Allocate memory for the char* buffer
    char* buffer = new char[bufferSize];

    // Convert float to char* with one decimal place accuracy
    snprintf(buffer, bufferSize, "%.1f", value);

    // Return the char* buffer
    return buffer;
}

//========================= helper functions =======================================//
float InverseTI_Circuit(float R1, float R2, float R3, float R4, float Vref, float Vout){
  // manipulation of the TI circuit equation to estimate the input voltage
  float Vin, temp;
  Vout = Vout + Vref*(R1/R2);
  temp = (R4/(R3+R4))*((R1+R2)/R2);
  Vin = Vout/temp;
  return Vin;
}

void handleInterrupt(){
  wheelRevolutions++;
}

//======================== SD card file generation ================================//
void createNewFile(){
  for (uint8_t i=0; i<=maxFiles; i++){
    if(i == maxFiles) {
      // maximum files on SD card reached, informs user to clear all files
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.send_string("Maximum files");
      lcd.setCursor(0,1);
      lcd.send_string("on SD card");
      delay(3000);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.send_string("Must delete");
      lcd.setCursor(0,1);
      lcd.send_string("files for log");
      delay(3000);
      lcd.clear();
    }
    // generates filename by basefilename + index i + .csv
    fullFileName = concatenateStrings(concatenateStrings(baseFileName,intToCharArray(i)),".csv");
    // checks if file exists, if it doesn't then create new file
    if(!SD.exists(fullFileName)) {
      dataFile = SD.open(fullFileName, FILE_WRITE);
      break; // no need to iterate through for loop anymore
    }
  }
}

void setup() {

  // pin allocation

  pinMode(dac_reference, OUTPUT);
  analogWrite(dac_reference,round(((dac_voltage/3.3)*256)-1));

  // LOW if no card is installed and HIGH if one is installed
  pinMode(SPI_SD_DET, INPUT_PULLUP);

  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH); // must be set to HIGH initially otherwise code will be stored on SD card instead of ESP32

  pinMode(opt1, INPUT); // unused
  pinMode(opt2, INPUT); // unused

  pinMode(int1, INPUT); // external interrupt 1
  pinMode(int2, INPUT); // external interrupt 2

  //pinMode(currentOCD, INPUT); // overcurrent detection for current sensor - does nothing so far
  pinMode(currentStandBy, OUTPUT);
  digitalWrite(currentStandBy, LOW); // datasheet is actually quite shit and doesnt say anything about digital signals
  pinMode(currentMeasure, INPUT);

  pinMode(simulatedInterrupt, INPUT_PULLUP);

  pinMode(voltageMeasure, INPUT);

  attachInterrupt(digitalPinToInterrupt(int1),handleInterrupt, RISING); // wheel revolution with hall-effect interrupt
  attachInterrupt(digitalPinToInterrupt(simulatedInterrupt),handleInterrupt, RISING);
 
  // GPIO based method to check if SD card is installed via DET pin
  const char* SDSTATE = boolToString(digitalRead(SPI_SD_DET));

  // initialise UART
  Serial.begin(115200);

  // sometimes ADC resolution is less bits than provided by µC, define as 12-bit as ESP32 has 2x 12-bit ADCs
  analogReadResolution(12);

  // initialise ESP32 RTC
  rtc.setTime(0, 0, 0, 1, 1, 1970); // only the OGs will understand

  // initialise display
  lcd.init();
    
  // display startup text
  lcd.setCursor(4,0);
  lcd.send_string("IEM 23-24");
  lcd.setCursor(0,1);
  lcd.send_string("Joulemeter v2.0");
  delay(4000);
  lcd.clear(); // this clears the display

  // inform user if SD card is detected on the display
  lcd.setCursor(0,0);
  lcd.send_string("SD Card present?");
  lcd.setCursor(0,1);
  lcd.send_string(SDSTATE);

  // must be complemented so SD card can be detected and data is written
  digitalWrite(SPI_CS, LOW);

  // SPI based method to check if SD card is installed via CS pin
  SD.begin(SPI_CS);  
  if(!SD.begin(SPI_CS)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  Serial.println("Initializing SD card...");
  if (!SD.begin(SPI_CS)) {
    Serial.println("ERROR - SD card initialization failed!");
    return;    
  }

  delay(1500);
  
  createNewFile();
  Serial.print("Created file: ");
  Serial.print(fullFileName);
  Serial.println("\n");
  delete[] fullFileName;

}

void loop() { 

  // =============================================== measure voltage and current ==========================================================================

  unsigned long prevTime = rtc.getSecond();

  inputCurrent = 0;
  for (int i =0; i<=100; i++){
    calibratedCurrentMeasure = (3.3/4095)*ADC_LUT[analogRead(currentMeasure)];

    inputCurrent += abs(1.65-calibratedCurrentMeasure);
    delay(100);
  }

  inputCurrent = (inputCurrent/100)*current2VoltageStep;

  //float inputCurrent = calibratedCurrentMeasure;// = InverseTI_Circuit(currentR1,currentR2,currentR3,currentR4,dac_voltage,calibratedCurrentMeasure);
  //inputCurrent = abs(1.65-inputCurrent);
  //inputCurrent = inputCurrent;//current2VoltageStep;
  /*
    output voltage when 0 current = 1.65
    Maximum measurable primary current = +37.5
    Vout-vref max = 1.15, 1.15+1.65=2.8
    23/750 is the voltage step per A
    So 15A would correspond to 0.46+1.65=2.11 V from ADC
    May appear Confusing!!!
  */
  float calibratedVoltageMeasure = ADC_LUT[analogRead(voltageMeasure)];
  calibratedVoltageMeasure = calibratedVoltageMeasure*(3.3/4095);

  float inputVoltage = InverseTI_Circuit(voltageR1,voltageR2,voltageR3,voltageR4,dac_voltage,calibratedVoltageMeasure);
  inputVoltage = inputVoltage*((voltageR5+voltageR6)/voltageR6);

  float inputPower = inputCurrent*inputVoltage;

  loggedCurrent = inputCurrent;
  loggedVoltage = inputVoltage;
  loggedPower = inputPower; // power in watts

  unsigned long currentTime = rtc.getSecond();

  float elapsedTime = (currentTime-prevTime); // time in seconds
  Serial.println(elapsedTime*1000);
  loggedEnergy += (inputPower*elapsedTime)/3600000; // energy in kWh

  unsigned long currentWheelRevolutions = wheelRevolutions;
  float incrementDistance = (2*3.1416*wheelRadius*(currentWheelRevolutions-prevWheelRevolutions))/1000; // distance in km
  totalDistance += incrementDistance;
  prevWheelRevolutions = currentWheelRevolutions;

  if(totalDistance!=0 && loggedEnergy!=0){
    loggedEfficiency = totalDistance/loggedEnergy; // efficiency km/kwh
  }
  else {
    loggedEfficiency = 0; // avoid div 0 error
  }

  Serial.println(rtc.getTime());
  Serial.println(loggedCurrent);
  Serial.println(loggedVoltage);
  Serial.println(loggedPower);
  Serial.println(loggedEnergy);
  Serial.println(loggedEfficiency);
  Serial.println(totalDistance);
  Serial.println();

  // ================================================================================= write data to SD card as a .csv file ====================================================================
  dataFile.print(rtc.getTime());
  dataFile.print(",");
  dataFile.print(loggedCurrent);
  dataFile.print(",");
  dataFile.print(loggedVoltage);
  dataFile.print(",");
  dataFile.print(loggedPower);
  dataFile.print(",");
  dataFile.print(loggedEnergy);
  dataFile.print(",");
  dataFile.print(loggedEfficiency);
  dataFile.print(",");
  dataFile.println(totalDistance);
  dataFile.flush(); // physically write data to SD card rather than hold within a buffer

  Serial.println("Data logged successfully\n");

  // ================================================================================== display update ==========================================================================================

  lcd.clear();

  // lcd.setCursor(0,0);
  // lcd.send_string("NRG:");

  // lcd.setCursor(0,1);
  // lcd.send_string("EFF:");

  // lcd.setCursor(13,0);
  // lcd.send_string("kWh");

  // lcd.setCursor(10,1);
  // lcd.send_string("km/kWh");

  // char* energy = floatToChar(loggedEnergy);
  // char* efficiency = floatToChar(loggedEfficiency);

  // lcd.setCursor(4,0);
  // lcd.send_string(energy);

  // lcd.setCursor(4,1);
  // lcd.send_string(efficiency);

  // delete[] efficiency, energy; // deallocate memory for char* buffer

  char* current = floatToChar(loggedCurrent);
  char* voltage = floatToChar(loggedVoltage);

  lcd.setCursor(0,0);
  lcd.send_string("I (A):");

  lcd.setCursor(10,0);
  lcd.send_string(current);

  lcd.setCursor(0,1);
  lcd.send_string("V (V):");
  
  lcd.setCursor(10,1);
  lcd.send_string(voltage);

  delay(1000); // adjust accordingly - affects datalogging rate and display refresh rate

}
