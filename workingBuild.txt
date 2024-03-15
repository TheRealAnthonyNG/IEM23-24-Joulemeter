#include <Arduino.h>
#include <Wire.h>
#include "Waveshare_LCD1602_RGB.h"
#include <SD.h>
#include <FS.h>
#include <SPI.h>
#include <string>
#include <Constants.h> // for ADC LUT
#include <ESP32Time.h>

// pin definitions
const uint8_t SPI_CS = D3;
const uint8_t ADC2_CH4 = D7;
const uint8_t ADC2_CH0 = D12;
const uint8_t ADC1_CH6 = A2;
const uint8_t SPI_SD_DET = A3;
const uint8_t ADC2_CH3 = A4; // these are temporary values for testing on breadboard

// sampling definitions
const float freqSamp = 10000; // sampling frequency in hertz
const float Ts = 1000/freqSamp; // sampling period in milliseconds

// sd card write interval
const float sdPeriodModifier = 200; // multiplier to change sd card writing frequency, adjust accordingly
const float sdTs = Ts*sdPeriodModifier; // determines the time in milliseconds between consecutive writes

// circuit parameters
const float DAC_Vref = 1.5; // reference voltage for TI circuit
const float vR1=21.5,vR2=7.87,vR3=6.19,vR4=9.53; // voltage TI circuit resistors
const float vR5=160, vR6=10; // high voltage bus potential divider for voltage measurement
const float cR1=28.7,cR2=6.8,cR3=7.32,cR4=16.9; // current TI circuit resistors
const double iVoltageStep = 23/750; // Current sensor voltage step per Ampere of current

// distance calculation
const float wheelRadius=0.3; // Radius of rear wheel in meters (put real value here)
volatile unsigned long prevWheelRevolutions=0, wheelRevolutions=0; // wheel revolutions from interrupt
volatile float totalDistance=0; // total distance travelled

// display config
uint8_t g,b,t=0;
uint8_t r = 255;

// LCD object
Waveshare_LCD1602_RGB lcd(16,2);  

// use of internal ESP32 RTC
ESP32Time rtc(0); // for GMT leave as 0, for GMT+1 (BST) change to 3600

// file object
File dataFile; 

// baseline Filename
const char baseFileName[] = "/ datalog";
char* fullFileName;
const uint8_t maxFiles = 50; // maximum number of files stored

uint8_t writeIterations = 0;


//======================== diagnostic functions ====================================//
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

  // LOW if no card is installed and HIGH if one is installed
  pinMode(SPI_SD_DET, INPUT_PULLUP);

  pinMode(ADC1_CH6, INPUT);
  pinMode(ADC2_CH0, INPUT);
  pinMode(ADC2_CH3, INPUT);
  pinMode(ADC2_CH4, INPUT);

  pinMode(SPI_CS, OUTPUT);

  // must be set to HIGH initially otherwise code will be stored on SD card instead of ESP32
  digitalWrite(SPI_CS, HIGH);
 
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
  delay(5000);
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

  delay(3000);
  
  createNewFile();
  Serial.print("Created file: ");
  Serial.print(fullFileName);
  Serial.println("\n");
  delete[] fullFileName;
    


}

void loop() {
  





  float A1CH6 = analogRead(ADC1_CH6)*3.3/4096;
  float A2CH0 = analogRead(ADC2_CH0)*3.3/4096;
  float A2CH3 = analogRead(ADC2_CH3)*3.3/4096;
  float A2CH4 = analogRead(ADC2_CH4)*3.3/4096;

  Serial.println(rtc.getTime()); // prints in this format: Thursday, January 01 1970 00:00:30
  Serial.println(A1CH6);
  Serial.println(A2CH0);
  Serial.println(A2CH3);
  Serial.println(A2CH4);

  dataFile.print(rtc.getTime()); // prints in this format: Thursday, January 01 1970 00:00:30
  dataFile.print(",");
  dataFile.print(A1CH6);
  dataFile.print(",");
  dataFile.print(A2CH0);
  dataFile.print(",");
  dataFile.print(A2CH3);
  dataFile.print(",");
  dataFile.println(A2CH4);
  dataFile.flush(); 

  writeIterations++;

  char* result = intToCharArray(writeIterations);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.send_string("Write Iterations:");
  lcd.setCursor(0,1);
  lcd.send_string(result);

  // free up memory allocated for char array
  delete[] result;

  Serial.println("Data written\n");




  delay(5000);



}
