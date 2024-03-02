#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// i2c addresses that are read when connecting display: 0x03, 0x3E, 0x60, 0x70

const float freqSamp = 10000; // sampling frequency in hertz
const float Ts = 1000/freqSamp; // sampling period in milliseconds
const float sdPeriodModifier = 200; // multiplier to change sd card writing frequency, adjust accordingly
const float sdTs = Ts*sdPeriodModifier; // determines the time in milliseconds between consecutive writes
const float DAC_Vref = 1.5; // reference voltage for TI circuit
const float vR1=21.5,vR2=7.87,vR3=6.19,vR4=9.53; // voltage TI circuit resistors
const float vR5=160, vR6=10; // high voltage bus potential divider for voltage measurement
const float cR1=28.7,cR2=6.8,cR3=7.32,cR4=16.9; // current TI circuit resistors
const float wheelRadius=0.3; // Radius of rear wheel in meters (put real value here)
const double iVoltageStep = 23/750; // Current sensor voltage step per Ampere of current
const char baseFileName[] = "/log"; // Base file name for datalogging
volatile float eXi, eXv, eXp, eXE=0, eXEf; //for safe access in ISR
volatile unsigned long prevWheelRevolutions=0, wheelRevolutions=0; // wheel revolutions from interrupt
volatile float totalDistance=0; // total distance travelled
volatile float cumulativeEnergy=0; // total energy consumed
SemaphoreHandle_t eXMutex; // Mutex for synchronisation

RTC_DS3231 rtc; // rtc object
File dataFile; // File object for the current log file

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

void samplePower(void *parameter){
  (void)parameter; // suppress unused parameter compiler warning

  unsigned long lastExecTime = millis();

  for(;;){ // infinite loop within task, iterate indefinitely
    float measuredCurrent = analogRead(4)*(3.3/4095); // convert digital reading into voltage
    float currentTI_input = InverseTI_Circuit(cR1, cR2, cR3, cR4, DAC_Vref, measuredCurrent); // range of 1.65-3.3 approx
    currentTI_input = currentTI_input-1.65;
    float estCurrent = currentTI_input/iVoltageStep;
    /*
    output voltage when 0 current = 1.65
    Maximum measurable primary current = +37.5
    Vout-vref max = 1.15, 1.15+1.65=2.8
    23/750 is the voltage step per A
    So 15A would correspond to 0.46+1.65=2.11 V from ADC
    May appear Confusing!!!
  */
    float measuredVoltage = analogRead(15)*(3.3/4095); // convert digital reading into voltage
    float voltageTI_input = InverseTI_Circuit(vR1, vR2, vR3, vR4, DAC_Vref, measuredVoltage); // range of 1.8-3.3 approx
    float estVoltage = voltageTI_input*((vR5+vR6)/vR6); // ignoring op-amp rail to rail limitation

    float estPower = estVoltage*estCurrent; // P = V*I

    if(xSemaphoreTake(eXMutex, portMAX_DELAY)){ // acquires mutex before sharing variables
      eXi = estCurrent;
      eXv = estVoltage;
      eXp = estPower;

      unsigned long currentTime = millis();
      float elaspedTime = (currentTime - lastExecTime)/1000; // convert to seconds
      eXE += (estPower*elaspedTime)/3600000; // convert to kWh

      unsigned long currentWheelRevolutions = wheelRevolutions;
      float incrementalDistance = (2*3.1416*wheelRadius*(currentWheelRevolutions-prevWheelRevolutions))/1000; // convert distance into km
      totalDistance += incrementalDistance;
      prevWheelRevolutions = currentWheelRevolutions;

      if(totalDistance!=0){ // efficiency calculation, km per kWh
        eXEf = totalDistance/eXE;
      }
      else{
        eXEf = 0; // avoids the cannot divide 0 error
      }
      xSemaphoreGive(eXMutex); // release the mutex
      vTaskDelay(pdMS_TO_TICKS(Ts)); // Delay for 0.1 milliseconds -> to achieve fs = 10kHz
    }
  }
}

void logData(void *parameter){
  (void)parameter;

  for(;;){
    if(digitalRead(26)==HIGH){ // checks if sd card is present
      Serial.println("SD card is inserted.");

      if(!dataFile){ // if the file is not open, open a new file for writing
        
        DateTime now = rtc.now(); // obtain current date and time from rtc

        char fileName[20]; // create a file name with current timestamp
        snprintf(fileName, sizeof(fileName), "%s_%04d%02d%02d_%02d%02d%02d.csv", baseFileName, now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
        // %04d is four digit decimal number placeholder and %02d is two digit decimal number placeholder

        dataFile = SD.open(fileName, FILE_WRITE); // open the file in write mode

        if(dataFile){
          Serial.println("New log file created.");
        } 
        else{
          Serial.print("Error opening file for writing.");
        }
      }

      if(xSemaphoreTake(eXMutex, portMAX_DELAY)){ // acquire mutex before reading shared variables
          float exportedCurrent = eXi;
          float exportedVoltage = eXv;
          float exportedPower = eXp;
          float exportedEnergy = eXE; 
          float exportedDistance = totalDistance;
          float exportedEfficiency = eXEf;

          // write timestamps and measured and calculated quantities
          // time,current(A),voltage(V),power(W),cumulative energy(kWh),cumulative distance(km),Efficiency(km/kWh)
          dataFile.print(millis()); 
          dataFile.print(",");
          dataFile.print(exportedCurrent);
          dataFile.print(",");
          dataFile.print(exportedVoltage);
          dataFile.print(",");
          dataFile.print(exportedPower);
          dataFile.print(",");
          dataFile.print(exportedEnergy);
          dataFile.print(",");
          dataFile.print(exportedDistance);
          dataFile.print(",");
          dataFile.println(exportedEfficiency);

          xSemaphoreGive(eXMutex); // release mutex

          dataFile.flush(); // flush data to ensure it is physically stored to sd card
          Serial.println("Data logged successfully.");
      }
    }
    else{
      Serial.println("No SD card present.");
    }
    vTaskDelay(pdMS_TO_TICKS(sdTs)); // don't want to write at 10kHz, will quickly fill up storage and some computers will struggle with number crunching csv files with a million plus entries
    // also faster writing consumes more resources and reduces sd longevity
  }
}

void setup() {
  //Wire.begin();
  Serial.begin(115200);

  pinMode(26, INPUT_PULLUP); // SD Card Detection, Logic LOW when no card and Logic HIGH when card is present
  // Format with FAT16 or FAT32 only

  pinMode(15, INPUT); // Voltage measure compensated output voltage
  
  pinMode(4, INPUT); // Current sensor compensated output voltage
  pinMode(17, INPUT); // Current sensor Over Current Detection
  pinMode(16, OUTPUT); // Current sensor standby/EN pin
  

  pinMode(13, INPUT); // External Interrupt 1
  pinMode(14, INPUT); // External Interrupt 2

  pinMode(25, OUTPUT); // DAC Reference voltage for TI circuit
  float dacValue = round(((DAC_Vref/3.3)*256)-1); // convert DAC reference voltage into value in range 0-255
  analogWrite(25, dacValue); // Write a DC value of DAC_VREF to pin 25

  // initialise SD card
  if(!SD.begin(12, SPI)){ // pin 12 corresponds to GPIO turned CS for SPI
    Serial.println("SD card initialisation failed.");
    while(1);
  }
  // initialise RTC
  if(!rtc.begin()){ // 
    Serial.println("Couldn't find RTC.");
    while(1);
  }
  // check if RTC loses power and if so set the time
  if(rtc.lostPower()){
    Serial.println("RTC lost power, setting time.");
    rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));
  }

  attachInterrupt(digitalPinToInterrupt(13), handleInterrupt, RISING); // wheel revolution hall effect interrupt

  eXMutex = xSemaphoreCreateMutex(); // create a mutex for synchronisation
  
  xTaskCreatePinnedToCore(
    samplePower, // function to be called
    "Sample Power Readings", // Task Name
    1024, // Stack size (bytes in ESP32, words FreeRTOS)
    NULL, // Parameter to pass to function (pointer)
    24, // Task priority (0 to configMAX_PRIORITIES - 1), 24 is highest priority and 0 is lowest priority
    NULL, // Task handle
    0 // Run on core x
  ); 

  xTaskCreatePinnedToCore(
    logData,
    "Datalogging",
    1024,
    NULL,
    24,
    NULL,
    1
  );

  // include SCL, SDA -> i2c
  // need to work with interfacing with an i2c display
}

void loop() {
  
  // FUCK OFF

}
