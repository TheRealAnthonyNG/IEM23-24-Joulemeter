#include <Wire.h>
#include "Waveshare_LCD1602_RGB.h"


Waveshare_LCD1602_RGB lcd(16,2);  //16 characters and 2 lines of show
int g,b,t=0;
int r = 255;
void setup() {
  Serial.begin(115200);






    // initialize
    lcd.init();
    
    lcd.setCursor(0,0);
    lcd.send_string("IEM 23-24");
    lcd.setCursor(0,1);
    lcd.send_string("Joulemeter tst");

}

void loop() {
  // put your main code here, to run repeatedly:

}
