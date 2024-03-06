// DS18B20 Temperature Sensor to Arduino with LCD  , debounce , without delay ,3 input button
////////////////////// LCD //////////////////////
#include <LiquidCrystal.h>
LiquidCrystal lcd(3,4,5,6,7,8); // (rs, en, d4, d5, d6, d7)
/////////////////////////////////////////////////
/////////////// Temp Sensor /////////////////////
#include <OneWire.h>
#include <Wire.h>
OneWire  ds(2);                 
double tempC = 0;
double tempF = 0;
byte i;
byte present = 0;
byte type_s;
byte data[12];
byte addr[8];
/////////////////////////////////////////////////
//////////////////// Rotary_Encoder /////////////
// push button variabls
int ledState = HIGH;
int presentButtonState;
int lastState = HIGH;
// Debounce Variabls
unsigned long lastDebounceTime = 0;
unsigned int countDelay = 0;
unsigned int debounceDelay = 50;
int count = 0;
int rise = 0;
int fall = 0;
unsigned long lastCountTime = 0;
long lastFallTime = 0;
long lastRotateTime = 0;
int counter = 0;
#define CLK 11
#define DT 12
#define SW 13
int currentStateCLK;
int lastStateCLK;
int StateDT;
int lastStateDT;
String currentDir ="";
byte encoderSt=0;
/////////////////////////////////////////////////
/////////////////// SETUP ///////////////////////
void setup() {
  Serial.begin(9600);
  Wire.begin();
  lcd.begin(20,4);
  pinMode(CLK,INPUT);
	pinMode(DT,INPUT);
	pinMode(SW, INPUT_PULLUP);
  lcd.clear();
  lastStateCLK = digitalRead(CLK);
  lastStateDT = digitalRead(DT);
  attachInterrupt(0, updateEncoder, CHANGE);
	attachInterrupt(1, updateEncoder, CHANGE);
}
////////////////////////////////////////////////
///// Calculating Temp from Temp Sensor/////////
void DS18B20() {
  ds.search(addr);
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 0);        
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  int16_t raw = (data[1] << 8) | data[0];
  raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } 
  else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  tempC= (float)raw / 16.0;
  tempF = (tempC * 9.0) / 5.0 + 32.0;
}
////////////////////////////////////////////////
//////////////// LOOP //////////////////////////
void loop() {
  DS18B20();
  currentStateCLK = digitalRead(CLK);
  StateDT = digitalRead(DT);
  presentButtonState = digitalRead(SW);
  if (presentButtonState == LOW && lastState == HIGH){
    if ( (millis() - lastDebounceTime) > debounceDelay) {
      ledState = !ledState;
    }
    lastState = presentButtonState;
    lastDebounceTime = millis();
  }
  else if (presentButtonState == HIGH && lastState == LOW){
    if ( (millis() - lastFallTime) > debounceDelay) {
      fall ++;
      lastFallTime = millis();
    }
    lastState = presentButtonState;
  }
  else if (presentButtonState == LOW && lastState == LOW){
      if ( (millis() - lastCountTime) > countDelay) {
        count ++;
        lastCountTime = millis();
      }
      lastState = presentButtonState;
  }
  if(count >= 50){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Setting : ");
      lcd.setCursor(0,1);
      lcd.print("Press To Change.");
      while(presentButtonState != LOW){
        presentButtonState = digitalRead(SW);
        count = 0;
      }
  }
  if (counter >= 30){
    count = 0;  
  }
  if (ledState == HIGH){
    lcd.setCursor(0,0);
    lcd.print("TempC   ");
    lcd.print(" Dir ");
    if (currentDir == "CW"){
      lcd.print(" ");
      lcd.print(currentDir);
      lcd.print(" ");
    }
    else{
      lcd.print(currentDir);
    }
    lcd.setCursor(0,1);
    lcd.print(tempC);
    lcd.print(" dg  ");
    lcd.print(counter);
    lcd.print("  ");
    lcd.print(count);
    lcd.print("  ");
  }
  else {
    lcd.setCursor(0,0);
    lcd.print("TempF   ");
    lcd.print(" Dir ");
    if (currentDir == "CW"){
      lcd.print(" ");
      lcd.print(currentDir);
      lcd.print(" ");
    }
    else{
      lcd.print(currentDir);
    }
    lcd.setCursor(0,1);
    lcd.print(tempF);
    lcd.print(" dg  ");
    lcd.print(counter);
    lcd.print("  ");
    lcd.print(count);
    lcd.print("  ");
  } 
}
////////////////////////////////////////////////
//////////////// ROTARY ENCODER //////////////// 
void updateEncoder(){
	currentStateCLK = digitalRead(CLK);
	if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
		if ( (millis() - lastRotateTime) > debounceDelay) {
      if (digitalRead(DT) != currentStateCLK) {
			  counter --;
			  currentDir ="CCW";
		  } else if (digitalRead(DT) == currentStateCLK) {
			  counter ++;
			  currentDir ="CW";
		  }
    lastRotateTime = millis();
    } 
	}
  lastStateCLK = currentStateCLK;
}
////////////////////////////////////////////////