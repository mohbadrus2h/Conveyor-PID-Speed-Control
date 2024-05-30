#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include "Dimmer.h"

#define outputPin 5
#define encoderPin 3
#define down_but 11
#define up_but 10

Dimmer dimmer(outputPin, DIMMER_RAMP, 1.5);

LiquidCrystal_I2C lcd(0x27,20,4);

volatile unsigned int counter = 0;

uint8_t input = 0;
uint8_t i = 0;
uint8_t address = 0;

uint8_t value;
uint8_t setpoint[] = {100,105,110};

 static float Kp = 0.03;
 static float Ki = 0.03;
 static float Kd = 0.00001;
// static float Kd = 0;

float error, last_error = 0, integral = 0;
float proportional;
float p_error;
float dimVal;
bool p_time = true;

unsigned long pid_lastTime = 0;       
unsigned long lcd_lastTime = 0;       
const int pid_interval = 100;       
const int lcd_interval = 500;       
const int pprEnco = 360; 
float speedOut, speedLast, nRevol;

uint8_t down_state = 0;
uint8_t up_state = 0;

void setup() {
  
  lcd.init();
  
  Serial.begin (9600);
  dimmer.begin();
  
  pinMode(encoderPin, INPUT_PULLUP);   
  pinMode(down_but, INPUT);
  pinMode(up_but, INPUT);

  EEPROM.get(address, i);
          
  attachInterrupt(digitalPinToInterrupt(encoderPin), ai0, RISING);

  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Setpoint");
  lcd.setCursor(12,0);
  lcd.print(":");
  
  lcd.setCursor(0,1);
  lcd.print("Speed (RPM)");
  lcd.setCursor(12,1);
  lcd.print(":");
  
  lcd.setCursor(0,2);
  lcd.print("Error");
  lcd.setCursor(12,2);
  lcd.print(":");

  lcd.setCursor(0,3);
  lcd.print("PID Out");
  lcd.setCursor(12,3);
  lcd.print(":");
}

void loop() {

//  tombol tambah kecepatan
  down_state = digitalRead(down_but);

//  tombol turun kecepatan
  up_state = digitalRead(up_but);

//membaca nilai tombol turun
  if (down_state == HIGH){
        
    if (i > 0){
      i--;
      }     
    delay(200);
   }

//membaca nilai tombol tambah
  if (up_state == HIGH){
    
    if (i < 2){
      i++;
    }
    delay(200);
   }

//mengisi nilai value dengan setpoint
   value = setpoint[i];

//   menyimpan index kecepatan pada eeprom
   EEPROM.put(address, i);

//   menjalankan PID
  if(pidTime()) {    
    
    speedLast = encoData();
    dimVal = PID(speedLast, value, integral, last_error);              
//    Serial.println(speedLast);
    dimmer.set(dimVal);
    resetSampling();    
  } 
   
//  menjalankan LCd
  if(lcdTime()){
    lcd.setCursor(14,0);
    lcd.print(value);
    lcd.setCursor(14,1);
    lcd.print(speedLast);
    lcd.setCursor(14,2);
    lcd.print(last_error);
    lcd.setCursor(14,3);
    lcd.print(dimVal);      
  }
}

//membaca pulsa encoder
void ai0() {
  counter++;
  
}

float encoData() {

//  menghitung kecepatan encoder
  nRevol = (float)counter / (float)pprEnco;

//  kecepatan encoder dalam 1 menit
  speedOut = nRevol * 1000.0 / (float)pid_interval * 60.0;

//  perbandingan diameter motor dan encoder = 0.846
  speedLast = speedOut * 0.8466;

  // Serial.print("\t Rev = ");
  // Serial.print(nRevol);
  // Serial.print("\t Speed = ");
//  Serial.println(speedLast);

  return speedLast;
}

// Fungsi reset pembacaan encoder
void resetSampling() {
  counter = 0;
}

// Timing eksekusi fungsi PID (100ms)
boolean pidTime() {

  unsigned long pid_currentTime = millis();

  if (pid_currentTime - pid_lastTime >= pid_interval) {
    
    pid_lastTime = pid_lastTime + pid_interval;

    return true;
  } 
  else {
    return false;
  }
}

// Timing upload data ke LCD (7 detik)
boolean lcdTime() {
  unsigned long lcd_currentTime = millis();

  if (lcd_currentTime - lcd_lastTime >= lcd_interval) {
    lcd_lastTime += lcd_interval;

    return true;
  } else{

    return false;
  }
}

// Fungsi PID
float PID(float speepLast, float setpoint, float &integral, float &last_error) {
  
  float dt = pid_interval / 1000.0;

  float error = setpoint - speedLast;
  proportional = Kp * error;
  integral += Ki * error * dt;

  float derivative = Kd * (error - last_error) / dt;
  last_error = error;

  if (integral >= 100){
    integral = 100;
  }
  else if(integral <= 0){
    integral = 0;    
  }

  float output = proportional + integral + derivative;
     
  if (output >= 100){
    output = 100;
  }
  else {}  
  
  return output;
}
