#include "SoftwareSerial.h"
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

Servo S1,S2;
Adafruit_NeoPixel strip(256, 5, NEO_GRB + NEO_KHZ800);
SoftwareSerial BTserial(8, 9);
bool is_open=0, izm=0;
uint32_t fillColor;
bool n1=0, flag=0;

unsigned long time = 0;
unsigned long Time = 0;
int is_pow=1;

void msg(int x) {
  strip.clear();
    if (x == 0) { //Коптер внутри
      fillColor = strip.Color(150, 150, 150);
    }
    if (x == 1) { //Открытие
      fillColor = strip.Color(0, 0, 150);
    }
    if (x == 2) fillColor = strip.Color(150, 0, 0); //Закрытие
    if (x == 3) fillColor = strip.Color(0, 150, 0);//Зарядка
  for (int i=0; i<256; i=i+2) {
        strip.setPixelColor(i, fillColor);}
    if (x == 4) { //Открытие2
      strip.clear();
      fillColor = strip.Color(0, 0, 150);
      for (int i=1; i<255; i=i+2) {
        strip.setPixelColor(i, fillColor);}
    }
    if (x == 5) { //Закрытие2
      strip.clear();
      fillColor = strip.Color(150, 0, 0);
      for (int i=1; i<255; i=i+2) {
        strip.setPixelColor(i, fillColor);}
    }
  strip.show();
}

void setup() {
  strip.begin();
  strip.show();
  Serial.begin(9600);
  BTserial.begin(115200);
  pinMode(A0, INPUT); 
  S1.attach(A4);
  S2.attach(A5);
  is_open=0;
  msg(3);
}
void loop() {

    if (BTserial.available()) { 
      int p=BTserial.read();
      if(p==65) { //Хотим закрыть
        if (is_open==1) izm=1;
        is_open=0;
      }
      if(p==66) { //Хотим открыть
        if (is_open==0) izm=1;
        is_open=1;
      }
    }

  if (n1 && millis() - Time > 200) {
      strip.clear();
      if (flag) {
        for (int i=0; i<256; i=i+2) {
          strip.setPixelColor(i, fillColor);
        }
        flag = 0;
      }
      else {
        for (int i=1; i<255; i=i+2) {
          strip.setPixelColor(i, fillColor);
        }
        flag = 1;
      }
    Time = millis();
    strip.show();
  }

  if (is_open==0 && izm==1) {//закрытие короба
    S1.write(100);
    S2.write(100);
    for (int x=0; x<5; x++){
    msg(2);
    delay(150);
    msg(5);
    delay(150);
    }
    strip.clear();
    strip.show();
    izm=0;
  }
  if (is_open==1 && izm==1)//открытие короба
  {
    for (int x=0; x<5; x++){
    S1.write(0);
    S2.write(0);
    msg(1);
    delay(150);
    msg(4);
    delay(150);
    }
    strip.clear();
    strip.show();
    izm=0;
  }

  if (analogRead(A0) > 780) {//Детект взлетa
    is_pow=0;
    time=0;
    n1 = 0;
    strip.clear();
    strip.show();
  }

  if (is_pow==0 && izm==0) {//Логика подключения питания
    if (analogRead(A0) < 780 && time==0) {
      msg(0);
      n1 = 1;
      is_pow=0;
      time=millis();
    }

    if (analogRead(A0) < 780 && time!=0 && millis() - time >5000) {
      msg(3);
      is_pow=1;
      time=0;
    }
  }
}
