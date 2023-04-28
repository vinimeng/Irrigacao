#include <Stepper.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <RTClib.h>
#include <Wire.h>

#define HIGROMETRO A2
#define LEDVERMELHO 6
#define LEDAMARELO 5
#define LEDVERDE 7
#define BAUDRATE 9600
#define IN1 9
#define IN2 10
#define IN3 8
#define IN4 11
#define FONTE u8g2_font_t0_12b_tr
#define SIMBOLOAGUA 50

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0);

Stepper MotorFrente(64, IN1, IN3, IN2, IN4);
int passoAtual = 0;
int higrometro = 0;
bool diminui = true;
bool aumenta = true;
char buf[20];
DS1307 rtc;
DateTime now;

void setup() {
  u8g2.begin();
  pinMode(HIGROMETRO, INPUT);
  pinMode(LEDVERMELHO, OUTPUT);
  pinMode(LEDAMARELO, OUTPUT);
  pinMode(LEDVERDE, OUTPUT);
  #ifdef AVR
    Wire.begin();
  #else
    Wire1.begin();
  #endif
  rtc.begin();
  rtc.adjust(DateTime(__DATE__, __TIME__));
  MotorFrente.setSpeed(256);
  higrometro = analogRead(HIGROMETRO);
  moverMotor();
}

void loop() {
  higrometro = analogRead(HIGROMETRO);
  Wire.setClock(100000);
  now = rtc.now();
  moverMotor();
  display();
  delay(10);
}

void ligarLed(int led) {
  digitalWrite(LEDVERMELHO, LOW);
  digitalWrite(LEDAMARELO, LOW);
  digitalWrite(LEDVERDE, LOW);

  switch (led) {
    case LEDVERDE:
      digitalWrite(LEDVERMELHO, HIGH);
      break;
    case LEDAMARELO:
      digitalWrite(LEDAMARELO, HIGH);
      break;
    case LEDVERMELHO:
    default:
      digitalWrite(LEDVERDE, HIGH);
      break;
  }
}

void moverMotor() {
  if (higrometro >= 0 && higrometro < 100 && diminui) {
    ligarLed(LEDVERDE);
    MotorFrente.step(1024 - passoAtual);
    passoAtual = 1024;
    diminui = false;
    aumenta = true;
  } else if (higrometro >= 100 && higrometro < 250 && (aumenta || diminui)) {
    ligarLed(LEDAMARELO);
    MotorFrente.step(512 - passoAtual);
    passoAtual = 512;
    diminui = true;
    aumenta = true;
  } else if (higrometro >= 250 && aumenta) {
    ligarLed(LEDVERMELHO);
    MotorFrente.step(0 - passoAtual);
    passoAtual = 0;
    diminui = true;
    aumenta = false;
  }
}

void display() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_unifont_t_weather);
  u8g2.drawGlyph(0, 12, 50);
  if (passoAtual == 1024) {
    u8g2.drawGlyph(50, 14, 33);
    u8g2.setFont(FONTE);
    u8g2.drawStr(68, 12, "0 graus");
  } else if (passoAtual == 512) {
    u8g2.drawGlyph(50, 14, 35);
    u8g2.setFont(FONTE);
    u8g2.drawStr(68, 12, "90 graus");
  } else {
    u8g2.drawGlyph(50, 14, 37);
    u8g2.setFont(FONTE);
    u8g2.drawStr(68, 12, "180 graus");
  }
  if (higrometro >= 400) {
    u8g2.drawStr(18, 12, "100%");
  } else {
    int porcentagem = higrometro * 0.253;
    String s = String(porcentagem);
    s.concat("%");
    u8g2.drawStr(18, 12, s.c_str());
  }
  u8g2.drawStr(0, 32, now.tostr(buf));
  u8g2.sendBuffer();
}
