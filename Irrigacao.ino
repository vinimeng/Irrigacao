#include <Stepper.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <I2C_RTC.h>

static DS1307 RTC; //0x3C //0x50 //0x58

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

void setup() {
  u8g2.begin();
  pinMode(HIGROMETRO, INPUT);
  pinMode(LEDVERMELHO, OUTPUT);
  pinMode(LEDAMARELO, OUTPUT);
  pinMode(LEDVERDE, OUTPUT);
  //MotorFrente.setSpeed(256);
  higrometro = analogRead(HIGROMETRO);
  //moverMotor();
  Serial.begin(9600);
  RTC.begin();
  //RTC.setHourMode(CLOCK_H24);
  //RTC.setWeek(1);

  //RTC.setDate(22,07,21);
  //RTC.setTime(23,00,00);
  Serial.println("No");
  Serial.println("Setting Time");
  //RTC.setHourMode(CLOCK_H12); //Comment if RTC PCF8563
  RTC.setHourMode(CLOCK_H24);  
  RTC.setDateTime(__DATE__, __TIME__);
  RTC.updateWeek();           //Update Weekdaybased on new date.    
  Serial.println("New Time Set");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);
  RTC.startClock(); //Start the Clock;
}

void loop() {
  higrometro = analogRead(HIGROMETRO);
  //moverMotor();
  display();
  int nDevices = 0;
  Serial.println(RTC.getSeconds());
  Serial.println("Scanning...");

  for (byte address = 1; address < 127; ++address) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");

      ++nDevices;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
  delay(5000); // Wait 5 seconds for next scan
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
  u8g2.sendBuffer();
}
