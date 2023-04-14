#include <Stepper.h>

#define POTENCIOMETRO A1
#define HIGROMETRO A2
#define LEDVERMELHO 6
#define LEDAMARELO 5
#define LEDVERDE 7
#define BAUDRATE 9600
#define IN1 9
#define IN2 10
#define IN3 8
#define IN4 11

Stepper MotorFrente(64, IN1, IN3, IN2, IN4);
int passoAtual = 0;
int potenciometro = 0;
int higrometro = 0;
int historicoPotenciometro = 0;
bool seco = false;
bool meio = false;
bool molhado = false;

void setup() {
  pinMode(POTENCIOMETRO, INPUT);
  pinMode(HIGROMETRO, INPUT);
  pinMode(LEDVERMELHO, OUTPUT);
  pinMode(LEDAMARELO, OUTPUT);
  pinMode(LEDVERDE, OUTPUT);
  Serial.begin(BAUDRATE);
  MotorFrente.setSpeed(256);
}

void loop() {
  higrometro = analogRead(HIGROMETRO);

  if (higrometro >= 0 && higrometro < 50 && seco) {
    ligarLed(LEDVERDE);
    MotorFrente.step(1024 - passoAtual);
    passoAtual = 1024;
    seco = false;
    meio = true;
    molhado = true;
  } else if (higrometro >= 50 && higrometro < 200  && meio) {
    ligarLed(LEDAMARELO);
    MotorFrente.step(512 - passoAtual);
    passoAtual = 512;
    seco = true;
    meio = false;
    molhado = true;
  } else if (higrometro >= 200  && molhado) {
    ligarLed(LEDVERMELHO);
    MotorFrente.step(0 - passoAtual);
    passoAtual = 0;
    seco = true;
    meio = true;
    molhado = false;
  }  
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
