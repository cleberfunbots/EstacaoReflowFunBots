/*

  Programa para Controle PID de Estação de Solda Reflow

      Vídeos de referência:
        - Controle PID de Ventilador: https://youtu.be/K0G01H5wj1Y
        - Solda Reflow: https://youtu.be/JNrhAweeBnA
        - SSR: https://youtu.be/hI_d4a834PE

      Componentes:
        - Arduino Uno ou outro qualquer
        - Base aquecida para Reflow de 220V/400W
        - Módulo MAX6675 com termopar
        - SSR de 40A
        - Fan e Módulo Relê
        - Display LCD 16x2 I2C
        - Botão tipo táctil
        
      Versão 1.0 - Versão inicial - 3/Jul/2022

 *    * Criado por Cleber Borges - FunBots - @cleber.funbots  *     *

      Instagram: https://www.instagram.com/cleber.funbots/
      Facebook: https://www.facebook.com/cleber.funbots
      YouTube: https://www.youtube.com/channel/UCKs2l5weIqgJQxiLj0A6Atw
      Telegram: https://t.me/cleberfunbots

   *** Tempos e Temperaturas de cada Fase *** 
   
   Fase 1 - Pre-Heater
   Temp = 150 DGC (130-180)
   Duração = 180 seg (90-180)

   Fase 2 - Reflow
   Temp = 230 DGC (183-245)
   Duração = 90 seg (60-90)

   Fase 3 - Pos-Reflow
   Temp = 245 DGC (225-245)
   Duração = 30 seg (10-30)

   Fonte: https://www.youtube.com/watch?v=Ffji0cagLpk

*/



// Inclusão de Bibliotecas
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include "max6675.h"
#include "TimerOne.h"

// Pinos
const int pinoSO = 5;
const int pinoCS = 6;
const int pinoCLK = 7;
const int pinoSSR = 9;
const int pinoFan = 8;
const int pinoBotaoInicio = 10;

// Declara objeto para ler termopar
MAX6675 termopar(pinoCLK, pinoCS, pinoSO);

//Inicializa o display no endereco 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Constantes do Controle PID
const int MIN_PWM = 0;
const int MAX_PWM = 512;
const float KP = 25;
const float KI = .05;
const float KD = .05;

// Variáveis para Controle de Temperatura
double pwmSSR = 0;
double tempPlate = 0;
double TempSetpoint[4] = {0, 150, 230, 245};
double TempSetpointAtual = 0;
int fase = 0;
float tempo = 0.0;
bool estadoBotao = false;

// PID
PID reflowPID(&tempPlate, &pwmSSR, &TempSetpointAtual, KP, KI, KD, DIRECT);

// Variáveis para temporização
unsigned long tempoInicio = 0;
unsigned long tempoFim = 0;
unsigned long tempoInicioFaseAtual = 0;
unsigned long tempoFaseAtual = 0;
const long tempoLoop = 500;
const long tempoFase[5] = {0, 180000, 90000, 30000, 1000};

void setup() {
  // Inicia Wire I2C
  Wire.begin();

  //Inicializa display
  lcd.init();
  lcd.setBacklight(HIGH);

  // Inicia Serial
  Serial.begin(9600);

  // Configura Pinos
  pinMode(pinoSSR, OUTPUT);
  pinMode(pinoFan, OUTPUT);
  pinMode(pinoBotaoInicio, INPUT_PULLUP);

  digitalWrite(pinoFan, HIGH);

  // Inicia Timer 1 com periodo de 3 segundos
  Timer1.initialize(3000000);

  // Inicia Temp Setpoint com fase 0;
  TempSetpointAtual = TempSetpoint[fase];

  // Configura controle PID
  reflowPID.SetOutputLimits(MIN_PWM, MAX_PWM);
  reflowPID.SetMode(AUTOMATIC);

}


void loop() {
  tempoInicio = millis();

  // Leitura do botao de inicio
  estadoBotao = !digitalRead(pinoBotaoInicio);

  // Leitura do termopar em Celsius
  tempPlate = termopar.readCelsius();

  if (estadoBotao || (fase > 0 && fase < 4)) {

    if (fase == 0) {
      fase++;
      tempoInicioFaseAtual = millis();
    }

    // Reconfigura PWM Máximo apenas para fase 2 e 3
    if (fase == 2 || fase == 3) {
      reflowPID.SetOutputLimits(MIN_PWM, 1023);
    } else {
      reflowPID.SetOutputLimits(MIN_PWM, MAX_PWM);
    }

    if (fase == 3 && tempo > 290) {
      digitalWrite(pinoFan, LOW);
    } else {
      digitalWrite(pinoFan, HIGH);
    }

    // Seta Setpoint com temperatura da fase corrente
    TempSetpointAtual = TempSetpoint[fase];
    // Calcula o PWM para o SSR conforme Controle PID
    reflowPID.Compute();
    // Aciona SSR com PWM calculado
    Timer1.pwm(pinoSSR, pwmSSR);

    tempoFaseAtual = millis();
    if ((tempoFaseAtual - tempoInicioFaseAtual) > tempoFase[fase]) {
      fase++;
      tempoInicioFaseAtual = millis();
    }

  } else if (fase == 4) {

    // COOL DOWN
    digitalWrite(pinoFan, LOW);
    pwmSSR = 0;
    Timer1.pwm(pinoSSR, pwmSSR);
    if (tempo > 400) fase++;

  } else {

    // Aguardando pressionar botão para Iniciar o processo de Reflow
    if (tempPlate > 80) {
      digitalWrite(pinoFan, LOW);
    } else {
      digitalWrite(pinoFan, HIGH);
    }

  }

  lcd.setCursor(0, 0);
  lcd.print("Fase:");
  lcd.print(fase);
  lcd.print(" t:");
  lcd.print(tempo);
  lcd.setCursor(0, 1);
  lcd.print("Temp:");
  lcd.print(tempPlate, 1);

  Serial.print("Fase: ");
  Serial.print(fase);
  Serial.print(" ; t: ");
  Serial.print(tempo);
  Serial.print(" ; Temp: ");
  Serial.print(tempPlate);
  Serial.print(" DGC");
  Serial.print(" ; PWM: ");
  Serial.println(pwmSSR);

  if (fase > 0 && fase < 5) tempo = tempo + 0.5;
  tempoFim = millis();
  if ((tempoFim - tempoInicio) < tempoLoop) {
    delay(tempoLoop - (tempoFim - tempoInicio));
  }
}
