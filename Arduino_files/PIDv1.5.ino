///// LIBRERIAS //////////////////
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>
#include <Servo.h>
#include <Sabertooth.h>

enum State {
  INICIO,
  BAJANDO,
  SUBIENDO,
  MOVIENDOSE,
  STOP
};

State state = INICIO;

////// PINES /////////////////

//// Salida ////

//Servo
const int Servo_PWM = 22;

//Stepper
const int DIR = 24;
const int PUL = 26;

//Bomba
const int bomba = 30;

//// Entrada ////
const int boton_mover = 53;
const int boton_subir = 51;

//Encoders
#define encoder0PinA 19   // INT4
#define encoder0PinB 18   // INT5

#define encoder1PinA 2    // INT0
#define encoder1PinB 3    // INT1

//Limit Switchs
const int limite_inferior = 23;
const int limite_lateral = 25;

//Hall Sensor
const int sensor_hall = A15;

////// VARIABLES //////////////

//// Comunicacion ////
String buffer = "";
String orden = "";

//// Control PID ////
const float kp1 = 0.05;
const float ki1 = 0;
const float kd1 = 0.003;

const float kp2 = 0.05;
const float ki2 = 0;
const float kd2 = 0.003;

//Estado angular
float angulo_actual1 = 0.0;
float angulo_actual2 = 0.0;

float angulo_bolsa = 0.0;
float angulo_objetivo1 = 0.0;
float angulo_objetivo2 = 0.0;

//PID interno
float error_pid1 = 0;
float error_anterior1 = 0;
float integral_pid1 = 0;
float derivada_pid1 = 0;

float error_pid2 = 0;
float error_anterior2 = 0;
float integral_pid2 = 0;
float derivada_pid2 = 0;

//PID salida
float pid_output1 = 0;
float pid_output2 = 0;

//// Movimiento ////
volatile bool mov_z = false;
volatile bool mov = false;
volatile int pasos_restantes = 1000;

//// Limites ////
volatile bool flag_limite_inferior = false;
volatile bool flag_limite_lateral  = false;

//// Sensor Hall ////
int valor_sensor_hall = 0;

// Tiempo
unsigned long time_ant = 0;
const unsigned long Period = 10000;     // 10 ms (100 Hz)
const float dt = Period * 0.000001f;    // 0.01 s

// Encoder Pololu 4752
const float CPR_MOTOR = 256.0;
const float GEAR_RATIO = 131.0;
const float CPR_OUTPUT = 1920.0;

volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;

long newposition0 = 0;
long newposition1 = 0;
long oldposition0 = 0;
long oldposition1 = 0;

float vel1_rpm = 0;
float vel2_rpm = 0;

// -----------------------
// INTERRUPCIONES
void doEncoder0A() {
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) encoder0Pos++;
  else encoder0Pos--;
}

void doEncoder0B() {
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) encoder0Pos--;
  else encoder0Pos++;
}

void doEncoder1A() {
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) encoder1Pos++;
  else encoder1Pos--;
}

void doEncoder1B() {
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) encoder1Pos--;
  else encoder1Pos++;
}

void leerLimites_ISR() {
  flag_limite_inferior = digitalRead(limite_inferior);
  flag_limite_lateral  = digitalRead(limite_lateral);
}

///// COMPONENTES //////

Sabertooth ST(128, Serial2); // pines 16 y 17
Servo MG995_Servo;
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);

  lcd.init();
  lcd.clear();
  lcd.backlight();

  //// Configurar pines ////
  ///Encoder
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);

  ///Limits
  pinMode(limite_inferior, INPUT);
  pinMode(limite_lateral, INPUT);

  ///Stepper, Servo y bomba
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(Servo_PWM, OUTPUT);
  pinMode(bomba, OUTPUT);

  ///Botones
  pinMode(boton_subir, INPUT);
  pinMode(boton_mover, INPUT);

  /// Activar interrupciones ////
  Timer1.initialize(1000); // 1 ms
  Timer1.attachInterrupt(leerLimites_ISR);

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoder0B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder1B, CHANGE);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("INICIO");
  lcd.setCursor(0, 1);
  lcd.print("ESPERANDO");
}

// -----------------------
void loop() {

  // Recepción serial
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      procesarMensaje(buffer);
      buffer = "";
    } else {
      buffer += c;
    }
  }

  switch (state) {

    case INICIO:
      mov_z = digitalRead(boton_subir);
      mov = digitalRead(boton_mover);

      if (mov_z) {
        flag_limite_inferior = digitalRead(limite_inferior);
        flag_limite_lateral  = digitalRead(limite_lateral);

        if (flag_limite_inferior) {
          state = SUBIENDO;
          lcd.clear();
          lcd.print("SUBIENDO");
        } else {
          state = BAJANDO;
          lcd.clear();
          lcd.print("BAJANDO");
        }
      }

      if (mov) {
        if (flag_limite_lateral) {
          state = STOP;
        } else {
          state = MOVIENDOSE;
          lcd.clear();
          lcd.print("MOVIENDOSE");
        }
      }

      break;

    // --------------------
    case BAJANDO:
      digitalWrite(DIR, LOW);
      if (flag_limite_inferior) {
        state = INICIO;
        lcd.clear();
        lcd.print("INICIO");
      } else {
        lcd.clear();
        lcd.print("Paso bajo");
        digitalWrite(PUL, HIGH);
        delayMicroseconds(2000);
        digitalWrite(PUL, LOW);
        delayMicroseconds(2000);
      }
      break;

    case SUBIENDO:
      digitalWrite(DIR, HIGH);
      if (pasos_restantes == 0) {
        state = INICIO;
        lcd.clear();
        lcd.print("INICIO");
        pasos_restantes = 100;
      } else {
        digitalWrite(PUL, HIGH);
        delayMicroseconds(2000);
        digitalWrite(PUL, LOW);
        delayMicroseconds(2000);
        lcd.clear();
        lcd.print("PASO up");
        pasos_restantes--;
      }
      break;

    // --------------------
    case MOVIENDOSE:

      if (micros() - time_ant >= Period) {

        if (flag_limite_lateral) {
          state = STOP;
        } else {

          newposition0 = encoder0Pos;
          newposition1 = encoder1Pos;

          vel1_rpm = (newposition0 - oldposition0) * (60.0 / (CPR_OUTPUT * dt));
          vel2_rpm = (newposition1 - oldposition1) * (60.0 / (CPR_OUTPUT * dt));

          oldposition0 = newposition0;
          oldposition1 = newposition1;

          float vel1_rad_s = vel1_rpm * 2.0 * PI / 60.0;
          float vel2_rad_s = vel2_rpm * 2.0 * PI / 60.0;

          angulo_actual1 += vel1_rad_s * dt;
          angulo_actual2 += vel2_rad_s * dt;

          error_pid1 = angulo_objetivo1 - angulo_actual1;
          integral_pid1 += error_pid1 * dt;
          derivada_pid1 = (error_pid1 - error_anterior1) / dt;
          error_anterior1 = error_pid1;

          error_pid2 = angulo_objetivo2 - angulo_actual2;
          integral_pid2 += error_pid2 * dt;
          derivada_pid2 = (error_pid2 - error_anterior2) / dt;
          error_anterior2 = error_pid2;

          pid_output1 = kp1 * error_pid1 + ki1 * integral_pid1 + kd1 * derivada_pid1;
          pid_output2 = kp2 * error_pid2 + ki2 * integral_pid2 + kd2 * derivada_pid2;

          if (pid_output1 > 40) pid_output1 = 40;
          if (pid_output1 < -40) pid_output1 = -40;

          ST.motor(1, pid_output1);

          if (pid_output2 > 40) pid_output2 = 40;
          if (pid_output2 < -40) pid_output2 = -40;

          ST.motor(2, pid_output2);

          time_ant = micros();

          if (abs(error_pid1) < 0.02 && abs(error_pid2) < 0.02) {

            ST.motor(1, 0);
            ST.motor(2, 0);

            angulo_actual1 = 0;
            error_anterior1 = 0;
            integral_pid1 = 0;

            angulo_actual2 = 0;
            error_anterior2 = 0;
            integral_pid2 = 0;

            state = INICIO;
          }
        }
      }
      break;

    // --------------------
    case STOP:
      ST.motor(1, 0);
      ST.motor(2, 0);
      lcd.clear();
      lcd.print("STOP");
      delay(1000);
      lcd.clear();
      lcd.print("INICIO");
      state = INICIO;
      break;
  }
}

////// FUNCIONES /////////////////
String extraerCampo(String msg, String etiqueta) {
  int start = msg.indexOf(etiqueta);
  if (start < 0) return "0";

  start += etiqueta.length();
  int end = msg.indexOf(';', start);
  if (end < 0) end = msg.length();

  return msg.substring(start, end);
}

void procesarMensaje(String msg) {
  msg.trim();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Msg: ");
  lcd.print(msg);

  int p1 = msg.indexOf(';');
  if (p1 < 0) return;

  orden = msg.substring(0, p1);

  int m1_start = msg.indexOf("M1:");
  int m2_start = msg.indexOf("M2:");
  int s_start  = msg.indexOf("S:");

  if (m1_start < 0 || m2_start < 0 || s_start < 0) {
    lcd.clear();
    lcd.print("Error campos");
    return;
  }

  String strM1 = extraerCampo(msg, "M1:");
  String strM2 = extraerCampo(msg, "M2:");
  String strS  = extraerCampo(msg, "S:");

  angulo_objetivo1 = strM1.toFloat();
  angulo_objetivo2 = strM2.toFloat();
  angulo_bolsa     = strS.toFloat();
}
