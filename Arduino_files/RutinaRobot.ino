///// LIBRERIAS //////////////////
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>
#include <Servo.h>
#include <Sabertooth.h>
#include <Encoder.h>

// ===== CONFIGURACIÓN ROBOT =====
const float k_coreccion= 1.02;
const float COUNTS_PER_REV_ARM = 1920*3; 
const float COUNTS_PER_DEGREE1 = COUNTS_PER_REV_ARM*k_coreccion / 360.0;
const float COUNTS_PER_DEGREE2 = COUNTS_PER_REV_ARM / 360.0;

int canalM1 = 1;
int canalM2 = 2;

enum State {
  INICIO,
  CENTRANDO,
  ESPERANDO_PAPA,
  MOVIENDOSE,
  ESPERAR,
  RECOGER_PAPA,
  SUBIENDO_CON_PAPA,
  SOLTAR_PAPA,
  STOP
};

enum Homing_State {
  HOME_START,
  HOME_Z_DOWN,  // Bajar hasta límite inferior
  HOME_M1_FIND, // Girar base (M1) hasta sensor
  HOME_M2_FIND, // Mover extremo (M2) hasta límite lateral
  HOME_CENTRAR, // Ir a pose conocida
  HOME_SUBIR,
  HOME_DONE
};

Homing_State homing_sub_state = HOME_START;

State state = INICIO;
State lastState = INICIO;

// ===== PINES =====
const int Servo_PWM = 42;
const int DIR = 24;
const int PUL = 28;
const int bomba = 31;
const int boton_mover = 50;
const int boton_subir = 52;
const int pinSensorHall = A0; 

// CR-TOUCH
const int CRTouch_Detection = 4; // Verificado segun tu codigo
const int CRTouch_Control = 9;

// Limites
const int limite_inferior = 23;
const int limite_lateral1 = 53;
const int limite_lateral2 = 51;

// ===== VARIABLES PID =====
float kp1 = 4.0, ki1 = 0.05, kd1 = 0.01;
float kp2 = 4.0, ki2 = 0.5, kd2 = 0.02;

const unsigned long Period_PID = 10000UL;

// Variables de Control
float angulo_actual1 = 0.0;
float angulo_actual2 = 0.0;
float angulo_objetivo1 = 0.0; 
float angulo_objetivo2 = 0.0;
float angulo_objetivo_servo = 0.0;

// Posición de la caja (Coordenadas Absolutas)
float angulo_caja1 = 0.0;
float angulo_caja2 = 250;

// Buffer Serial
float next_M1 = 0.0;
float next_M2 = 0.0;
float next_S = 0.0;
bool flag_comando_recibido = false; 

float error_pid1 = 0, integral_pid1 = 0, error_anterior1 = 0;
float error_pid2 = 0, integral_pid2 = 0, error_anterior2 = 0;

float pid_output1 = 0;
float pid_output2 = 0;

// ===== VARIABLES SISTEMA =====
bool flag_limite_inferior = false;
bool flag_limite_lateral1  = false;
bool flag_limite_lateral2  = false;
bool flag_servo_listo = false;
bool flag_centro = false;
bool flag_cambio = true;
bool flag_bolsa = false;
const int limit_vel1 = 22;
const int limit_vel2 = 35; // Tu valor actualizado
int current_limit2 = 0;

// Variables Stepper
volatile long pasos_restantes = 0;
volatile bool enable_step = false;
volatile bool step_pin_state = false;

// Objetos Encoder
Encoder myEnc1(3, 2); // Pines actualizados segun tu codigo
Encoder myEnc2(18, 19); 

// Tiempos
unsigned long time_ant_pid = 0;

// Comunicación
String buffer = "";
volatile bool flag_detener = false;

// COMPONENTES
Sabertooth ST(128, Serial2);
Servo MG995_Servo;
Servo cr_touch;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// =====================
// PROTOTIPOS DE FUNCIONES
// =====================
void procesarMensaje(String msg);
void setState(State s);
void leerLimites();
bool controlPID(float p1, float i1, float d1, float p2, float i2, float d2, int lim1, int lim2);
void leerSerialNoBloqueante();

// =====================
// INTERRUPCIONES
// =====================
void stepISR() {
  if (!enable_step) return;

  step_pin_state = !step_pin_state;
  digitalWrite(PUL, step_pin_state);

  if (step_pin_state && pasos_restantes > 0) {
    pasos_restantes--;
  } else if (step_pin_state && pasos_restantes <= 0) {
     enable_step = false; 
  }
}

void leerLimites() {
  flag_limite_inferior = digitalRead(limite_inferior);
  flag_limite_lateral1 = digitalRead(limite_lateral1);
  flag_limite_lateral2 = digitalRead(limite_lateral2);
  
  if (analogRead(pinSensorHall) <= 30) {
    flag_centro = true;
  } else {
    flag_centro = false;
  }
}

// =====================
// SETUP
// =====================
void setup() {
  Serial.begin(115200);
  Serial2.begin(38400); // Sabertooth
  ST.autobaud();
  delay(500);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("INICIADO");

  // Pines Sensores
  pinMode(limite_inferior, INPUT); 
  pinMode(limite_lateral1, INPUT);
  
  // Pines Actuadores
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(Servo_PWM, OUTPUT);
  pinMode(bomba, OUTPUT);
  pinMode(boton_subir, INPUT);
  pinMode(boton_mover, INPUT);

  // Servos
  MG995_Servo.attach(Servo_PWM);
  MG995_Servo.write(0);
  
  cr_touch.attach(CRTouch_Control);
  cr_touch.write(90); // Sonda arriba
  delay(50);
  cr_touch.write(160);  
  delay(500);

  cr_touch.write(10); // Desplegar
  delay(500);   
  pinMode(CRTouch_Detection, INPUT_PULLUP);

  // Reset Encoders
  myEnc1.write(0);
  myEnc2.write(0);

  Timer1.initialize(500); // Velocidad Stepper
  Timer1.attachInterrupt(stepISR);

  setState(INICIO);
}

// =====================
// LOOP PRINCIPAL
// =====================
void loop() {
  leerSerialNoBloqueante();

  switch (state) {
    case INICIO:
      ST.motor(canalM1, 0); ST.motor(canalM2, 0);
      enable_step = false;
      flag_comando_recibido = false;
      leerLimites();
      
      if (digitalRead(boton_mover)){
          setState(CENTRANDO);
          lcd.clear();
          lcd.print("CENTRANDO");
      }
      break;

    case CENTRANDO:
      leerLimites();

      switch (homing_sub_state) {
        case HOME_START:
          ST.motor(canalM1, 0);
          ST.motor(canalM2, 0);
          enable_step = false;
          homing_sub_state = HOME_Z_DOWN;
          break;

        case HOME_Z_DOWN:
          leerLimites();
          if (!flag_limite_inferior) {
            digitalWrite(DIR, LOW); 
            pasos_restantes = 999999; 
            enable_step = true;
          } else {
            enable_step = false;
            homing_sub_state = HOME_M1_FIND;
          }
          break;

        case HOME_M1_FIND:
        leerLimites();
          if (!flag_centro) {
            ST.motor(canalM1, 20); 
            leerLimites();
          } else {
            ST.motor(canalM1, 0);
           
            homing_sub_state = HOME_M2_FIND;
          }
          break;

        case HOME_M2_FIND:
          leerLimites();
          if (digitalRead(limite_lateral1)){
            leerLimites();
            ST.motor(canalM2, 0);
            myEnc1.write(0); 
            myEnc2.write(0); 
            
            digitalWrite(DIR, HIGH); 
            pasos_restantes = 1000;
            enable_step = true;
            
            // Configurar pose conocida
            angulo_objetivo1 = 90.0;
            angulo_objetivo2 = 153.0; // Tu valor actualizado
            
            homing_sub_state = HOME_CENTRAR;
          } else {
            ST.motor(canalM2, -13); 
            leerLimites();
          }
          break;

        case HOME_CENTRAR:
          if (controlPID(kp1, ki1, kd1, kp2, ki2, kd2, limit_vel1, limit_vel2)){
            // digitalWrite(DIR, HIGH); 
            // pasos_restantes = 1000;
            // enable_step = true;
            homing_sub_state = HOME_SUBIR;
          }
          break;

        case HOME_SUBIR:
          ST.motor(canalM1, 0);
          ST.motor(canalM2, 0);
          
          if (pasos_restantes <= 0 && enable_step == false) {
            digitalWrite(DIR, LOW);
            homing_sub_state = HOME_DONE;
            flag_limite_inferior = false;
          }
          break;

        case HOME_DONE:
          homing_sub_state = HOME_START;
          lcd.clear();
          lcd.print("IR POR ");
          lcd.setCursor(0,1);
          lcd.print("PAPA");
          setState(ESPERANDO_PAPA);
          break;
      }
      break;

    case ESPERANDO_PAPA:
      // --- LOGICA CORREGIDA ---
      // 1. Si llega un comando, lo mostramos en pantalla (sin bloquear interrupciones del LCD)
      if (flag_comando_recibido){
        // Solo asignamos variables, NO usamos noInterrupts() aquí para el LCD
        angulo_objetivo1 = next_M1;
        angulo_objetivo2 = next_M2;
        angulo_objetivo_servo = next_S;
        
        flag_comando_recibido = false;
        
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("M1:"); lcd.print(next_M1, 1);
        lcd.setCursor(8,0);
        lcd.print("M2:"); lcd.print(next_M2, 1);
      } 
      
      // 2. Independientemente, si pulsan el botón, nos movemos
      if (digitalRead(boton_mover) == HIGH) {
        lcd.clear();
        lcd.print("MOVIENDOSE"); 
        setState(MOVIENDOSE);
           
      }  
      break;

    case MOVIENDOSE:
      leerLimites();

      if (flag_detener || digitalRead(limite_lateral1)) {
        setState(STOP); 
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("DETENER");  
      }

      if (!flag_servo_listo){
          MG995_Servo.write(angulo_objetivo_servo);
          flag_servo_listo = true;
      } 

      if (controlPID(kp1, ki1, kd1, kp2, ki2, kd2, limit_vel1, limit_vel2)){
        setState(ESPERAR);
        lcd.clear();
        lcd.print("EN POSICION");
      }
      break;

    case ESPERAR:
      if (digitalRead(boton_mover)){
        setState(RECOGER_PAPA);
        lcd.clear();
        lcd.print("BAJANDO...");
      }
      break;

    case RECOGER_PAPA:
      digitalWrite(DIR, LOW); 
      leerLimites();
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Ang1:");
      lcd.setCursor(6,0);
      lcd.print(angulo_actual1);
      lcd.setCursor(0,1);
      lcd.print("Ang2:");
      lcd.setCursor(6,2);
      lcd.print(angulo_actual2);


      if (!flag_limite_inferior) { 
        if (!digitalRead(CRTouch_Detection)){
          digitalWrite(DIR, LOW); 
          pasos_restantes = 999999;
          enable_step = true;
        }       
        
        if (digitalRead(CRTouch_Detection)){
          digitalWrite(bomba, HIGH);
          
          if (!flag_bolsa){
            pasos_restantes = 200; 
            enable_step = true;
            flag_bolsa = true;
          }else{
            if (pasos_restantes <=0){
              setState(SUBIENDO_CON_PAPA);
              flag_comando_recibido = false;
              lcd.clear();
              lcd.print("SUBIENDO");
              delay(100);    
            }
          }
        }
      } else {
        enable_step = false;
        setState(CENTRANDO); 
      }
      break;

    case SUBIENDO_CON_PAPA:
        digitalWrite(DIR, HIGH); 
        if (!enable_step) {
            pasos_restantes = 1500; 
            enable_step = true;
        }
        
        if (pasos_restantes <= 0) {
            enable_step = false;
            setState(SOLTAR_PAPA); 
            delay(500);
        }
        break;
    
    case SOLTAR_PAPA:
      if (!flag_comando_recibido){
        angulo_objetivo1 = angulo_caja1;
        angulo_objetivo2 = angulo_caja2;
        flag_comando_recibido = true;
        lcd.clear();
        lcd.print("YENDO A CAJA");
      }

      if (controlPID(kp1, ki1, kd1, kp2, ki2, kd2, limit_vel1, limit_vel2)){
        digitalWrite(bomba, LOW);
        flag_bolsa = false;
        setState(INICIO); 
      }
      break;

    case STOP:
        MG995_Servo.detach();
        ST.motor(canalM1, 0);
        ST.motor(canalM2, 0);
        enable_step = false;
        digitalWrite(bomba, LOW);
        lcd.clear();
        lcd.print("STOP");
        delay(1000);

        if (!flag_detener){
          setState(INICIO);
        }
      break;
  }
}

// =====================
// FUNCIONES AUXILIARES
// =====================

void leerSerialNoBloqueante() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
       procesarMensaje(buffer);
       buffer = "";
    } else {
       buffer += c;
    }
  }
}

void procesarMensaje(String msg) {
  if (msg.startsWith("Mover;") && msg.indexOf("M1:") > 0) {
    
    // CORRECCIÓN: Usar float para no perder decimales
    float tempM1 = 0.0;
    float tempM2 = 0.0;
    float tempS = 0.0;

    int startM1 = msg.indexOf("M1:") + 3; 
    int endM1 = msg.indexOf(";", startM1); 
    if (endM1 != -1) {
      tempM1 = msg.substring(startM1, endM1).toFloat();
    }

    int startM2 = msg.indexOf("M2:", endM1) + 3; 
    int endM2 = msg.indexOf(";", startM2);
    if (endM2 != -1) {
      tempM2 = msg.substring(startM2, endM2).toFloat();
    }

    int startS = msg.indexOf("S:", endM2) + 2; 
    int endS = msg.indexOf(";", startS);
    
    if (endS != -1) {
      String strS = msg.substring(startS, endS);
      tempS = strS.toFloat();
    
      int startBit1 = endS + 1; 
      int endBit1 = msg.indexOf(";", startBit1);
      
      if (endBit1 != -1) {
         int signM1 = msg.substring(startBit1, endBit1).toInt();
         if (signM1 == 1) tempM1 = tempM1 * -1.0;
      }

      int startBit2 = endBit1 + 1; 
      if (startBit2 > 0 && startBit2 < msg.length()) {
          int signM2 = msg.substring(startBit2).toInt();
          if (signM2 == 1) tempM2 = tempM2 * -1.0;
      }
    }

    next_M1 = tempM1;
    next_M2 = tempM2;
    next_S = tempS;
    
    flag_comando_recibido = true; 
    
    // Serial.print("Cmd Recibido -> M1: "); Serial.print(next_M1);
    // Serial.print(" | M2: "); Serial.println(next_M2);

  } else if (msg.indexOf("Stop") >= 0) {
    flag_detener = true;
    setState(STOP);
  }
}

bool controlPID(float p1, float i1, float d1, float p2, float i2, float d2, int lim1, int lim2) {
  
  if (micros() - time_ant_pid < Period_PID) return false; 
  
  time_ant_pid = micros();
  float dt = Period_PID / 1000000.0;

  noInterrupts();
  long pos0 = myEnc1.read(); 
  long pos1 = myEnc2.read();
  interrupts();

  int out1 = 0;
  int out2 = 0;

  angulo_actual1 = (float)pos0 / COUNTS_PER_DEGREE1;
  angulo_actual2 = (float)pos1 / COUNTS_PER_DEGREE2;

  // PID M1
  error_pid1 = angulo_objetivo1 - angulo_actual1;
  integral_pid1 += error_pid1 * dt;
  integral_pid1 = constrain(integral_pid1, -60, 60); 
  float derivada1 = (error_pid1 - error_anterior1) / dt;
  pid_output1 = (p1 * error_pid1) + (i1 * integral_pid1) + (d1 * derivada1);
  error_anterior1 = error_pid1;

  // PID M2
  error_pid2 = angulo_objetivo2 - angulo_actual2;
  integral_pid2 += error_pid2 * dt;
  integral_pid2 = constrain(integral_pid2, -50, 50); 
  float derivada2 = (error_pid2 - error_anterior2) / dt;
  pid_output2 = (p2 * error_pid2) + (i2 * integral_pid2) + (d2 * derivada2);
  error_anterior2 = error_pid2;


  if (flag_bolsa) {
      // Si hay bolsa, limitamos la velocidad del motor 2 a la mitad (según tu lógica)
    current_limit2 = (int)(limit_vel2 * 0.5);
      // current_limit1 se queda igual según tu código anterior, pero puedes bajarlo aquí si quieres
    // Si hay bolsa, limitamos la velocidad del motor 2 a la mitad
    out1 = constrain((int)pid_output1, -(limit_vel1/1.15), (limit_vel1/1.15)); 
    out2 = constrain((int)pid_output2, -(limit_vel2/2), (limit_vel2/2)); 
    Serial.println("Limitando motores por papa --------------------------------");
    out2 = out2/1.5;
  } else{
    out1 = constrain((int)pid_output1, -limit_vel1, limit_vel1); 
    out2 = constrain((int)pid_output2, -limit_vel2, limit_vel2);
  }
  

  // Zona muerta
  // if (abs(error_pid1) < 1.0) out1 = 0;
  // else if (out1 > 0 && out1 < 16) out1 = 20; 
  // else if (out1 < 0 && out1 > -16) out1 = -20; 

  // if (abs(error_pid2) < 1.0) out2 = 0;
  // else if (out1 > 0 && out1 < 10) out1 = 15; 
  // else if (out1 < 0 && out1 > -10) out1 = -15; 

  ST.motor(canalM1, -out1);
  ST.motor(canalM2, out2);

  if (abs(error_pid1) < 1.0 && abs(error_pid2) < 1.5) {
      return true;
  } else {
      return false;
  }
}

void setState(State s) {
  if (state != s) {
    // Serial.print("Estado: "); Serial.println(s);
    lastState = state;
    state = s;
  }
}
