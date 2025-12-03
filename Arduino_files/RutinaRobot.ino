///// LIBRERIAS //////////////////
#include <LiquidCrystal_I2C.h> 
#include <TimerOne.h>
#include <Servo.h>
#include <Sabertooth.h>
#include <Encoder.h>

// =====================
// DEFINICIONES DE COORDENADAS
// =====================
struct Coordenada {
  int m1;
  int m2;
};

const int NUM_PASOS = 6;
int safe_index = 0;

// Lista de coordenadas para soltar en la caja
Coordenada coordenadas_caja[NUM_PASOS] = {
  {13, 187},  // Paso 0
  {31, 228},  // Paso 1
  {28, 257},  // Paso 2
  {13, 224},  // Paso 3
  {11, 281},  // Paso 4
  {1,  249}   // Paso 5
};

// ===== CONFIGURACIÓN ROBOT =====
// Variables para el filtro de la derivada 
float last_derivative1 = 0.0;
float last_derivative2 = 0.0;
const float alpha = 0.7; // Factor de suavizado (0.0 a 1.0)

const float k_coreccion = 1.00;
const float COUNTS_PER_REV_ARM = 1920 * 3; 
const float COUNTS_PER_DEGREE1 = COUNTS_PER_REV_ARM * k_coreccion / 360.0;
const float COUNTS_PER_DEGREE2 = COUNTS_PER_REV_ARM / 360.0;

int canalM1 = 1;
int canalM2 = 2;

// ===== MÁQUINA DE ESTADOS =====
enum State {
  INICIO,
  CENTRANDO,
  ESPERANDO_PAPA,
  MOVIENDOSE,
  ESPERAR,
  RECOGER_PAPA,
  SUBIENDO_CON_PAPA,
  SOLTAR_PAPA_1, // Etapa 1: Mover M1
  SOLTAR_PAPA_2, // Etapa 2: Mover M2
  BAJAR_PAPA,
  SOLTAR_PAPA,   // Acción de soltar (Servo)
  FIN_RUTINA,
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
const int Servo_PWM = 40;
const int DIR = 24;
const int PUL = 28;
const int bomba = 31;
const int boton_mover = 50;
const int boton_subir = 52;
const int pinSensorHall = A0; 

// CR-TOUCH
const int CRTouch_Detection = 4; 
const int CRTouch_Control = 9;

// Limites
const int limite_inferior = 23;
const int limite_lateral1 = 53;
const int limite_lateral2 = 51;

// ===== VARIABLES PID =====
// VARIABLES CON PAPA
float kp1_papa = 3.2, ki1_papa = 0.00001, kd1_papa = 0.00003;
float kp2_papa = 3.5, ki2_papa = 0.00001, kd2_papa = 0.000001;

// VARIABLES SIN PAPA
float kp1 = 3.0, ki1 = 0.000005, kd1 = 0.001;
float kp2 = 2.7, ki2 = 0.000001, kd2 = 0.000;

const unsigned long Period_PID = 10000UL; // 10ms
unsigned long time_ant_pid = 0;

// Variables de Control KI Dinámico Motor 1
float window_error1 = 6.0;
int counter_window1 = 0;
int window_length1 = 20;
float ki_actual1 = 0.0;
int amplificador1 = 0.0;

// Variables de Control KI Dinámico Motor 2
float window_error2 = 6.0;
int counter_window2 = 0;
int window_length2 = 10;
float ki_actual2 = 0.0;
int amplificador2 = 0.0;

float angulo_actual1 = 0.0;
float angulo_actual2 = 0.0;
float angulo_objetivo1 = 0.0; 
float angulo_objetivo2 = 0.0;
float angulo_objetivo_servo = 0.0;
float angulo_max1 = 0.0;
float angulo_max2 = 0.0;

float error_pid1 = 0, integral_pid1 = 0, error_anterior1 = 0;
float error_pid2 = 0, integral_pid2 = 0, error_anterior2 = 0;
float pid_output1 = 0;
float pid_output2 = 0;

const int limit_vel1 = 20;
const int limit_vel2 = 35;
int l1_soltar = 0;
int l2_soltar = 0;

// Buffer Serial
float next_M1 = 0.0;
float next_M2 = 0.0;
float next_S = 0.0;
bool flag_comando_recibido = false; 
String buffer = "";
volatile bool flag_detener = false;

// ===== VARIABLES SISTEMA =====
bool flag_limite_inferior = false;
bool flag_limite_lateral1 = false;
bool flag_limite_lateral2 = false;
bool flag_servo_listo = false;
bool flag_centro = false;
bool flag_cambio = true;
bool flag_bolsa = false;

int current_limit2 = 0;
int counter_papas = 0;
int max_papas = 20;

// Variables Stepper
volatile long pasos_restantes = 0;
volatile bool enable_step = false;
volatile bool step_pin_state = false;

// COMPONENTES
Sabertooth ST(128, Serial2);
Servo MG995_Servo;
Servo cr_touch;
Encoder myEnc1(3, 2);
Encoder myEnc2(18, 19); 
LiquidCrystal_I2C lcd(0x27, 16, 2);

// =====================
// PROTOTIPOS DE FUNCIONES
// =====================
void procesarMensaje(String msg);
void setState(State s);
void leerLimites();
bool controlPID(float p1, float i1, float d1, float p2, float i2, float d2, int lim1, int lim2, int amp1, int amp2);
void leerSerialNoBloqueante();
void resetPID();

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
  pinMode(limite_lateral2, INPUT); 
  
  // Pines Actuadores
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(Servo_PWM, OUTPUT);
  pinMode(bomba, OUTPUT);
  pinMode(boton_subir, INPUT);
  pinMode(boton_mover, INPUT);
  
  // Sensor Hall
  pinMode(pinSensorHall, INPUT);

  // Servos
  MG995_Servo.attach(Servo_PWM);
  MG995_Servo.write(0);
  MG995_Servo.detach();
  
  delay(100);
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

  Timer1.initialize(500); 
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
    flag_servo_listo = false;
    flag_centro = false;
    flag_cambio = true;
    flag_bolsa = false;
    ST.motor(canalM1, 0); ST.motor(canalM2, 0);
    // enable_step = false;
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
          homing_sub_state = HOME_SUBIR;
          digitalWrite(DIR, HIGH); 
          pasos_restantes = 1000;
          enable_step = true;
        }
      break;

      case HOME_SUBIR:
        ST.motor(canalM1, 0);
        ST.motor(canalM2, 0);
        
        if (pasos_restantes <= 0 && enable_step == false) {
            digitalWrite(DIR, LOW);
            homing_sub_state = HOME_M1_FIND;
            flag_limite_inferior = false;
        }
      break;

      case HOME_M1_FIND:
      leerLimites();
        if (!flag_centro) {
          ST.motor(canalM1, 18); 
          leerLimites();
        } else {
          ST.motor(canalM1, 0);
          myEnc1.write(0); 

          homing_sub_state = HOME_M2_FIND;
        }
        break;

      case HOME_M2_FIND:
        leerLimites();
        if (digitalRead(limite_lateral1)){
          leerLimites();
          ST.motor(canalM2, 0);
          myEnc2.write(0); 
          // Configurar pose conocida
          angulo_objetivo1 = 45.0;
          angulo_objetivo2 = 155.0;       
          
          resetPID(); 
          homing_sub_state = HOME_CENTRAR;
          delay(100);
        } else {
          ST.motor(canalM2, -13); 
          leerLimites();
        }
        break;

      case HOME_CENTRAR:
      amplificador1 = 1500;
      amplificador2 = 1500;
        if (flag_detener || digitalRead(limite_lateral2) ){ 
          setState(STOP); 
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("DETENER");  
        }

        if (controlPID(kp1, ki1, kd1, kp2, ki2, kd2, limit_vel1, limit_vel2, amplificador1, amplificador2)){
           resetPID(); 
           homing_sub_state = HOME_DONE;
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
    if (flag_comando_recibido){
      angulo_objetivo1 = next_M1;
      angulo_objetivo2 = next_M2;
      angulo_objetivo_servo = next_S;
      flag_comando_recibido = false;
      
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("M1:"); lcd.print(next_M1, 1);
      lcd.setCursor(8,0);
      lcd.print("M2:"); lcd.print(next_M2, 1);
      lcd.clear();
      lcd.print("MOVIENDOSE"); 
      setState(MOVIENDOSE);
    } 
    break;

  case MOVIENDOSE:
    leerLimites();

    amplificador1 = 1500;
    amplificador2 = 1500;

    if (flag_detener || digitalRead(limite_lateral1) || digitalRead(limite_lateral2)) {
      setState(STOP); 
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("DETENER");  
    }

    if (controlPID(kp1, ki1, kd1, kp2, ki2, kd2, limit_vel1, limit_vel1, amplificador1, amplificador2)){
      setState(ESPERAR);
      lcd.clear();
      lcd.print("ESPERAR");
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
            lcd.clear();
            lcd.print("SUBIENDO");
            delay(150);    
          }
        }
      }
    } else {
      pasos_restantes = 500;
      digitalWrite(DIR, HIGH); 
      // enable_step = false;
      setState(INICIO); 
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
          digitalWrite(DIR, LOW); 
          safe_index = counter_papas % NUM_PASOS;          
          angulo_objetivo1 = coordenadas_caja[safe_index].m1;
          // SOLO Movemos M1 en la etapa 1
          
          resetPID();
          setState(SOLTAR_PAPA_1); 
          lcd.clear();
          lcd.print("YENDO A ANGULO M1");
          lcd.setCursor(0, 1);
          lcd.print("POS: "); lcd.print(safe_index);
          delay(500);
      }
      break;

  case SOLTAR_PAPA_1:
    leerLimites(); // SEGURIDAD AGREGADA
    if (flag_detener || digitalRead(limite_lateral1) || digitalRead(limite_lateral2)) {
      setState(STOP); 
    }

    amplificador1 = 1500;
    amplificador2 = 1500;
    if (flag_bolsa) {
        l1_soltar = (int)(limit_vel1 / 1.0); 
        l2_soltar = (int)(limit_vel2 / 2.3);  
    }else {
        l1_soltar = limit_vel1;
        l2_soltar = limit_vel2;
    }

    if (controlPID(kp1_papa, ki1_papa, kd1_papa, kp2_papa, ki2_papa, kd2_papa, l1_soltar, l2_soltar, amplificador1, amplificador2)){
      resetPID();
      angulo_objetivo2 = coordenadas_caja[safe_index].m2; // Ahora preparamos M2
      setState(SOLTAR_PAPA_2); 
      lcd.clear();
      lcd.print("YENDO A ANGULO M2");
      lcd.setCursor(0, 1);
      lcd.print("POS: "); lcd.print(safe_index);
      delay(100);
    }
  break;

  case SOLTAR_PAPA_2:
    leerLimites(); // SEGURIDAD AGREGADA
    if (flag_detener || digitalRead(limite_lateral1) || digitalRead(limite_lateral2)) {
      setState(STOP); 
    }

    if (controlPID(kp1_papa, ki1_papa, kd1_papa, kp2_papa, ki2_papa, kd2_papa, l1_soltar, l2_soltar, amplificador1, amplificador2)){
      resetPID();
      setState(BAJAR_PAPA); 
      pasos_restantes = 1500; 
      enable_step = true;
      lcd.clear();
      lcd.print("SOLTANDO...");
      delay(500);
      MG995_Servo.attach(Servo_PWM);
      MG995_Servo.write(angulo_objetivo_servo);
      delay(50);
    }
  break;
  
  case BAJAR_PAPA:
    leerLimites();

    if (!flag_limite_inferior && enable_step) {     
      digitalWrite(DIR, LOW); 
      // enable_step = true;
    }else {
      enable_step = false;
      setState(SOLTAR_PAPA); 
    }

  break;
  case SOLTAR_PAPA:
    counter_papas++;
    flag_bolsa = false;
    digitalWrite(bomba, LOW);
    // Acciones físicas de soltar
    cr_touch.write(90); // Sonda arriba
    delay(50);
    cr_touch.write(160);  
    delay(500);
    cr_touch.write(10); // Desplegar
    delay(500);
    
    
    if(counter_papas < max_papas){
      // EFICIENCIA: Volver a esperar orden en lugar de recalibrar
      setState(INICIO); 
      lcd.clear();
      lcd.print("LISTO SIGUIENTE");
    } else{
      setState(FIN_RUTINA);
    }
    break;

  case FIN_RUTINA:
     lcd.clear();
     lcd.print("FIN RUTINA");
     // Detener todo
     ST.motor(canalM1, 0); ST.motor(canalM2, 0);
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

    // Transformación de coordenadas del usuario
    next_M1 = tempM1 + 45; 
    next_M2 = 155 - tempM2;
    next_S = tempS;
    
    flag_comando_recibido = true; 
  } else if (msg.indexOf("Stop RASPI") >= 0) {
    flag_detener = true;
    setState(STOP);
  }
}

// === RESET PID COMPLETO ===
void resetPID() {
  integral_pid1 = 0;
  integral_pid2 = 0;
  error_anterior1 = 0; 
  error_anterior2 = 0;
  last_derivative1 = 0; 
  last_derivative2 = 0; 
  
  // Reset Ki dinamico M1
  counter_window1 = 0;
  ki_actual1 = ki1;

  // Reset Ki dinamico M2
  counter_window2 = 0;
  ki_actual2 = ki2;
  
  time_ant_pid = micros(); 
}

// === PID OPTIMIZADO CON KI DINÁMICO DOBLE ===
bool controlPID(float p1, float i1, float d1, float p2, float i2, float d2, int lim1, int lim2, int amp1, int amp2) {
  
  if (micros() - time_ant_pid < Period_PID) return false; 
  
  time_ant_pid = micros();
  float dt = Period_PID / 1000000.0;

  noInterrupts();
  long pos0 = myEnc1.read(); 
  long pos1 = myEnc2.read();
  interrupts();

  angulo_actual1 = (float)pos0 / COUNTS_PER_DEGREE1;
  angulo_actual2 = (float)pos1 / COUNTS_PER_DEGREE2;

  // Seguimiento de máximos
  if (angulo_max1 < angulo_actual1) angulo_max1 = angulo_actual1;
  if (angulo_max2 < angulo_actual2) angulo_max2 = angulo_actual2;

  // --- PID M1 (con Ki Dinámico) ---
  error_pid1 = angulo_objetivo1 - angulo_actual1;
  ki_actual1 = i1;

  if (abs(error_pid1) < window_error1 && abs(error_pid1) > 1.0) {
    counter_window1++;
    if (counter_window1 >= window_length1){
      ki_actual1 = amp1 * i1; 
    }
  } else {
    counter_window1 = 0; 
  }

  // --- PID M2 (con Ki Dinámico) ---
  error_pid2 = angulo_objetivo2 - angulo_actual2;
  ki_actual2 = i2;

  if (abs(error_pid2) < window_error2 && abs(error_pid2) > 1.0) {
    counter_window2++;
    if (counter_window2 >= window_length2){
      ki_actual2 = amp2 * i2; 
    }
  } else {
    counter_window2 = 0; 
  }

  // Integrales
  integral_pid1 += error_pid1 * dt;
  integral_pid1 = constrain(integral_pid1, -60, 60); 

  integral_pid2 += error_pid2 * dt;
  integral_pid2 = constrain(integral_pid2, -50, 50); 
  
  // Derivadas Filtradas
  float raw_derivative1 = (error_pid1 - error_anterior1) / dt;
  float derivative1 = alpha * last_derivative1 + (1.0 - alpha) * raw_derivative1;
  last_derivative1 = derivative1;
  
  float raw_derivative2 = (error_pid2 - error_anterior2) / dt;
  float derivative2 = alpha * last_derivative2 + (1.0 - alpha) * raw_derivative2;
  last_derivative2 = derivative2;
  
  // Calculo Salidas
  pid_output1 = (p1 * error_pid1) + (ki_actual1 * integral_pid1) + (d1 * derivative1);
  pid_output2 = (p2 * error_pid2) + (ki_actual2 * integral_pid2) + (d2 * derivative2);
  
  error_anterior1 = error_pid1;
  error_anterior2 = error_pid2;

  // --- LÓGICA DE SALIDAS (LIMITADORES) ---
  int out1 = constrain((int)pid_output1, -lim1, lim1); 
  int out2 = constrain((int)pid_output2, -lim2, lim2);
  
  // Zona muerta
  if (abs(error_pid1) < 1.5) out1 = 0; 
  if (abs(error_pid2) < 1.5) out2 = 0;

  ST.motor(canalM1, -out1);
  ST.motor(canalM2, out2);

  // Verificación de llegada a posición estable
  if (abs(error_pid1) < 1.5 && abs(error_pid2) < 1.5) { 
      if (abs(derivative1) < 5.0 && abs(derivative2) < 5.0) {
          ST.motor(canalM1, 0);
          ST.motor(canalM2, 0);
          return true; // LLegó
      }
  }
  
  return false; 
}

void setState(State s) {
  if (state != s) {
    lastState = state;
    state = s;
  }
}
