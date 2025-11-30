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
// const float COUNTS_PER_REV_ARM = 1920 * 3;
// const float COUNTS_PER_DEGREE = COUNTS_PER_REV_ARM / 360.0;
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
const int CRTouch_Detection = 2;
const int CRTouch_Control = 9;

// Limites
const int limite_inferior = 23;
const int limite_lateral1 = 53;
// const int limite_lateral2 = 51;

// ===== VARIABLES PID =====
float kp1 = 4.0, ki1 = 0.15, kd1 = 0.01;
float kp2 = 2.0, ki2 = 0.25, kd2 = 0.03;


const unsigned long Period_PID = 10000UL;
float angulo_actual1 = 0.0;
float angulo_actual2 = 0.0;
float angulo_objetivo1 = 30.0; 
float angulo_objetivo2 = 30.0;
float angulo_objetivo_servo = 30.0;

float error_pid1 = 0, integral_pid1 = 0, error_anterior1 = 0;
float error_pid2 = 0, integral_pid2 = 0, error_anterior2 = 0;

float pid_output1 = 0;
float pid_output2 = 0;

// ===== VARIABLES SISTEMA =====
bool flag_limite_inferior = false;
bool flag_limite_lateral1  = false;
// bool flag_limite_lateral2  = false;
bool flag_servo_listo = false;
bool flag_centro = false;
bool flag_cambio = true;
bool flag_bolsa = false;
const int limit_vel1 = 25;
const int limit_vel2 = 60;

// Variables Stepper
volatile long pasos_restantes = 0;
volatile bool enable_step = false;
volatile bool step_pin_state = false;

// Objetos Encoder (Pines con Interrupción en MEGA: 2, 3, 18, 19, 20, 21)
Encoder myEnc1(18, 19); // M1 Base
Encoder myEnc2(20, 21); // M2 Brazo

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
void controlPID();
void leerSerialNoBloqueante();

// =====================
// INTERRUPCIONES
// =====================

// Interrupción del Timer1 para el Stepper
void stepISR() {
  if (!enable_step) return;

  step_pin_state = !step_pin_state;
  digitalWrite(PUL, step_pin_state);

  // Solo descontamos pasos si el pin pasa a HIGH (un pulso completo)
  if (step_pin_state && pasos_restantes > 0) {
    pasos_restantes--;
  } else if (step_pin_state && pasos_restantes <= 0) {
     // Si llegamos a 0 pasos, detenemos.
     // NOTA: Para movimiento continuo (buscar limite), iniciamos pasos_restantes en 999999
     enable_step = false; 
  }
}

void leerLimites() {
  flag_limite_inferior = digitalRead(limite_inferior);
  flag_limite_lateral1 = digitalRead(limite_lateral1);
  // flag_limite_lateral2 = digitalRead(limite_lateral2);
  
  // Sensor Hall/IR Analogico
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

  // Pines Sensores
  pinMode(limite_inferior, INPUT); 
  pinMode(limite_lateral1, INPUT);
  // pinMode(limite_lateral2, INPUT);
  
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
  cr_touch.write(160);  // Comando de Reseteo de Alarma
  delay(500);

  Serial.println("Enviando comando de despliegue CR-Touch...");
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
      leerLimites();
      
      if (digitalRead(boton_mover)){
          setState(CENTRANDO);
      }
        
      break;

    case CENTRANDO:
      leerLimites();

      switch (homing_sub_state) {
        case HOME_START:
          ST.motor(canalM1, 0);
          ST.motor(canalM2, 0);
          enable_step = false;
          Serial.println("Homing: Buscando limite inferior Z");
          homing_sub_state = HOME_Z_DOWN;
          break;

        case HOME_Z_DOWN:
          leerLimites();
          // Bajar hasta encontrar limite
          if (!flag_limite_inferior) {
            digitalWrite(DIR, LOW); // Dirección BAJAR
            pasos_restantes = 999999; 
            enable_step = true;
          } else {
            // Límite encontrado
            enable_step = false;
            Serial.println("Homing: Z Listo. Buscando Centro M1");
            homing_sub_state = HOME_M1_FIND;
          }
          break;

        case HOME_M1_FIND:
        leerLimites();
          // Mover M1 hasta sensor Hall
          if (!flag_centro) {
            ST.motor(canalM1, 19); // Velocidad lenta
            leerLimites();
          } else {
            ST.motor(canalM1, 0);
            myEnc1.write(0); // Reset Encoder 1
            
            Serial.println("Homing: M1 Listo. Buscando Tope M2");
            homing_sub_state = HOME_M2_FIND;
          }
          break;

        case HOME_M2_FIND:
          leerLimites();
          // Mover M2 hasta switch lateral 2
          if (digitalRead(limite_lateral1)){
            leerLimites();
            ST.motor(canalM2, 0);
            myEnc2.write(0); // Reset Encoder 2
            Serial.println("Homing: M2 Listo. Subiendo Z...");

            if (flag_limite_lateral1){
              Serial.println("Por Limit 1");
            }
            // if (flag_limite_lateral2){
            //   Serial.println("Por Limit 2");
            // }
            
            // Subir Z un poco para no arrastrar
            digitalWrite(DIR, HIGH);
            pasos_restantes = 1000;
            enable_step = true;
            homing_sub_state = HOME_SUBIR;
          } else {
            ST.motor(canalM2, -15); // Velocidad negativa hacia el switch
            leerLimites();
          }
          break;

        case HOME_SUBIR:
          ST.motor(canalM1, 0);
          ST.motor(canalM2, 0);
          
          // Esperar a que el stepper termine de subir
          if (pasos_restantes <= 0 && enable_step == false) {
             homing_sub_state = HOME_DONE;
             flag_limite_inferior = false;
             Serial.println("Homing COMPLETO.");
          }
          break;

        case HOME_DONE:
          homing_sub_state = HOME_START;
          setState(ESPERANDO_PAPA);
          Serial.println("ESPERANDO PAPA.");
          break;
      }
      break;

    case ESPERANDO_PAPA:
      ST.motor(canalM1, 0); ST.motor(canalM2, 0);
      enable_step = false;

      // Opción de comando manual de prueba con botón (opcional)
      if (digitalRead(boton_mover) == HIGH) {
        myEnc1.write(0); 
        myEnc2.write(0); 
        setState(MOVIENDOSE);
        Serial.println("Realizando PID");
      }
      break;

    case MOVIENDOSE:
      leerLimites();
      if (!flag_servo_listo){
          MG995_Servo.write(angulo_objetivo_servo);
          flag_servo_listo = true;
      }  
      controlPID(); 
    
      // Salida de seguridad
      if (flag_detener || digitalRead(limite_lateral1)) {
        // setState(STOP); // Descomentar si se quiere stop estricto por límites
      }
      break;
    case ESPERAR:
      if (digitalRead(boton_mover)){
        setState(RECOGER_PAPA);
      }

      break;
    case RECOGER_PAPA:
      digitalWrite(DIR, LOW); 
      leerLimites();

      if (!flag_limite_inferior) { 

        if (!digitalRead(CRTouch_Detection)){
          digitalWrite(DIR, LOW); // Dirección BAJAR
          pasos_restantes = 999999;
          enable_step = true;
        }       
        
        if (digitalRead(CRTouch_Detection)){
          // CR-Touch detectó objeto
          Serial.println("Papa detectada");
          digitalWrite(bomba, HIGH);
          if (!flag_bolsa){
            pasos_restantes = 200;
            enable_step = true;
            flag_bolsa = true;
          }else{
            if (pasos_restantes <=0){
              Serial.println("Subiendo Papa");
              setState(SUBIENDO_CON_PAPA);
              delay(50);    
            }
          }
          
          
        }
      } else {
        enable_step = false;
        Serial.println("PAPA NO ENCONTRADA (Límite Z alcanzado).");
        setState(CENTRANDO); // O volver a inicio
      }
      break;

    case SUBIENDO_CON_PAPA:
        digitalWrite(DIR, HIGH); // Dirección ARRIBA
        if (!enable_step) {
            pasos_restantes = 1000; // Subir lo suficiente
            enable_step = true;
        }
        
        // Cuando termine de subir
        if (pasos_restantes <= 0) {
            enable_step = false;
            // Aquí decides qué hacer después de recogerla (ej: ir a dejarla)
            Serial.println("Subida completa. Esperando nueva orden.");
            setState(SOLTAR_PAPA); 
            delay(50);
        }
        break;
    
    case SOLTAR_PAPA:
      controlPID();

      break;

    case STOP:
        MG995_Servo.detach();
        ST.motor(canalM1, 0);
        ST.motor(canalM2, 0);
        enable_step = false;
        digitalWrite(bomba, LOW);
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
  // Aseguramos que la orden sea "Mover" y tenga estructura básica
  // Formato esperado: "Mover;M1:XX;M2:XX;S:XX;bit1;bit2"
  if (msg.startsWith("Mover;") && msg.indexOf("M1:") > 0) {
    
    // Variables temporales para magnitud
    float tempM1 = 0.0;
    float tempM2 = 0.0;

    // --- 1. Extracción de Magnitud M1 ---
    int startM1 = msg.indexOf("M1:") + 3; // +3 salta "M1:"
    int endM1 = msg.indexOf(";", startM1); 
    if (endM1 != -1) {
      tempM1 = msg.substring(startM1, endM1).toFloat();
    }

    // --- 2. Extracción de Magnitud M2 ---
    int startM2 = msg.indexOf("M2:", endM1) + 3; 
    int endM2 = msg.indexOf(";", startM2);
    if (endM2 != -1) {
      tempM2 = msg.substring(startM2, endM2).toFloat();
    }

    // --- 3. Extracción de S (Servo) ---
    // Buscamos el siguiente ';'
    int startS = msg.indexOf("S:", endM2) + 2; 
    int endS = msg.indexOf(";", startS);
    
    if (endS != -1) {
      String strS = msg.substring(startS, endS);
      angulo_objetivo_servo = strS.toFloat();
    
      // --- 4. Extracción Bit Signo M1 ---
      // Esta justo después del ';' que cerró a S
      int startBit1 = endS + 1; 
      int endBit1 = msg.indexOf(";", startBit1);
      
      if (endBit1 != -1) {
         int signM1 = msg.substring(startBit1, endBit1).toInt();
         // Si el bit es 1, invertimos el signo (Negativo)
         // Si el bit es 0, dejamos positivo
         if (signM1 == 1) {
            tempM1 = tempM1 * -1.0;
         }
      }

      // --- 5. Extracción Bit Signo M2 ---
      // Esta justo después del ';' del bit anterior hasta el final
      int startBit2 = endBit1 + 1;
      // Como es el último, vamos hasta el final del string si no hay más separadores
      if (startBit2 > 0 && startBit2 < msg.length()) {
          int signM2 = msg.substring(startBit2).toInt();
          
          if (signM2 == 1) {
             tempM2 = tempM2 * -1.0;
          }
      }
    }
    // --- Asignación Final ---
    angulo_objetivo1 = tempM1;
    angulo_objetivo2 = tempM2;
    flag_servo_listo = false; // Resetear bandera servo
    flag_detener = false;
    
    Serial.print("Cmd Recibido -> M1: "); Serial.print(angulo_objetivo1);
    Serial.print(" | M2: "); Serial.println(angulo_objetivo2);

    setState(MOVIENDOSE);

  } else if (msg.indexOf("Stop") >= 0) {
    flag_detener = true;
    setState(STOP);
  }
}

void controlPID() {
  if (micros() - time_ant_pid < Period_PID) return; // Ejecutar solo cada 10ms
  
  time_ant_pid = micros();
  float dt = Period_PID / 1000000.0;

  // 1. Leer Posición REAL (Snapshot atómico)
  // long pos0, pos1;
  noInterrupts();
  long pos0 = myEnc1.read(); 
  long pos1 = myEnc2.read();
  interrupts();

  // 2. Convertir
  angulo_actual1 = (float)pos0 / COUNTS_PER_DEGREE1;
  angulo_actual2 = (float)pos1 / COUNTS_PER_DEGREE2;

  // 3. PID M1
  error_pid1 = angulo_objetivo1 - angulo_actual1;
  integral_pid1 += error_pid1 * dt;
  integral_pid1 = constrain(integral_pid1, -60, 60);
  float derivada1 = (error_pid1 - error_anterior1) / dt;
  pid_output1 = (kp1 * error_pid1) + (ki1 * integral_pid1) + (kd1 * derivada1);
  error_anterior1 = error_pid1;

  // 4. PID M2
  error_pid2 = angulo_objetivo2 - angulo_actual2;
  integral_pid2 += error_pid2 * dt;
  integral_pid2 = constrain(integral_pid2, -50, 50);
  float derivada2 = (error_pid2 - error_anterior2) / dt;
  pid_output2 = (kp2 * error_pid2) + (ki2 * integral_pid2) + (kd2 * derivada2);
  error_anterior2 = error_pid2;

  // 5. Salidas

  int current_limit1 = limit_vel1;
  int current_limit2 = limit_vel2;

  if (flag_bolsa) {
      // Si hay bolsa, limitamos la velocidad del motor 2 a la mitad (según tu lógica)
      current_limit2 = (int)(limit_vel2 * 0.5);
      // current_limit1 se queda igual según tu código anterior, pero puedes bajarlo aquí si quieres
  }
  int out1 = constrain((int)pid_output1, -limit_vel1, limit_vel1); 
  int out2 = constrain((int)pid_output2, -current_limit2, current_limit2);

  if (abs(error_pid1) < 1.0) {
      out1 = 0;
  } else {
      // Si estamos lejos, aseguramos que la potencia mínima sea 50
      if (out1 > 0 && out1 <= 16)  out1 = 20;  // Empujón mínimo positivo
      if (out1 < 0 && pid_output1 >= -16) out1 = -20; // Empujón mínimo negativo
      // No necesitamos limitar hacia arriba aquí, el constrain final lo hará
  }

  if (abs(error_pid2) < 1.0) {
      out2 = 0;
  } else {
      // Si estamos lejos, aseguramos que la potencia mínima sea 50
      if (out2 > 0 && out2 <= 17)  out2 = 18;  // Empujón mínimo positivo
      if (out2 < 0 && pid_output2 >= -17) out2 = -18; // Empujón mínimo negativo
      // No necesitamos limitar hacia arriba aquí, el constrain final lo hará
  }

  ST.motor(canalM1, -out1);
  ST.motor(canalM2, out2);


  Serial.println("Error 1");
  Serial.println(error_pid1);
  Serial.println("Aactuacion 1");
  Serial.println(-out1);
  Serial.println("Error 2");
  Serial.println(error_pid2);
  Serial.println("Aactuacion 2");
  Serial.println(out2);
 

  // 6. Llegada al objetivo y transición
  if (abs(error_pid1) < 1.0 && abs(error_pid2) < 1.0) {
    if (!flag_bolsa){
      error_pid1 = 0;
      error_pid2 = 0;
      error_anterior1 = 0;
      error_anterior2 = 0;
      Serial.println("RECOGIENDO PAPA");
      myEnc1.write(0);
      myEnc2.write(0);
      angulo_objetivo1 = -20.0;
      angulo_objetivo2 = 60.0;
      setState(ESPERAR);

    } else{
      Serial.println("SOLTANDO SOBRE LA CAJA");
      myEnc1.write(0);
      myEnc2.write(0);
      error_pid1 = 0;
      error_pid2 = 0;
      error_anterior1 = 0;
      error_anterior2 = 0;
      delay(50);
      digitalWrite(bomba, LOW);
      setState(ESPERANDO_PAPA);
      delay(50);
    } 
  }
}

void setState(State s) {
  if (state != s) {
    Serial.print("Estado: "); Serial.println(s);
    lastState = state;
    state = s;
  }
}
