///// LIBRERIAS //////////////////
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <Servo.h>
#include <Sabertooth.h>

// ===== CONFIGURACIÓN ROBOT =====
// Pololu 4752 (30:1) + Encoder 64 CPR + Reducción externa 3:1
// Total CPR = 64 * 30 * 3 = 25200 cuentas por 360 grados del brazo
const float COUNTS_PER_REV_ARM = 1920*3; 
const float COUNTS_PER_DEGREE = COUNTS_PER_REV_ARM / 360.0;

enum State {
  INICIO,
  BAJANDO,
  SUBIENDO,
  MOVIENDOSE,
  STOP
};

State state = INICIO;
State lastState = INICIO;

// ===== PINES =====
const int Servo_PWM = 22;
const int DIR = 30;
const int PUL = 28;
const int bomba = 31; // OJO: Cambié a 31 porque tenías repetido el pin 30 para DIR y bomba
const int boton_mover = 50;
const int boton_subir = 52;

// Encoders
#define encoder0PinA 19
#define encoder0PinB 18
#define encoder1PinA 20
#define encoder1PinB 21

const int limite_inferior = 23;
const int limite_lateral = 53;
const int limite_lateral2 = 51;
// ===== VARIABLES PID =====
// Ajuste estos valores: 
float kp1 = 1, ki1 = 0.0, kd1 = 0.0; // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< VALORES PID M1 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> M1 motor de base
float kp2 = 0.5, ki2 = 0.0, kd2 = 0.0;// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< VALORES PID M2 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> M2 motore de extremo efector

float angulo_actual1 = 0.0;
float angulo_actual2 = 0.0;
float angulo_objetivo1 = 0.0; // Iniciar en 0 para evitar salto brusco
float angulo_objetivo2 = 0.0;
float angulo_objetivo_servo = 45.0;

float error_pid1 = 0, integral_pid1 = 0, error_anterior1 = 0;
float error_pid2 = 0, integral_pid2 = 0, error_anterior2 = 0;

float pid_output1 = 0;
float pid_output2 = 0;

// ===== VARIABLES SISTEMA =====
volatile bool flag_limite_inferior = false;
volatile bool flag_limite_lateral  = false;
bool flag_fondo = false;
volatile long pasos_restantes = 100; // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< SI RUTINA DE SUBIR Y BAJAR FUNCIONA AUMENTAR PASOS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><
volatile bool enable_step = false;
volatile bool step_pin_state = false;

// Variables encoder (volatile para interrupciones)
volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;
int dir_lectura = 1;

// Tiempos
unsigned long time_ant_pid = 0;
unsigned long time_ant_lcd = 0;
const unsigned long Period_PID = 10000UL; // 10 ms
// const unsigned long Period_LCD = 300000UL; // 300 ms (refresco pantalla)

// Comunicación
String buffer = "";
volatile bool flag_detener = false;

// COMPONENTES
Sabertooth ST(128, Serial2);
Servo MG995_Servo;
// LiquidCrystal_I2C lcd(0x27, 16, 2);

// =====================
// INTERRUPCIONES
// =====================
void doEncoder0A() {
  // Lectura eficiente usando bitwise si es posible, o digitalRead rápido
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
void stepISR() {
  if (!enable_step) return;
  step_pin_state = !step_pin_state;
  digitalWrite(PUL, step_pin_state);
  if (step_pin_state && pasos_restantes > 0) pasos_restantes--;
}

void leerLimites_ISR() {
  flag_limite_inferior = digitalRead(limite_inferior); // Asumiendo active HIGH
  flag_limite_lateral  = digitalRead(limite_lateral);
}

// =====================
// SETUP
// =====================
void setup() {
  Serial.begin(115200); 
  Serial2.begin(9600); // Sabertooth
  ST.autobaud();
  delay(500); // Esperar a que Sabertooth inicie bien

  // Configurar Pines
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(limite_inferior, INPUT);
  pinMode(limite_lateral, INPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(Servo_PWM, OUTPUT);
  pinMode(bomba, OUTPUT);
  pinMode(boton_subir, INPUT);
  pinMode(boton_mover, INPUT);

  // Timers
  Timer1.initialize(500); // Stepper velocidad media
  Timer1.attachInterrupt(stepISR);
  Timer3.initialize(100); // Limites cada 0.1ms
  Timer3.attachInterrupt(leerLimites_ISR);

  // Interrupts Encoders
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoder0B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder1B, CHANGE);

  MG995_Servo.attach(Servo_PWM);
  MG995_Servo.write(0);
  // MG995_Servo.detach();
  
  // Resetear contadores para que la posición actual sea 0
  encoder0Pos = 0;
  encoder1Pos = 0;

  setState(INICIO);
}

// =====================
// LOOP PRINCIPAL
// =====================
void loop() {
  leerSerialNoBloqueante();

  // Gestión de Estados
  switch (state) {
    case INICIO:
      ST.motor(1, 0); ST.motor(2, 0); // Motores relajados
      
      if (flag_detener) { setState(STOP); break; }
      
      if (digitalRead(boton_subir)) {
        pasos_restantes = 200; // Resetear pasos
        if (flag_fondo){
          setState(SUBIENDO);
        } else{
          setState(flag_limite_inferior ? SUBIENDO : BAJANDO);
        }  
      } 
      else if (digitalRead(boton_mover)) {
        setState(flag_limite_lateral ? STOP : MOVIENDOSE);
      }
      break;

    case BAJANDO:
      digitalWrite(DIR, LOW); // Dirección abajo
      enable_step = true;

      if (flag_detener){
        enable_step = false;
        setState(STOP);
      } else{
        if (flag_limite_inferior){
          flag_fondo = true;
          enable_step = false;
          setState(INICIO);
        }
      }
      break;

    case SUBIENDO:
      digitalWrite(DIR, HIGH); // Dirección arriba
      enable_step = true;
      if (flag_detener || flag_limite_lateral || pasos_restantes <= 0) {
        enable_step = false;

        if (pasos_restantes <=0){
          flag_fondo = false;
          setState(INICIO);
        }
        setState(pasos_restantes <= 0 ? INICIO : STOP);
      }
      break;

    case MOVIENDOSE:
      
      // MG995_Servo.attach(Servo_PWM);
      MG995_Servo.write(angulo_objetivo_servo);
      controlPID(); // Función separada para orden
      
      if (flag_detener || flag_limite_lateral) {
        setState(STOP);
      }
      break;

    case STOP:
      MG995_Servo.detach();
      ST.motor(1, 0);
      ST.motor(2, 0);
      enable_step = false;
      digitalWrite(bomba, LOW);
      flag_detener = false;
      delay(1000); // Pequeña pausa de seguridad
      setState(INICIO);
      break;
  }
}

// =====================
// FUNCIÓN PID MEJORADA
// =====================
void controlPID() {
  if (micros() - time_ant_pid < Period_PID) return; // Ejecutar solo cada 10ms
  
  time_ant_pid = micros();
  float dt = Period_PID / 1000000.0;

  // 1. Leer Posición REAL (Snapshot atómico)
  long pos0, pos1;
  noInterrupts();
  pos0 = encoder0Pos;
  pos1 = encoder1Pos;
  interrupts();

  // 2. Convertir Cuentas a Grados
  angulo_actual1 = (float)pos0 / COUNTS_PER_DEGREE;
  angulo_actual2 = (float)pos1 / COUNTS_PER_DEGREE;

  Serial.println("angulo actual1");
  Serial.println(angulo_actual1);
  Serial.println("angulo actua2");
  Serial.println(angulo_actual2);


  // 3. Cálculo PID Motor 1
  error_pid1 = angulo_objetivo1 - angulo_actual1;
  Serial.println("error pid1");
  Serial.println(error_pid1);
  integral_pid1 += error_pid1 * dt;
  // Anti-windup para integral
  integral_pid1 = constrain(integral_pid1, -50, 50); 
  
  float derivada1 = (error_pid1 - error_anterior1) / dt;
  pid_output1 = (kp1 * error_pid1) + (ki1 * integral_pid1) + (kd1 * derivada1);
  error_anterior1 = error_pid1;

  // 4. Cálculo PID Motor 2
  error_pid2 = angulo_objetivo2 - angulo_actual2;
  Serial.println("error pid2");
  Serial.println(error_pid2);
  integral_pid2 += error_pid2 * dt;
  integral_pid2 = constrain(integral_pid2, -50, 50);
  
  float derivada2 = (error_pid2 - error_anterior2) / dt;
  pid_output2 = (kp2 * error_pid2) + (ki2 * integral_pid2) + (kd2 * derivada2);
  error_anterior2 = error_pid2;

  if (abs(error_pid1) < 1.0) {
      pid_output1 = 0;
  } else {
      // Si estamos lejos, aseguramos que la potencia mínima sea 50
      if (pid_output1 > 0 && pid_output1 < 50)  pid_output1 = 50;  // Empujón mínimo positivo
      if (pid_output1 < 0 && pid_output1 > -50) pid_output1 = -50; // Empujón mínimo negativo
      // No necesitamos limitar hacia arriba aquí, el constrain final lo hará
  }

  // 5. Saturación para Sabertooth (-127 a 127)
  // Aumentamos el límite para que tenga fuerza de moverse 
  int out1 = constrain((int)pid_output1, -100, 100);
  int out2 = constrain((int)pid_output2, -80, 80);


  // Zona muerta (Deadband): si la potencia es muy baja, el motor solo zumba
  // Ayuda a evitar calentamiento cerca del objetivo
  if (abs(out1) < 5) out1 = 0;
  if (abs(out2) < 5) out2 = 0;

  Serial.println("pid output 1");
  Serial.println(out1);
  Serial.println("pid output 1");
  Serial.println(out2);
  // 6. Enviar a Motores
  ST.motor(1, out1);
  ST.motor(2, out2);

  // 7. Chequear llegada
  if (abs(error_pid1) < 1.0 && abs(error_pid2) < 1.0) {
    // Tolerancia de 1 grado
    ST.motor(1, 0);
    ST.motor(2, 0);
    // Opcional: setState(INICIO) si queremos terminar el movimiento
  }
}


void setState(State s) {
  Serial.println(s);
  Serial.println("INICIA PID");
  lastState = state;
  state = s;
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
  // ... (Tu misma lógica de parseo va aquí) ...
  // Solo asegúrate de actualizar angulo_objetivo1 y angulo_objetivo2
  // Ejemplo rápido:
  if (msg.indexOf("Mover") >= 0) {
     // Parsear valores y asignar a angulo_objetivo
     // angulo_objetivo1 = ...
     setState(MOVIENDOSE);
  }
}