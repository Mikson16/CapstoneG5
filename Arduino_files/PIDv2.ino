///// LIBRERIAS ////////////////// ORDEN;M1:XX;M2:XX;S:XX
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>
#include <TimerThree.h>
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

// Helper to change state and update LCD only on change
State lastState = INICIO;
void setState(State s);

// ===== PINES =====
// Servo
const int Servo_PWM = 22;

// Stepper
const int DIR = 24;
const int PUL = 26;

// Bomba
const int bomba = 30;

// Botones
const int boton_mover = 53;
const int boton_subir = 51;

// Encoders (Mega interrupt-capable pins)
#define encoder0PinA 19   // INT4
#define encoder0PinB 18   // INT5
#define encoder1PinA 2    // INT0
#define encoder1PinB 3    // INT1

// Limit switches
const int limite_inferior = 23;
const int limite_lateral = 25;

// Hall sensor (analog)
const int sensor_hall = A15;

// ===== VARIABLES =====
// Comunicacion
String buffer = "";
String orden = "";

// Control PID (floats)
const float kp1 = 0.05f;
const float ki1 = 0.0f;
const float kd1 = 0.003f;

const float kp2 = 0.05f;
const float ki2 = 0.0f;
const float kd2 = 0.003f;

// Estado angular
float angulo_actual1 = 0.0f;
float angulo_actual2 = 0.0f;

float angulo_bolsa = 0.0f;
float angulo_objetivo1 = 0.0f;
float angulo_objetivo2 = 0.0f;

// PID interno
float error_pid1 = 0;
float error_anterior1 = 0;
float integral_pid1 = 0;
float derivada_pid1 = 0;

float error_pid2 = 0;
float error_anterior2 = 0;
float integral_pid2 = 0;
float derivada_pid2 = 0;

// PID salida
float pid_output1 = 0;
float pid_output2 = 0;

// Movimiento
volatile bool mov_z = false;
volatile bool mov = false;
volatile long pasos_restantes = 1000;
volatile bool enable_step = false;    // unificado nombre
volatile bool step_pin_state = false; // controla el PUL toggle

// Limites (actualizados por ISR cada 1 ms)
volatile bool flag_limite_inferior = false;
volatile bool flag_limite_lateral  = false;

// Sensor Hall
int valor_sensor_hall = 0;

// Tiempo
unsigned long time_ant = 0;
const unsigned long Period = 10000UL;     // 10000 us = 10 ms
const float dt = Period * 1e-6f;          // 0.01 s

// Encoder Pololu 4752 (datos)
const float CPR_MOTOR = 256.0f;
const float GEAR_RATIO = 131.0f;
const float CPR_OUTPUT = 1920.0f;

// Variables encoder
volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;

long newposition0 = 0;
long newposition1 = 0;
long oldposition0 = 0;
long oldposition1 = 0;

float vel1_rpm = 0;
float vel2_rpm = 0;

// FLAG para detener enviado por la Raspi
volatile bool flag_detener = false; // puede no necesitar volatile, pero por seguridad

// COMPONENTES
Sabertooth ST(128, Serial2); // Sabertooth en Serial2
Servo MG995_Servo;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// -----------------------
// INTERRUPCIONES ENC
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

void stepISR() {
  if (!enable_step) {
    // Asegurar que la salida quede en LOW
    digitalWrite(PUL, LOW);
    step_pin_state = false;
    return;
  }

  // Duty cycle 50% (toggle)
  step_pin_state = !step_pin_state;
  digitalWrite(PUL, step_pin_state ? HIGH : LOW);

  // Contar solo en flanco ascendente (cuando pulse pasa a HIGH)
  if (step_pin_state) {
    if (pasos_restantes > 0) pasos_restantes--;
  }
}

void leerLimites_ISR() {
  flag_limite_inferior = (digitalRead(limite_inferior) == HIGH);
  flag_limite_lateral  = (digitalRead(limite_lateral) == HIGH);
}
// =====================
// SETUP
void setup() {
  // COMMS
  Serial.begin(115200);    // puerto Serial0 con Raspberry Pi: NO imprimir por aquí si lo deseas
  Serial2.begin(9600);     // Sabertooth

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Pines encoders
  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);

  // Limit switches (asume pulled externamente; reads HIGH when pressed)
  pinMode(limite_inferior, INPUT);
  pinMode(limite_lateral, INPUT);

  // Stepper, Servo, Bomba
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(Servo_PWM, OUTPUT);
  pinMode(bomba, OUTPUT);

  // Botones (tú indicaste 5V al pulsar)
  pinMode(boton_subir, INPUT);
  pinMode(boton_mover, INPUT);

  // Attach interrupts timers (TimerOne & TimerTwo)
  // TimerOne: usada para el stepper (frecuencia alta)
  Timer1.initialize(500); // 500 us period (0.5 ms). Ajusta según necesites.
  Timer1.attachInterrupt(stepISR);

  // TimerTwo: para muestreo de límites (ej. 1 ms)
  Timer3.initialize(1000); // 1000 us = 1 ms
  Timer3.attachInterrupt(leerLimites_ISR);

  // Attach hardware interrupts para encoders
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoder0B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder1B, CHANGE);

  // Servo attach
  MG995_Servo.attach(Servo_PWM);

  // Inicial
  lastState = state;
  setState(INICIO);
  time_ant = micros();
}

// =====================
// LOOP
void loop() {
  // 1) Recepción serial desde Raspberry (Serial0)
  leerSerialNoBloqueante();   // SIEMPRE ACTIVO

  // 2) Lógica de estados
  switch (state) {

    case INICIO: {
      // Leer botones (no usamos Serial prints)
      mov_z = (digitalRead(boton_subir) == HIGH);
      mov   = (digitalRead(boton_mover) == HIGH);

      // Si llegó comando detener, pasar a STOP
      if (flag_detener) {
        setState(STOP);
        break;
      }

      // Si se solicita subir/bajar
      if (mov_z) {
        // Usar flags actualizados por ISR (no volver a digitalRead aquí)
        if (flag_limite_inferior) {
          setState(SUBIENDO);
        } else {
          setState(BAJANDO);
        }
      } else if (mov) {
        if (flag_limite_lateral) {
          setState(STOP);
        } else {
          setState(MOVIENDOSE);
        }
      }
      break;
    }

    case BAJANDO: {
      // Mover stepper hacia abajo
      digitalWrite(DIR, LOW);
      enable_step = true;

      // Si llegó comando detener -> para y STOP
      if (flag_detener) {
        setState(STOP);
        break;
      }

      // Si el límite inferior ya está activo -> detener
      if (flag_limite_inferior) {
        enable_step = false;
        setState(INICIO);
        break;
      }

      break;
    }

    case SUBIENDO: {
      digitalWrite(DIR, HIGH);
      enable_step = true;
      // Stop por comando rasp
      if (flag_detener) {
        setState(STOP);
        break;
      }

      if (pasos_restantes <= 0) {
        pasos_restantes = 1000;
        enable_step = false;
        setState(INICIO);
        break;
      }
      if (flag_limite_lateral) {
        enable_step = false;
        setState(STOP);
        break;
      }

      break;
    }

    case MOVIENDOSE: {
      // Control PID ejecutado cada Period (10 ms)
      if ((micros() - time_ant) >= Period) {

        // Stop por comando rasp
        if (flag_detener) {
          setState(STOP);
          break;
        }

        // Comprueba límites (ISR mantiene flags)
        if (flag_limite_lateral) {
          setState(STOP);
          break;
        }

        // Actualizar posiciones leídas de los encoders (volatile -> leer una vez)
        noInterrupts();
        newposition0 = encoder0Pos;
        newposition1 = encoder1Pos;
        interrupts();

        // Calcular RPM según tu fórmula
        vel1_rpm = (newposition0 - oldposition0) * (60.0f / (CPR_OUTPUT * dt));
        vel2_rpm = (newposition1 - oldposition1) * (60.0f / (CPR_OUTPUT * dt));

        oldposition0 = newposition0;
        oldposition1 = newposition1;

        // RPM -> rad/s
        float vel1_rad_s = vel1_rpm * 2.0f * PI / 60.0f;
        float vel2_rad_s = vel2_rpm * 2.0f * PI / 60.0f;

        // Integración para ángulo (simple integrado desde velocidad medida)
        angulo_actual1 += vel1_rad_s * dt;
        angulo_actual2 += vel2_rad_s * dt;

        // PID para motor 1
        error_pid1 = angulo_objetivo1 - angulo_actual1;
        integral_pid1 += error_pid1 * dt;
        derivada_pid1 = (error_pid1 - error_anterior1) / dt;
        error_anterior1 = error_pid1;
        pid_output1 = kp1 * error_pid1 + ki1 * integral_pid1 + kd1 * derivada_pid1;

        // PID para motor 2
        error_pid2 = angulo_objetivo2 - angulo_actual2;
        integral_pid2 += error_pid2 * dt;
        derivada_pid2 = (error_pid2 - error_anterior2) / dt;
        error_anterior2 = error_pid2;
        pid_output2 = kp2 * error_pid2 + ki2 * integral_pid2 + kd2 * derivada_pid2;

        // Saturación en rango Sabertooth convencional -127..127 (tú usas +/-40 limite)
        const float LIM = 40.0f;
        if (pid_output1 > LIM) pid_output1 = LIM;
        if (pid_output1 < -LIM) pid_output1 = -LIM;
        if (pid_output2 > LIM) pid_output2 = LIM;
        if (pid_output2 < -LIM) pid_output2 = -LIM;

        // Enviar comandos (cast a int)
        ST.motor(1, (int)round(pid_output1));
        ST.motor(2, (int)round(pid_output2));

        // Evaluar llegada a metas (ambos ejes dentro de tolerancia)
        bool fin1 = (fabs(error_pid1) < 0.02f); // ~1 grado
        bool fin2 = (fabs(error_pid2) < 0.02f);

        time_ant = micros(); // actualizar timestamp aquí

        if (fin1 && fin2) {
          // detener motores
          ST.motor(1, 0);
          ST.motor(2, 0);

          // reset estados PID/ángulos si quieres considerar "home" al llegar
          angulo_actual1 = 0.0f;
          angulo_actual2 = 0.0f;
          integral_pid1 = 0.0f;
          integral_pid2 = 0.0f;
          error_anterior1 = 0.0f;
          error_anterior2 = 0.0f;

          setState(INICIO);
          break;
        }
      }
      break;
    }

    case STOP: {
      // Parar motores inmediatamente
      ST.motor(1, 0);
      ST.motor(2, 0);

      // Mantener bomba off por seguridad
      digitalWrite(bomba, LOW);

      // Reiniciar banderas de movimiento
      mov = false;
      mov_z = false;

      flag_detener = false;

      // Volver a INICIO
      setState(INICIO);
      break;
    }

    default:
      setState(INICIO);
      break;
  } // switch

} // loop

void setState(State s) {
  if (s == lastState) {
    state = s;
    return;
  }
  state = s;
  lastState = s;

  lcd.clear();
  switch (s) {
    case INICIO:
      lcd.setCursor(0, 0); lcd.print("INICIO");
      lcd.setCursor(0, 1); lcd.print("ESPERANDO");
      break;
    case BAJANDO:
      lcd.setCursor(0, 0); lcd.print("BAJANDO");
      break;
    case SUBIENDO:
      lcd.setCursor(0, 0); lcd.print("SUBIENDO");
      break;
    case MOVIENDOSE:
      lcd.setCursor(0, 0); lcd.print("MOVIENDOSE");
      break;
    case STOP:
      lcd.setCursor(0, 0); lcd.print("STOP");
      break;
  }
}

// Extrae campo "M1:", "M2:", "S:"
String extraerCampo(String msg, String etiqueta) {
  int start = msg.indexOf(etiqueta);
  if (start < 0) return "0";
  start += etiqueta.length();
  int end = msg.indexOf(';', start);
  if (end < 0) end = msg.length();
  return msg.substring(start, end);
}

// Parseo y acciones según mensaje recibido
void procesarMensaje(String msg) {
  msg.trim();
  if (msg.length() == 0) return;

  // Estructura: ORDEN;M1:XX;M2:XX;S:XX;
  int p1 = msg.indexOf(';');
  if (p1 < 0) return;

  orden = msg.substring(0, p1);

  int m1_start = msg.indexOf("M1:");
  int m2_start = msg.indexOf("M2:");
  int s_start  = msg.indexOf("S:");

  if (m1_start < 0 || m2_start < 0 || s_start < 0) {
    // Campo mal formado: ignorar
    return;
  }

  String strM1 = extraerCampo(msg, "M1:");
  String strM2 = extraerCampo(msg, "M2:");
  String strS  = extraerCampo(msg, "S:");

  // Convertir a float (si la conversión falla, toFloat devuelve 0.0)
  angulo_objetivo1 = strM1.toFloat();
  angulo_objetivo2 = strM2.toFloat();
  angulo_bolsa     = strS.toFloat();

  // Gestión de ordenes
  if (orden.equalsIgnoreCase("detenerse")) {
    flag_detener = true;
    // Parada inmediata por seguridad
    ST.motor(1, 0);
    ST.motor(2, 0);
  } else if (orden.equalsIgnoreCase("Mover") || orden.equalsIgnoreCase("mover")) {
    flag_detener = false;
    // Cambiar a MOVIENDOSE si no hay límites
    if (!flag_limite_lateral) {
      setState(MOVIENDOSE);
    } else {
      setState(STOP);
    }
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
