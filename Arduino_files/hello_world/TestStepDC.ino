#include <TimerOne.h>
// #include <Servo.h> 
#include <Sabertooth.h>



unsigned long tiempoAnterior = 0;
const unsigned long intervalo = 500;  

const int boton_subir = 2;
const int boton_bajar = 3;
const int boton_rotar1 = 4;
const int boton_rotar2 = 5;

// const int PUL = 7;
// const int DIR = 8;
// const int EN = 9;
const int EN = 7;
const int DIR = 8;
const int PUL = 9;

int subir = 0;  
int bajar = 0;
int rotar1 = 0;
int rotar2 = 0;
// int pulsado_subir = 0;
// int pulsado_bajar = 0;

Sabertooth ST(128); 
// Servo MG995_Servo; 
// #define Servo_PWM 6


void setup() {
  Serial.begin(9600);  
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(boton_subir, INPUT);
  pinMode(boton_bajar, INPUT);
  pinMode(boton_rotar1, INPUT);
  pinMode(boton_rotar2, INPUT);
  Serial.println("Sistema iniciado. Esperando pulsaciones...");
  SabertoothTXPinSerial.begin(9600); // 9600 is the default baud rate for Sabertooth packet serial.
  ST.autobaud();
  digitalWrite(EN, HIGH);
  // MG995_Servo.attach(Servo_PWM); 
  // MG995_Servo.detach();
}

void loop() {
  unsigned long tiempoActual = millis();   // Tiempo actual en milisegundos

  // Si han pasado 500 ms desde la última vez
  if (tiempoActual - tiempoAnterior >= intervalo) {
    tiempoAnterior = tiempoActual;

    rotar1 = digitalRead(boton_rotar1);
    rotar2 = digitalRead(boton_rotar2); 

    if (rotar1 == HIGH){
      ST.motor(2, 50);
      Serial.println("rotar mov 1");
      delay(200);
      ST.motor(2, 0); 
      delay(200);
    }

    if (rotar2 == HIGH){
      ST.motor(2, -50);
      Serial.println("rotar mov 1");
      delay(200);
      ST.motor(2, 0); 
      delay(200);
    }

    subir = digitalRead(boton_subir);
    bajar = digitalRead(boton_bajar);


    if (subir == HIGH){
      for (int i = 0; i < 40; i++){
        digitalWrite(DIR, LOW);
        Serial.println("Avanzando PASO");
        digitalWrite(PUL, HIGH);
        delayMicroseconds(500);
        digitalWrite(PUL, LOW);
        delayMicroseconds(500);
      }
      
    }

    if (bajar == HIGH){
      for (int i = 0; i < 40; i++){
        digitalWrite(DIR, HIGH);
        Serial.println("Bajar PASO");
        digitalWrite(PUL, HIGH);
        delayMicroseconds(500);
        digitalWrite(PUL, LOW);
        delayMicroseconds(500);
      }
    }
    
  }
}

// void miISR_Stepper(){
//   subir = digitalRead(boton_subir);
//   bajar = digitalRead(boton_bajar);


//   if (subir == HIGH){
//     digitalWrite(DIR, LOW);
//       Serial.println("Avanzando PASO");
//       digitalWrite(PUL, HIGH);
//       delayMicroseconds(500);
//       digitalWrite(PUL, LOW);
//       delayMicroseconds(500);
//   }

//   if (bajar == HIGH){
//     digitalWrite(DIR, HIGH);
//       Serial.println("Bajar PASO");
//       digitalWrite(PUL, HIGH);
//       delayMicroseconds(500);
//       digitalWrite(PUL, LOW);
//       delayMicroseconds(500);
//   }
// }