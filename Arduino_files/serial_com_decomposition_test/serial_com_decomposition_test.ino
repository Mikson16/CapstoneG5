#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// --- CONFIGURACIÓN ---
// Dirección I2C: 0x27 o 0x3F suelen ser las más comunes.
LiquidCrystal_I2C lcd(0x27, 16, 2);  

// Variables Globales para almacenar los ángulos
float valM1 = 0.0;
float valM2 = 0.0;
float valServo = 0.0;

void setup() {
  // Iniciar comunicación serial a 115200 (Coincide con tu Python)
  Serial.begin(115200); 

  // Iniciar LCD
  lcd.init();
  lcd.backlight();
  
  // Mensaje de bienvenida
  lcd.setCursor(0,0);
  lcd.print("Sistema Listo");
  lcd.setCursor(0,1);
  lcd.print("Esperando RX...");
  
  delay(1500);
  lcd.clear();
  lcd.print("Esperando CMD...");
}

// --- FUNCIÓN PARA MOSTRAR DATOS (SCROLL PAGINADO) ---
void mostrarScrollLCD(float m1, float m2, float s) {
  // Esta función bloquea el loop brevemente para mostrar la secuencia
  // Pagina 1: M1
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("DATO RECIBIDO:");
  lcd.setCursor(0,1);
  lcd.print("M1: ");
  lcd.print(m1);
  delay(1500); // Muestra por 1.5 segundos

  // Pagina 2: M2
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("DATO RECIBIDO:");
  lcd.setCursor(0,1);
  lcd.print("M2: ");
  lcd.print(m2);
  delay(1500); // Muestra por 1.5 segundos

  // Pagina 3: Servo
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("DATO RECIBIDO:");
  lcd.setCursor(0,1);
  lcd.print("Servo: ");
  lcd.print(s);
  delay(1500); // Muestra por 1.5 segundos

  // Volver a estado de espera
  lcd.clear();
  lcd.print("Esperando CMD...");
}

// --- TU LÓGICA DE PARSEO ---
void procesarMensaje(String msg) {
  // Formato esperado: "Mover;M1:XX;M2:XX;S:XX;bit1;bit2"
  
  // Validamos estructura básica
  if (msg.startsWith("Mover;") && msg.indexOf("M1:") > 0) {
    
    // Variables temporales
    float tempM1 = 0.0;
    float tempM2 = 0.0;
    float tempS = 0.0;

    // --- 1. Extracción de Magnitud M1 ---
    int startM1 = msg.indexOf("M1:") + 3; 
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
    int startS = msg.indexOf("S:", endM2) + 2; 
    int endS = msg.indexOf(";", startS);
    
    if (endS != -1) {
      String strS = msg.substring(startS, endS);
      tempS = strS.toFloat();
    
      // --- 4. Extracción Bit Signo M1 ---
      int startBit1 = endS + 1; 
      int endBit1 = msg.indexOf(";", startBit1);
      
      if (endBit1 != -1) {
         int signM1 = msg.substring(startBit1, endBit1).toInt();
         if (signM1 == 1) {
            tempM1 = tempM1 * -1.0;
         }
      }

      // --- 5. Extracción Bit Signo M2 ---
      int startBit2 = endBit1 + 1;
      if (startBit2 > 0 && startBit2 < msg.length()) {
          int signM2 = msg.substring(startBit2).toInt();
          if (signM2 == 1) {
             tempM2 = tempM2 * -1.0;
          }
      }
      
      // --- FINALIZAR: Actualizar globales y mostrar en LCD ---
      valM1 = tempM1;
      valM2 = tempM2;
      valServo = tempS;
      
      // Llamamos a la función visual
      mostrarScrollLCD(valM1, valM2, valServo);
    }
  } else if (msg.indexOf("Stop") >= 0) {
    lcd.clear();
    lcd.setCursor(0,0); 
    lcd.print("DATO RECIBIDO:");
    lcd.setCursor(0,1);
    lcd.print("STOP ");
    delay(2000); // Muestra por 1.5 segundos
  } else {
    // Si el mensaje no es válido (ej. ruido o formato incorrecto)
    lcd.clear();
    lcd.print("Error Formato");
    delay(1000);
    lcd.clear();
    lcd.print("Esperando CMD...");
  }
}

void loop() {
  // Verificar si hay datos entrando por el puerto Serial
  if (Serial.available() > 0) {
    
    // Leemos todo el mensaje hasta el salto de línea
    String mensaje = Serial.readStringUntil('\n');
    mensaje.trim(); // Limpiamos espacios

    if (mensaje.length() > 0) {
      // Enviamos el string crudo a procesar
      procesarMensaje(mensaje);
      
      // Debug opcional en Serial Monitor
      Serial.print("Procesado: ");
      Serial.println(mensaje);
    }
  }
}
