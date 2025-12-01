#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// --- CONFIGURACIÓN ---
// La dirección I2C suele ser 0x27 o 0x3F. Si no funciona con una, prueba la otra.
LiquidCrystal_I2C lcd(0x27, 16, 2);  

void setup() {
  // Iniciar comunicación serial (Asegúrate que coincida con tu código de Python)
  // Si usas 115200 en Python, cambia aquí a 115200
  Serial.begin(115200); 

  // Iniciar LCD
  lcd.init();
  lcd.backlight();
  
  // Mensaje de bienvenida
  lcd.setCursor(0,0);
  lcd.print("Sistema Listo");
  lcd.setCursor(0,1);
  lcd.print("Esperando RX...");
  
  delay(2000);
  lcd.clear();
}

void loop() {
  // Verificar si hay datos entrando por el puerto Serial
  if (Serial.available() > 0) {
    
    // Leemos todo el mensaje hasta que encuentre un salto de línea (\n)
    // Esto es ideal porque 'Serial.println()' o tu código Python envían \n al final
    String mensaje = Serial.readStringUntil('\n');
    
    // Limpiamos espacios en blanco o caracteres basura al inicio/final
    mensaje.trim(); 

    // Solo si el mensaje no está vacío, lo mostramos
    if (mensaje.length() > 0) {
      lcd.clear();
      
      // Imprimir etiqueta en fila 0
      lcd.setCursor(0,0); 
      lcd.print("RX Datos:");
      
      // Imprimir el mensaje recibido en fila 1
      lcd.setCursor(0,1);
      
      // La LCD solo muestra 16 caracteres, cortamos si es muy largo para que no se vea raro
      if(mensaje.length() > 16){
        lcd.print(mensaje.substring(0, 16));
      } else {
        lcd.print(mensaje);
      }
      
      // Debug en el puerto serial (para ver en PC si llegó bien)
      Serial.print("Recibido y mostrado: ");
      Serial.println(mensaje);
    }
  }
}
