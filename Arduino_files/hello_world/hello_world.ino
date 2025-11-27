

//#include <Sabertooth.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Inicio aa");
  
  

}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0){
    String line = Serial.readStringUntil('\n');
    line.trim();
    Serial.print("Recibido: ");
    Serial.println(line);
    };
}
