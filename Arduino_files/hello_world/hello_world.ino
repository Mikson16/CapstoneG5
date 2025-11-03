
//#include <Sabertooth.h>
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

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
