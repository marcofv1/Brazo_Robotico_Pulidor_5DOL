#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN  150  // pulso para 0°
#define SERVO_MAX  600  // pulso para 180°
#define NUM_SERVOS 5
#define SERVO_OFFSET 8  // Usar canales 8 a 12 porque del 0 al 7 se lastimaron algunos pines.

void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  pwm.begin();
  pwm.setPWMFreq(50);

  delay(10);
  Serial.println("Listo. Enviar 5 ángulos separados por coma, ejemplo:");
  Serial.println("90,45,135,90,0");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int angulos[NUM_SERVOS];
    int index = 0;
    int lastIndex = 0;

    // Parsear ángulos separados por coma
    while (index < NUM_SERVOS) {
      int commaIndex = input.indexOf(',', lastIndex);
      String val;
      if (commaIndex == -1) {
        val = input.substring(lastIndex);
      } else {
        val = input.substring(lastIndex, commaIndex);
      }
      val.trim();
      angulos[index] = val.toInt();

      lastIndex = commaIndex + 1;
      if (commaIndex == -1) break;
      index++;
    }

    // Validar que se recibieron 5 ángulos
    if (index == NUM_SERVOS -1 || (index == NUM_SERVOS && lastIndex >= input.length())) {
      for (int i = 0; i < NUM_SERVOS; i++) {
        if (angulos[i] < 0) angulos[i] = 0;
        if (angulos[i] > 180) angulos[i] = 180;

        int pulso = map(angulos[i], 0, 180, SERVO_MIN, SERVO_MAX);
        pwm.setPWM(i + SERVO_OFFSET, 0, pulso);  // Ahora en canales 9 a 13
      }
      Serial.print("Moviendo servos a: ");
      for (int i = 0; i < NUM_SERVOS; i++) {
        Serial.print(angulos[i]);
        if (i < NUM_SERVOS -1) Serial.print(", ");
      }
      Serial.println(" grados.");
    } else {
      Serial.println("⚠️ Debes enviar 5 ángulos separados por coma.");
    }
  }
}
