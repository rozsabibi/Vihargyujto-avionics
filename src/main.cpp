#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>

#define PIN_BUZZ 16
#define SDA_PIN  21
#define SCL_PIN  22

static const uint16_t BUZZ_ON_MS_MAX   = 100; 
static const uint16_t BUZZ_COOLDOWN_MS = 600;

void buzz_on()  { digitalWrite(PIN_BUZZ, LOW);  }
void buzz_off() { digitalWrite(PIN_BUZZ, HIGH); }

void beep_ms(uint16_t ms = 120) {
  if (ms > BUZZ_ON_MS_MAX) ms = BUZZ_ON_MS_MAX;
  buzz_on();
  delay(ms);
  buzz_off();
  delay(BUZZ_COOLDOWN_MS);
}


Adafruit_BMP280 bmp;
Adafruit_BME280 bme;

void setup() {
  pinMode(PIN_BUZZ, OUTPUT);
  buzz_off();
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  beep_ms(80);

  Serial.println("I2C scan:");
  for (uint8_t a=3; a<0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission()==0){ 
      Serial.printf("  Found: 0x%02X\n", a);
    }
  }

 
  bool ok = bmp.begin(0x76) || bmp.begin(0x77);
  if (!ok) {
    Serial.println("BMP280 nem reagál, próbálom BME280-ként...");
    ok = bme.begin(0x76) || bme.begin(0x77);
  }
  if (!ok) {
    Serial.println("Nincs BMP/BME a buszon :(");
    while(1){ beep_ms(70); }
  }
  beep_ms(90);
}

void loop() {
  if (bmp.sensorID() == 0x58) {
    float p = bmp.readPressure();
    float t = bmp.readTemperature();
    Serial.printf("BMP280  p=%.0f Pa  T=%.2f C\n", p, t);
  } else {
    float p = bme.readPressure();
    float t = bme.readTemperature();
    float h = bme.readHumidity();
    Serial.printf("BME280  p=%.0f Pa  T=%.2f C  RH=%.1f%%\n", p, t, h);
  }
  delay(200);
}
