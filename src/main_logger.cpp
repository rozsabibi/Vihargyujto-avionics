#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>

#define SDA_PIN   21
#define SCL_PIN   22
#define USE_SIM   0     // 0 = valós szenzor, 1 = szoftveres szimulátor (0..300 m fel/le)
#define PRINT_HEADER 1

static const float P0_hPa = 1013.25f;
static const float EMA_A  = 0.10f;

Adafruit_BMP280 bmp;
Adafruit_BME280 bme;

float h_filt = 0.0f, h_prev = 0.0f;
unsigned long t_prev = 0;
bool firstSample = true;

#if USE_SIM
// Egyszerű szimulátor: ~8 s alatt 0→300 m, aztán vissza. ~20 Hz.
float sim_t = 0.0f;
static void sim_read(float &p_Pa, float &T_C){
  float h = (sim_t < 8.0f) ? (300.0f * (sim_t/8.0f))
                           : (300.0f * max(0.0f, 2.0f - (sim_t/8.0f)));
  h += 0.5f * sinf(sim_t * 3.7f);                  // kis zaj
  p_Pa = (P0_hPa*100.0f) * expf(-h/8434.5f);       // Pa
  T_C  = 22.0f + 0.2f * sinf(sim_t * 1.1f);
  sim_t += 0.05f; // 20 Hz
}
#endif

void setup(){
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

#if !USE_SIM
  bool ok = bmp.begin(0x76) || bmp.begin(0x77);
  if(!ok){
    Serial.println("# BMP280 nem megy, próbálom BME280-ként...");
    ok = bme.begin(0x76) || bme.begin(0x77);
  }
  if(!ok){
    Serial.println("# Nincs BMP/BME az I2C buszon, állok.");
    while(1) delay(1000);
  }
#endif

  if(PRINT_HEADER){
    Serial.println("t_ms,p_Pa,T_C,alt_m_filt,vz_mps");
  }
  t_prev = millis();
}

void loop(){
  float p_Pa = 0.0f;
  float T_C  = 0.0f;

#if USE_SIM
  sim_read(p_Pa, T_C);
#else
  if (bmp.sensorID() == 0x58) {
    p_Pa = bmp.readPressure();
    T_C  = bmp.readTemperature();
  } else {
    p_Pa = bme.readPressure();
    T_C  = bme.readTemperature();
  }
#endif

  float p_hPa = p_Pa / 100.0f;
  float h     = 8434.5f * logf(P0_hPa / p_hPa);

  if(firstSample){
    h_filt = h;
    h_prev = h_filt;
    firstSample = false;
    delay(50);
    return;
  }

  h_filt = EMA_A*h + (1.0f-EMA_A)*h_filt;

  unsigned long t = millis();
  float dt = (t - t_prev) / 1000.0f; if(dt <= 0) dt = 1e-3f;
  float vz = (h_filt - h_prev) / dt;

  Serial.printf("%lu,%.0f,%.2f,%.2f,%.2f\n",
                (unsigned long)t, p_Pa, T_C, h_filt, vz);

  h_prev = h_filt; t_prev = t;
  delay(50); // ~20 Hz
}
