#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

// Offsets
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

// Sensibilité
const float accelScale = 16384.0; // ±2g
const float gyroScale = 131.0;    // ±250°/s

void calibrateSensor() {
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  const int samples = 200;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax_sum += ax;
    ay_sum += ay;
    az_sum += (az - 16384); // Retirer la gravité
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    delay(5);
  }

  ax_offset = ax_sum / samples;
  ay_offset = ay_sum / samples;
  az_offset = az_sum / samples;
  gx_offset = gx_sum / samples;
  gy_offset = gy_sum / samples;
  gz_offset = gz_sum / samples;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA, SCL pour ESP32
  
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Erreur : MPU non détecté");
    while (1);
  }

  Serial.println("Calibrage automatique... ne touchez pas l'appareil.");
  delay(2000);
  calibrateSensor();
  Serial.println("Calibration terminée !");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Retirer les offsets
  ax -= ax_offset;
  ay -= ay_offset;
  az -= az_offset;
  gx -= gx_offset;
  gy -= gy_offset;
  gz -= gz_offset;

  // Convertir en unités physiques
  float ax_ms2 = (ax / accelScale) * 9.80665;
  float ay_ms2 = (ay / accelScale) * 9.80665;
  float az_ms2 = (az / accelScale) * 9.80665;

  float gx_dps = gx / gyroScale;
  float gy_dps = gy / gyroScale;
  float gz_dps = gz / gyroScale;

  // Filtrage simple (moyenne glissante)
  static float ax_f = ax_ms2, ay_f = ay_ms2, az_f = az_ms2;
  static float gx_f = gx_dps, gy_f = gy_dps, gz_f = gz_dps;

  ax_f = (ax_f + ax_ms2) / 2;
  ay_f = (ay_f + ay_ms2) / 2;
  az_f = (az_f + az_ms2) / 2;

  gx_f = (gx_f + gx_dps) / 2;
  gy_f = (gy_f + gy_dps) / 2;
  gz_f = (gz_f + gz_dps) / 2;

  // Affichage
  Serial.print("Acc (m/s²): ");
  Serial.print(ax_f); Serial.print(", ");
  Serial.print(ay_f); Serial.print(", ");
  Serial.print(az_f); Serial.print(" | ");

  Serial.print("Gyro (°/s): ");
  Serial.print(gx_f); Serial.print(", ");
  Serial.print(gy_f); Serial.print(", ");
  Serial.println(gz_f);

  delay(1000);
}
