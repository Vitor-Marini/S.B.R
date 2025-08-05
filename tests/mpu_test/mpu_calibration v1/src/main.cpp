#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


void calibrarSensor();
void mostrarOffsets();


Adafruit_MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

long ax_offset = 0, ay_offset = 0, az_offset = 0;
long gx_offset = 0, gy_offset = 0, gz_offset = 0;

bool calibrated = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println(" Falha na conexão com o MPU6050.");
    while (1);
  }

  Serial.println(" Conexão com MPU6050 estabelecida.");
  Serial.println("Digite:");
  Serial.println("  y  → para calibrar");
  Serial.println("  o  → para mostrar os offsets");
  Serial.println("  r  → para recalibrar");
  Serial.println("Aguardando comando...");
}

void loop() {
  if (Serial.available()) {
    char comando = Serial.read();

    switch (comando) {
      case 'y':
      case 'Y':
      case 'r':
      case 'R':
        calibrarSensor();
        break;

      case 'o':
      case 'O':
        mostrarOffsets();
        break;

      default:
        Serial.println("Comando inválido. Use:");
        Serial.println("  y  → calibrar");
        Serial.println("  o  → mostrar offsets");
        Serial.println("  r  → recalibrar");
        break;
    }
  }
}

void calibrarSensor() {
  Serial.println(" Iniciando calibração. Mantenha o sensor PARADO e NIVELADO...");
  delay(2000);

  ax_offset = ay_offset = az_offset = 0;
  gx_offset = gy_offset = gz_offset = 0;

  for (int i = 0; i < 1000; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_offset += ax;
    ay_offset += ay;
    az_offset += az - 16384;  // gravidade
    gx_offset += gx;
    gy_offset += gy;
    gz_offset += gz;
    delay(2);
  }

  ax_offset /= 1000;
  ay_offset /= 1000;
  az_offset /= 1000;
  gx_offset /= 1000;
  gy_offset /= 1000;
  gz_offset /= 1000;

  calibrated = true;

  Serial.println(" Calibração concluída.");
  mostrarOffsets();
}

void mostrarOffsets() {
  if (!calibrated) {
    Serial.println(" Sensor ainda não calibrado.");
  }
  Serial.println(" Offsets atuais:");
  Serial.print("  Accel: ");
  Serial.print(ax_offset); Serial.print(", ");
  Serial.print(ay_offset); Serial.print(", ");
  Serial.println(az_offset);

  Serial.print("  Gyro : ");
  Serial.print(gx_offset); Serial.print(", ");
  Serial.print(gy_offset); Serial.print(", ");
  Serial.println(gz_offset);
}
