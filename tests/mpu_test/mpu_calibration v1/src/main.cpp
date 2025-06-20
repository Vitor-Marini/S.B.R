#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Variáveis para armazenar os offsets de calibração
float accel_offset_x = 0;
float accel_offset_y = 0;
float accel_offset_z = 0;
float gyro_offset_x = 0;
float gyro_offset_y = 0;
float gyro_offset_z = 0;

const int numReadings = 1000; // Número de leituras para calcular a média

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Aguarda a porta serial abrir
  }

  Serial.println("Inicializando MPU6050...");

  // Tenta inicializar o MPU6050
  if (!mpu.begin()) {
    Serial.println("Falha ao encontrar chip MPU6050");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 encontrado!");

  // Configura a faixa do acelerômetro e giroscópio (opcional, mas recomendado)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // Escolha a faixa apropriada
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);     // Escolha a faixa apropriada
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);  // Define o filtro digital

  Serial.println("\nColoque o MPU em uma superficie PLANA e ESTAVEL.");
  Serial.println("A calibracao comeca em 5 segundos...");
  delay(5000);

  calibrateMPU();

  Serial.println("\nCalibracao concluida!");
  Serial.print("Offset Acelerometro X: "); Serial.println(accel_offset_x);
  Serial.print("Offset Acelerometro Y: "); Serial.println(accel_offset_y);
  Serial.print("Offset Acelerometro Z: "); Serial.println(accel_offset_z);
  Serial.print("Offset Giroscopio X: "); Serial.println(gyro_offset_x);
  Serial.print("Offset Giroscopio Y: "); Serial.println(gyro_offset_y);
  Serial.print("Offset Giroscopio Z: "); Serial.println(gyro_offset_z);

  Serial.println("\nIniciando leituras com offsets aplicados...");
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Aplica os offsets de calibração
  float calibrated_accel_x = a.acceleration.x - accel_offset_x;
  float calibrated_accel_y = a.acceleration.y - accel_offset_y;
  float calibrated_accel_z = a.acceleration.z - accel_offset_z; // Para Z, o offset deve ser subtraído de (leitura - 9.81) ou (leitura - 1g de sensibilidade)

  // O ideal é que a leitura de Z, quando parado e apontando para cima, seja +9.81 m/s^2.
  // Então, se seu MPU estiver "para cima" e você quiser que a leitura seja próxima de 0,0,0
  // Você precisará subtrair o valor de 1G (aproximadamente 9.81 m/s^2) da leitura do eixo Z,
  // ou ajustar o offset de Z para considerar isso.
  // Por exemplo, se a leitura do Z parado é 9.85, o offset seria 0.04 (9.85 - 9.81).
  // Para simplificar, neste exemplo o offset do Z é a leitura média, mas para uso real,
  // considere o valor da gravidade no eixo Z.

  float calibrated_gyro_x = g.gyro.x - gyro_offset_x;
  float calibrated_gyro_y = g.gyro.y - gyro_offset_y;
  float calibrated_gyro_z = g.gyro.z - gyro_offset_z;

  Serial.print("Accel (calib) X: "); Serial.print(calibrated_accel_x, 2);
  Serial.print(", Y: "); Serial.print(calibrated_accel_y, 2);
  Serial.print(", Z: "); Serial.print(calibrated_accel_z, 2);
  Serial.print(" m/s^2\t");

  Serial.print("Gyro (calib) X: "); Serial.print(calibrated_gyro_x, 2);
  Serial.print(", Y: "); Serial.print(calibrated_gyro_y, 2);
  Serial.print(", Z: "); Serial.print(calibrated_gyro_z, 2);
  Serial.println(" rad/s");

  delay(100);
}

void calibrateMPU() {
  Serial.println("Calibrando Giroscopio e Acelerometro...");
  Serial.println("Nao mova o sensor durante a calibracao!");

  long sum_accel_x = 0;
  long sum_accel_y = 0;
  long sum_accel_z = 0;
  long sum_gyro_x = 0;
  long sum_gyro_y = 0;
  long sum_gyro_z = 0;

  for (int i = 0; i < numReadings; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sum_accel_x += a.acceleration.x;
    sum_accel_y += a.acceleration.y;
    sum_accel_z += a.acceleration.z; // Note: For accelerometer Z, you'll ideally want to calibrate relative to 1G or -1G.
                                     // This simple average will give the offset from the *current* Z reading.
                                     // For proper accelerometer calibration, you'd position it in 6 ways.

    sum_gyro_x += g.gyro.x;
    sum_gyro_y += g.gyro.y;
    sum_gyro_z += g.gyro.z;

    delay(10); // Pequeno atraso entre as leituras
    if (i % 100 == 0) {
      Serial.print("."); // Indicador de progresso
    }
  }

  accel_offset_x = (float)sum_accel_x / numReadings;
  accel_offset_y = (float)sum_accel_y / numReadings;
  // Para o eixo Z do acelerômetro, se o MPU estiver virado para cima,
  // a leitura nominal de Z será próxima de 9.81 m/s^2 (1G).
  // Se você quiser que o valor calibrado seja 0 quando parado e para cima,
  // você subtrairia (media_Z - 9.81). No entanto, para simplicidade,
  // este código apenas calcula o offset médio da leitura bruta.
  // Para aplicações de AHRS, o tratamento do 1G no Z é mais complexo.
  accel_offset_z = (float)sum_accel_z / numReadings;

  gyro_offset_x = (float)sum_gyro_x / numReadings;
  gyro_offset_y = (float)sum_gyro_y / numReadings;
  gyro_offset_z = (float)sum_gyro_z / numReadings;

  Serial.println("\nCalibracao MPU finalizada. Offsets calculados.");
}