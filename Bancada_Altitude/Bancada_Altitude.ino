#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "SparkFunMPL3115A2.h"
#include <LittleFS.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>

// ============ CONFIGURAÇÃO WIFI =============
const char *ssid = "A15Jonas";
const char *password = "cqjd9qchmn9wdua";

// ============ SENSORES E VARIÁVEIS GLOBAIS =============
MPL3115A2 myPressure;
Adafruit_BNO055 myIMU = Adafruit_BNO055(55, 0x29); // Ajuste o endereço se necessário

float roll_offset = 0, pitch_offset = 0, yaw_offset = 0;
float accel_offset = 0, altitude_offset = 0;
float z_filtered = 0, z_filtered_previous = 0, z_accel =0, z_fusion = 0;
float current_raw_altitude_cm = 0, distance_ultra = 0, average_ultra = 0;
const float alpha = 0.3;

float roll = 0, pitch = 0, yaw = 0;
//float raw_roll = 0, raw_pitch = 0, raw_yaw = 0;
float roll_filtered = 0, pitch_filtered = 0, yaw_filtered = 0;
const float imu_alpha = 0.2;

sensors_event_t orientationData, linearAccelData;
bool imu_calibrated = false;

float setpointRoll = 0, setpointPitch = 0, setpointYaw = 0, setpointZ = 0;

Servo ESC1, ESC2, ESC3, ESC4; 
int pwmValue = 0;
const int pwmPin1 = 4;
const int pwmPin2 = 16;
const int pwmPin3 = 17;
const int pwmPin4 = 18;
volatile int setup_motor_flag = 0; // 1 = armado, 0 = desarmado

#define p_trigPin_  26
#define p_echoPin_  25

//-#MY AVERAGE
#define n_my_data 20
float my_data[n_my_data];
int i_my_data = 0;

int baro_counter = 0;
int ultra_counter = 0;

const float D_FILTER_ALPHA = 0.6f;

AsyncWebServer server(80);

// ============ PID =============
struct PIDGains {
  float kp, ki, kd;
  float prevError;
  float prevIterm;
  float prevFilteredDerivative;
};

PIDGains pidRoll  = { 2.35, 1.8, 0.4, 0, 0, 0.0f }; //ga deu 9.9833 0.0023 1.003, instavel
PIDGains pidPitch = { 2.35, 1.8, 0.4, 0, 0, 0.0f }; 
PIDGains pidYaw   = { 1.7, 0.85, 1.0, 0, 0, 0.0f };
PIDGains pidZ     = { 5.0, 2.0, 1.0, 0, 0, 0.0f };
//PIDGains pidZ     = { 9.99, 4.218, 3.549, 0, 0, 0.0f }; //Valor obtido pelo algoritmo genetico, deu instavel

float computePID(float error, PIDGains &gains, float dt = 0.01) {

  float pTerm = gains.kp * error;

  gains.prevIterm += gains.ki * (error + gains.prevError) * dt / 2.0;
  gains.prevIterm = constrain(gains.prevIterm, -500, 500);
  float iTerm = gains.prevIterm;

  //float dTerm = (dt > 0) ? gains.kd * (error - gains.prevError) / dt : 0;
  float rawDerivative = 0.0f;
  if (dt > 0.00001f) { // Evita divisão por zero ou dt muito pequeno
    rawDerivative = (error - gains.prevError) / dt;
  }
  float filteredDerivative = D_FILTER_ALPHA * rawDerivative + (1.0f - D_FILTER_ALPHA) * gains.prevFilteredDerivative;
  float dTerm = gains.kd * filteredDerivative;
  gains.prevFilteredDerivative = filteredDerivative;

  float output = pTerm + iTerm + dTerm;
  gains.prevError = error;
  return output;
}

void zerarPID() {

  pidRoll.prevError = 0.0f;
  pidRoll.prevIterm = 0.0f;
  pidRoll.prevFilteredDerivative = 0.0f;

  pidPitch.prevError = 0.0f;
  pidPitch.prevIterm = 0.0f;
  pidPitch.prevFilteredDerivative = 0.0f;

  pidYaw.prevError = 0.0f;
  pidYaw.prevIterm = 0.0f;
  pidYaw.prevFilteredDerivative = 0.0f;

  pidZ.prevError = 0.0f;
  pidZ.prevIterm = 0.0f;
  pidZ.prevFilteredDerivative = 0.0f;
}


float my_moving_average(float in_data, int debug){

  float my_ave;
  float my_sum;

  my_data[i_my_data] = in_data;

  if (my_data[n_my_data - 1] == 0)
  {
    my_sum = 0;

    for (int i = 0; i <= i_my_data; i++)
    {
      my_sum += my_data[i];
    }
    my_ave = my_sum / (i_my_data + 1);
  }
  else
  {
    my_sum = 0;
    for (int i = 0; i < n_my_data; i++)
    {
      my_sum += my_data[i];
    }
    my_ave = my_sum / n_my_data;
  }


  //#Debug
  if (debug)
  {
    for (int i = 0; i < n_my_data; i++)
    {
      Serial.print(i); Serial.print(" "); Serial.println(my_data[i]);
    }
    Serial.print("i"); Serial.print(" "); Serial.println(i_my_data);
    Serial.print("Last"); Serial.print(" "); Serial.println(my_data[n_my_data - 1]);

    Serial.print("Sum"); Serial.print(" "); Serial.println(my_sum);
    Serial.print("Ave"); Serial.print(" "); Serial.println(my_ave);
    Serial.println();
  }

  i_my_data++;
  if (i_my_data >= n_my_data)
    i_my_data = 0;

  return my_ave;
}

void my_moving_ave_clear(){
  //Make sure All Array is 0
  for (int i = 0; i < n_my_data; i++)
  {
    my_data[i] = 0;
  }}

bool shouldReadThisCycle(int freq, int &counter) {
    if (freq <= 0 || freq > 100) return false; // Frequência inválida
    int divider = 100 / freq;
    bool trigger = (++counter >= divider);
    if (trigger) counter = 0;
    return trigger;
}

// ============ TASK LOOP =============
void myloopTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);
  const float dt = 0.01;

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    if (!setup_motor_flag) {
      // Leitura apenas para gráficos/web
      if (imu_calibrated) {
        myIMU.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        roll = orientationData.orientation.z - roll_offset;
        pitch = orientationData.orientation.y - pitch_offset;
        yaw = orientationData.orientation.x;
      }

      if (shouldReadThisCycle(40, ultra_counter)){
        distance_ultra = us_get_distancia(p_trigPin_, p_echoPin_);
        average_ultra = my_moving_average(distance_ultra, 0);
      }
      z_fusion = average_ultra;

      continue;
    }

    if (imu_calibrated) {
      myIMU.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    roll = orientationData.orientation.z - roll_offset;
    pitch = orientationData.orientation.y - pitch_offset;
    yaw = orientationData.orientation.x;
    }

    if (setup_motor_flag && (abs(roll) > 50 || abs(pitch) > 50)) {
      setup_motor_flag = 0;
      ESC1.writeMicroseconds(1000);ESC2.writeMicroseconds(1000);ESC3.writeMicroseconds(1000);ESC4.writeMicroseconds(1000);
      Serial.println("!!! Motores DESARMADOS por segurança: Roll/Pitch excessivo !!!");
}

    if (shouldReadThisCycle(40, ultra_counter)){
      distance_ultra = us_get_distancia(p_trigPin_, p_echoPin_);
      average_ultra = my_moving_average(distance_ultra, 0);
    }

    z_fusion = average_ultra;

    float errorRoll = setpointRoll - roll;
    float errorPitch = setpointPitch - pitch;
    float errorYaw = setpointYaw - yaw;
    if (errorYaw > 180.0) errorYaw -= 360.0;
    else if (errorYaw < -180.0) errorYaw += 360.0;
    float errorZ = setpointZ - z_fusion;

    float outputRoll = computePID(errorRoll, pidRoll, dt);
    float outputPitch = computePID(errorPitch, pidPitch, dt);
    float outputYaw = computePID(errorYaw, pidYaw, dt);
    float outputZ = computePID(errorZ, pidZ, dt);

    outputZ = 0;
    //outputYaw = 0;
    //outputRoll = 0;
    //outputPitch = 0;

    int base_throttle_us = 1300;

    float motor_1  = outputZ - outputRoll + outputPitch + outputYaw + base_throttle_us;
    float motor_2  = outputZ - outputRoll - outputPitch - outputYaw + base_throttle_us;
    float motor_3  = outputZ + outputRoll - outputPitch + outputYaw + base_throttle_us;
    float motor_4  = outputZ + outputRoll + outputPitch - outputYaw + base_throttle_us;

    int motor_1_pulse = constrain(motor_1, 1100, 1999);
    int motor_2_pulse = constrain(motor_2, 1100, 1999);
    int motor_3_pulse = constrain(motor_3, 1100, 1999);
    int motor_4_pulse = constrain(motor_4, 1100, 1999);

    ESC1.writeMicroseconds(motor_1_pulse);
    ESC2.writeMicroseconds(motor_2_pulse);
    ESC3.writeMicroseconds(motor_3_pulse);
    ESC4.writeMicroseconds(motor_4_pulse);

    //Serial.print(errorZ);     Serial.print(',');
    //Serial.print(setpointZ);  Serial.print(',');
    //Serial.print(z_fusion);   Serial.println(' ');
    Serial.print(errorRoll);    Serial.print(',');
    Serial.print(setpointRoll); Serial.print(',');
    Serial.print(roll);         Serial.print(',');
    Serial.print(errorPitch);   Serial.print(',');
    Serial.print(setpointPitch);Serial.print(',');
    Serial.print(pitch);        Serial.print(',');
    Serial.print(errorYaw);     Serial.print(',');
    Serial.print(setpointYaw);  Serial.print(',');
    Serial.print(yaw);          Serial.println(' ');
    
  }
}

// ============ SETUP PWM ============
void setupPWM() {
  ESC1.setPeriodHertz(250);
  ESC2.setPeriodHertz(250);
  ESC3.setPeriodHertz(250);
  ESC4.setPeriodHertz(250);
  ESC1.attach(pwmPin1, 1000, 2000);
  ESC2.attach(pwmPin2, 1000, 2000);
  ESC3.attach(pwmPin3, 1000, 2000);
  ESC4.attach(pwmPin4, 1000, 2000);
  pwmValue = 0;
  int initialPulse = map(pwmValue, 0, 100, 1000, 2000);
  ESC1.writeMicroseconds(initialPulse);
  ESC2.writeMicroseconds(initialPulse);
  ESC3.writeMicroseconds(initialPulse);
  ESC4.writeMicroseconds(initialPulse);
}

void calibrateAltimeterBias() {
  // Faz várias leituras e calcula a média para melhor precisão
  const int num_samples = 10;
  float sum = 0;

  Serial.println("Calibrando bias do altimetro... Não mexa o dispositivo!");

  for (int i = 0; i < num_samples; i++) {
    sum += myPressure.readAltitude() * 100.0;
    delay(1000);
  }
  altitude_offset = sum / num_samples;
}

void calibrateBNO055Bias() {
  const int num_samples = 1000;
  float roll_sum = 0, pitch_sum = 0, yaw_sum = 0, accel_sum = 0;

  //sensors_event_t orientationData;
  //ensors_event_t linearAccelData;

  Serial.println("Calibrando bias do BNO055... Não mexa o dispositivo!");

  for (int i = 0; i < num_samples; i++) {
    myIMU.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    myIMU.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    roll_sum  += orientationData.orientation.z;
    pitch_sum += orientationData.orientation.y;
    accel_sum += linearAccelData.acceleration.z;

    delay(10);
  }

  roll_offset = roll_sum / num_samples;
  pitch_offset = pitch_sum / num_samples;
  accel_offset = accel_sum / num_samples;
  imu_calibrated = 1;

}


// ============ SETUP WEBSERVER ============
// Veja a documentação dos endpoints no início do arquivo!

void setupWebServer() {
  if (!LittleFS.begin(true)) {
    Serial.println("Erro ao montar LittleFS");
    return;
  }

  WiFi.begin(ssid, password);
  Serial.print("Conectando ao WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado. IP: " + WiFi.localIP().toString());

  // Rota principal
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  // Endpoint: GET /data
  // Saída: roll,pitch,yaw,z_filtered,setpointRoll,setpointPitch,setpointYaw,setpointZ,imuCalibrated
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
    String response = String(roll, 2) + "," + String(pitch, 2) + "," + String(yaw, 2) + "," + String(z_fusion, 2) + "," +
                      String(setpointRoll, 2) + "," + String(setpointPitch, 2) + "," + String(setpointYaw, 2) + "," + String(setpointZ, 2) + "," +
                      (imu_calibrated ? "1" : "0");
    request->send(200, "text/plain", response);
  });

  // Endpoint: GET /setpoint?roll=..&pitch=..&yaw=..&z=..
  server.on("/setpoint", HTTP_GET, [](AsyncWebServerRequest *request) {
    bool success = true;
    if (request->hasParam("roll")) setpointRoll = request->getParam("roll")->value().toFloat(); else success = false;
    if (request->hasParam("pitch")) setpointPitch = request->getParam("pitch")->value().toFloat(); else success = false;
    if (request->hasParam("yaw")) setpointYaw = request->getParam("yaw")->value().toFloat(); else success = false;
    if (request->hasParam("z")) setpointZ = request->getParam("z")->value().toFloat(); else success = false;

    if (success) {
      request->send(200, "text/plain", "Setpoints atualizados");
    } else {
      request->send(400, "text/plain", "Erro: Parametros de setpoint ausentes.");
    }
  });

  // Endpoint: GET /pwm (retorna valor atual do PWM)
  server.on("/pwm", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(pwmValue));
  });

  server.on("/get_pid_gains", HTTP_GET, [](AsyncWebServerRequest *request) {
    String response = "";
    response += String(pidRoll.kp, 2) + ",";
    response += String(pidRoll.ki, 2) + ",";
    response += String(pidRoll.kd, 2) + ",";
    response += String(pidPitch.kp, 2) + ",";
    response += String(pidPitch.ki, 2) + ",";
    response += String(pidPitch.kd, 2) + ",";
    response += String(pidYaw.kp, 2) + ",";
    response += String(pidYaw.ki, 2) + ",";
    response += String(pidYaw.kd, 2) + ",";
    response += String(pidZ.kp, 2) + ",";
    response += String(pidZ.ki, 2) + ",";
    response += String(pidZ.kd, 2);
    request->send(200, "text/plain", response);
  });

  // Endpoint: POST /pwm (corpo = valor int 0-100)
  server.on("/pwm", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      String body = "";
      for (size_t i = 0; i < len; i++) body += (char)data[i];
      bool valid = true;
      for (size_t i = 0; i < body.length(); i++) {
        if (!isDigit(body[i]) && !(i == 0 && body[i] == '-')) valid = false;
      }
      int newPwmPercentage = valid ? constrain(body.toInt(), 0, 100) : 0;
      pwmValue = newPwmPercentage;
      if (!setup_motor_flag) {
        int pulseWidth = map(pwmValue, 0, 100, 1000, 2000);
        ESC1.writeMicroseconds(pulseWidth);ESC2.writeMicroseconds(pulseWidth);ESC3.writeMicroseconds(pulseWidth);ESC4.writeMicroseconds(pulseWidth);
        request->send(200, "text/plain", "PWM manual atualizado. Motores DESARMADOS. PWM: " + String(pwmValue));
      } else {
        request->send(200, "text/plain", "PWM base para PID atualizado. Motores ARMADOS. PWM: " + String(pwmValue));
      }
    });

  // Endpoint: POST /setup_motor (corpo = "1" para armar, "0" para desarmar)
  server.on("/setup_motor", HTTP_POST,
    [](AsyncWebServerRequest *request) {},
    NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      String body_content = "";
      for (size_t i = 0; i < len; i++) body_content += (char)data[i];
      if (index + len == total) {
        if (body_content == "1") {
          setup_motor_flag = 1;
          Serial.println("Motores ARMADOS via Web!");
          request->send(200, "text/plain", "Motores ARMADOS!");
        } else if (body_content == "0") {
          setup_motor_flag = 0;
          Serial.println("Motores DESARMADOS via Web!");
          ESC1.writeMicroseconds(map(0, 0, 100, 1000, 2000));
          ESC2.writeMicroseconds(map(0, 0, 100, 1000, 2000));
          ESC3.writeMicroseconds(map(0, 0, 100, 1000, 2000));
          ESC4.writeMicroseconds(map(0, 0, 100, 1000, 2000));

          zerarPID();

          request->send(200, "text/plain", "Motores DESARMADOS!");
        } else {
          request->send(400, "text/plain", "Comando invalido (corpo deve ser '1' ou '0').");
        }
      }
    }
  );

  // Endpoint: POST /pid (campos kp_roll, ki_roll, kd_roll, ...)
  server.on("/pid", HTTP_POST, [](AsyncWebServerRequest *request) {
    bool all_ok = true;
    if (request->hasParam("kp_roll", true) && request->hasParam("ki_roll", true) && request->hasParam("kd_roll", true)) {
      pidRoll.kp = request->getParam("kp_roll", true)->value().toFloat();
      pidRoll.ki = request->getParam("ki_roll", true)->value().toFloat();
      pidRoll.kd = request->getParam("kd_roll", true)->value().toFloat();
    } else all_ok = false;
    if (request->hasParam("kp_pitch", true) && request->hasParam("ki_pitch", true) && request->hasParam("kd_pitch", true)) {
      pidPitch.kp = request->getParam("kp_pitch", true)->value().toFloat();
      pidPitch.ki = request->getParam("ki_pitch", true)->value().toFloat();
      pidPitch.kd = request->getParam("kd_pitch", true)->value().toFloat();
    } else all_ok = false;
    if (request->hasParam("kp_yaw", true) && request->hasParam("ki_yaw", true) && request->hasParam("kd_yaw", true)) {
      pidYaw.kp = request->getParam("kp_yaw", true)->value().toFloat();
      pidYaw.ki = request->getParam("ki_yaw", true)->value().toFloat();
      pidYaw.kd = request->getParam("kd_yaw", true)->value().toFloat();
    } else all_ok = false;
    if (request->hasParam("kp_z", true) && request->hasParam("ki_z", true) && request->hasParam("kd_z", true)) {
      pidZ.kp = request->getParam("kp_z", true)->value().toFloat();
      pidZ.ki = request->getParam("ki_z", true)->value().toFloat();
      pidZ.kd = request->getParam("kd_z", true)->value().toFloat();
    } else all_ok = false;
    if (all_ok) {
      Serial.println("Ganhos PID atualizados via Web.");
      request->send(200, "text/plain", "Ganhos PID atualizados com sucesso!");
    } else {
      Serial.println("Falha ao atualizar ganhos PID: parâmetros ausentes.");
      request->send(400, "text/plain", "Erro: Parametros PID ausentes ou invalidos.");
    }
  });

  server.begin();
}

float us_get_distancia(int triger, int echo)
{
  long duration;
  float distance;
  int i;

  digitalWrite(triger, LOW);
  delayMicroseconds(2);
  digitalWrite(triger, HIGH);
  delayMicroseconds(10);
  digitalWrite(triger, LOW);
  duration =  pulseIn(echo, HIGH,10000); //uS

  distance = duration * 0.034 / 2;

  return distance;
}

// ============ SETUP PRINCIPAL ============
void setup() {
  pinMode(p_trigPin_, OUTPUT);
  pinMode(p_echoPin_, INPUT);
  my_moving_ave_clear();
  Wire.begin();
  Serial.begin(115200);
  Serial.println("\nIniciando Setup do Drone...");

  if (!myIMU.begin()) {
    Serial.println("BNO055 não detectado! Verifique conexões e endereço I2C.");
    while (1) delay(10);
  }
  Serial.println("BNO055 detectado!");
  myIMU.setExtCrystalUse(true);

  myPressure.begin();
  myPressure.setModeAltimeter();
  myPressure.setOversampleRate(7);
  myPressure.enableEventFlags();
  delay(100);
  //calibrateAltimeterBias();
  calibrateBNO055Bias();
  z_filtered_previous = 0;

  setupPWM();
  setupWebServer();
  zerarPID();

  Serial.println("Setup inicial completo. Aguardando comando de armar motores via /setup_motor ou interface.");

  xTaskCreatePinnedToCore(
    myloopTask, "ControlLoop", 10000, NULL, 1, NULL, 1);
  Serial.println("Task de controle iniciada.");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
