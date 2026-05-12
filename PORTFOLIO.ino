// =====================================================
// Floor Cleaning Robot – With MQTT for Node-RED Dashboard
// =====================================================

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ------------------- WiFi -------------------
const char* ssid = "WiFi_ID";
const char* password = "password";

// ------------------- MQTT Server -------------------
const char* mqtt_server = "broker.hivemq.com";

// MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);

// =====================================================
// Sensors & Actuators
// =====================================================
#define LDR_PIN     34
#define LED_PIN     32

#define TRIG_PIN    13
#define ECHO_PIN    26

#define CLIFF_PIN   33

#define BUZZER_PIN   4

// Motor pins
#define AIN1 23
#define AIN2 19
#define PWMA 18

#define BIN1 14
#define BIN2 12
#define PWMB 27

#define STBY_PIN 2

bool emergencyStop = false;  // true = 立即停止，false = 正常自动运行
unsigned long runtimeSec = 0;
String lastState = "";

// MPU6050
Adafruit_MPU6050 mpu;
float TILT_THRESHOLD = 10.0;

// =====================================================
// WiFi Setup
// =====================================================
void setup_wifi() {
  delay(10);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  msg.trim();

  Serial.print("MQTT Cmd: ");
  Serial.println(msg);

  if (msg == "STOP") {
    emergencyStop = true;
    motorStop();
  } else if (msg == "AUTO") {
    emergencyStop = false;
  }

  if (String(topic) == "robot/reset_runtime") {
    runtimeSec = 0;
    Serial.println("Runtime Reset!");
  }
}

// =====================================================
// MQTT Reconnect
// =====================================================
void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32_Robot")) {
      Serial.println(" connected!");
      client.subscribe("robot/cmd");  // 订阅来自 Node-RED 的指令
      client.subscribe("robot/reset_runtime");  // ✅ 加这一行
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

// =====================================================
// Ultrasonic
// =====================================================
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return 400.0;
  return duration * 0.0343 / 2.0;
}

void publishState(String newState) {
  if (newState != lastState) {
    client.publish("robot/state", newState.c_str());
    lastState = newState;
  }
}

// =====================================================
// Motor 控制
// =====================================================
void motorDrive(int leftSpeed, int rightSpeed) {

  // Left motor
  if (leftSpeed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, leftSpeed);
  } else if (leftSpeed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -leftSpeed);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }

  // Right motor
  if (rightSpeed > 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, rightSpeed);
  } else if (rightSpeed < 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, -rightSpeed);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}

void motorStop() {
  motorDrive(0, 0);
}

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(115200);

  // ---------------- WiFi + MQTT ----------------
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // ---------------- IO pins ----------------
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(CLIFF_PIN, INPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);

  // ---------------- MPU6050 ----------------
  Wire.begin(22, 21);
  mpu.begin();
  delay(200);
}

// =====================================================
// Loop
// =====================================================
void loop() {

  // Ensure MQTT connected
  if (!client.connected()) reconnect();
  client.loop();

  // ---------------- LDR ----------------
  int ldrVal = analogRead(LDR_PIN);
  digitalWrite(LED_PIN, (ldrVal > 3000));
  client.publish("robot/led", (ldrVal > 3000) ? "ON" : "OFF");

  // ---------------- Cliff ----------------
  int cliff = digitalRead(CLIFF_PIN);
  bool cliffDetected = (cliff == HIGH);

  // ---------------- MPU6050 Tilt ----------------
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 57.3;
  float roll  = atan2(ay, sqrt(ax * ax + az * az)) * 57.3;

  bool tilted = (abs(pitch) > TILT_THRESHOLD || abs(roll) > TILT_THRESHOLD);
  if (tilted) {
    client.publish("robot/tilted", "1");
    publishState("STOP_TILT");
    tone(BUZZER_PIN, 2000);
    motorStop();
    delay(50);
    client.publish("robot/buzzer", "ON");
    return;
  } else {
    noTone(BUZZER_PIN);
    client.publish("robot/tilted", "0");
    client.publish("robot/buzzer", "OFF");
}

  // ---------------- Ultrasonic ----------------
  float dist = readUltrasonic();

  // =====================================================
  // 🟦 MQTT Publish → Node-RED Dashboard
  // =====================================================
  static unsigned long lastPub = 0;
  if (millis() - lastPub > 1000) {
    runtimeSec++;
    client.publish("robot/distance", String(dist).c_str());
    client.publish("robot/cliff", cliffDetected ? "1" : "0");
    client.publish("robot/ldr", String(ldrVal).c_str());
    client.publish("robot/mode", emergencyStop ? "STOP" : "AUTO");
    client.publish("robot/runtime", String(runtimeSec).c_str());
    lastPub = millis();
  }

  // -------------------------------------------------
  // Emergency Stop
  // -------------------------------------------------
  if (emergencyStop) {
    publishState("STOP_EMERGENCY");
    motorStop();
    delay(50);
    return;
  }

  // -------------------------------------------------
  // Cliff detected
  // -------------------------------------------------
  if (cliffDetected) {
    publishState("STOP_CLIFF");
    motorStop();
    delay(50);
    return;
  }

  // -------------------------------------------------
  // Obstacle avoidance
  // -------------------------------------------------
  if (dist < 20.0) {
    publishState("REVERSING");
    motorDrive(-180, -180);
    delay(100);

    publishState("TURNING");
    motorDrive(-180, 180);
    delay(100);

    return;
  }

  // -------------------------------------------------
  // Normal forward driving
  // -------------------------------------------------
  publishState("FORWARD");
  motorDrive(200, 200);
  delay(100);
}