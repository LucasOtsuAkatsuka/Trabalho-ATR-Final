// DataCollector adaptado para o modelo do trabalho
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include "DHT.h"

// ---------------- CONFIG WIFI ----------------
const char* ssid = "Lucas";
const char* password = "Qualquer123!";

// ---------------- CONFIG MQTT ----------------
const char* mqtt_server = "192.168.0.38";
const int mqtt_port = 1884;

// ---------------- MQTT TOPICOS ----------------
const char* mqtt_topic_proximidade = "/sensors/esp32/proximidade";
const char* mqtt_topic_temperatura = "/sensors/esp32/temperatura";
const char* mqtt_topic_acelerador = "/sensors/esp32/acelerador";
const char* mqtt_topic_freio = "/sensors/esp32/freio";
const char* mqtt_topic_sensores = "/sensor_monitors";

WiFiClient espClient;
PubSubClient client(espClient);

// ---------------- DEFININDO PINOS ----------------
#define TRIG_PIN 5
#define ECHO_PIN 25
#define LED_CARRO 2
#define LED_AIRBAG 4
#define DHTPIN 18
#define DHTTYPE DHT11
#define PINO_ACELERADOR 34
#define PINO_FREIO 17


DHT dht(DHTPIN, DHTTYPE);

// ---------------- INSTANCIANDO MUTEX ----------------
SemaphoreHandle_t xMutex;


// ---------------- INICIALIZANDO VARIÁVEIS GLOBAIS ----------------
float distancia_cm = 0.0;
float temperatura = 0.0;
float velocidade = 0.0;
bool carroLigado = false;
bool airbagAtivado = false;
bool colisaoDetectada = false;
float pedal = 0.0;
const float VELOCIDADE_MAXIMA = 200.0;
const float ACELERACAO_MAX = 2.0;
const float FREIO_MOTOR = 0.5;


// ---------------- FUNÇÃO WIFI ----------------
void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");
  Serial.println(WiFi.localIP());
  configTime(0, 0, "pool.ntp.org", "time.google.com", "time.windows.com");
  time_t now = time(nullptr);
  while (now < 24 * 3600) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("\nHora sincronizada!");
}


// ---------------- FUNÇÃO MQTT ----------------
void connectToMQTT() {
  while (!client.connected()) {
    if (client.connect("ESP32Ultrassom")) {
      Serial.println("MQTT conectado.");
    } else {
      Serial.print("Erro MQTT: ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}


// ---------------- FUNÇÃO MQTT DOS SENSORES E SEUS PERIODOS ----------------
void publishSensorMonitorInfo() {
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  StaticJsonDocument<512> doc;
  #pragma GCC diagnostic pop
  doc["machine_id"] = "esp32";
  JsonArray sensors = doc.createNestedArray("sensors");

  JsonObject s1 = sensors.createNestedObject();
  s1["sensor_id"] = "proximidade";
  s1["data_type"] = "float";
  s1["data_interval"] = 200;

  JsonObject s2 = sensors.createNestedObject();
  s2["sensor_id"] = "temperatura";
  s2["data_type"] = "float";
  s2["data_interval"] = 2000;

  JsonObject s3 = sensors.createNestedObject();
  s3["sensor_id"] = "acelerador";
  s3["data_type"] = "float";
  s3["data_interval"] = 500;

  JsonObject s4 = sensors.createNestedObject();
  s4["sensor_id"] = "freio";
  s4["data_type"] = "float";
  s4["data_interval"] = 300;

  char payload[512];
  serializeJson(doc, payload);
  client.publish(mqtt_topic_sensores, payload);
  Serial.println("Mensagem inicial dos sensores publicada:");
  Serial.println(payload);
}


// ---------------- FUNÇÃO PARA USAR O SENSOR DE PROXIMIDADE ----------------
float readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return duration * 0.034 / 2.0;
}

// ---------------- FUNÇÃO PARA PUBLICAR VALORES DOS SENSORES NOS RESPECTIVOS TOPICOS ----------------
void publishSensorData(const char* topic, float value) {
  char timestamp[30];
  time_t now;
  struct tm timeinfo;
  time(&now);
  gmtime_r(&now, &timeinfo);
  strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &timeinfo);

  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  StaticJsonDocument<512> doc;
  #pragma GCC diagnostic pop
  doc["timestamp"] = timestamp;
  doc["value"] = value;

  char payload[256];
  serializeJson(doc, payload);
  client.publish(topic, payload);
  Serial.printf("Enviado para %s: %s\n", topic, payload);
}

// ---------------- TAREFA QUEPUBLICAPARA O TÓPICO DE SENSORES ----------------
void taskSensorMonitorInfo(void* pvParameters) {
  for (;;) {
    if (!client.connected()) connectToMQTT();
    publishSensorMonitorInfo();
    vTaskDelay(pdMS_TO_TICKS(60000)); 
  }
}

// ---------------- FUNÇÃO PARA QUE ENVIA PARA O TÓPICO DE PROXIMIDADE SEU VALOR ----------------
void taskProximidade(void* pvParameters) {
  for (;;) {
    if (!client.connected()) connectToMQTT();
    client.loop();

    float leitura = readUltrasonicDistance();
    if (carroLigado && !colisaoDetectada){
        publishSensorData(mqtt_topic_proximidade, leitura);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}



// ---------------- FUNÇÃO PARA QUE ENVIA PARA O TÓPICO DE TEMPERATURA SEU VALOR ----------------
void taskTemperatura(void* pvParameters) {
  for (;;) {
    if (carroLigado && !colisaoDetectada) {
      float temp = dht.readTemperature();
      if (!isnan(temp)) publishSensorData(mqtt_topic_temperatura, temp);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// ---------------- FUNÇÃO PARA QUE ENVIA PARA O TÓPICO DE ACELERAÇÃO SEU VALOR ----------------
void taskAceleradorLeitura(void* pvParameters) {
  for (;;) {
    float leitura = analogRead(PINO_ACELERADOR) / 4095.0;
    if (carroLigado && !colisaoDetectada) {
      float acel = leitura * ACELERACAO_MAX;
      velocidade += (leitura > 0.02) ? acel : 0;
      velocidade = constrain(velocidade, 0, VELOCIDADE_MAXIMA);
      publishSensorData(mqtt_topic_acelerador, acel);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ---------------- FUNÇÃO PARA QUE ENVIA PARA O TÓPICO DE FREIO SEU VALOR ----------------
void taskFreioLeitura(void* pvParameters) {
  for (;;) {
    float leitura = analogRead(PINO_FREIO) / 4095.0;
    if (carroLigado && !colisaoDetectada) {
      float acel = leitura * FREIO_MOTOR;
      velocidade -= (leitura > 0.02) ? acel : 0;
      velocidade = constrain(velocidade, 0, VELOCIDADE_MAXIMA);
      publishSensorData(mqtt_topic_freio, acel);
    }
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

void taskIgnicao(void* pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(5000));
  carroLigado = true;
  digitalWrite(LED_CARRO, HIGH);
  Serial.println("Carro ligado!");
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_CARRO, OUTPUT);
  pinMode(LED_AIRBAG, OUTPUT);
  digitalWrite(LED_CARRO, LOW);
  digitalWrite(LED_AIRBAG, LOW);
  dht.begin();
  connectToWiFi();
  client.setServer(mqtt_server, mqtt_port);
  connectToMQTT();
  publishSensorMonitorInfo();
  xMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(taskSensorMonitorInfo, "MQTT_Sensores", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskIgnicao, "Ignicao", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskProximidade, "MQTT_Proximidade", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskTemperatura, "MQTT_Temp", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskAceleradorLeitura, "MQTT_Acelerador", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskFreioLeitura, "MQTT_Freio", 4096, NULL, 2, NULL, 1);
}

void loop() {}
