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
const char* mqtt_topic_velocidade = "/sensors/esp32/velocidade";
const char* mqtt_topic_sensores = "/sensor_monitors";
const char* mqtt_topic_alarmefreio = "/alarmes/freio";

// ---------------- Filas para os tópicos ----------------

QueueHandle_t queue_alarme_freio = xQueueCreate(2, sizeof(u32_t));

// ---------------- DEFININDO PINOS ----------------
#define TRIG_PIN 5
#define ECHO_PIN 25
#define LED_CARRO 2
#define LED_AIRBAG 4  
#define LED_FREIO 15
#define LED_ERRO 0
#define PINO_ACELERADOR 34
#define PINO_FREIO 17

/* SENSOR TEMPERATURA */
#define DHTPIN 18
#define DHTTYPE DHT11

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);

// ---------------- INSTANCIANDO MUTEX ----------------
SemaphoreHandle_t xMutex;


// ---------------- INICIALIZANDO VARIÁVEIS GLOBAIS ----------------
float distancia_cm = 0.0;
float temperatura = 0.0;
float velocidade = 0.0;
float pedal = 0.0;
float freio = 0.0;
float acel = 0.0;
bool carroLigado = false;
bool airbagAtivado = false;
bool colisaoDetectada = false;
bool frenagem = false;
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


// ---------------- FUNÇÃO MQTT CALLBACK ----------------

void callback(char* topic, byte* payload, unsigned int length) {

  String mensagem;
  for (int i = 0; i < length; i++) {
    mensagem += (char)payload[i];
  }

  if (strcmp(topic, mqtt_topic_alarmefreio) == 0) {
    xQueueSend(queue_alarme_freio, &mensagem, portMAX_DELAY);
  } 
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

// ---------------- FUNÇÃO QUE LÊ O SENSOR DE TEMPERATURA  ----------------

void taskTemperatura(void* pvParameters) {
  for (;;) {
    bool ligado = false;
    bool colisao = false;

    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      ligado = carroLigado;
      colisao = colisaoDetectada;
      xSemaphoreGive(xMutex);
    }

    if (ligado && !colisao) {
      float temp = dht.readTemperature();
      if (!isnan(temp)) {
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
          temperatura = temp;
          xSemaphoreGive(xMutex);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// ---------------- FUNÇÃO QUE ENVIA PARA O TÓPICO DE TEMPERATURA SEU VALOR ----------------
void taskMQTTTemperatura(void* pvParameters) {
  for (;;) {
    if (!client.connected()) connectToMQTT();
    client.loop();

    float temp = 0.0;
    bool ligado = false;
    bool colisao = false;

    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      ligado = carroLigado;
      colisao = colisaoDetectada;
      temp = temperatura;
      xSemaphoreGive(xMutex);
    }

    // Só publica se carro estiver ligado e sem colisão
    if (!ligado || colisao) {
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

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
    doc["timestamp"] = JsonArray(); doc["timestamp"].add(timestamp);
    doc["temperatura"] = JsonArray(); doc["temperatura"].add(temp);
    doc["dispositivo"] = JsonArray(); doc["dispositivo"].add("esp32");
    doc["topico"] = JsonArray(); doc["topico"].add(mqtt_topic_temperatura);

    char payload[256];
    serializeJson(doc, payload);
    client.publish(mqtt_topic_temperatura, payload);
    Serial.println(payload);

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// ---------------- FUNÇÃO LEITURA PEDAL DO ACELERADOR ----------------
void taskPedalLeitura(void* pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      pedal = 100*(analogRead(PINO_ACELERADOR) / 4095.0);
      if(pedal < 5){
        pedal = 0;
      }
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ---------------- FUNÇÃO QUE ENVIA PARA O TÓPICO DE ACELERAÇÃO SEU VALOR ----------------
void taskAceleradorLeitura(void* pvParameters) {
  for (;;) {
    if (carroLigado && !colisaoDetectada) {
      if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        if(frenagem){
          acel = 0;
        }else{
          acel = pedal * ACELERACAO_MAX;
        }
        publishSensorData(mqtt_topic_acelerador, acel);
      xSemaphoreGive(xMutex);
      }
    }else{
      acel = 0;
      publishSensorData(mqtt_topic_acelerador, acel);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ---------------- FUNÇÃO QUE ENVIA PARA O TÓPICO DE FREIO SEU VALOR ----------------
void taskFreioLeitura(void* pvParameters) {
  for (;;) { 
    if (carroLigado && !colisaoDetectada) {
      if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        freio = 100*(analogRead(PINO_FREIO) / 4095.0);
        if(freio >= 10){
          frenagem = true;
          digitalWrite(LED_FREIO, HIGH);
        }else{
          frenagem = false;
          digitalWrite(LED_FREIO, LOW);
        }
        publishSensorData(mqtt_topic_freio, frenagem);
      xSemaphoreGive(xMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

// ---------------- FUNÇÃO QUE ENVIA PARA O TÓPICO DE VELOCIDADE SEU VALOR ----------------
void taskVelocidadeLeitura(void* pvParameters){
  for(;;){
    if (carroLigado && !colisaoDetectada) {
      if(acel > 0){
        velocidade += (pedal > 0.02) ? acel : 0;
        velocidade = constrain(velocidade, 0, VELOCIDADE_MAXIMA);
      }else if (acel == 0 && frenagem == true){
        velocidade -= freio * FREIO_MOTOR;
        velocidade = constrain(velocidade, 0, VELOCIDADE_MAXIMA);
      }else{
        velocidade -= FREIO_MOTOR;
        velocidade = constrain(velocidade, 0, VELOCIDADE_MAXIMA);
      }
      publishSensorData(mqtt_topic_velocidade, velocidade);
    }
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

      
// ---------------- FUNÇÃO QUE LIGA O CARRO ----------------

void taskIgnicao(void* pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(5000));
  carroLigado = true;
  digitalWrite(LED_CARRO, HIGH);
  Serial.println("Carro ligado!");
  vTaskDelete(NULL);
}

// ---------------- TRATAMENTO DOS ALARMES -----------------

// ---------------- FREIO E ACELERADOR ACIONADOS -----------------

void alarmeFreio(void* pvParameters) {
  for(;;){
    char msg[5];
    xQueueReceive(queue_alarme_freio, &msg, portMAX_DELAY);
    if(strcmp(msg, "true ") == 0){
      digitalWrite(LED_ERRO, HIGH);
    }else{
      digitalWrite(LED_ERRO, LOW);
    }
  }
}



void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_CARRO, OUTPUT);
  pinMode(LED_AIRBAG, OUTPUT);
  pinMode(LED_FREIO, OUTPUT);
  pinMode(LED_ERRO, OUTPUT);
  digitalWrite(LED_CARRO, LOW);
  digitalWrite(LED_AIRBAG, LOW);
  digitalWrite(LED_FREIO, LOW);
  digitalWrite(LED_ERRO, LOW);
  connectToWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  connectToMQTT();
  publishSensorMonitorInfo();
  xMutex = xSemaphoreCreateMutex();

  client.subscribe(mqtt_topic_alarmefreio);

  xTaskCreatePinnedToCore(taskSensorMonitorInfo, "MQTT_Sensores", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskIgnicao, "Ignicao", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskProximidade, "MQTT_Proximidade", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskTemperatura, "MQTT_Temp", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskAceleradorLeitura, "MQTT_Acelerador", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskFreioLeitura, "MQTT_Freio", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskPedalLeitura, "MQTT_Pedal", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskVelocidadeLeitura, "MQTT_Velocidade", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskMQTTTemperatura, "MQTT_Temp", 4096, NULL, 1, NULL, 1);
}

void loop() {}
