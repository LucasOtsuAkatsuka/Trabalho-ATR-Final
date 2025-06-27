// DataCollector adaptado para o modelo do trabalho
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include "DHT.h"

// ---------------- CONFIG WIFI ----------------
const char* ssid = "Iphone do Japa";
const char* password = "qualquer123!";

// ---------------- CONFIG MQTT ----------------
const char* mqtt_server = "172.20.10.2";
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
#define PINO_FREIO 35

/* SENSOR TEMPERATURA */
#define DHTPIN 18
#define DHTTYPE DHT11

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);


// ---------------- PERIODO PRIORIDADE ----------------
#define timePriotidadeAlta 3000
#define timePriotidadeMedia 4000
#define timePriotidadeBaixa 5000


// ---------------- INSTANCIANDO MUTEX ----------------
SemaphoreHandle_t xMutex;


// ---------------- INICIALIZANDO VARIÁVEIS GLOBAIS ----------------
float distancia_cm = 0.0;
float temperatura = 0.0;
float velocidade = 0.0;
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
  client.publish(mqtt_topic_sensores, payload, true);
  Serial.println("Mensagem inicial dos sensores publicada:");
  Serial.println(payload);
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

// ---------------- FUNÇÃO PARA QUE ENVIA PARA O TÓPICO DE PROXIMIDADE SEU VALOR ----------------
void taskProximidade(void* pvParameters) {
  for (;;) {
    if (!client.connected()) connectToMQTT();
    client.loop();

    float leitura = readUltrasonicDistance();
    if (carroLigado && !colisaoDetectada){
        publishSensorData(mqtt_topic_proximidade, leitura);
    }
    vTaskDelay(pdMS_TO_TICKS(timePriotidadeAlta));
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
      xSemaphoreGive(xMutex);
    }

    // Só publica se carro estiver ligado e sem colisão
    if (ligado || !colisao) {
       temp = dht.readTemperature();
       publishSensorData(mqtt_topic_temperatura, temp);
    }

    vTaskDelay(pdMS_TO_TICKS(timePriotidadeBaixa));
  }
}

// ---------------- FUNÇÃO QUE ENVIA PARA O TÓPICO DE ACELERAÇÃO SEU VALOR ----------------
void taskAceleradorLeitura(void* pvParameters) {
  for (;;) {
    float pedal;
    if (carroLigado && !colisaoDetectada) {
      pedal = 100*(analogRead(PINO_ACELERADOR) / 4095.0);
      if(pedal < 10){
        pedal = 0;
      }
      publishSensorData(mqtt_topic_acelerador, acel);
    }
    vTaskDelay(pdMS_TO_TICKS(timePriotidadeMedia));
  }
}

// ---------------- FUNÇÃO QUE ENVIA PARA O TÓPICO DE FREIO SEU VALOR ----------------
void taskFreioLeitura(void* pvParameters) {
  for (;;) { 
    if (carroLigado && !colisaoDetectada) {
      float pedalfreio = 100*(analogRead(PINO_FREIO) / 4095.0);
      if(pedalfreio < 10){
        pedalfreio = 0;
      }
      publishSensorData(mqtt_topic_freio, pedalfreio);
    }
    vTaskDelay(pdMS_TO_TICKS(timePriotidadeAlta));
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

// ---------------- FUNÇÃO MQTT CALLBACK ----------------

void callback(char* topic, byte* payload, unsigned int length) {
  String mensagem;
  mensagem.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) {
    mensagem += (char)payload[i];
  }

  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, mensagem);
  if (err) {
    Serial.println("-------------------------------------------------");
    Serial.print("Erro JSON: ");
    Serial.println(err.c_str());
    return;
  }

  if (strcmp(topic, mqtt_topic_freio) == 0) {
    freio = doc["value"].as<float>();
  } else if(strcmp(topic, mqtt_topic_acelerador)==0){
    acel = doc["value"].as<float>();
  } else if(strcmp(topic, mqtt_topic_temperatura)==0){
    temperatura = doc["value"].as<float>();
  } else if(strcmp(topic, mqtt_topic_proximidade)==0){
    distancia_cm = doc["value"].as<float>();
  }
}

// ---------------- TRATAR FREIO -----------------

void tasktratarFreio(void* pvParameters){
  for(;;){
    if(freio != 0){
      frenagem = true;
      digitalWrite(LED_FREIO, HIGH);
    }else{
      frenagem = false;
      digitalWrite(LED_FREIO, LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(timePriotidadeAlta));
  }
}

// ---------------- FREIO E ACELERADOR ACIONADOS -----------------

void tasktratarCondInv(void* pvParameters){
  for(;;){
    if(frenagem && acel != 0){
      digitalWrite(LED_ERRO, HIGH);
    }else{
      digitalWrite(LED_ERRO, LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(timePriotidadeAlta));
  }
}

// ---------------- Airbag -----------------
void tasktratarAirbag(void* pvParameters){
  for(;;){
    if(distancia_cm == 0){
      digitalWrite(LED_ERRO, HIGH);
      if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        colisaoDetectada = true;
        xSemaphoreGive(xMutex);
      }
    }else{
      digitalWrite(LED_ERRO, LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(timePriotidadeAlta));
  }
}

// ---------------- FUNÇÃO QUE ENVIA PARA O TÓPICO DE VELOCIDADE SEU VALOR ----------------

void taskVelocidadeLeitura(void* pvParameters){
  for(;;){
    if (carroLigado && !colisaoDetectada) {
      if(acel > 0){
        velocidade += acel * ACELERACAO_MAX;
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
    vTaskDelay(pdMS_TO_TICKS(timePriotidadeBaixa));
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
  xMutex = xSemaphoreCreateMutex();

  client.subscribe(mqtt_topic_freio);
  client.subscribe(mqtt_topic_proximidade);
  client.subscribe(mqtt_topic_temperatura);
  client.subscribe(mqtt_topic_acelerador);

  xTaskCreate(taskSensorMonitorInfo, "MQTT_Sensores", 4096, NULL, 1, NULL);
  xTaskCreate(taskIgnicao, "Ignicao", 2048, NULL, 2, NULL);
  xTaskCreate(taskProximidade, "MQTT_Proximidade", 4096, NULL, 3, NULL);
  xTaskCreate(taskAceleradorLeitura, "MQTT_Acelerador", 4096, NULL, 2, NULL);
  xTaskCreate(taskFreioLeitura, "MQTT_Freio", 4096, NULL, 3, NULL);
  xTaskCreate(taskVelocidadeLeitura, "MQTT_Velocidade", 4096, NULL, 1, NULL);
  xTaskCreate(tasktratarFreio, "MQTT_Temp", 4096, NULL, 3, NULL);
  xTaskCreate(tasktratarCondInv, "MQTT_Temp", 4096, NULL, 3, NULL);
  xTaskCreate(tasktratarAirbag, "MQTT_Temp", 4096, NULL, 3, NULL);
}

void loop() {}