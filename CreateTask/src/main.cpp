#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include "DHT.h"

// ---------------- CONFIG WIFI + MQTT ----------------
const char* ssid = "Lucas";
const char* password = "Qualquer123!";
const char* mqtt_server = "192.168.0.38";
const int mqtt_port = 1884;
const char* mqtt_topic_proximidade_airbag = "car/proximidade";
const char* mqtt_topic_temperatura = "car/temperatura";
const char* mqtt_topic_acelerador = "car/acelerador";
const char* mqtt_topic_freio = "car/freio";

// ---------------- PINOS ----------------

/* SENSOR PROXIMIDADE */
#define TRIG_PIN 5
#define ECHO_PIN 25

/* LEDS */
#define LED_CARRO 2
#define LED_AIRBAG 4

/* SENSOR TEMPERATURA */
#define DHTPIN 18
#define DHTTYPE DHT11

/* POTENCIOMETRO ACELERADOR */
#define PINO_ACELERADOR 34

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);

// ---------------- VARIÁVEIS ----------------
SemaphoreHandle_t xMutex;
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

// ---------------- WIFI ----------------
void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");
  Serial.println(WiFi.localIP());

  configTime(0, 0, "pool.ntp.org", "time.google.com", "time.windows.com");

  Serial.print("Sincronizando horário");
  time_t now = time(nullptr);
  while (now < 24 * 3600) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("\nHora sincronizada!");
}

// ---------------- MQTT ----------------
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

// ---------------- SENSOR ULTRASSÔNICO ----------------
float readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return duration * 0.034 / 2.0;
}

// ---------------- TASK IGNICAO ----------------
void taskIgnicao(void* pvParameters) {
  Serial.println("Aguardando partida...");
  vTaskDelay(pdMS_TO_TICKS(5000));

  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    carroLigado = true;
    airbagAtivado = false;
    xSemaphoreGive(xMutex);
  }

  digitalWrite(LED_CARRO, HIGH);
  digitalWrite(LED_AIRBAG, LOW);
  Serial.println("🚗 Carro ligado!");
  vTaskDelete(NULL);
}

// ---------------- TASK SENSOR PROXIMIDADE ----------------
void taskSensor(void* pvParameters) {
  for (;;) {
    float leitura = readUltrasonicDistance();

    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      if (carroLigado) {
        distancia_cm = leitura;

        if (leitura == 0.0 && !airbagAtivado) {
          carroLigado = false;
          airbagAtivado = true;
          colisaoDetectada = true;
          Serial.println("🛑 Colisão! Airbag ativado.");
        }
      }
      xSemaphoreGive(xMutex);
    }

    digitalWrite(LED_CARRO, carroLigado ? HIGH : LOW);
    digitalWrite(LED_AIRBAG, airbagAtivado ? HIGH : LOW);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ---------------- TASK SENSOR TEMPERATURA ----------------
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

// ---------------- TASK POTENCIOMETRO ACELERAÇÃO ----------------
void taskAceleradorLeitura(void* pvParameters) {
  for (;;) {
    bool ligado = false;
    bool colisao = false;
    float leitura = analogRead(PINO_ACELERADOR) / 4095.0;

    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      ligado = carroLigado;
      colisao = colisaoDetectada;
      xSemaphoreGive(xMutex);
    }

    if (ligado && !colisao) {
      float acel = leitura * ACELERACAO_MAX;

      if (leitura > 0.02) {
        velocidade += acel;
      } else {
        velocidade -= FREIO_MOTOR;
      }

      if (velocidade < 0) velocidade = 0;
      if (velocidade > VELOCIDADE_MAXIMA) velocidade = VELOCIDADE_MAXIMA;

      if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        pedal = acel;
        xSemaphoreGive(xMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ---------------- TASK MQTT PROXIMIDADE ----------------
void taskMQTT(void* pvParameters) {
  for (;;) {
    if (!client.connected()) connectToMQTT();
    client.loop();

    float distancia;
    bool ligado, colisao;
    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      distancia = distancia_cm;
      ligado = carroLigado;
      colisao = colisaoDetectada;
      xSemaphoreGive(xMutex);
    }

    if (!ligado && !colisao) {
      vTaskDelay(pdMS_TO_TICKS(500));
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
    doc["sensor_proximidade"] = JsonArray(); doc["sensor_proximidade"].add(distancia);
    doc["airbag"] = JsonArray(); doc["airbag"].add(airbagAtivado);
    doc["dispositivo"] = JsonArray(); doc["dispositivo"].add("esp32");
    doc["topico"] = JsonArray(); doc["topico"].add(mqtt_topic_proximidade_airbag);

    char payload[512];
    serializeJson(doc, payload);
    client.publish(mqtt_topic_proximidade_airbag, payload);
    Serial.println(payload);

    if (colisao) {
      if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
        colisaoDetectada = false;
        xSemaphoreGive(xMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ---------------- TASK MQTT TEMPERATURA ----------------
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

// ---------------- TASK MQTT ACELERAÇÃO ----------------
void taskMQTTAcelerador(void* pvParameters) {
  for (;;) {
    if (!client.connected()) connectToMQTT();
    client.loop();

    float pedal_local = 0.0;
    float velocidade_local = 0.0;
    bool ligado = false;
    bool colisao = false;

    if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
      pedal_local = pedal;
      velocidade_local = velocidade;
      ligado = carroLigado;
      colisao = colisaoDetectada;
      xSemaphoreGive(xMutex);
    }

    if (ligado && !colisao) {
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
      doc["aceleracao"] = JsonArray(); doc["aceleracao"].add(pedal_local);
      doc["velocidade"] = JsonArray(); doc["velocidade"].add(velocidade_local);
      doc["dispositivo"] = JsonArray(); doc["dispositivo"].add("esp32");
      doc["topico"] = JsonArray(); doc["topico"].add(mqtt_topic_acelerador);

      char payload[256];
      serializeJson(doc, payload);
      client.publish(mqtt_topic_acelerador, payload);
      Serial.println(payload);
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ---------------- SETUP ----------------
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
  xMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(taskIgnicao, "Ignicao", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskSensor, "Sensor", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskTemperatura, "TempSensor", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskAceleradorLeitura, "AceleradorLeitura", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskMQTT, "MQTT_Proximidade", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskMQTTTemperatura, "MQTT_Temp", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskMQTTAcelerador, "MQTT_Acelerador", 4096, NULL, 1, NULL, 1);

}

void loop() {}