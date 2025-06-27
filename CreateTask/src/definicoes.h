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


// ------- Conexão ----------
void connectToMQTT();
void connectToWiFi();
void callback(char* topic, byte* payload, unsigned int length);

// ------- Comunicação ----------
void publishSensorMonitorInfo();
void publishSensorData(const char* topic, float value);
void taskSensorMonitorInfo(void* pvParameters);

// ------- Sensores ----------
void taskProximidade(void* pvParameters);
void taskMQTTTemperatura(void* pvParameters);
void taskAceleradorLeitura(void* pvParameters);
void taskFreioLeitura(void* pvParameters);
void taskVelocidadeLeitura(void* pvParameters);

// ------- Controle ----------
void tasktratarFreio(void* pvParameters);
void tasktratarCondInv(void* pvParameters);
void tasktratarAirbag(void* pvParameters);