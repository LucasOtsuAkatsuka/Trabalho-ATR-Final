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
const char* mqtt_topic_alarme = "/sensors/alarme";

//-------- RECEBER ---------------
const char* mqtt_topic_freio_rec = "/valor/freio";


// ---------------- DEFININDO PINOS ----------------
#define TRIG_PIN 5
#define ECHO_PIN 25
#define LED_CARRO 2
#define LED_AIRBAG 4  
#define LED_FREIO 15
#define LED_ERRO 14
#define LED_TEMPERATURA 26
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

SemaphoreHandle_t mqttMutex;

// ---------------- INICIALIZANDO VARIÁVEIS GLOBAIS ----------------
bool carroLigado = false;
bool airbagAtivado = false;
float distancia_cm = 10.0;
float temperatura = 0.0;
float velocidade = 0.0;
float freio = 0.0;
float acel = 0.0;
bool colisaoDetectada = false;
bool frenagem = false;
const float VELOCIDADE_MAXIMA = 200.0;
const float ACELERACAO_MAX = 2.0;
const float FREIO_MOTOR = 0.5;
float freioAnalogico = 0.0;
float freioRecebidoMQTT = 0.0;

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
      delay(2000); 
    } else {
      Serial.print("Erro MQTT: ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}


void taskMQTTLoop(void* pvParameters){
  for(;;){
    if (!client.connected()) {
      connectToMQTT();
    }
    client.loop(); 
    vTaskDelay(pdMS_TO_TICKS(10));
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

// ---------------- FUNÇÃO PARA PUBLICAR VALORES DOS SENSORES NOS RESPECTIVOS TOPICOS ----------------
void publishSensorData(const char* topic, float value) {
  if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(300))) {
    
    char timestamp[30];
    time_t now;
    struct tm timeinfo;
    time(&now);
    gmtime_r(&now, &timeinfo);
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &timeinfo);

    StaticJsonDocument<128> doc;
    doc["timestamp"] = timestamp;
    doc["value"] = value;

    String payload;
    serializeJson(doc, payload);

    client.publish(topic, payload.c_str());
    Serial.printf("Enviado para %s: %s\n", topic, payload.c_str());

    xSemaphoreGive(mqttMutex);
  } else {
    Serial.println("Não foi possível obter o mutex para publicar MQTT.");
  }
}

// ---------------- TAREFA QUEPUBLICAPARA O TÓPICO DE SENSORES ----------------
void taskSensorMonitorInfo(void* pvParameters) {
  for (;;) {
    if (!client.connected()) connectToMQTT();

    if (carroLigado && !colisaoDetectada){
        publishSensorMonitorInfo();
    }
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

    float leitura = readUltrasonicDistance();
    if (carroLigado && !colisaoDetectada){
        publishSensorData(mqtt_topic_proximidade, leitura);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ---------------- FUNÇÃO QUE ENVIA PARA O TÓPICO DE TEMPERATURA SEU VALOR ----------------
void taskMQTTTemperatura(void* pvParameters) {
  for (;;) {
    if (!client.connected()) connectToMQTT();

    // Só publica se carro estiver ligado e sem colisão
    if (carroLigado && !colisaoDetectada) {
       float temp = dht.readTemperature();
       publishSensorData(mqtt_topic_temperatura, temp);
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// ---------------- FUNÇÃO QUE ENVIA PARA O TÓPICO DE ACELERAÇÃO SEU VALOR ----------------
void taskAceleradorLeitura(void* pvParameters) {
  for (;;) {
    if (!client.connected()) {
      connectToMQTT();
    }

    if (carroLigado && !colisaoDetectada) {
      float pedalAcelerador = 100*(analogRead(PINO_ACELERADOR) / 4095.0);
      if(pedalAcelerador < 10){
        pedalAcelerador = 0;
      }
      acel = pedalAcelerador;
      publishSensorData(mqtt_topic_acelerador, acel);
    }
    vTaskDelay(pdMS_TO_TICKS(600));
  }
}

// ---------------- FUNÇÃO QUE ENVIA PARA O TÓPICO DE FREIO SEU VALOR ----------------
void taskFreioLeitura(void* pvParameters) {
  for (;;) { 
    if (!client.connected()) {
      connectToMQTT();
    }

    if (carroLigado && !colisaoDetectada) {
      freioAnalogico = 100*(analogRead(PINO_FREIO) / 4095.0);
      if(freioAnalogico < 10){
        freioAnalogico = 0;
      }
      publishSensorData(mqtt_topic_freio, freioAnalogico);
    }
    vTaskDelay(pdMS_TO_TICKS(550));
  }
}
 
// ---------------- FUNÇÃO QUE LIGA O CARRO ----------------

void taskIgnicao(void* pvParameters) {
  Serial.println("Carro Ligando");
  vTaskDelay(pdMS_TO_TICKS(5000));
  carroLigado = true;
  digitalWrite(LED_CARRO, HIGH);
  Serial.println("Carro ligado!");
  vTaskDelete(NULL);
}


//----------------------------------------------------------
// ---------------- TRATAMENTO DOS ALARMES -----------------
//----------------------------------------------------------


// ---------------- FUNÇÃO MQTT CALLBACK ----------------

void callback(char* topic, byte* payload, unsigned int length) {
  // 1) Converte para String
  String msg((char*)payload, length);


  // 3) Valida formato: deve começar com '{' e terminar com '}'
  if (length == 0 || msg[0] != '{' || msg[msg.length() - 1] != '}') {
    Serial.println("Payload ignorado – não parece JSON completo.");
    return;
  }

  // 4) Faz o parsing seguro
  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, msg);
  if (err) {
    Serial.print("Erro JSON: ");
    Serial.println(err.c_str());
    return;
  }

  // 5) Atualiza variáveis
  if (strcmp(topic, mqtt_topic_freio_rec) == 0)
    freioRecebidoMQTT = doc["value"] | 0.0;
  else if (strcmp(topic, mqtt_topic_acelerador)==0) 
    acel  = doc["value"] | 0.0;
  else if (strcmp(topic, mqtt_topic_temperatura)==0) 
    temperatura = doc["value"] | 0.0;
  else if (strcmp(topic, mqtt_topic_proximidade)==0) 
    distancia_cm = doc["value"] | 0.0;
}

//------------------FUNÇÃO PARA PUBLICAR ALARMES---------------
void publishAlarm(const char* alarme) {
  if (alarme == nullptr) {
    Serial.println("ERRO: alarme NULL recebido, ignorando.");
    return;
  }

  if (xSemaphoreTake(mqttMutex, pdMS_TO_TICKS(300))) {
    char timestamp[30];
    time_t now; struct tm tm_utc;
    time(&now); gmtime_r(&now, &tm_utc);
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &tm_utc);

    StaticJsonDocument<128> doc;
    doc["timestemp"] = timestamp;
    doc["alarme"] = alarme; 

    String payload;
    serializeJson(doc, payload);
    client.publish(mqtt_topic_alarme, payload.c_str());
    Serial.printf("ALARME: %s\n", payload.c_str());

    xSemaphoreGive(mqttMutex);
  } else {
    Serial.println("Mutex ocupado – alarme não publicado.");
  }
}

// ---------------- TRATAR FREIO -----------------

void tasktratarFreio(void* pvParameters){
  for(;;){
    if (!client.connected()) connectToMQTT();
    if(carroLigado && !colisaoDetectada){
      Serial.println("-------------------------------------------");
      Serial.println(freioRecebidoMQTT);
      if (carroLigado && !colisaoDetectada) {
        if (freioRecebidoMQTT > 0.0) {
          publishAlarm("freio ativo");      
          digitalWrite(LED_FREIO, HIGH);   
          frenagem = true;
        }else{
          digitalWrite(LED_FREIO, LOW);
          frenagem = false;
        }
      }
    }else{
      digitalWrite(LED_FREIO, LOW);
      frenagem = false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(550));
  }
}

// ---------------- FREIO E ACELERADOR ACIONADOS -----------------

void tasktratarCondInv(void* pvParameters){
  for(;;){
    if (!client.connected()) {
      connectToMQTT();
    }

    if(carroLigado && !colisaoDetectada){
      if(frenagem && acel > 0.0){
        publishAlarm("freio+acel");
        digitalWrite(LED_ERRO, HIGH);
      }else{
        digitalWrite(LED_ERRO, LOW);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(700));
  }
}

// ---------------- Airbag -----------------
void tasktratarAirbag(void* pvParameters){
  for(;;){
    if (!client.connected()) {
      connectToMQTT();
    }

    if(carroLigado and !colisaoDetectada){
      if(distancia_cm == 0.0){
        publishAlarm("airbag ativado");
        digitalWrite(LED_AIRBAG, HIGH);
        colisaoDetectada = true;
      }else{
        digitalWrite(LED_ERRO, LOW);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(520));
  }
}

// ---------------- FUNÇÃO QUE TRATA E ENVIA PARA O TÓPICO DE VELOCIDADE SEU VALOR ----------------

void taskVelocidadeLeitura(void* pvParameters){
  for(;;){
    if (!client.connected()) {
      connectToMQTT();
    }

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
    vTaskDelay(pdMS_TO_TICKS(650));
  }
}

// ---------------- Alarme Temperatura -----------------
void tasktratarTemp(void* pvParameters){
  for(;;){
    if (!client.connected()) {
      connectToMQTT();
    }

    if(carroLigado and !colisaoDetectada){
      if(temperatura > 30.0){
        publishAlarm("temperatura Alta");
        digitalWrite(LED_TEMPERATURA, HIGH); 
      }else{
        digitalWrite(LED_ERRO, LOW);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2100));
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
  pinMode(LED_TEMPERATURA, OUTPUT);
  digitalWrite(LED_CARRO, LOW);
  digitalWrite(LED_AIRBAG, LOW);
  digitalWrite(LED_FREIO, LOW);
  digitalWrite(LED_ERRO, LOW);
  digitalWrite(LED_TEMPERATURA, LOW);
  mqttMutex = xSemaphoreCreateMutex();
  dht.begin();
  connectToWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  connectToMQTT();

  client.subscribe(mqtt_topic_freio_rec);
  client.subscribe(mqtt_topic_proximidade);
  client.subscribe(mqtt_topic_temperatura);
  client.subscribe(mqtt_topic_acelerador);

  xTaskCreatePinnedToCore(taskMQTTLoop, "MQTT_Loop", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskSensorMonitorInfo, "Sensores", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskIgnicao, "Ignicao", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskMQTTTemperatura, "Temperatura", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskProximidade, "MProximidade", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskAceleradorLeitura, "Acelerador", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskFreioLeitura, "Freio",4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskVelocidadeLeitura, "Velocidade", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(tasktratarFreio, "Tratar_Freio", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(tasktratarCondInv, "Tratar_CondInv", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(tasktratarAirbag, "Airbag", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(tasktratarTemp, "Tratar_Temp", 4096, NULL, 1, NULL, 1);
}

void loop() {
 
}
