#include "definicoes.h"

// ---------------- FUNÇÃO QUE LIGA O CARRO ----------------

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
  xTaskCreate(taskMQTTTemperatura, "MQTT_Temp", 4096, NULL, 1, NULL);

  xTaskCreate(tasktratarFreio, "TratarFreio", 4096, NULL, 3, NULL);
  xTaskCreate(tasktratarCondInv, "TratarCondInv", 4096, NULL, 3, NULL);
  xTaskCreate(tasktratarAirbag, "TratarAitbag", 4096, NULL, 3, NULL);
}

void loop() {}
