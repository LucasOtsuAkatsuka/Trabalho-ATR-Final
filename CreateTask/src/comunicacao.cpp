#include "definicoes.h"

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

// ---------------- TAREFA QUE PUBLICAPARA O TÓPICO DE SENSORES ----------------
void taskSensorMonitorInfo(void* pvParameters) {
  for (;;) {
    if (!client.connected()) connectToMQTT();
    publishSensorMonitorInfo();
    vTaskDelay(pdMS_TO_TICKS(60000)); 
  }
}
