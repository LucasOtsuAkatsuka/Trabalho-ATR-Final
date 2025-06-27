#include "definicoes.h"

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