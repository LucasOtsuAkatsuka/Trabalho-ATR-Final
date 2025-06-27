#include "definicoes.h"

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