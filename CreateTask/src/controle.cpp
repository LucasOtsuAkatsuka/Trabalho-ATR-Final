#include "definicoes.h"

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
  }
  vTaskDelay(pdMS_TO_TICKS(timePriotidadeAlta));
}

// ---------------- FREIO E ACELERADOR ACIONADOS -----------------

void tasktratarCondInv(void* pvParameters){
  for(;;){
    if(frenagem && acel != 0){
      digitalWrite(LED_ERRO, HIGH);
    }else{
      digitalWrite(LED_ERRO, LOW);
    }
  }
  vTaskDelay(pdMS_TO_TICKS(timePriotidadeAlta));
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
  }
  vTaskDelay(pdMS_TO_TICKS(timePriotidadeAlta));
}