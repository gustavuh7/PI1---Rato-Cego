/**
 * @file WebInterface.h
 * @brief Interface Visual Web (Wi-Fi Access Point + Dashboard em Tempo Real)
 *        para controle de Início, Parada de Emergência, Reset e visualização
 *        do mapeamento 18x18 do robô Rato Cego no ESP32.
 */

#ifndef WEB_INTERFACE_H_
#define WEB_INTERFACE_H_

#include <Arduino.h>
#include "Navigation.h"

// =============================================================================
// CONFIGURAÇÕES DA REDE WI-FI DO ROBÔ (ACCESS POINT)
// Conecte o celular ou notebook nesta rede e acesse: http://192.168.4.1
// =============================================================================
#define WIFI_AP_SSID        "RatoCego-ESP32"
#define WIFI_AP_PASSWORD    "12345678"
#define WIFI_AP_CHANNEL     1

/**
 * @brief Inicializa o Wi-Fi Access Point (`RatoCego-ESP32`), configura as rotas HTTP
 *        e cria uma tarefa FreeRTOS dedicada no Core 0 do ESP32 para atender
 *        os cliques da interface visual em tempo real (mesmo durante curvas/retas).
 */
void WebInterface_Init(void);

#endif /* WEB_INTERFACE_H_ */

