/**
 * @file umart_ino_seguidor_parede.h
 * @brief Cabeçalho principal do firmware Micromouse Bidirecional no ESP32
 *        (Refatorado a partir de uMaRT-INO Seguidor de Parede).
 */

#ifndef _umart_ino_seguidor_parede_H_
#define _umart_ino_seguidor_parede_H_

#include <Arduino.h>
#include <Ticker.h>
#include <stdint.h>
#include "motores.h"
#include "sensores.h"
#include "controlador.h"
#include "Navigation.h"

// =============================================================================
// PINOS DE INTERFACE DE USUÁRIO NO ESP32
// =============================================================================
#define LED_STATUS      2   // LED onboard do ESP32 (indica conclusão do labirinto)
#define SW_START        0   // Botão BOOT/SW1 para início ou seleção de canto inicial

// Configuração padrão do labirinto (pode ser ajustada para qualquer MxM <= 18)
#define DEFAULT_MAZE_DIMENSION  18
#define DEFAULT_START_CORNER    START_BOTTOM_LEFT

// Protótipos das Funções
void setup(void);
void loop(void);
void callbackTimerControle(void);

#endif /* _umart_ino_seguidor_parede_H_ */
