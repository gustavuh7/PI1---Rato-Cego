/**
 * @file sensores.h
 * @brief Leitura e processamento dos 4 sensores infravermelhos FC-51 via ADC do ESP32.
 *        Integrado à camada de Frente Virtual (`Navigation.h`).
 */

#ifndef SENSORES_H_
#define SENSORES_H_

#include <Arduino.h>
#include "Navigation.h"

// =============================================================================
// MAPEAMENTO DOS PINOS FÍSICOS DOS SENSORES FC-51 (ADC1 ESP32)
// =============================================================================
#define SENSOR_FRONTAL_PIN  PIN_SENSOR_FRONT // GPIO 34 (ADC ref: 27 a 34)
#define SENSOR_TRASEIRO_PIN PIN_SENSOR_REAR  // GPIO 35 (ADC ref: 2815 a 2820)
#define SENSOR_ESQUERDO_PIN PIN_SENSOR_LEFT  // GPIO 39 (ADC ref: 32 a 40)
#define SENSOR_DIREITO_PIN  PIN_SENSOR_RIGHT // GPIO 36 (ADC ref: 45 a 52)

// Constantes de alinhamento central e detecção baseadas nas referências ADC
const int16_t CENTRO_LATERAL_ESQ = (ADC_REF_LEFT_MIN  + ADC_REF_LEFT_MAX)  / 2; // ~36
const int16_t CENTRO_LATERAL_DIR = (ADC_REF_RIGHT_MIN + ADC_REF_RIGHT_MAX) / 2; // ~48
const int16_t CENTRO_FRONTAL_FIS = (ADC_REF_FRONT_MIN + ADC_REF_FRONT_MAX) / 2; // ~30
const int16_t CENTRO_TRASEIRO_FIS = (ADC_REF_REAR_MIN + ADC_REF_REAR_MAX)  / 2; // ~2817

// =============================================================================
// PROTÓTIPOS DAS FUNÇÕES
// =============================================================================
uint8_t getSensoresParede(void);
int16_t getErroSensores(void);
void printSensores(void);

// Variáveis globais atualizadas com os valores virtuais (relativos à frente ativa)
extern int16_t sensor_frontal_virtual;
extern int16_t sensor_traseiro_virtual;
extern int16_t sensor_esquerdo_virtual;
extern int16_t sensor_direito_virtual;

#endif /* SENSORES_H_ */
