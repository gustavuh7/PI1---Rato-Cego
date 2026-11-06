/**
 * @file motores.h
 * @brief Driver de acionamento dos micromotores DC via ponte H dupla MX1508 no ESP32.
 *        Inclui abstração para inversão de frente virtual (Motor Esquerdo U3 e Motor Direito U6).
 */

#ifndef MOTORES_H_
#define MOTORES_H_

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// DEFINIÇÃO DE PINOS DO DRIVER MX1508 (ESP32)
// Ajuste os GPIOs conforme a trilha da PCB do seu robô para U3 (Esq) e U6 (Dir)
// =============================================================================
#define IN1_E   25  // Motor Esquerdo (U3) - Entrada IN1
#define IN2_E   26  // Motor Esquerdo (U3) - Entrada IN2
#define PWM_E   25  // Canal PWM Esquerdo (compatível com ponte H de 2 ou 3 fios)

#define IN1_D   27  // Motor Direito (U6)  - Entrada IN1
#define IN2_D   14  // Motor Direito (U6)  - Entrada IN2
#define PWM_D   27  // Canal PWM Direito

// =============================================================================
// CONSTANTES DE OPERAÇÃO PWM
// =============================================================================
#define PWM_MAX             255
#define PWM_DUTY_75_PERCENT 191 // 75% de duty cycle inicial (0.75 * 255 = 191)
#define PWM_FREQ_HZ         20000
#define PWM_RESOLUTION_BITS 8

// Canais LEDC do ESP32 para geração de PWM nas entradas do MX1508
#define LEDC_CH_E_IN1       0
#define LEDC_CH_E_IN2       1
#define LEDC_CH_D_IN1       2
#define LEDC_CH_D_IN2       3

// =============================================================================
// PROTÓTIPOS DAS FUNÇÕES
// =============================================================================

/**
 * @brief Inicializa os pinos e canais PWM do driver MX1508 no ESP32.
 */
void initMotores(void);

/**
 * @brief Aciona diretamente os motores físicos (Motor Esquerdo U3 e Motor Direito U6).
 * @param pwm_esquerda Valor entre -255 e +255.
 * @param pwm_direita  Valor entre -255 e +255.
 */
void setMotores(int16_t pwm_esquerda, int16_t pwm_direita);

/**
 * @brief Aciona os motores considerando o estado de `inverted_front`.
 *        - Quando `inverted_front == false`: aplica diretamente aos motores físicos.
 *        - Quando `inverted_front == true`: inverte o sentido de rotação e espelha
 *          o motor esquerdo com o motor direito (`phys_esq = -virt_dir`, `phys_dir = -virt_esq`).
 * @param pwm_virtual_esq Comando PWM para a roda esquerda virtual (-255 a +255).
 * @param pwm_virtual_dir Comando PWM para a roda direita virtual (-255 a +255).
 * @param inverted_front  Estado atual da frente virtual.
 */
void setMotoresVirtual(int16_t pwm_virtual_esq, int16_t pwm_virtual_dir, bool inverted_front);

#endif /* MOTORES_H_ */

