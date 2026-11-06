/**
 * @file controlador.h
 * @brief Controle cinemático e PID de velocidade para o Micromouse Bidirecional no ESP32.
 *        Parametrizado para rodas de 44 mm, bitola de 90 mm, encoders de 590 PPR e célula de 165 mm.
 */

#ifndef CONTROLADOR_H_
#define CONTROLADOR_H_

#include <Arduino.h>
#include <stdint.h>
#include <math.h>
#include "motores.h"
#include "sensores.h"
#include "Navigation.h"

// =============================================================================
// DEFINIÇÃO DOS PINOS DOS ENCODERS QUADRATURA NO ESP32
// Ajuste conforme a pinagem física dos encoders dos motores U3 e U6
// =============================================================================
#define ENCODER_A_E     18  // Encoder do motor esquerdo U3 (Sinal A)
#define ENCODER_B_E     19  // Encoder do motor esquerdo U3 (Sinal B)
#define ENCODER_A_D     16  // Encoder do motor direito U6  (Sinal A)
#define ENCODER_B_D     17  // Encoder do motor direito U6  (Sinal B)

// =============================================================================
// CONSTANTES FÍSICAS E DE CONTROLE DO ROBÔ
// =============================================================================
// Ganhos do controlador PD de velocidade
#define KP              5       // Ganho Proporcional
#define KD              10      // Ganho Derivativo

// Tipos de movimento
#define TRANSLACIONAL   0
#define ROTACIONAL      1

// Parâmetros mecânicos e de odometria especificados para o projeto:
// - Encoders: ~580 a 600 pulsos por revolução (valor médio nominal: 590 PPR)
// - Diâmetro das rodas: 44 mm (circunferência = 44 * PI = 138,23 mm)
// - Bitola entre rodas: ~9 cm (90 mm)
// - Célula do labirinto: 16,5 cm (165 mm)
#define ENCODER_PPR     590     // Resolução do Encoder [pulsos/volta]
#define DIAMETRO_RODA   44      // Diâmetro da roda [mm]
#define RODA_RODA       90      // Distância entre as rodas (bitola) [mm]
#define CNT_POR_1000MM  4268    // = (1000 * 590) / (44 * PI)
#define CNT_POR_360DEG  1207    // = (PI * RODA_RODA * CNT_POR_1000MM) / 1000

// Parâmetros temporais e de velocidade
#define TS              10      // Período de amostragem do loop de controle [ms]
#define K_SENSORES      10      // Peso de correção do alinhamento lateral no PID
#define SPEED_RETA      250     // Velocidade linear nominal nas retas [mm/s]
#define SPEED_CURVA     200     // Velocidade nominal nas curvas de 90° [mm/s]

#define CELULA          165     // Tamanho da célula do labirinto [mm] (16,5 cm)

// =============================================================================
// MACROS DE CONVERSÃO DE UNIDADES
// =============================================================================
#define COUNTS_TO_MM(cnt)       (((cnt) * 1000) / CNT_POR_1000MM)
#define COUNTS_TO_DEG(cnt)      (((cnt) * 360) / CNT_POR_360DEG)
#define DIST_TO_COUNTS(mm)      (((mm) * CNT_POR_1000MM) / 1000)
#define SPEED_TO_COUNTS(speed)  ((CNT_POR_1000MM * (speed) * TS) / 1000000)
#define RAD_S_TO_COUNTS(rad_s)  (SPEED_TO_COUNTS((rad_s) * RODA_RODA))

// =============================================================================
// PROTÓTIPOS DAS FUNÇÕES
// =============================================================================
void initEncoders(void);
int32_t lerEncoderEsquerdaFisico(void);
int32_t lerEncoderDireitaFisico(void);
void resetEncodersFisicos(void);

void controleVelocidade(void);
void setMovimento(int32_t raio, int32_t dist_ou_angulo, int32_t speed);
void updateEncoders(void);
void controladorPID(void);
bool isFinalMovimento(void);
void resetControlador(void);
void frente(int32_t mm);
void curvaPivot(int16_t graus);
void curva(int16_t graus);

// Variáveis Externas
extern volatile int32_t distancia_mm;
extern volatile int32_t distancia_deg;

#endif /* CONTROLADOR_H_ */
