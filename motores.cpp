/**
 * @file motores.cpp
 * @brief Implementação do acionamento de motores via MX1508 no ESP32 e abstração
 *        de inversão bidirecional (`inverted_front`).
 */

#include "motores.h"

void initMotores(void)
{
    pinMode(IN1_E, OUTPUT);
    pinMode(IN2_E, OUTPUT);
    pinMode(IN1_D, OUTPUT);
    pinMode(IN2_D, OUTPUT);

    digitalWrite(IN1_E, LOW);
    digitalWrite(IN2_E, LOW);
    digitalWrite(IN1_D, LOW);
    digitalWrite(IN2_D, LOW);
}

void setMotores(int16_t pwm_esquerda, int16_t pwm_direita)
{
    // Saturação em [-255, 255]
    if (pwm_esquerda > PWM_MAX)  pwm_esquerda = PWM_MAX;
    if (pwm_esquerda < -PWM_MAX) pwm_esquerda = -PWM_MAX;
    if (pwm_direita > PWM_MAX)   pwm_direita  = PWM_MAX;
    if (pwm_direita < -PWM_MAX)  pwm_direita  = -PWM_MAX;

    // Acionamento do Motor Esquerdo Físico (U3) via MX1508
    if (pwm_esquerda >= 0)
    {
        analogWrite(IN1_E, pwm_esquerda);
        analogWrite(IN2_E, 0);
    }
    else
    {
        analogWrite(IN1_E, 0);
        analogWrite(IN2_E, -pwm_esquerda);
    }

    // Acionamento do Motor Direito Físico (U6) via MX1508
    if (pwm_direita >= 0)
    {
        analogWrite(IN1_D, pwm_direita);
        analogWrite(IN2_D, 0);
    }
    else
    {
        analogWrite(IN1_D, 0);
        analogWrite(IN2_D, -pwm_direita);
    }
}

void setMotoresVirtual(int16_t pwm_virtual_esq, int16_t pwm_virtual_dir, bool inverted_front)
{
    if (!inverted_front)
    {
        // Frente Física ativa: mapeamento direto
        setMotores(pwm_virtual_esq, pwm_virtual_dir);
    }
    else
    {
        // Frente Virtual invertida (Traseira Física atua como Frente):
        // - O sentido de rotação linear é invertido (sinal negativo)
        // - A roda esquerda virtual corresponde fisicamente ao Motor Direito (U6)
        // - A roda direita virtual corresponde fisicamente ao Motor Esquerdo (U3)
        int16_t phys_esquerda = -pwm_virtual_dir;
        int16_t phys_direita  = -pwm_virtual_esq;
        setMotores(phys_esquerda, phys_direita);
    }
}
