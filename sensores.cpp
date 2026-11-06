/**
 * @file sensores.cpp
 * @brief Implementação da leitura dos 4 sensores FC-51 no ESP32 e cálculo de erro
 *        de alinhamento lateral respeitando a frente virtual (`inverted_front`).
 */

#include "sensores.h"

// Variáveis externas com as leituras atuais já espelhadas pela Frente Virtual
int16_t sensor_frontal_virtual  = 0;
int16_t sensor_traseiro_virtual = 0;
int16_t sensor_esquerdo_virtual = 0;
int16_t sensor_direito_virtual  = 0;

static VirtualSensors_t s_last_virtual_sensors;

uint8_t getSensoresParede(void)
{
    uint8_t mask = Navigation_ReadVirtualSensors(&s_last_virtual_sensors);

    sensor_frontal_virtual  = (int16_t)s_last_virtual_sensors.raw_front;
    sensor_traseiro_virtual = (int16_t)s_last_virtual_sensors.raw_rear;
    sensor_esquerdo_virtual = (int16_t)s_last_virtual_sensors.raw_left;
    sensor_direito_virtual  = (int16_t)s_last_virtual_sensors.raw_right;

    return mask;
}

int16_t getErroSensores(void)
{
    int16_t erro = 0;
    bool inverted = Navigation_IsFrontInverted();

    // Seleciona a referência nominal do centro para cada lateral conforme a frente virtual
    int16_t ref_esq = inverted ? CENTRO_LATERAL_DIR : CENTRO_LATERAL_ESQ;
    int16_t ref_dir = inverted ? CENTRO_LATERAL_ESQ : CENTRO_LATERAL_DIR;

    // Alinhamento lateral quando não há parede bloqueando a frente virtual
    if (!s_last_virtual_sensors.wall_front)
    {
        if (s_last_virtual_sensors.wall_left && !s_last_virtual_sensors.wall_right)
        {
            erro = sensor_esquerdo_virtual - ref_esq;
        }
        else if (s_last_virtual_sensors.wall_right && !s_last_virtual_sensors.wall_left)
        {
            erro = ref_dir - sensor_direito_virtual;
        }
        else if (s_last_virtual_sensors.wall_left && s_last_virtual_sensors.wall_right)
        {
            erro = (sensor_esquerdo_virtual - ref_esq) - (sensor_direito_virtual - ref_dir);
        }
    }

    return erro;
}
