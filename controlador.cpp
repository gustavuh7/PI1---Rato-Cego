/**
 * @file controlador.cpp
 * @brief Implementação do controle de velocidade PID, leitura de encoders quadratura
 *        por interrupção no ESP32 e abstração cinemática bidirecional (`inverted_front`).
 */

#include "controlador.h"

// =============================================================================
// VARIÁVEIS GLOBAIS E DE ESTADO DO CONTROLADOR
// =============================================================================
volatile int32_t distancia_mm  = 0;
volatile int32_t distancia_deg = 0;

// Contadores brutos dos encoders físicos (atualizados via ISR no ESP32)
static volatile int32_t s_enc_esq_cnt = 0;
static volatile int32_t s_enc_dir_cnt = 0;

// Variáveis internas do controlador PID
static int8_t  tipo_movimento = TRANSLACIONAL;
static int32_t setpoint_dist = 0;
static int32_t setpoint_speed_esquerda_cnt = 0;
static int32_t setpoint_speed_direita_cnt  = 0;
static int32_t encoder_esquerda_delta = 0;
static int32_t encoder_direita_delta  = 0;
static int32_t encoder_esquerda_anterior_cnt = 0;
static int32_t encoder_direita_anterior_cnt  = 0;
static int32_t pid_esquerda_out = 0;
static int32_t pid_direita_out  = 0;
static int32_t erro_esquerda_anterior = 0;
static int32_t erro_direita_anterior  = 0;
static bool    bUsarSensores = false;

// =============================================================================
// INTERRUPÇÕES DOS ENCODERS QUADRATURA (ESP32 IRAM_ATTR)
// =============================================================================
static void IRAM_ATTR isrEncoderEsquerdaA(void)
{
    bool a = digitalRead(ENCODER_A_E);
    bool b = digitalRead(ENCODER_B_E);
    if (a == b) s_enc_esq_cnt++;
    else        s_enc_esq_cnt--;
}

static void IRAM_ATTR isrEncoderDireitaA(void)
{
    bool a = digitalRead(ENCODER_A_D);
    bool b = digitalRead(ENCODER_B_D);
    if (a != b) s_enc_dir_cnt++;
    else        s_enc_dir_cnt--;
}

void initEncoders(void)
{
    pinMode(ENCODER_A_E, INPUT_PULLUP);
    pinMode(ENCODER_B_E, INPUT_PULLUP);
    pinMode(ENCODER_A_D, INPUT_PULLUP);
    pinMode(ENCODER_B_D, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_E), isrEncoderEsquerdaA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_D), isrEncoderDireitaA,  CHANGE);
}

int32_t lerEncoderEsquerdaFisico(void)
{
    return s_enc_esq_cnt;
}

int32_t lerEncoderDireitaFisico(void)
{
    return s_enc_dir_cnt;
}

void resetEncodersFisicos(void)
{
    noInterrupts();
    s_enc_esq_cnt = 0;
    s_enc_dir_cnt = 0;
    interrupts();
}

// =============================================================================
// MALHA DE CONTROLE DE VELOCIDADE E ODOMETRIA VIRTUAL
// =============================================================================

void controleVelocidade(void)
{
    updateEncoders();
    controladorPID();
}

void setMovimento(int32_t raio, int32_t dist_ou_angulo, int32_t speed)
{
    if (raio == 0)
    {
        tipo_movimento = TRANSLACIONAL;

        setpoint_speed_esquerda_cnt = SPEED_TO_COUNTS(speed);
        setpoint_speed_direita_cnt  = setpoint_speed_esquerda_cnt;

        // Pré-carrega o integrador PD com o PWM base de 75% duty cycle na direção do movimento
        int16_t pwm_init = (speed >= 0) ? PWM_DUTY_75_PERCENT : -PWM_DUTY_75_PERCENT;
        if (speed == 0) pwm_init = 0;
        pid_esquerda_out = pwm_init;
        pid_direita_out  = pwm_init;
    }
    else
    {
        tipo_movimento = ROTACIONAL;

        float speedX_mm_s  = (float)speed;
        float speedW_rad_s = speedX_mm_s / (float)raio;

        int32_t speedX_cnt = SPEED_TO_COUNTS(abs(speed));
        int32_t speedW_cnt = RAD_S_TO_COUNTS(speedW_rad_s);

        if (raio < RODA_RODA)
        {
            speedX_cnt -= (abs(speedW_cnt) / 2);
            speedW_cnt /= 2;
        }

        setpoint_speed_esquerda_cnt = speedX_cnt - speedW_cnt;
        setpoint_speed_direita_cnt  = speedX_cnt + speedW_cnt;

        // Pré-carrega o PWM inicial de 75% nas rodas para o giro de 90°
        pid_esquerda_out = (setpoint_speed_esquerda_cnt >= 0) ? PWM_DUTY_75_PERCENT : -PWM_DUTY_75_PERCENT;
        pid_direita_out  = (setpoint_speed_direita_cnt  >= 0) ? PWM_DUTY_75_PERCENT : -PWM_DUTY_75_PERCENT;
    }

    setpoint_dist = dist_ou_angulo;
}

void updateEncoders(void)
{
    int32_t phys_esq = lerEncoderEsquerdaFisico();
    int32_t phys_dir = lerEncoderDireitaFisico();

    int32_t virt_esq_cnt;
    int32_t virt_dir_cnt;

    // Espelha e inverte a leitura dos encoders quando `inverted_front == true`
    if (!Navigation_IsFrontInverted())
    {
        virt_esq_cnt = phys_esq;
        virt_dir_cnt = phys_dir;
    }
    else
    {
        virt_esq_cnt = -phys_dir;
        virt_dir_cnt = -phys_esq;
    }

    encoder_esquerda_delta = virt_esq_cnt - encoder_esquerda_anterior_cnt;
    encoder_direita_delta  = virt_dir_cnt - encoder_direita_anterior_cnt;

    encoder_esquerda_anterior_cnt = virt_esq_cnt;
    encoder_direita_anterior_cnt  = virt_dir_cnt;

    distancia_mm  = COUNTS_TO_MM((virt_dir_cnt + virt_esq_cnt) / 2);
    distancia_deg = COUNTS_TO_DEG((virt_dir_cnt - virt_esq_cnt) / 2);
}

void controladorPID(void)
{
    int32_t erro_esquerda = 0;
    int32_t erro_direita  = 0;

    if (bUsarSensores)
    {
        int32_t sensor_feedback = (int32_t)getErroSensores() / K_SENSORES;
        erro_esquerda += sensor_feedback;
        erro_direita  -= sensor_feedback;
    }

    erro_esquerda += (setpoint_speed_esquerda_cnt - encoder_esquerda_delta);
    erro_direita  += (setpoint_speed_direita_cnt  - encoder_direita_delta);

    pid_esquerda_out += (KP * erro_esquerda) + (KD * (erro_esquerda - erro_esquerda_anterior));
    pid_direita_out  += (KP * erro_direita)  + (KD * (erro_direita  - erro_direita_anterior));

    // Saturação do acumulador PID
    if (pid_esquerda_out > PWM_MAX)  pid_esquerda_out = PWM_MAX;
    if (pid_esquerda_out < -PWM_MAX) pid_esquerda_out = -PWM_MAX;
    if (pid_direita_out > PWM_MAX)   pid_direita_out  = PWM_MAX;
    if (pid_direita_out < -PWM_MAX)  pid_direita_out  = -PWM_MAX;

    erro_esquerda_anterior = erro_esquerda;
    erro_direita_anterior  = erro_direita;

    // Aciona os motores traduzindo pela camada de frente virtual (`inverted_front`)
    setMotoresVirtual((int16_t)pid_esquerda_out, (int16_t)pid_direita_out, Navigation_IsFrontInverted());
}

bool isFinalMovimento(void)
{
    // Interrompe imediatamente qualquer movimento se o usuário clicou em PARAR ou concluiu o labirinto
    if (!Navigation_IsRunning() || Navigation_IsMazeCompleted())
    {
        setMotores(0, 0);
        return true;
    }

    if (tipo_movimento == TRANSLACIONAL && abs(distancia_mm) >= setpoint_dist)
    {
        return true;
    }
    else if (tipo_movimento == ROTACIONAL && abs(distancia_deg) >= setpoint_dist)
    {
        return true;
    }

    return false;
}

void resetControlador(void)
{
    resetEncodersFisicos();

    encoder_esquerda_anterior_cnt = 0;
    encoder_direita_anterior_cnt  = 0;
    encoder_esquerda_delta        = 0;
    encoder_direita_delta         = 0;

    distancia_mm  = 0;
    distancia_deg = 0;

    pid_esquerda_out       = 0;
    pid_direita_out        = 0;
    erro_esquerda_anterior = 0;
    erro_direita_anterior  = 0;

    setpoint_speed_esquerda_cnt = 0;
    setpoint_speed_direita_cnt  = 0;
}

void frente(int32_t mm)
{
    if (mm != 0)
    {
        bUsarSensores = true;
        resetControlador();
        setMovimento(0, mm, SPEED_RETA);

        do
        {
            delay(1);
        } while (!isFinalMovimento());
    }
    else
    {
        bUsarSensores = true;
        setMovimento(0, 0, SPEED_RETA);
        delay(1);
    }
}

void curvaPivot(int16_t graus)
{
    bUsarSensores = false;
    resetControlador();
    setMotores(0, 0);
    delay(100);

    // Nota: Curvas permitidas são apenas de +90° (esquerda) ou -90° (direita).
    // O giro de 180° foi eliminado pela arquitetura de Frente Virtual.
    if (graus > 0)
    {
        setMovimento(RODA_RODA / 2, graus, SPEED_CURVA);
    }
    else
    {
        setMovimento(RODA_RODA / 2, -graus, -SPEED_CURVA);
    }

    do
    {
        delay(1);
    } while (!isFinalMovimento());
}

void curva(int16_t graus)
{
    // Desloca meia célula até o centro da célula atual (na direção da frente virtual)
    frente(CELULA / 2);
    // Realiza apenas curva de 90° (+90° Esquerda ou -90° Direita)
    curvaPivot(graus);
    // Avança meia célula até a fronteira da próxima célula
    frente(CELULA / 2);
}
