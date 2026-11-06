/**
 * @file PI1---Rato-Cego.ino
 * @brief Sketch principal (Arduino IDE) do robô Micromouse Bidirecional no ESP32.
 *        Utiliza a arquitetura de Frente Virtual (`Navigation.h` / `Navigation.cpp`),
 *        mapeamento matricial 18x18, algoritmo Seguidor de Parede Esquerda sem giro de 180°
 *        e Interface Visual Web (`WebInterface.h` / `WebInterface.cpp`).
 *
 * Modos de Início e Parada no Labirinto:
 * 1. INTERFACE VISUAL WI-FI (Sem tocar no robô):
 *    - Conecte o celular ou PC no Wi-Fi: "RatoCego-ESP32" (Senha: "12345678")
 *    - Abra no navegador: http://192.168.4.1
 *    - Use os botões "INICIAR ROBÔ", "PARAR AGORA" e "ZERAR MAPA", e acompanhe o mapa ao vivo.
 * 2. BOTÃO FÍSICO BOOT DA ESP32 (GPIO 0):
 *    - Robô parado: Pressione BOOT -> O LED pisca por 2 segundos (tempo para tirar a mão) e o robô inicia.
 *    - Robô em movimento: Pressione BOOT -> Parada imediata dos motores.
 */

#include "umart_ino_seguidor_parede.h"

// =============================================================================
// VARIÁVEIS PRIVADAS
// =============================================================================
static Ticker s_timerControle;
static bool bSinalizouVitoria = false;
static volatile bool s_btnBootPressed = false;
static volatile uint32_t s_lastBtnIrqMs = 0;

// Interrupção do botão físico BOOT (GPIO 0) para Parada Imediata ou Partida
static void IRAM_ATTR isrBotaoBoot(void)
{
    uint32_t now = millis();
    if ((now - s_lastBtnIrqMs) > 300)
    {
        s_lastBtnIrqMs = now;
        if (Navigation_IsRunning())
        {
            // Se já estiver andando, para imediatamente!
            Navigation_Stop();
        }
        else
        {
            // Se estiver parado, agenda partida com contagem regressiva de 2s
            s_btnBootPressed = true;
        }
    }
}

// =============================================================================
// INICIALIZAÇÃO DO SISTEMA (SETUP)
// =============================================================================
void setup(void)
{
    Serial.begin(115200);

    pinMode(LED_STATUS, OUTPUT);
    pinMode(SW_START, INPUT_PULLUP);
    digitalWrite(LED_STATUS, LOW);

    // 1. Inicializa driver MX1508 (motores U3 e U6) e encoders em quadratura
    initMotores();
    initEncoders();

    // 2. Inicializa camada de Frente Virtual, Sensores FC-51 (GPIOs 34, 35, 39, 36) e Matriz 18x18
    //    Nota: O robô inicia no estado PARADO (is_running = false) aguardando comando!
    Navigation_Init(DEFAULT_START_CORNER, DEFAULT_MAZE_DIMENSION, HEADING_NORTH);

    // 3. Inicializa o Painel Visual Web via Wi-Fi Access Point (http://192.168.4.1)
    WebInterface_Init();

    // 4. Configura interrupção do botão físico BOOT (GPIO 0)
    attachInterrupt(digitalPinToInterrupt(SW_START), isrBotaoBoot, FALLING);

    // 5. Inicia o Timer periódico de controle de velocidade e leitura de sensores (a cada TS = 10 ms)
    s_timerControle.attach_ms(TS, callbackTimerControle);

    Serial.println("[MICROMOUSE ESP32] Pronto e aguardando comando de INICIO (Web UI http://192.168.4.1 ou Botao BOOT).");
}

// =============================================================================
// LOOP PRINCIPAL (MÁQUINA DE ESTADOS E MAPEAMENTO)
// =============================================================================
void loop(void)
{
    // 1. Verifica condição de parada por Vitória (chegada à célula diagonalmente oposta MxM)
    if (Navigation_IsMazeCompleted())
    {
        Navigation_Stop();

        if (!bSinalizouVitoria)
        {
            bSinalizouVitoria = true;
            const NavigationState_t* st = Navigation_GetState();
            Serial.printf("[VITORIA] Labirinto concluido na celula (%d, %d)!\n", st->x, st->y);
            Navigation_PrintMazeASCII();
        }

        // Sinalização luminosa contínua indicando conclusão do labirinto
        digitalWrite(LED_STATUS, HIGH);
        delay(200);
        digitalWrite(LED_STATUS, LOW);
        delay(200);
        return;
    }

    bSinalizouVitoria = false;

    // 2. Se o usuário apertou o botão físico BOOT (GPIO 0) com o robô parado:
    //    Aguarda 2 segundos piscando o LED (para dar tempo de afastar a mão) e inicia!
    if (s_btnBootPressed)
    {
        s_btnBootPressed = false;
        Serial.println("[BOTAO BOOT] Iniciando em 2 segundos...");
        for (int i = 0; i < 4; i++)
        {
            digitalWrite(LED_STATUS, HIGH);
            delay(250);
            digitalWrite(LED_STATUS, LOW);
            delay(250);
        }
        Navigation_Start();
    }

    // 3. Se estiver aguardando início (parado), mantém motores desligados
    if (!Navigation_IsRunning())
    {
        setMotores(0, 0);
        digitalWrite(LED_STATUS, LOW);
        delay(50);
        return;
    }

    // 4. Robô em operação no labirinto: LED aceso e execução passo a passo celular
    digitalWrite(LED_STATUS, HIGH);
    Navigation_PrintTelemetry();
    Navigation_Step();
}

// =============================================================================
// CALLBACK DO TIMER DE CONTROLE (10 ms)
// =============================================================================
void callbackTimerControle(void)
{
    if (Navigation_IsRunning() && !Navigation_IsMazeCompleted())
    {
        getSensoresParede();
        controleVelocidade();
    }
    else
    {
        setMotores(0, 0);
    }
}
