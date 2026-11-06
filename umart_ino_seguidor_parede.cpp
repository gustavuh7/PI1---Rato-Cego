/**
 * @file umart_ino_seguidor_parede.cpp
 * @brief Loop principal do robô Micromouse Bidirecional no ESP32.
 *        Utiliza a arquitetura de Frente Virtual (`Navigation.h` / `Navigation.cpp`),
 *        mapeamento matricial 18x18 e algoritmo Seguidor de Parede Esquerda sem giro de 180°.
 */

#include "umart_ino_seguidor_parede.h"

// =============================================================================
// VARIÁVEIS PRIVADAS
// =============================================================================
static Ticker s_timerControle;
static volatile bool bRodarControle = false;
static bool bSinalizouVitoria = false;

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

    // 2. Seleciona o canto inicial:
    //    - Padrão: Canto Inferior Esquerdo (0, 0) -> Alvo em (M-1, M-1)
    //    - Se o botão SW_START estiver pressionado no boot: Canto Inferior Direito (M-1, 0) -> Alvo em (0, M-1)
    StartCorner_t canto_inicial = (digitalRead(SW_START) == LOW)
                                  ? START_BOTTOM_RIGHT
                                  : DEFAULT_START_CORNER;

    // 3. Inicializa camada de Frente Virtual, Sensores FC-51 (GPIOs 34, 35, 39, 36) e Matriz 18x18
    Navigation_Init(canto_inicial, DEFAULT_MAZE_DIMENSION, HEADING_NORTH);

    Serial.println("[MICROMOUSE ESP32] Sistema inicializado. Aguardando partida...");
    delay(2000);

    // 4. Inicia o Timer periódico de controle de velocidade e leitura de sensores (a cada TS = 10 ms)
    bRodarControle = true;
    s_timerControle.attach_ms(TS, callbackTimerControle);

    // 5. Desloca meia célula (82,5 mm) do centro da célula inicial até a fronteira da próxima célula
    frente(CELULA / 2);
}

// =============================================================================
// LOOP PRINCIPAL (MÁQUINA DE ESTADOS E MAPEAMENTO)
// =============================================================================
void loop(void)
{
    // Verifica condição de parada (chegada à célula diagonalmente oposta MxM)
    if (Navigation_IsMazeCompleted())
    {
        bRodarControle = false;
        resetControlador();
        setMotores(0, 0);

        if (!bSinalizouVitoria)
        {
            bSinalizouVitoria = true;
            const NavigationState_t* st = Navigation_GetState();
            Serial.printf("[VITORIA] Labirinto concluido na celula (%d, %d)!\n", st->x, st->y);
        }

        // Sinalização luminosa contínua indicando conclusão do labirinto
        digitalWrite(LED_STATUS, HIGH);
        delay(200);
        digitalWrite(LED_STATUS, LOW);
        delay(200);
        return;
    }

    // Executa um ciclo completo de navegação celular:
    // - Leitura dos 4 sensores FC-51 mapeados pela Frente Virtual (`inverted_front`)
    // - Atualização das paredes e visitação na matriz `maze[18][18]`
    // - Decisão Left-Hand Rule sem giro de 180° (inversão de frente virtual em becos sem saída)
    // - Acionamento dos motores e atualização de coordenadas `(x, y)` e `heading`
    Navigation_Step();
}

// =============================================================================
// CALLBACK DO TIMER DE CONTROLE (10 ms)
// =============================================================================
void callbackTimerControle(void)
{
    if (bRodarControle && !Navigation_IsMazeCompleted())
    {
        getSensoresParede();
        controleVelocidade();
    }
    else
    {
        setMotores(0, 0);
    }
}
