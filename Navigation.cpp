/**
 * @file Navigation.cpp
 * @brief Implementação da camada de Frente Virtual, Mapeamento Matricial 18x18
 *        e Máquina de Estados do Seguidor de Parede Esquerda (Left-Hand Rule)
 *        sem giro de 180° para ESP32.
 */

#include "Navigation.h"
#include "controlador.h"
#include "motores.h"
#include "sensores.h"

// =============================================================================
// ESTADO PRIVADO DO MÓDULO DE NAVEGAÇÃO
// =============================================================================
static NavigationState_t g_nav;

// Vetores de deslocamento cartesiano para (HEADING_NORTH, HEADING_EAST, HEADING_SOUTH, HEADING_WEST)
static const int8_t DX[4] = { 0,  1,  0, -1 };
static const int8_t DY[4] = { 1,  0, -1,  0 };

// Bit de parede global correspondente a cada Heading_t
static const uint8_t HEADING_TO_WALL_BIT[4] = {
    MAZE_WALL_NORTH, // HEADING_NORTH (0)
    MAZE_WALL_EAST,  // HEADING_EAST  (1)
    MAZE_WALL_SOUTH, // HEADING_SOUTH (2)
    MAZE_WALL_WEST   // HEADING_WEST  (3)
};

// =============================================================================
// FUNÇÕES AUXILIARES INTERNAS
// =============================================================================

/**
 * @brief Avalia se a leitura ADC de um sensor físico FC-51 indica presença de parede.
 *        Considera os valores de referência do ADC do ESP32 especificados para cada pino.
 */
static bool evaluatePhysicalWall(uint8_t gpio_pin, uint16_t adc_val)
{
    switch (gpio_pin)
    {
        case PIN_SENSOR_FRONT: // GPIO 34 (ADC ref: 27 a 34)
            return (adc_val >= ADC_REF_FRONT_MIN && adc_val <= TH_SENSOR_FRONT);

        case PIN_SENSOR_REAR:  // GPIO 35 (ADC ref: 2815 a 2820)
            return (adc_val >= ADC_REF_REAR_MIN && adc_val <= TH_SENSOR_REAR);

        case PIN_SENSOR_LEFT:  // GPIO 39 (ADC ref: 32 a 40)
            return (adc_val >= ADC_REF_LEFT_MIN && adc_val <= TH_SENSOR_LEFT);

        case PIN_SENSOR_RIGHT: // GPIO 36 (ADC ref: 45 a 52)
            return (adc_val >= ADC_REF_RIGHT_MIN && adc_val <= TH_SENSOR_RIGHT);

        default:
            return false;
    }
}

/**
 * @brief Registra uma parede global na célula `(cx, cy)` e espelha a parede oposta
 *        na célula adjacente correspondente para manter a consistência do mapa.
 */
static void setGlobalWall(int8_t cx, int8_t cy, Heading_t global_dir)
{
    if (cx < 0 || cx >= g_nav.maze_size || cy < 0 || cy >= g_nav.maze_size)
    {
        return;
    }

    // Registra a parede na célula atual
    g_nav.maze[cx][cy] |= HEADING_TO_WALL_BIT[global_dir];

    // Registra a parede complementar na célula vizinha
    int8_t nx = cx + DX[global_dir];
    int8_t ny = cy + DY[global_dir];
    if (nx >= 0 && nx < g_nav.maze_size && ny >= 0 && ny < g_nav.maze_size)
    {
        Heading_t opposite_dir = (Heading_t)((global_dir + 2) % 4);
        g_nav.maze[nx][ny] |= HEADING_TO_WALL_BIT[opposite_dir];
    }
}

/**
 * @brief Atualiza a coordenada `(x, y)` após o avanço de 1 célula na direção `heading`.
 */
static void advanceCoordinatesOneCell(void)
{
    int8_t next_x = g_nav.x + DX[g_nav.heading];
    int8_t next_y = g_nav.y + DY[g_nav.heading];

    if (next_x >= 0 && next_x < g_nav.maze_size)
    {
        g_nav.x = next_x;
    }
    if (next_y >= 0 && next_y < g_nav.maze_size)
    {
        g_nav.y = next_y;
    }

    g_nav.maze[g_nav.x][g_nav.y] |= MAZE_CELL_VISITED;
}

// =============================================================================
// IMPLEMENTAÇÃO DAS FUNÇÕES PÚBLICAS
// =============================================================================

void Navigation_Init(StartCorner_t corner, uint8_t effective_maze_size, Heading_t initial_heading)
{
    // Configura os pinos ADC1 do ESP32 (somente entrada)
    pinMode(PIN_SENSOR_FRONT, INPUT);
    pinMode(PIN_SENSOR_REAR,  INPUT);
    pinMode(PIN_SENSOR_LEFT,  INPUT);
    pinMode(PIN_SENSOR_RIGHT, INPUT);

    // Resolução padrão de 12 bits (0..4095) compatível com a referência até 2820
    analogReadResolution(12);

    if (effective_maze_size == 0 || effective_maze_size > MAZE_MAX_SIZE)
    {
        effective_maze_size = MAZE_MAX_SIZE;
    }

    g_nav.maze_size      = effective_maze_size;
    g_nav.heading        = initial_heading;
    g_nav.inverted_front = false; // Inicia com a frente física ativa (GPIO 34)
    g_nav.maze_completed = false;

    // Limpa a matriz 18x18 e configura as bordas externas do labirinto
    for (uint8_t x = 0; x < MAZE_MAX_SIZE; x++)
    {
        for (uint8_t y = 0; y < MAZE_MAX_SIZE; y++)
        {
            g_nav.maze[x][y] = 0;

            if (x == 0)                       g_nav.maze[x][y] |= MAZE_WALL_WEST;
            if (x == effective_maze_size - 1) g_nav.maze[x][y] |= MAZE_WALL_EAST;
            if (y == 0)                       g_nav.maze[x][y] |= MAZE_WALL_SOUTH;
            if (y == effective_maze_size - 1) g_nav.maze[x][y] |= MAZE_WALL_NORTH;
        }
    }

    // Define posição inicial e célula alvo diagonalmente oposta (MxM)
    if (corner == START_BOTTOM_LEFT)
    {
        g_nav.x        = 0;
        g_nav.y        = 0;
        g_nav.target_x = effective_maze_size - 1;
        g_nav.target_y = effective_maze_size - 1;
    }
    else // START_BOTTOM_RIGHT
    {
        g_nav.x        = effective_maze_size - 1;
        g_nav.y        = 0;
        g_nav.target_x = 0;
        g_nav.target_y = effective_maze_size - 1;
    }

    // Marca célula inicial como visitada
    g_nav.maze[g_nav.x][g_nav.y] |= MAZE_CELL_VISITED;
}

uint8_t Navigation_ReadVirtualSensors(VirtualSensors_t* out_sensors)
{
    // 1. Leitura física direta dos canais ADC do ESP32
    uint16_t phys_front = analogRead(PIN_SENSOR_FRONT); // GPIO 34
    uint16_t phys_rear  = analogRead(PIN_SENSOR_REAR);  // GPIO 35
    uint16_t phys_left  = analogRead(PIN_SENSOR_LEFT);  // GPIO 39
    uint16_t phys_right = analogRead(PIN_SENSOR_RIGHT); // GPIO 36

    bool w_phys_front = evaluatePhysicalWall(PIN_SENSOR_FRONT, phys_front);
    bool w_phys_rear  = evaluatePhysicalWall(PIN_SENSOR_REAR,  phys_rear);
    bool w_phys_left  = evaluatePhysicalWall(PIN_SENSOR_LEFT,  phys_left);
    bool w_phys_right = evaluatePhysicalWall(PIN_SENSOR_RIGHT, phys_right);

    VirtualSensors_t v;

    // 2. Translação para os sensores virtuais conforme o estado de `inverted_front`
    if (!g_nav.inverted_front)
    {
        // Frente Física ativa:
        // Frente = GPIO 34, Traseira = GPIO 35, Esquerda = GPIO 39, Direita = GPIO 36
        v.raw_front  = phys_front;
        v.raw_rear   = phys_rear;
        v.raw_left   = phys_left;
        v.raw_right  = phys_right;

        v.wall_front = w_phys_front;
        v.wall_rear  = w_phys_rear;
        v.wall_left  = w_phys_left;
        v.wall_right = w_phys_right;
    }
    else
    {
        // Frente Virtual Invertida (Traseira Física atua como Frente):
        // Frente = GPIO 35, Traseira = GPIO 34, Esquerda = GPIO 36 (espelhado), Direita = GPIO 39 (espelhado)
        v.raw_front  = phys_rear;
        v.raw_rear   = phys_front;
        v.raw_left   = phys_right;
        v.raw_right  = phys_left;

        v.wall_front = w_phys_rear;
        v.wall_rear  = w_phys_front;
        v.wall_left  = w_phys_right;
        v.wall_right = w_phys_left;
    }

    // 3. Monta a máscara de bits relativa (Bit 2: Esquerda, Bit 1: Frente, Bit 0: Direita)
    v.wall_mask = REL_WALL_NONE;
    if (v.wall_left)  v.wall_mask |= REL_WALL_LEFT;
    if (v.wall_front) v.wall_mask |= REL_WALL_FRONT;
    if (v.wall_right) v.wall_mask |= REL_WALL_RIGHT;

    if (out_sensors != NULL)
    {
        *out_sensors = v;
    }

    return v.wall_mask;
}

void Navigation_UpdateMazeMap(const VirtualSensors_t* v_sensors)
{
    if (v_sensors == NULL)
    {
        return;
    }

    // Marca a célula atual como visitada
    g_nav.maze[g_nav.x][g_nav.y] |= MAZE_CELL_VISITED;

    // Converte as direções relativas (Frente, Direita, Traseira, Esquerda) para globais (N, L, S, O)
    Heading_t dir_front = g_nav.heading;
    Heading_t dir_right = (Heading_t)((g_nav.heading + 1) % 4);
    Heading_t dir_rear  = (Heading_t)((g_nav.heading + 2) % 4);
    Heading_t dir_left  = (Heading_t)((g_nav.heading + 3) % 4);

    if (v_sensors->wall_front) setGlobalWall(g_nav.x, g_nav.y, dir_front);
    if (v_sensors->wall_right) setGlobalWall(g_nav.x, g_nav.y, dir_right);
    if (v_sensors->wall_rear)  setGlobalWall(g_nav.x, g_nav.y, dir_rear);
    if (v_sensors->wall_left)  setGlobalWall(g_nav.x, g_nav.y, dir_left);
}

RobotAction_t Navigation_DecideNextAction(const VirtualSensors_t* v_sensors)
{
    if (v_sensors == NULL)
    {
        return ACTION_IDLE;
    }

    // Verifica condição de vitória antes de decidir o próximo movimento
    if (g_nav.x == g_nav.target_x && g_nav.y == g_nav.target_y)
    {
        g_nav.maze_completed = true;
        return ACTION_STOP_VICTORY;
    }

    // Máquina de Estados - Seguidor de Parede Esquerda (Left-Hand Rule)
    // Prioridade: 1º Esquerda livre -> 2º Frente livre -> 3º Direita livre -> 4º Beco sem saída (Inversão Virtual)
    switch (v_sensors->wall_mask)
    {
        case 0b000: // Sem paredes
        case 0b001: // Apenas parede à direita
        case 0b010: // Apenas parede frontal
        case 0b011: // Paredes frontal e à direita (esquerda livre)
            // Atualiza orientação global (-90° / +270° = Esquerda)
            g_nav.heading = (Heading_t)((g_nav.heading + 3) % 4);
            return ACTION_TURN_LEFT;

        case 0b100: // Apenas parede à esquerda (frente livre)
        case 0b101: // Paredes à esquerda e à direita - corredor (frente livre)
            // Mantém a orientação global atual
            return ACTION_FORWARD;

        case 0b110: // Paredes à esquerda e à frente (direita livre)
            // Atualiza orientação global (+90° = Direita)
            g_nav.heading = (Heading_t)((g_nav.heading + 1) % 4);
            return ACTION_TURN_RIGHT;

        case 0b111: // Beco sem saída (paredes à esquerda, frente e direita na visão virtual)
        default:
            // RESTRIÇÃO FÍSICA CRÍTICA:
            // O robô NÃO gira 180° sobre o próprio eixo.
            // Alterna a frente virtual e inverte a orientação global em 180°.
            g_nav.inverted_front = !g_nav.inverted_front;
            g_nav.heading        = (Heading_t)((g_nav.heading + 2) % 4);
            return ACTION_INVERT_AND_EXIT;
    }
}

void Navigation_ExecuteAction(RobotAction_t action)
{
    // A camada de controle de motores e encoders lê `Navigation_IsFrontInverted()`
    // através de `setMotoresVirtual` / `frente` / `curva`, garantindo que:
    // - Quando `inverted_front == true`, o avanço linear inverta o sentido de rotação
    //   dos motores e as curvas de 90° espelhem a roda de tração correspondente.
    switch (action)
    {
        case ACTION_FORWARD:
            // Avança 1 célula (165 mm) mantendo a frente virtual atual
            frente(CELL_SIZE_MM);
            advanceCoordinatesOneCell();
            break;

        case ACTION_TURN_LEFT:
            // Curva de +90° (Esquerda relativa à frente virtual) e avança até a fronteira
            curva(90);
            advanceCoordinatesOneCell();
            break;

        case ACTION_TURN_RIGHT:
            // Curva de -90° (Direita relativa à frente virtual) e avança até a fronteira
            curva(-90);
            advanceCoordinatesOneCell();
            break;

        case ACTION_INVERT_AND_EXIT:
            // Em beco sem saída, `inverted_front` já foi alternado em Navigation_DecideNextAction().
            // Para brevemente os motores para evitar pico de corrente na reversão e avança
            // linearmente na nova frente virtual para sair da célula sem girar 180°.
            resetControlador();
            setMotores(0, 0);
            delay(100);
            frente(CELL_SIZE_MM);
            advanceCoordinatesOneCell();
            break;

        case ACTION_STOP_VICTORY:
        case ACTION_IDLE:
        default:
            resetControlador();
            setMotores(0, 0);
            break;
    }
}

void Navigation_Step(void)
{
    if (g_nav.maze_completed)
    {
        setMotores(0, 0);
        return;
    }

    // 1. Lê sensores já mapeados para a frente virtual ativa
    VirtualSensors_t v_sensors;
    Navigation_ReadVirtualSensors(&v_sensors);

    // 2. Registra as paredes detectadas na matriz maze[18][18]
    Navigation_UpdateMazeMap(&v_sensors);

    // 3. Verifica se atingiu a célula objetivo (target_x, target_y)
    if (g_nav.x == g_nav.target_x && g_nav.y == g_nav.target_y)
    {
        g_nav.maze_completed = true;
        Navigation_ExecuteAction(ACTION_STOP_VICTORY);
        return;
    }

    // 4. Decide o próximo movimento (Left-Hand Rule sem giro de 180°) e aciona os motores
    RobotAction_t next_action = Navigation_DecideNextAction(&v_sensors);
    Navigation_ExecuteAction(next_action);

    // 5. Verifica novamente após avançar para a nova célula se chegou ao destino
    if (g_nav.x == g_nav.target_x && g_nav.y == g_nav.target_y)
    {
        g_nav.maze_completed = true;
        Navigation_ExecuteAction(ACTION_STOP_VICTORY);
    }
}

bool Navigation_IsFrontInverted(void)
{
    return g_nav.inverted_front;
}

bool Navigation_IsMazeCompleted(void)
{
    return g_nav.maze_completed;
}

const NavigationState_t* Navigation_GetState(void)
{
    return &g_nav;
}

void Navigation_PrintTelemetry(void)
{
    static const char* HEADING_NAMES[4] = { "NORTE", "LESTE", "SUL", "OESTE" };
    VirtualSensors_t v;
    Navigation_ReadVirtualSensors(&v);

    Serial.printf("[NAV] Pos:(%2d,%2d) Alvo:(%2d,%2d) | Dir:%-5s | FrenteInv:%s | "
                  "Virt[E:%4u(%d) F:%4u(%d) D:%4u(%d) T:%4u(%d)] Mask:0b%d%d%d\n",
                  g_nav.x, g_nav.y,
                  g_nav.target_x, g_nav.target_y,
                  HEADING_NAMES[g_nav.heading % 4],
                  g_nav.inverted_front ? "SIM (TRASEIRA)" : "NAO (FRONTAL)",
                  v.raw_left,  v.wall_left  ? 1 : 0,
                  v.raw_front, v.wall_front ? 1 : 0,
                  v.raw_right, v.wall_right ? 1 : 0,
                  v.raw_rear,  v.wall_rear  ? 1 : 0,
                  (v.wall_mask >> 2) & 1,
                  (v.wall_mask >> 1) & 1,
                  v.wall_mask & 1);
}

