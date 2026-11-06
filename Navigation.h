/**
 * @file Navigation.h
 * @brief Camada de abstração de frente virtual, mapeamento matricial (18x18)
 *        e máquina de estados do Seguidor de Parede Esquerda (Left-Hand Rule)
 *        para robô Micromouse bidirecional baseado em ESP32.
 */

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// CONSTANTES DO LABIRINTO E HARDWARE
// =============================================================================
#define MAZE_MAX_SIZE       18      // Dimensão máxima da matriz do labirinto (18x18)
#define CELL_SIZE_MM        165     // Dimensão da célula: 16,5 cm (165 mm)

// Pinos físicos dos sensores infravermelhos FC-51 (ADC1 do ESP32)
#define PIN_SENSOR_FRONT    34      // Sensor Frontal Físico  (ADC ref: 27 a 34)
#define PIN_SENSOR_REAR     35      // Sensor Traseiro Físico (ADC ref: 2815 a 2820)
#define PIN_SENSOR_LEFT     39      // Sensor Esquerdo Físico (ADC ref: 32 a 40)
#define PIN_SENSOR_RIGHT    36      // Sensor Direito Físico  (ADC ref: 45 a 52)

// Valores de referência ADC e tolerância para detecção de parede nos sensores FC-51
#define ADC_REF_FRONT_MIN   27
#define ADC_REF_FRONT_MAX   34
#define ADC_REF_REAR_MIN    2815
#define ADC_REF_REAR_MAX    2820
#define ADC_REF_LEFT_MIN    32
#define ADC_REF_LEFT_MAX    40
#define ADC_REF_RIGHT_MIN   45
#define ADC_REF_RIGHT_MAX   52

// Limiares (Thresholds) configuráveis para detecção de parede por canal físico
// Nota: Nos módulos FC-51 com saída analógica, a aproximação do obstáculo altera
// a tensão lida pelo ADC em torno da faixa de referência calibrada.
#define TH_SENSOR_FRONT     60
#define TH_SENSOR_REAR      2850
#define TH_SENSOR_LEFT      65
#define TH_SENSOR_RIGHT     80

// Máscaras de bits de paredes relativas (compatível com padrão uMaRT: E-F-D)
#define REL_WALL_NONE       0b000
#define REL_WALL_RIGHT      0b001
#define REL_WALL_FRONT      0b010
#define REL_WALL_LEFT       0b100
#define REL_WALL_DEAD_END   0b111

// Máscaras de bits para registro na matriz maze[18][18]
#define MAZE_WALL_NORTH     (1 << 0) // 0x01
#define MAZE_WALL_EAST      (1 << 1) // 0x02
#define MAZE_WALL_SOUTH     (1 << 2) // 0x04
#define MAZE_WALL_WEST      (1 << 3) // 0x08
#define MAZE_CELL_VISITED   (1 << 4) // 0x10

// =============================================================================
// ENUMERAÇÕES E ESTRUTURAS DE DADOS
// =============================================================================

/**
 * @brief Orientação global absoluta do robô no labirinto (Rosa dos Ventos).
 */
typedef enum : uint8_t {
    HEADING_NORTH = 0, // +Y
    HEADING_EAST  = 1, // +X
    HEADING_SOUTH = 2, // -Y
    HEADING_WEST  = 3  // -X
} Heading_t;

/**
 * @brief Comandos de ação de alto nível para abstração dos motores.
 */
typedef enum : uint8_t {
    ACTION_IDLE = 0,
    ACTION_FORWARD,         // Avança 1 célula na direção da frente virtual ativa
    ACTION_TURN_LEFT,       // Curva de 90° à esquerda (relativa à frente virtual) e avança
    ACTION_TURN_RIGHT,      // Curva de 90° à direita (relativa à frente virtual) e avança
    ACTION_INVERT_AND_EXIT, // Beco sem saída: inverte frente virtual sem giro de 180° e avança
    ACTION_STOP_VICTORY     // Para imediatamente os motores ao atingir o destino
} RobotAction_t;

/**
 * @brief Configuração da posição inicial do robô no labirinto.
 */
typedef enum : uint8_t {
    START_BOTTOM_LEFT = 0,  // Posição inicial (0, 0) -> Alvo em (M-1, M-1)
    START_BOTTOM_RIGHT = 1  // Posição inicial (M-1, 0) -> Alvo em (0, M-1)
} StartCorner_t;

/**
 * @brief Leituras brutas e booleanas mapeadas para a orientação virtual ativa.
 */
typedef struct {
    uint16_t raw_front;
    uint16_t raw_rear;
    uint16_t raw_left;
    uint16_t raw_right;
    bool wall_front;
    bool wall_rear;
    bool wall_left;
    bool wall_right;
    uint8_t wall_mask; // Formato binário: [Bit2: Esquerda | Bit1: Frente | Bit0: Direita]
} VirtualSensors_t;

/**
 * @brief Estrutura de estado completo de navegação e mapeamento do robô.
 */
typedef struct {
    int8_t x;                       // Coordenada X atual [0 .. maze_size-1]
    int8_t y;                       // Coordenada Y atual [0 .. maze_size-1]
    int8_t target_x;                // Coordenada X da célula alvo (diagonalmente oposta)
    int8_t target_y;                // Coordenada Y da célula alvo (diagonalmente oposta)
    uint8_t maze_size;              // Tamanho efetivo do grid (M <= 18)
    Heading_t heading;              // Orientação global absoluta (N, L, S, O)
    bool inverted_front;            // false = Frente Física (GPIO 34), true = Traseira Física (GPIO 35)
    bool maze_completed;            // Flag de vitória (labirinto concluído)
    uint8_t maze[MAZE_MAX_SIZE][MAZE_MAX_SIZE]; // Matriz de paredes e células visitadas
} NavigationState_t;

// =============================================================================
// PROTÓTIPOS DAS FUNÇÕES PÚBLICAS
// =============================================================================

/**
 * @brief Inicializa os pinos ADC dos sensores e reseta o estado de navegação e a matriz.
 * @param corner Canto inicial (START_BOTTOM_LEFT ou START_BOTTOM_RIGHT).
 * @param effective_maze_size Dimensão M do labirinto (padrão: 18).
 * @param initial_heading Orientação inicial do robô (padrão: HEADING_NORTH).
 */
void Navigation_Init(StartCorner_t corner = START_BOTTOM_LEFT,
                     uint8_t effective_maze_size = MAZE_MAX_SIZE,
                     Heading_t initial_heading = HEADING_NORTH);

/**
 * @brief Lê os 4 sensores FC-51 no ADC do ESP32 e realiza o mapeamento relativo
 *        de acordo com o estado de `inverted_front`.
 * @param out_sensors Ponteiro para a estrutura que receberá as leituras virtuais.
 * @return Máscara de bits de paredes relativas (0bEFD: Bit2=Esq, Bit1=Frente, Bit0=Dir).
 */
uint8_t Navigation_ReadVirtualSensors(VirtualSensors_t* out_sensors);

/**
 * @brief Registra as paredes detectadas na célula atual `(x, y)` dentro da matriz
 *        `maze[18][18]` convertendo da referência relativa (virtual) para a global
 *        (Norte, Leste, Sul, Oeste) e sincronizando com as células vizinhas.
 * @param v_sensors Leituras atuais dos sensores virtuais.
 */
void Navigation_UpdateMazeMap(const VirtualSensors_t* v_sensors);

/**
 * @brief Executa a lógica de decisão do Seguidor de Parede Esquerda (Left-Hand Rule),
 *        eliminando o giro físico de 180° em becos sem saída através da inversão
 *        da frente virtual (`inverted_front`).
 * @param v_sensors Leituras atuais dos sensores virtuais.
 * @return Comando de ação (`RobotAction_t`) a ser executado pelos motores.
 */
RobotAction_t Navigation_DecideNextAction(const VirtualSensors_t* v_sensors);

/**
 * @brief Camada de abstração de acionamento dos motores. Traduz `ACTION_FORWARD`,
 *        `ACTION_TURN_LEFT` e `ACTION_TURN_RIGHT` considerando `inverted_front`.
 * @param action Ação decidida pela máquina de estados.
 */
void Navigation_ExecuteAction(RobotAction_t action);

/**
 * @brief Executa um passo completo de navegação na célula atual:
 *        1. Lê sensores virtuais
 *        2. Atualiza matriz `maze[18][18]`
 *        3. Verifica condição de vitória `(target_x, target_y)`
 *        4. Decide e executa a próxima ação e atualiza coordenadas `(x, y)`
 */
void Navigation_Step(void);

/**
 * @brief Retorna se a frente virtual está invertida (`true` = traseira física é a frente).
 */
bool Navigation_IsFrontInverted(void);

/**
 * @brief Retorna se o robô já atingiu a célula final e concluiu o labirinto.
 */
bool Navigation_IsMazeCompleted(void);

/**
 * @brief Acesso somente-leitura ao estado global da navegação.
 */
const NavigationState_t* Navigation_GetState(void);

#endif /* NAVIGATION_H_ */

