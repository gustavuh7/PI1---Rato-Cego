# PI1 - Rato Cego (Micromouse Bidirecional ESP32)

Firmware refatorado a partir do projeto `umart_ino_seguidor_parede` (Micromouse Brasil) para o robô autônomo **Rato Cego** baseado em **ESP32** com arquitetura de **Frente Virtual Bidirecional**.

## Especificações de Hardware

- **Microcontrolador:** ESP32 (Arduino IDE 2.x - pacote `esp32:esp32`)
- **Driver de Motores:** Ponte H Dupla MX1508 (Motor Esquerdo `U3` e Motor Direito `U6`)
- **PWM Inicial de Operação:** 75% Duty Cycle (`191 / 255`)
- **Encoders:** Quadratura (~580 a 600 pulsos/volta, nominal `590 PPR`)
- **Geometria Mecânica:** Rodas de `44 mm` de diâmetro, bitola de `90 mm` (~9 cm)
- **Labirinto:** Células de `16,5 cm x 16,5 cm` (`165 mm`), matriz de até `18x18`
- **Sensores Infravermelhos (4x FC-51 Analógicos no ADC1 do ESP32):**
  - Frontal Físico: `GPIO 34` (ADC ref: 27 a 34)
  - Traseiro Físico: `GPIO 35` (ADC ref: 2815 a 2820)
  - Esquerdo Físico: `GPIO 39` (ADC ref: 32 a 40)
  - Direito Físico: `GPIO 36` (ADC ref: 45 a 52)

## Arquitetura de Frente Virtual (Sem Giro de 180°)

Devido às dimensões físicas do chassi em relação à célula de `16,5 cm`, o robô **não realiza curvas físicas de 180°**. Ao encontrar um beco sem saída (`0b111`):
1. Alterna o estado `inverted_front = !inverted_front`.
2. Atualiza a orientação global (`heading = (heading + 2) % 4`).
3. Espelha o mapeamento dos 4 sensores FC-51, dos motores e dos encoders.
4. Executa avanço linear na nova frente virtual para sair da célula sem girar sobre o próprio eixo.

## Estrutura de Arquivos

- `Navigation.h` / `Navigation.cpp`: Abstração de Frente Virtual, mapeamento matricial `maze[18][18]`, detecção de vitória `(target_x, target_y)` e máquina de estados *Left-Hand Rule*.
- `motores.h` / `motores.cpp`: Acionamento PWM do driver MX1508 e espelhamento de tração `setMotoresVirtual()`.
- `sensores.h` / `sensores.cpp`: Leitura dos 4 sensores FC-51 e cálculo do erro de alinhamento lateral virtual.
- `controlador.h` / `controlador.cpp`: Leitura de encoders quadratura via ISR (`IRAM_ATTR`), odometria em mm/graus e controle PD de velocidade.
- `umart_ino_seguidor_parede.h`: Cabeçalho de configuração geral do robô.
- `PI1---Rato-Cego.ino`: Sketch principal para a **Arduino IDE** (`setup`, timer periódico de `10 ms` e `loop`).
