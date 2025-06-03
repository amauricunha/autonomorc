# AutonomoRC

Projeto de controle embarcado para veículo RC autônomo/manual, utilizando Arduino, sensores e comunicação CAN.

## Funcionalidades

- Controle de motor (ESC) e direção (servo) via RC ou comandos CAN.
- Leitura de sensores:
  - Encoder óptico (TCRT5000) para velocidade da roda.
  - Sensor ultrassônico (HC-SR04) para detecção de obstáculos.
  - Fototransistor para detecção de linha/luz.
  - Mock de velocidade do veículo (MPU9250).
- Controle de tração (opcional, ativado por canal RC).
- Parada automática ao detectar obstáculos.
- Comunicação CAN com Raspberry Pi:
  - Recebe comandos de navegação (PWM/ângulo).
  - Envia dados de navegação (velocidade, pulsos, slip).
  - Heartbeat CAN para monitoramento.
- Diagnóstico de sensores (alerta por LED).

## Arquivos principais

- `arduino/controlerecu.ino`: Código principal do Arduino para controle do veículo.
- `README.md`: Este arquivo.

## Instalação

1. Instale as bibliotecas necessárias no Arduino IDE:
   - Servo
   - Wire
   - SPI
   - [MCP_CAN_lib](https://github.com/coryjfowler/MCP_CAN_lib)

2. Faça upload do arquivo `controlerecu.ino` para o Arduino.

3. Conecte os sensores e atuadores conforme os pinos definidos no código.

## Pinos e Conexões

| Função                | Pino Arduino | Pino MCP2515 | Pino Raspberry Pi (SPI) |
|-----------------------|--------------|--------------|-------------------------|
| TCRT5000 (encoder)    | 3            |              |                         |
| ESC (motor)           | 9            |              |                         |
| Servo direção         | 10           |              |                         |
| HC-SR04 Trigger       | 6            |              |                         |
| HC-SR04 Echo          | 7            |              |                         |
| Fototransistor        | 2            |              |                         |
| LED                   | 12           |              |                         |
| RC CH1 (direção)      | 3            |              |                         |
| RC CH2 (aceleração)   | 2            |              |                         |
| RC CH3 (trac. ctrl)   | 4            |              |                         |
| RC CH4 (modo)         | 5            |              |                         |
| CAN CS                | 8            | CS           | GPIO (ex: 8)            |
| CAN SO (MISO)         |              | SO           | MISO (GPIO 9)           |
| CAN SI (MOSI)         |              | SI           | MOSI (GPIO 10)          |
| CAN SCK               |              | SCK          | SCK (GPIO 11)           |
| CAN INT               |              | INT          | GPIO (ex: 25)           |
| CAN VCC/GND           |              | VCC/GND      | 3.3V/5V e GND           |

> **Atenção:** Ajuste os pinos e parâmetros conforme seu hardware.

### Ligação do MCP2515 ao Raspberry Pi (SPI)

Conecte o módulo MCP2515 ao Raspberry Pi usando a interface SPI. Exemplo de ligação:

| MCP2515 | Raspberry Pi GPIO | Função         |
|---------|------------------|----------------|
| VCC     | 3.3V ou 5V       | Alimentação    |
| GND     | GND              | Terra          |
| SCK     | GPIO 11 (SCLK)   | SPI Clock      |
| SI      | GPIO 10 (MOSI)   | SPI MOSI       |
| SO      | GPIO 9 (MISO)    | SPI MISO       |
| CS      | GPIO 8 (CE0)     | SPI Chip Select|
| INT     | GPIO 25 (ou outro disponível) | Interrupção |

> **Nota:** O MCP2515 pode ser alimentado com 5V, mas o Raspberry Pi trabalha com 3.3V nos pinos GPIO. Certifique-se de que o módulo MCP2515 possui conversores de nível lógico para comunicação segura.

### Ligação do MCP2515 ao Arduino

Para conectar o MCP2515 ao Arduino, você deve ligar os seguintes pinos:

| MCP2515 | Arduino         | Função                |
|---------|----------------|-----------------------|
| VCC     | 5V             | Alimentação           |
| GND     | GND            | Terra                 |
| SCK     | 13             | SPI Clock             |
| SI      | 11             | SPI MOSI              |
| SO      | 12             | SPI MISO              |
| CS      | 8 (exemplo)    | SPI Chip Select (CAN_CS_PIN no código) |
| INT     | (opcional, ex: 2) | Interrupção (pode não ser usado) |

- **CAN CS** é o pino de seleção do chip (CS), definido no código como `CAN_CS_PIN`.
- Os pinos SCK, MOSI (SI), MISO (SO) são os pinos padrão SPI do Arduino UNO (13, 11, 12).
- INT pode ser usado para interrupção, mas não é obrigatório para funcionamento básico.

> **Importante:** Todos os pinos de comunicação SPI devem ser conectados, não apenas o CS.

### Alimentação do Arduino via BEC do ESC

Você pode alimentar o Arduino usando o BEC do ESC, conectando o fio vermelho (5V) e preto (GND) do conector do ESC à entrada de alimentação do Arduino (pino VIN ou 5V, dependendo do modelo). Certifique-se de:

- O BEC fornece tensão estável de 5V.
- Não alimente o Arduino simultaneamente por USB e pelo BEC para evitar conflitos.
- O MCP2515 deve ser alimentado pelo mesmo 5V e GND do Arduino.

> **Atenção:** Se usar fontes diferentes para Arduino e MCP2515, conecte os GNDs para referência comum.

### Alimentação dos módulos MCP2515 (Arduino e Raspberry Pi)

Você pode alimentar cada módulo MCP2515 (o conectado ao Arduino e o conectado ao Raspberry Pi) com fontes diferentes, desde que cada um receba a tensão adequada ao seu lado (5V para Arduino, 3.3V ou 5V para Raspberry Pi, conforme o módulo). **O ponto mais importante é que todos os dispositivos conectados ao barramento CAN (incluindo ambos MCP2515, Arduino e Raspberry Pi) devem ter seus GNDs interligados** para garantir referência comum de sinal e comunicação confiável.

- O MCP2515 do Arduino normalmente é alimentado com 5V.
- O MCP2515 do Raspberry Pi pode ser alimentado com 3.3V ou 5V, dependendo do módulo e do circuito de interface de nível lógico.
- Sempre conecte os GNDs de todos os módulos e dispositivos CAN entre si.

> **Nunca conecte diretamente sinais de 5V aos pinos GPIO do Raspberry Pi.** Use módulos MCP2515 com conversores de nível lógico para proteger o Raspberry Pi.

### Por que não conectar sinais de 5V diretamente aos GPIO do Raspberry Pi?

Os pinos GPIO do Raspberry Pi operam em 3.3V e **não são tolerantes a 5V**. Se você conectar sinais de 5V diretamente a esses pinos, pode danificar permanentemente o processador do Raspberry Pi. Isso vale tanto para sinais de dados (como SPI, I2C, UART) quanto para sinais de controle.

#### Como proteger o Raspberry Pi?

- **Use módulos MCP2515 com conversores de nível lógico integrados:** Muitos módulos MCP2515 já vêm com circuitos de conversão de nível para adaptar sinais de 5V para 3.3V. Verifique a documentação do seu módulo.
- **Adicione conversores de nível lógico externos:** Se o seu módulo MCP2515 não possui conversão de nível, utilize circuitos de conversão (por exemplo, usando transistores MOSFET ou CI dedicados) entre o MCP2515 e o Raspberry Pi.
- **Nunca alimente o MCP2515 com 5V se ele estiver conectado diretamente ao Raspberry Pi sem conversão de nível.** Prefira alimentar o MCP2515 com 3.3V nesses casos.

#### Exemplos de conversão de nível

- **Conversor de nível lógico bidirecional:** Pequenos módulos com MOSFETs, facilmente encontrados em lojas de eletrônica, permitem adaptar sinais SPI/I2C entre 5V e 3.3V.
- **Divisor resistivo:** Para sinais unidirecionais (ex: MISO), um simples divisor de tensão com resistores pode ser usado, mas não é recomendado para sinais de alta velocidade como SPI.

> **Resumo:** Sempre confira se o seu módulo MCP2515 é compatível com 3.3V nos sinais SPI antes de conectar ao Raspberry Pi. Se não for, adicione conversores de nível lógico para evitar danos ao hardware.

## Modos de Operação

- **Manual:** Sinais do rádio controlam motor e direção.
- **Autônomo:** Comandos CAN controlam motor e direção.
- **Controle de tração:** Ativado pelo canal RC CH3.
- **Parada por obstáculo:** Motor é bloqueado se obstáculo detectado a menos de 20cm.

## Comunicação CAN

- **Comandos recebidos:**  
  - ID `0x100`: [ESC PWM, Servo Ângulo]
- **Dados enviados:**  
  - ID `0x200`: [Velocidade roda, Pulsos, Slip]
  - ID `0x300`: [Obstáculo detectado]
  - ID `0x400`: Heartbeat

## Observações

- O código inclui mocks para sensores não implementados (ex: MPU9250).
- Ajuste os valores de PWM, ângulos e limites conforme seu veículo.
- O LED pode indicar falha de sensores.

## Licença

MIT
