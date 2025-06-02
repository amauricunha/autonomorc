# Autonomous RC Car

Projeto de demonstração de ADAS e navegação autônoma em um RC impresso em 3D.

## Estrutura

- `arduino/`: Código embarcado para sensores, atuadores e CAN.
- `raspberry/`: Scripts para SLAM, sensor fusion, planejamento e CAN.

## Fluxo

1. Mapeamento (SLAM)
2. Localização (Sensor Fusion)
3. Planejamento de rota e obstáculos
4. Comunicação CAN
5. Controle de movimento

## Como rodar

Veja cada subdiretório para detalhes.

---

## Ligações: Arduino, ESC, Servo, Rádio Controle (RC) e CAN

O sistema permite alternar entre modo manual (controle RC) e modo autônomo (comandos do Raspberry Pi via CAN). O canal 4 (CH4) do receptor RC seleciona o modo, e o canal 3 (CH3) ativa/desativa o controle de tração.

### Esquema de ligação com CAN (Arduino MCP2515 ↔ Raspberry Pi)

```
+-------------------+         +-------------------+         +-------------------+
|   RC Receptor     |         |     Arduino       |         |      ESC/Servo    |
|-------------------|         |-------------------|         |-------------------|
| CH2 (aceleração) -|-------> | D2 (entrada)      |         |                   |
| CH1 (direção) ----|-------> | D3 (entrada)      |         |                   |
| CH3 (trac. ctrl)--|-------> | D4 (entrada)      |         |                   |
| CH4 (modo) -------|-------> | D5 (entrada)      |         |                   |
| GND --------------|-------> | GND               |         |                   |
| 5V ---------------|-------> | 5V                |         |                   |
+-------------------+         |                   |         |                   |
                              | D9 (saída PWM) ---|-------->| ESC (motor)       |
                              | D10 (saída PWM) --|-------->| Servo (direção)   |
                              |                   |         +-------------------+
                              | MCP2515 (SPI)     |
                              |  CAN_H ---------- |-------------------+
                              |  CAN_L ---------- |                   |
                              +-------------------+                   |
                                                           +-------------------+
                                                           |   Raspberry Pi    |
                                                           |-------------------|
                                                           | MCP2515 (SPI)     |
                                                           |  CAN_H ---------- |
                                                           |  CAN_L ---------- |
                                                           | (CAN0 interface)  |
                                                           +-------------------+
```

- **MCP2515 (Arduino):** Módulo CAN conectado ao Arduino via SPI (CS: D8, SCK/MISO/MOSI padrão).
- **MCP2515 (Raspberry Pi):** Módulo CAN conectado ao Raspberry Pi via SPI (ou CAN HAT).
- **CAN_H/CAN_L:** Barramento CAN conecta MCP2515 do Arduino ao MCP2515 do Raspberry Pi.
- **No modo autônomo:** Arduino recebe comandos do Raspberry Pi via CAN para ESC/servo.
- **No modo manual:** Arduino lê sinais do RC e repassa para ESC/servo.
- **Controle de tração pode ser ativado/desativado em ambos os modos via CH3.

### Observações

- O canal 4 do receptor RC (CH4) deve ser configurado como um switch no rádio para alternar entre modo manual e autônomo.
- O canal 3 do receptor RC (CH3) deve ser configurado como um switch ou potenciômetro para ativar/desativar o controle de tração.
- Todos os GNDs (Arduino, ESC, Servo, Receptor, MCP2515, Raspberry Pi) devem estar conectados.
- O MCP2515 deve ser alimentado com 5V (ou 3.3V conforme modelo).
- O Raspberry Pi deve ter interface CAN ativa (ex: CAN HAT ou MCP2515 via SPI).
- Para análise posterior, os dados da rede CAN podem ser salvos em um arquivo `.log` ou `.blf` usando ferramentas como `candump` ou python-can.
- **No modo manual, o Arduino envia periodicamente dados de navegação (odometria, slip, etc) via CAN para o Raspberry Pi, permitindo o mapeamento do ambiente com ORB-SLAM2 ou RTAB-Map enquanto o carrinho é pilotado manualmente.**

---
