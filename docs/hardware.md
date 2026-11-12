# Hardware do S.B.R

Referência do hardware **como está montado hoje** e do que falta melhorar.
O histórico de como cada coisa foi descoberta está em [diario.md](diario.md).

## Componentes

| Componente | Detalhe |
|------------|---------|
| MCU | ESP32-C3 Supermini (RISC-V, 160 MHz, **um núcleo**, WiFi + **só Bluetooth LE**) |
| IMU | MPU6050 (I2C 400 kHz, DLPF 44 Hz, ±8 g, ±500 °/s) |
| Motores | 2× NEMA17, 200 passos/volta |
| Drivers | 2× DRV8825, **microstepping 1/16** (3200 passos/volta) |
| Rodas | PLA impresso, diâmetro 74 mm |
| Bateria | Makita BL1016 (10,8–12 V); a ESP é alimentada por um DC-DC a partir dela |
| Chassi | peças em `hardware/3d-models/` |

## Pinagem

| Sinal | ESP32-C3 | Observação |
|-------|----------|------------|
| SDA (MPU) | GPIO 8 | |
| SCL (MPU) | GPIO 9 | |
| STEP motor "L" | GPIO 0 | **"L" do firmware é a roda DIREITA** do robô (olhando pra frente) |
| DIR motor "L" | GPIO 1 | |
| DIR motor "R" | GPIO 2 | |
| STEP motor "R" | GPIO 3 | |
| M2 dos dois drivers | 3,3 V (ou 5 V) | 1/16; M0 e M1 soltos (pull-down interno) |
| ENABLE dos drivers | não ligado | drivers sempre energizados (chiam e esquentam parados) |

Microstepping do DRV8825:

| M2 | M1 | M0 | Modo | `tune spr` |
|----|----|----|------|------------|
| 0 | 0 | 0 | full step | 200 |
| **1** | **0** | **0** | **1/16 (atual)** | **3200** |
| 1 | 0 | 1 | 1/32 | 6400 |

## Convenções verificadas no robô

- **Pitch positivo = inclinado pra frente**, o lado pra onde as rodas andam com comando positivo.
- Eixo de queda: acelerômetro X/Z + **gyro Y** (MPU deitado, chip pra cima).
- **Gyro Z positivo = girar pra esquerda** (anti-horário visto de cima).
- `motor both +N` faz as duas rodas andarem pra frente (`left_reversed`/`right_reversed` = falso).
- Com o MPU nessa orientação, a configuração `regs` esperada é `75=68 6B=01 19=00 1A=03 1B=08 1C=10`.

## O que falta melhorar (por prioridade)

1. **Chassi rígido.** Hoje ele flexiona: o corpo/IMU inclina sem o peso sair de cima do eixo, e por isso
   o robô não anda pra frente/trás (ver diário, 2026-11-10). Bateria, motores e MPU numa estrutura que não ceda.
2. **MPU firme e longe dos drivers.** Parafuso/espaçador ou cola dura, fios presos ao chassi.
   Perto dos drivers ele chega a 46 °C e o bias do gyro deriva (~0,14 °/s); faça `cal` depois de aquecer.
3. **Alimentação.** Já houve reset do MPU e reboots da ESP por mau contato; capacitor (100–470 µF)
   perto da ESP/MPU e conectores firmes. Bateria fraca derruba os motores.
4. **Monitor de bateria.** Divisor (ex.: 100 kΩ + 33 kΩ → ~3,1 V a 12,6 V) num pino ADC livre.
5. **ENABLE num GPIO**, pra desligar os motores em IDLE/FAULT (sem chiado e sem esquentar).
6. **Vref** dos drivers: ~0,6–0,7 V pra NEMA17 de ~1,2–1,5 A (nunca medido).
7. Opcional: **peso mais alto** (ex.: bateria em cima). Pêndulo mais alto cai mais devagar e tolera mais
   atraso — ver `docs/controle.html`, seção 13.

## Limitação importante

O **DS4 não conecta direto na ESP32-C3**: ele usa Bluetooth Clássico e a C3 só tem Bluetooth LE.
O controle é feito pelo PC (`tools/gamepad.py`), que manda os comandos ao robô pelo WiFi.
