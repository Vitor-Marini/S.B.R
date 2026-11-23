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
| Chassi | rígido desde 2026-11-21 (viga entre as placas laterais); peças em `hardware/3d-models/` |

## Pinagem

| Sinal | ESP32-C3 | Observação |
|-------|----------|------------|
| SDA (MPU) | GPIO 8 | |
| SCL (MPU) | GPIO 9 | |
| STEP motor "L" | GPIO 0 | **"L" do firmware é a roda ESQUERDA** do robô (olhando pra frente; medido 2026-11-21) |
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

- **Frente = lado da chave liga/desliga** (−Y no CAD). Pitch positivo = inclinado pra lá, e comando
  positivo nas rodas anda pra lá. Conferir com essa referência física, não com "frente" de memória.
- Eixo de queda: acelerômetro X/Z + **gyro Y** (MPU deitado, chip pra cima).
- **Gyro Z positivo = girar pra esquerda** (anti-horário visto de cima).
- Desde a remontagem de 2026-11-21 os dois motores estão invertidos: `tune lrev 1`, `tune rrev 1`
  (`motor both +N` anda pra frente). Se remontar e o robô "acelerar e cair", confira esse sinal primeiro.
- **"L" do firmware (GPIO0) = roda esquerda** olhando pra frente. Medido girando no lugar: `motors 1600 -1600`
  → giro horário (yaw cai). Se a curva "girar sem parar", esse mapeamento mudou: inverta o sinal em `balance.cpp`.
- Com o MPU nessa orientação, a configuração `regs` esperada é `75=68 6B=01 19=00 1A=03 1B=08 1C=10`.

## O que falta melhorar (por prioridade)

1. ✅ **Chassi rígido** (2026-11-21): viga entre as placas laterais; antes ele flexionava e o robô não andava.
2. ✅ **MPU baixo e rígido** (2026-11-21): na prateleira logo acima dos motores (~37 mm acima do eixo), mesma
   orientação de antes (eixos conferidos inclinando). Setpoint remedido: −0,9°. Ainda vale afastar dos drivers
   (calor → deriva do gyro): faça `cal` depois de aquecer.
3. **Alimentação.** Já houve reset do MPU e reboots da ESP por mau contato; capacitor (100–470 µF)
   perto da ESP/MPU e conectores firmes. Bateria fraca derruba os motores.
4. **Monitor de bateria.** Divisor (ex.: 100 kΩ + 33 kΩ → ~3,1 V a 12,6 V) num pino ADC livre.
5. **ENABLE num GPIO**, pra desligar os motores em IDLE/FAULT (sem chiado e sem esquentar).
6. **Vref** dos drivers: ~0,6–0,7 V pra NEMA17 de ~1,2–1,5 A (nunca medido).
7. ✅ **Bateria no topo** (2026-11-21). Mesmo assim o centro de massa do corpo fica só ~37 mm acima do eixo.

## Parâmetros físicos (modelo 3D do mk2, calibrado com a balança)

| Grandeza | Valor |
|---|---|
| Massa total / do corpo (sem rodas) | 1,06 kg / 1,02 kg |
| Centro de massa do corpo acima do eixo (d) | 37 mm (±15 %) |
| Inércia do corpo em torno do eixo (I) | 3,93e-3 kg·m² |
| **Comprimento efetivo do pêndulo L = I/(m·d)** | **≈ 0,10 m** (é o L de `docs/controle.html`) |
| Raio da roda / bitola (centro a centro) | 37,5 mm / 150 mm |
| Chip do MPU acima do eixo | 37 mm |

Fonte: modelo Fusion "S.B.R mk2" v4, massas calibradas com o peso real (precisão: massa ±5 %, CG e inércias ±15 %).

**L medido (2026-11-22), teste do pêndulo:** robô de cabeça pra baixo, balançando em torno do eixo das rodas,
gyro gravado pelo stream. Período com amplitude de 15–30°: **T ≈ 0,61 s** (0,58–0,63) → `L = g·(T/2π)² ≈ 0,09 m`
(0,084–0,099), ~8 % abaixo do CAD. Acima de ~50° o período cresce (pêndulo não linear) e o gyro **satura em
±500 °/s** (escala atual do MPU).
O firmware usa roda de 74 mm (medida); o CAD diz 75 mm (1 % de diferença na velocidade).

## Onde montar o IMU

- O **gyro** lê o mesmo em qualquer ponto de um corpo **rígido**: só a rigidez da montagem importa.
- O **acelerômetro** mede gravidade + a aceleração do ponto onde está. Num modelo linear do pêndulo
  (L = comprimento efetivo, a = aceleração da roda, h = altura do sensor acima do eixo):

  `leitura_x ≈ a·(1 − h/L) + g·θ·(1 + h/L)`

  - **No eixo (h = 0):** a aceleração da roda aparece inteira, mas ela é **conhecida** (stepper: comandada =
    real), então o firmware pode subtraí-la. Montagem recomendada, junto com rigidez.
  - **No "centro de percussão" (h ≈ L):** a aceleração da roda se cancela, mas o ângulo aparece em dobro,
    L é desconhecido e muda com a posição das baterias, e lá em cima o chassi flexiona. Na prática, não compensa.
- O filtro complementar (α = 0,995) confia no gyro abaixo de ~1 s, então o acelerômetro só pesa nos efeitos lentos
  (andar acelerando, deriva).

## Limitação importante

O **DS4 não conecta direto na ESP32-C3**: ele usa Bluetooth Clássico e a C3 só tem Bluetooth LE.
O controle é feito pelo PC (`tools/gamepad.py`), que manda os comandos ao robô pelo WiFi.
