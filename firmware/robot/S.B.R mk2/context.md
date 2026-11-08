# S.B.R mk2 — Contexto do Projeto

> Arquivo de contexto para manter continuidade entre sessões de desenvolvimento.

---

## Visão Geral

Robô equilibrista (self-balancing robot) sobre duas rodas, baseado em ESP32-C3 Supermini + MPU6050 + NEMA17 + DRV8825. Firmware refeito do zero (mk2) substituindo o mk1 que não funcionava.

---

## Hardware

| Componente | Especificação |
|------------|---------------|
| MCU | ESP32-C3 Supermini (RISC-V 32-bit, 160MHz, núcleo único, 400KB RAM) |
| IMU | MPU6050 (I2C, DLPF 44Hz, range ±8G / ±500°/s) |
| Motor | NEMA17 (200 passos/volta) × 2 |
| Driver | DRV8825 × 2 (microstepping por jumper, ENABLE não conectado) |
| Bateria | Makita BL1016 12V |
| Pinos I2C | SDA=GPIO8, SCL=GPIO9 |
| Pinos STEP | L=GPIO0, R=GPIO3 |
| Pinos DIR | L=GPIO1, R=GPIO2 |

---

## Decisões de Arquitetura

### LEDC PWM (em vez de timer ISR)
- **Problema mk1**: hw_timer ISR chamando digitalWrite — CPU 100% ocupada
- **Solução mk2**: Periférico LEDC gera pulsos de step em hardware. Frequência = steps/s, duty 50%. Zero ISR.

### Mahony Filter (em vez de Complementar)
- **Problema**: Complementar com DLPF 5Hz = ~40ms de atraso
- **Solução**: Mahony com quaternions, estima bias do gyro online, DLPF 44Hz

### PID Cascateado
- Malha interna: ângulo PD (100 Hz)
- Malha externa: yaw heading-hold PID (100 Hz)
- Malha de velocidade: PI com offset de ângulo (100 Hz)

### Máquina de Estados
- INIT → CALIBRATE → IDLE → STARTING → ACTIVE → MOVING → FAULT
- Rampa de 2s no STARTING (ganhos crescem de 0 a 100%)
- Failsafe: se ângulo > 45° → FAULT → após 3s estável → IDLE

### WiFi + BLE simultâneos
- WiFi AP: `SBR_ROBOT` / `12345678` — dashboard web + WebSocket
- BLE UART: `SBR_CONTROL` — Nordic UART Service
- NimBLE-Arduino para coexistência rádio

---

## Estrutura do Firmware

```
S.B.R mk2/
├── include/
│   ├── balance.hpp        — Controlador PID cascateado
│   ├── ble_gamepad.hpp    — BLE UART server
│   ├── cli.hpp            — Terminal interativo serial
│   ├── config.hpp         — Configuração persistente (LittleFS)
│   ├── ds4_controller.hpp — BLE HID client p/ DualShock 4
│   ├── filters.hpp        — Mahony + Complementar
│   ├── imu_manager.hpp    — MPU6050 init/calib/leitura
│   ├── motor.hpp          — LEDC PWM stepper control
│   ├── supervisor.hpp     — Máquina de estados
│   ├── velocity.hpp       — Controle de velocidade PI
│   └── web_interface.hpp  — WiFi AP + WebSocket
├── src/
│   ├── main.cpp           — setup/loop
│   ├── balance.cpp
│   ├── ble_gamepad.cpp
│   ├── cli.cpp
│   ├── config.cpp
│   ├── ds4_controller.cpp — BLE Central + parsing HID
│   ├── filters.cpp
│   ├── imu_manager.cpp
│   ├── motor.cpp
│   ├── supervisor.cpp
│   ├── velocity.cpp
│   └── web_interface.cpp
├── data/
│   └── index.html         — Dashboard web com joystick
├── docs/
│   ├── TECNOLOGIAS.md     — Documentação técnica completa
│   └── TUTORIAL_USO.md    — Guia prático de uso
├── todos/
│   └── hardware_checklist.md — Tarefas físicas pendentes
├── context.md             — Este arquivo
└── platformio.ini         — Configuração de build
```

---

## Configurações Padrão

```cpp
// PID de ângulo
anglePid = { 25.0f, 0.5f, 0.8f };

// PID de yaw (heading-hold)
yawPid   = { 2.0f, 0.02f, 0.5f };

// PID de velocidade
velocityPid = { 8.0f, 1.0f, 0.0f };

// Motor
stepsPerRev = 3200.0f;       // assumindo 1/16 microstepping
wheelDiameterMm = 80.0f;
maxAccelStepsS2 = 25000.0f;

// Filtro
mahonyKp = 0.5f;
mahonyKi = 0.001f;

// Limites
maxVelocity = 1.5f;  // m/s
maxTurnRate = 180.0f; // °/s
failsafeAngle = 45.0f;
```

---

## O que foi implementado

- [x] LEDC PWM para motores (sem ISR)
- [x] Mahony filter (quaternion)
- [x] PID cascateado (ângulo + yaw)
- [x] Controle de velocidade (PI, odometria)
- [x] Máquina de estados (INIT→CALIBRATE→IDLE→STARTING→ACTIVE→FAULT)
- [x] CLI interativo (serial)
- [x] Dashboard web (WiFi AP + WebSocket + Chart.js + joystick)
- [x] BLE UART (Nordic UART Service)
- [x] Config persistente (LittleFS + JSON)
- [x] Rampa de aceleração (motor)
- [x] DS4 gamepad (BLE HID client, scan/conexão/parsing)
- [x] Failsafe (ângulo > 45°)
- [x] Auto-recovery (3s em < 15°)

## O que está pendente

- [ ] **Medir hardware**: diâmetro roda, microstepping, sentido motor
- [ ] **Sintonizar PID**: ajustar Kp/Ki/Kd para o robô físico
- [ ] **Conectar ENABLE** dos DRV8825 a GPIOs (opcional, para desligar motores em IDLE)
- [x] **PS4 gamepad nativo**: HID client BLE (DS4 via NimBLE Central)
- [ ] **Monitor bateria**: divisor de tensão na ADC
- [ ] **Encoder externo**: para odometria precisa (se necessário)

---

## DS4 Controller (DualShock 4 via BLE)

O robô agora funciona como **BLE Central** para se conectar diretamente ao controle PS4, sem intermediários. Detalhes:

- **Scan**: comando `pair` no CLI, procura por "Wireless Controller" por 10s
- **Conexão**: NimBLE Central descobre serviço HID (0x1812) e se inscreve em notificações
- **Parsing**: suporta relatórios simple (0x01, 9 bytes) e full (0x11, 78 bytes)
- **Mapeamento**:
  | Controle | Função |
  |----------|--------|
  | Left Stick Y | Velocidade (−1 a +1) |
  | Right Stick X | Curva (−1 a +1) |
  | Triangle | Start (iniciar balanceamento) |
  | Circle | Stop |
  | Square | Calibrar |
- **Reconexão**: automática após 5s se a conexão cair
- **Edge detection**: botões agem apenas na borda de subida
- **Status no dashboard**: campo `ds4` no JSON de broadcast (conectado/desconectado + endereço)

### APIs NimBLE-Arduino descobertas (v1.4.3)

- `NimBLEScan::start(segundos, callback)` — scan end callback é função estática, não classe
- `NimBLEClientCallbacks` — só tem `onConnect`/`onDisconnect`, sem `onDisconnectReason`
- `NimBLERemoteService::getCharacteristics()` — retorna `vector<NimBLERemoteCharacteristic*>*` (ponteiro)
- `BLEDevice::deleteClient(cliente)` — único modo de deletar `NimBLEClient` (destrutor privado)

---

## Problemas Conhecidos

1. **D term usa derivada do erro filtrada, não gyro rate**
   - Funcionalmente equivalente em regime, mas introduz atraso
   - Pode ser alterado para gyro rate direto se necessário
   - Ver `balance.cpp` linha 34-35

2. **WiFi AP + BLE podem causar interferência**
   - ESP32-C3 tem um rádio compartilhado
   - NimBLE é projetado para coexistência, mas latência pode aumentar

3. **Odometia sem encoder = estimativa**
   - Se o motor perder passos (stall), a contagem fica incorreta
   - O robô pode andar "torto" sem perceber

4. **GPIO 0 usado como STEP**
   - GPIO 0 é strapping pin no ESP32-C3 (boot mode)
   - Configurado como output apenas após boot (em setup())
   - Não causa problemas, mas é bom saber

5. **Complementar vs Mahony: pitch rate em unidades diferentes**
   - Mahony: pitch rate em rad/s
   - Complementar: pitch rate em deg/s
   - Se trocar o filtro em runtime, o D term ficará incorreto
   - Solução futura: unificar para deg/s

---

## Build

```bash
pio run                    # Compilar
pio run --target upload    # Gravar firmware
pio run --target uploadfs  # Gravar LittleFS (dashboard)
pio device monitor -b 115200  # Monitor serial
```

| Métrica | Valor |
|---------|-------|
| Flash   | 87.0% (1.140.646 / 1.310.720 bytes) |
| RAM     | 13.8% (45.060 / 327.680 bytes) |
| Zero warnings | ✅ |

---

## Comandos Úteis (CLI)

```
start      → Iniciar balanceamento
stop       → Parar balanceamento
calibrate  → Recalibrar giroscópio
tune kp 30 → Ajustar PID em tempo real
save       → Salvar config na flash
status     → Mostrar todos parâmetros
motor L 500 → Testar motor
vel 0.5    → Andar a 0.5 m/s
turn -30   → Curvar à esquerda
pair       → Escanear e conectar DS4
ds4        → Mostrar status do DS4
```

---

## Frameworks e Bibliotecas

| Biblioteca | Versão | Uso |
|------------|--------|-----|
| ESPAsyncWebServer | 3.6.0 | Servidor web assíncrono |
| ArduinoJson | 7.4.2+ | Serialização JSON (config + WebSocket) |
| Adafruit MPU6050 | 2.2.6+ | Driver do sensor |
| Adafruit Unified Sensor | 1.1.15 | Interface unificada de sensores |
| NimBLE-Arduino | 1.4.1+ | BLE stack (coexistência WiFi) |
| LittleFS | 2.0.0 | Sistema de arquivos em flash |
| Chart.js | (CDN) | Gráfico no dashboard web |
