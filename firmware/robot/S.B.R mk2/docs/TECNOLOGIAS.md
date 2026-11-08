# Tecnologias e Técnicas — S.B.R mk2

## Controle de Motores com LEDC PWM (sem interrupções!)

No firmware original (mk1), os motores de passo eram controlados por interrupções de hardware
(timers). Cada pulso de passo gerava uma interrupção que chamava `digitalWrite()` dentro da ISR
— extremamente custoso para um microcontrolador de núcleo único (ESP32-C3).

No mk2, usamos o periférico **LEDC** (PWM) do ESP32-C3:

- Cada motor recebe um canal PWM independente
- A **frequência do PWM = steps por segundo** do motor
- Duty cycle fixo em 50% (cada borda de subida = um passo do motor)
- **Zero interrupções** — o hardware gera os pulsos sozinho
- A CPU fica livre para rodar o controle e a telemetria

```
LEDC Timer 0 ── Canal 0 ── GPIO 0 (STEP motor esquerdo)
LEDC Timer 1 ── Canal 3 ── GPIO 3 (STEP motor direito)
```

Para mudar a velocidade, basta chamar `ledc_set_freq()`. Para parar o motor,
colocamos o duty cycle em 0 (pino fica em LOW). Não há mais a sobrecarga
de interrupções a cada passo.

### Rampa de aceleração

Toda mudança de velocidade passa por um **rampeador** que limita a aceleração:

```
maxDelta = maxAccelStepsS2 × dt
current += constrain(target - current, -maxDelta, +maxDelta)
```

Isso evita que os steppers percam passos por aceleração brusca.
O valor padrão é 25000 steps/s² (0 a 6000 steps/s em ~240ms),
configurável em `config.hpp` como `motor.maxAccelStepsS2`.

---

## Filtro Mahony (fusão de sensores)

O MPU6050 tem dois sensores:

| Sensor | Bom para | Ruim para |
|--------|----------|-----------|
| Acelerômetro | Saber onde é "pra baixo" (gravidade) | Ruído alto, não responde rápido |
| Giroscópio | Medir rotação rápida com precisão | Deriva com o tempo (drift) |

O **Filtro Mahony** combina o melhor dos dois:

1. Mede o ângulo pelo acelerômetro (referência de gravidade)
2. Estima o ângulo pelo giroscópio (resposta rápida)
3. Calcula o erro entre os dois
4. Usa esse erro para corrigir o giroscópio (estimando o bias/deriva)
5. Integra o giroscópio corrigido para obter o ângulo final

O resultado é um ângulo de pitch preciso e com baixo atraso, essencial para o
equilíbrio. O filtro também estima automaticamente o bias do girospcópio
enquanto o robô opera, eliminando a necessidade de recalibração constante.

### Parâmetros do filtro

- **Mahony Kp** (0.5): velocidade com que o filtro corrige o ângulo usando o
  acelerômetro. Maior = resposta mais rápida, mas mais ruído.
- **Mahony Ki** (0.001): velocidade com que o filtro aprende o bias do
  giroscópio. Muito alto pode causar oscilações.

Configuráveis em `config.hpp` como `filter.mahonyKp` e `filter.mahonyKi`.

### Filtro Complementar (fallback)

Se o Mahony se mostrar pesado demais, o firmware também tem um
**Filtro Complementar** clássico:

```
ângulo = α × (ângulo + gyro × dt) + (1 − α) × accelAngle
```

O alpha padrão é 0.98 (98% giroscópio, 2% acelerômetro).
Pode ser ativado via `imu.setUseMahony(false)`.

---

## PID Cascateado (três níveis de controle)

O controle de equilíbrio usa três malhas aninhadas, cada uma rodando em uma
frequência diferente:

```
┌─────────────────────────────────────────────────────┐
│                    Malha de Yaw                       │
│  (heading-hold) mantém o robô apontando na direção   │
│  yaw_correção = Kp_y × erro_yaw + Kd_y × (-yaw_rate) │
└─────────────────────┬───────────────────────────────┘
                      │ comando diferencial
                      ▼
┌─────────────────────────────────────────────────────┐
│                  Malha de Ângulo                      │
│  (equilíbrio) mantém o robô na vertical              │
│  torque = Kp_a × erro_angulo + Kd_a × (-pitch_rate)  │
└─────────────────────┬───────────────────────────────┘
                      │ torque total
                      ▼
┌─────────────────────────────────────────────────────┐
│              Distribuição para os Motores             │
│  motor_esquerdo = torque + yaw_correção              │
│  motor_direito  = torque − yaw_correção              │
└─────────────────────────────────────────────────────┘
```

### 1. Malha de Ângulo (100 Hz)

- **Entrada**: pitch do filtro Mahony + pitch rate do giroscópio
- **Saída**: torque (0 a 100%)
- **Controle**: PD com derivativo do erro filtrado

```
torque = Kp × erro + Ki × integral + Kd × derivativo_filtrado(erro)
torque = -torque  (sinal invertido para equilíbrio)
```

O termo derivativo usa a derivada do erro (não o gyro diretamente),
passada por um filtro passa-baixa de 1ª ordem (α = 0.8):
```
rawDeriv = (erro_atual − erro_anterior) / dt
derivFiltrado = 0.8 × derivFiltrado + 0.2 × rawDeriv
```

Em regime permanente, `derivFiltrado ≈ −pitchRate`, equivalente a usar o
gyro rate diretamente, mas com a vantagem de rejeitar ruído.

O torque é invertido (`torque = -torque`) para que um erro positivo
(inclinação para frente) produza torque positivo (rodas giram para frente).

Padrões: Kp = 25.0, Ki = 0.5, Kd = 0.8

### 2. Malha de Yaw (100 Hz)

- **Entrada**: yaw estimado pelo Mahony + yaw rate do giroscópio
- **Saída**: correção diferencial (−30 a +30%)
- **Controle**: PID

Mantém o robô apontando na mesma direção (heading-hold).
O erro de yaw é normalizado para ±180° para evitar saltos no controle.

Padrões: Kp = 2.0, Ki = 0.02, Kd = 0.5

### 3. Malha de Velocidade (futuro — implementação parcial)

Preparada para o "andar". Quando ativada:
- Entrada: odometria (contagem de passos)
- Saída: setpoint de ângulo (inclinar para frente = andar para frente)

O controle de velocidade ainda não está ativo — os placeholders existem
no `config.hpp` como `velocityPid` para quando você quiser implementar.

---

## Odometria (contagem de passos)

Cada motor tem um contador de passos acumulados:

```cpp
_leftSteps += (int32_t)(currentSpeedL × dt);
_rightSteps += (int32_t)(currentSpeedR × dt);
```

A partir disso, podemos calcular:
- **Distância percorrida**: `passos / stepsPorVolta × π × diametroRoda`
- **Velocidade linear**: derivada da distância
- **Rotação do robô**: diferença entre as rodas

A odometria é usada no controle de velocidade (futuro) e pode ser visualizada
no CLI com o comando `status`. Use `reset` para zerar os contadores.

**ATENÇÃO**: A odometria com steppers sem encoder é estimada — se o motor
perder passos (situação de stall), a contagem fica incorreta.

---

## Máquina de Estados (Supervisor)

O robô não tenta mais equilibrar imediatamente ao ligar. Agora ele segue
uma máquina de estados:

```
INIT ──→ CALIBRATE ──→ IDLE ──→ STARTING ──→ ACTIVE ──→ MOVING
                           ↑                        │
                           └──── FAULT ←────────────┘
```

| Estado | O que acontece |
|--------|---------------|
| **INIT** | Inicializa MPU6050, configura filtros |
| **CALIBRATE** | Calibra o giroscópio (500 amostras, robô parado) |
| **IDLE** | Robô parado, WiFi ligado, só monitora ângulo |
| **STARTING** | Rampa de 2s: ganhos do PID crescem de 0 a 100% |
| **ACTIVE** | Equilíbrio ativo + heading-hold |
| **MOVING** | ACTIVE + controle de velocidade (futuro) |
| **FAULT** | Ângulo > 45°, motores desligados. Auto-recovery após 3s estável |

Isso evita que o robô "saia disparado" ao ligar e permite recuperação
automática de quedas.

---

## CLI (Terminal Interativo)

O terminal serial agora aceita comandos:

| Comando | Função |
|---------|--------|
| `start` | Inicia o balanceamento |
| `stop` | Para e volta a IDLE |
| `calibrate` | Recalibra o giroscópio |
| `status` | Mostra todos os parâmetros |
| `tune kp 30` | Ajusta Kp do ângulo em tempo real |
| `tune ykp 3` | Ajusta Kp do yaw |
| `tune sp 1` | Muda o setpoint (inclinação alvo) |
| `motor left 500` | Gira motor esquerdo a 500 steps/s |
| `motor right -300` | Gira motor direito ao contrário |
| `angle` | Mostra ângulo atual |
| `save` / `load` | Salva/carrega config na flash |
| `reset` | Zera odometria e integrais |
| `pair` | Escaneia e conecta DS4 via BLE |
| `ds4` | Mostra status do DS4 (conectado/endereço) |

---

## Dashboard Web (WiFi + WebSocket)

O robô cria uma rede WiFi (`SBR_ROBOT`, senha `12345678`).

O dashboard web (`192.168.4.1`) mostra em tempo real:
- **Gráfico do ângulo** (Chart.js, últimos 100 pontos)
- **Estado atual** do robô (ACTIVE, FAULT, etc)
- Telemetria: pitch, rate, yaw, output do PID, velocidades
- **Sliders** para ajustar Kp, Ki, Kd, setpoint
- **Botões**: INICIAR, PARAR, CALIBRAR
- **Exportar CSV** dos dados de ângulo

A comunicação é via WebSocket — os dados do PID e telemetria são enviados
a ~10 Hz.

---

## MPU6050 — Configuração

| Parâmetro | Mk1 | Mk2 |
|-----------|-----|-----|
| Range acelerômetro | ±8G | ±8G |
| Range giroscópio | ±500°/s | ±500°/s |
| **DLPF (filtro hardware)** | **5 Hz** | **44 Hz** |

O DLPF de 5Hz no mk1 adicionava ~40ms de atraso na leitura, o que
desestabilizava o controle. No mk2, subimos para 44Hz e deixamos o
filtro Mahony cuidar do ruído restante — resposta muito mais rápida.

---

## Componentes e Pinagem

```
ESP32-C3 Supermini
├── GPIO 0  → STEP motor L (LEDC Canal 0)
├── GPIO 1  → DIR motor L
├── GPIO 2  → DIR motor R
├── GPIO 3  → STEP motor R (LEDC Canal 3)
├── GPIO 8  → SDA (MPU6050)
└── GPIO 9  → SCL (MPU6050)
```

Os pinos M0, M1, M2 (microstepping) e ENABLE dos DRV8825 não estão
conectados ao ESP32. O microstepping é definido por jumpers nos próprios
módulos. **Verifique a configuração atual** (provavelmente full step).

---

## Resumo das melhorias do mk1 para o mk2

| Problema no mk1 | Solução no mk2 |
|-----------------|----------------|
| Stepper via ISR com digitalWrite (lento) | LEDC PWM sem interrupção |
| dt fixo de 10ms | dt medido em tempo real |
| Filtro complementar | Mahony com estimativa de bias |
| Sem controle de yaw | Heading-hold com PID |
| Inicialização abrupta | Máquina de estados com rampa |
| MPU6050 DLPF em 5Hz (lento) | DLPF em 44Hz |
| Ki = 0 (sem integral) | Ki = 0.5 com anti-windup |
| Só log serial | CLI interativo com comandos |
| Debug só por Serial | Dashboard Web + Serial CLI |
| Sem controle remoto | BLE UART + WiFi WebSocket |
| PWM por software (ISR) | LEDC por hardware (zero CPU)

---

## Configuração Inicial (após montagem)

Antes de tentar equilibrar, complete os passos em
[`todos/hardware_checklist.md`](../todos/hardware_checklist.md):

1. Medir diâmetro das rodas → `wheel_diameter_mm`
2. Verificar microstepping → `steps_per_rev`
3. Testar sentido dos motores via CLI → `left_reversed` / `right_reversed`
4. Com `start`, observar se o robô tenta se equilibrar
5. Ajustar Kp, Ki, Kd pelos sliders do dashboard ou CLI (`tune`)

---

## Controle de Velocidade (andar)

Para o robô se mover, ele precisa inclinar para frente — a gravidade puxa,
e os motores aceleram para "pegar" o robô, gerando movimento. Esse é o
mesmo princípio do Segway.

### Arquitetura de controle em cascata

```
Alvo de Velocidade (m/s)         Alvo de Curva (°/s)
         │                               │
         ▼                               ▼
┌──────────────────┐          ┌──────────────────────┐
│  Velocity PI      │          │  Turn Rate PI        │
│  erro = alvo -    │          │  erro = alvo - yaw   │
│  velocidade_real  │          │  rate_real           │
│  saída = offset   │          │  saída = correção    │
│  de ângulo (°)    │          │  diferencial (%)     │
└────────┬─────────┘          └──────────┬───────────┘
         │                               │
         ▼                               ▼
   ┌─────────────────────────────────────────┐
   │         Balance PD (ângulo)              │
   │  setpoint_efetivo = setpoint + offset    │
   │  torque = Kp × erro + Kd × (-pitchRate) │
   └────────────────┬────────────────────────┘
                    │ torque
                    ▼
   ┌─────────────────────────────────────────┐
   │         Distribuição Motor               │
   │  left  = torque + correção_diferencial   │
   │  right = torque - correção_diferencial   │
   └─────────────────────────────────────────┘
```

### Estimativa de velocidade (odometria)

A velocidade real é estimada pela contagem de passos dos steppers:

```
média_steps/s = (dLeft + dRight) / 2 / dt
velocidade = média_steps/s / stepsPorVolta × π × diametroRoda
```

Isso é feito a cada ciclo de controle (~100 Hz) pelo `VelocityController`.
A velocidade estimada é usada como feedback para o controle PI.

### Ganhos do controle de velocidade

Definidos em `config.hpp` como `velocityPid`:

| Ganho | Padrão | Função |
|-------|--------|--------|
| Kp    | 8.0    | Inclinação por m/s de erro |
| Ki    | 1.0    | Elimina erro estacionário |

O offset de ângulo máximo é limitado a ±8° (não importa o quanto o usuário
peça, o robô nunca vai inclinar mais que 8° para se mover).

---

## Controle por Joystick Virtual (Web)

O dashboard web (`192.168.4.1`) agora tem um **joystick virtual** no centro
da tela. Funciona:

- **Desktop**: clique e arraste com o mouse
- **Mobile**: toque e arraste com o dedo

| Eixo | Controle | Intervalo |
|------|----------|-----------|
| Y (cima/baixo) | Velocidade (frente/trás) | -1.0 a +1.0 m/s |
| X (esquerda/direita) | Curva (esquerda/direita) | -180 a +180 °/s |

Quando o joystick é solto, a velocidade e a curva voltam a zero
automaticamente.

Os comandos são enviados via WebSocket:
```json
{"v": 0.5, "turn": -30}
```

---

## Controle por Bluetooth (BLE UART)

O ESP32-C3 também funciona como um **servidor BLE UART**.

- Nome: `SBR_CONTROL`
- Serviço: Nordic UART Service (NUS)
- Conecte qualquer app de terminal BLE (ex: "Serial Bluetooth Terminal")
- Envie comandos de texto:

| Comando | Exemplo | Efeito |
|---------|---------|--------|
| `f 0.3` | `f 0.3` | Frente 0.3 m/s |
| `v -0.5` | `v -0.5` | Trás 0.5 m/s |
| `t 45` | `t 45` | Curva 45°/s direita |
| `turn -30` | `turn -30` | Curva 30°/s esquerda |
| `s` | `s` | Parar (zera velocidade e curva) |
| `start` | `start` | Iniciar balanceamento |
| `stop` | `stop` | Parar balanceamento |

### Suporte nativo a PS4 (DualShock 4 via BLE)

O robô agora funciona como **BLE Central** (cliente) para se conectar
diretamente a um controle DualShock 4 via Bluetooth, sem necessidade
de celular intermediário.

#### Como usar

1. Coloque o DS4 em modo de pareamento: pressione **PS + Share** por 3s
   (o LED começa a piscar rapidamente)
2. No CLI do robô, digite `pair`
3. O robô escaneia por 10s procurando por "Wireless Controller"
4. Quando encontrado, conecta automaticamente e se inscreve nas
   notificações HID do serviço 0x1812

#### Mapeamento de controles

| Controle | Função no robô |
|----------|----------------|
| Left Stick Y (cima/baixo) | Velocidade (−1 a +1) |
| Right Stick X (esquerda/direita) | Curva (−1 a +1) |
| Triangle | **Start** — inicia balanceamento |
| Circle | **Stop** — para balanceamento |
| Square | **Calibrate** — recalibra giroscópio |
| D-PAD | Reservado |
| L1/R1/L2/R2/Share/Options | Reservado |

#### Formato dos relatórios HID

O DS4 envia relatórios em dois formatos:

- **Simple (report ID 0x01)**: 9-11 bytes — usado quando não há entrada
  no touchpad/giroscópio. Sticks nos bytes 1-4, botões nos bytes 5-8.
- **Full (report ID 0x11)**: 78 bytes — ativado por leitura do feature
  report 0x05. Inclui touchpad, giroscópio, acelerômetro, bateria.
  Sticks nos bytes 3-6, botões nos bytes 7-9.

| Byte (simple) | Byte (full) | Campo |
|---------------|-------------|-------|
| 1 | 3 | Left Stick X |
| 2 | 4 | Left Stick Y |
| 3 | 5 | Right Stick X |
| 4 | 6 | Right Stick Y |
| 5 | 7 | Botões 0 (D-PAD, Square, Cross, Circle, Triangle) |
| 6 | 8 | Botões 1 (L1, R1, L2, R2, Share, Options, L3, R3) |
| 7 | 9 | Botões 2 (PS, Touchpad) |

#### Edge detection

Os comandos de botão (start/stop/calibrate) disparam apenas na borda
de subida — um `_prev` mantém o estado anterior e compara a cada ciclo.
Isso evita disparos múltiplos enquanto o botão está pressionado.

#### Reconexão automática

Se a conexão cair (DS4 desliga ou sai do alcance), o robô tenta
reconectar automaticamente após 5s, sem intervenção do usuário.
O endereço MAC do último DS4 pareado é mantido em memória.

#### Status no dashboard web

O campo `ds4` no JSON de broadcast indica:
- `"ds4":"conectado XX:XX:XX:XX:XX:XX"` — conectado
- `"ds4":"desconectado"` — desconectado
- `"ds4":"none"` — nunca pareou

---

## Resumo de todos os métodos de controle

| Método | Hardware necessário | Alcance | Latência |
|--------|-------------------|---------|----------|
| Dashboard web + joystick virtual | WiFi (já incluso) | ~30m | ~50ms |
| CLI serial | Cabo USB | 2m | <1ms |
| BLE UART (app celular) | BLE (já incluso) | ~10m | ~50ms |
| DS4 nativo (BLE HID) | BLE (já incluso) | ~10m | ~20ms |

---

## Correções de Build (pós-implementação)

Durante a primeira compilação, foram encontrados e corrigidos:

### 1. `AsyncWebSocket` com cópia deletada

```cpp
// ANTES (ERRO: copy constructor deleted − contém std::mutex)
AsyncWebSocket _ws = AsyncWebSocket("/ws");

// DEPOIS (inicialização direta com braces)
AsyncWebSocket _ws{"/ws"};
```

O mesmo vale para `AsyncWebServer \_server{80}`.

### 2. ArduinoJson v7 API obsoleta

A lib `ArduinoJson ^7.4.2` usa a nova API v7. Correções aplicadas:

| Antes (deprecated) | Depois |
|-------------------|--------|
| `StaticJsonDocument<N>` | `JsonDocument` |
| `DynamicJsonDocument(N)` | `JsonDocument` |
| `doc.containsKey("key")` | `!doc["key"].isNull()` |
| `doc["key"]` (leitura direta) | `doc["key"].as\<float\>()` |
| `doc["key"]` (bool) | `doc["key"].is\<bool\>() \&\& doc["key"].as\<bool\>()` |

Arquivos alterados: `config.cpp`, `web_interface.cpp`.

### 3. Pinos I2C explícitos para ESP32-C3

```cpp
// ANTES: usa pinos padrão da Wire (podem não ser GPIO 8/9)
Wire.begin();

// DEPOIS: explícito para o ESP32-C3 Supermini
Wire.begin(8, 9);
```

### 4. Variável morta removida

Em `balance.cpp`, a variável `dTerm` era calculada mas nunca usada
(o D term real vem do `\_filteredDerivative`). Removida.

### Métricas de build (release)

| Recurso | Uso | Total |
|---------|-----|-------|
| Flash   | 87.0% | 1.310.720 bytes |
| RAM     | 13.8% | 327.680 bytes |
