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
- **Controle**: PD com derivativo do gyro (não do ângulo)

```
torque = Kp × (setpoint − pitch) + Kd × (−pitch_rate)
```

Usar o gyro rate direto no termo derivativo elimina o ruído amplificado
que aconteceria ao derivar o ângulo.

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

---

## Configuração Inicial (após montagem)

Antes de tentar equilibrar, complete os passos em
[`todos/hardware_checklist.md`](../todos/hardware_checklist.md):

1. Medir diâmetro das rodas → `wheel_diameter_mm`
2. Verificar microstepping → `steps_per_rev`
3. Testar sentido dos motores via CLI → `left_reversed` / `right_reversed`
4. Com `start`, observar se o robô tenta se equilibrar
5. Ajustar Kp, Ki, Kd pelos sliders do dashboard ou CLI (`tune`)
