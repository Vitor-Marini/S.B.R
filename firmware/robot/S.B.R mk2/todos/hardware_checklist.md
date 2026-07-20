# Hardware Checklist — S.B.R mk2

Tarefas físicas para preparar o robô para o novo firmware.

---

## 1. Medir o diâmetro das rodas

- [ ] Medir o diâmetro externo da roda em **milímetros**
- [ ] Anotar o valor em `config.hpp` como `WHEEL_DIAMETER_MM`
- Necessário para odometria e futura velocidade linear

---

## 2. Descobrir o microstepping dos DRV8825

Os pinos M0, M1, M2 dos drivers **não estão conectados ao ESP32-C3**.  
Eles estão configurados via jumpers/solda nos próprios módulos DRV8825.

- [ ] Inspecionar fisicamente os dois drivers DRV8825
- [ ] Verificar se há jumpers (shunts) em M0, M1, M2
- [ ] Se não houver jumpers: provavelmente **full step** (200 passos/volta)
- [ ] Anotar a configuração e o steps_per_rev resultante

Tabela de configuração típica (DRV8825):

| M2  | M1  | M0  | Microstepping | Steps/rev |
|-----|-----|-----|---------------|-----------|
| LOW | LOW | LOW | Full step     | 200       |
| LOW | LOW | HIGH| 1/2 step      | 400       |
| LOW | HIGH| LOW | 1/4 step      | 800       |
| LOW | HIGH| HIGH| 1/8 step      | 1600      |
| HIGH| LOW | LOW | 1/16 step     | 3200      |
| HIGH| LOW | HIGH| 1/32 step     | 6400      |

Valores típicos para jumpers abertos (sem shunt): todos LOW = **full step**.

---

## 3. Verificar ENABLE dos DRV8825

- [ ] Verificar se o pino ENABLE de cada DRV8825 está com pull-up ou pull-down
- [ ] Se pino ENABLE solto: o módulo tem pull-up interno → motor habilitado
- [ ] Se pino ENABLE conectado ao GND via resistor: motor sempre habilitado
- [ **Ideal:** Conectar ENABLE a um GPIO do ESP32-C3 para desligar motores em IDLE/FAULT

Se não for possível conectar ao ESP32, deixar ENABLE como está (sempre habilitado).

---

## 4. Definir o sentido de rotação dos motores

- [ ] Alimentar o motor esquerdo com DIR=HIGH e verificar se a roda gira para **frente** (robô anda para frente)
- [ ] Se não: anotar que o motor esquerdo precisa de DIR invertido (`MOTOR_LEFT_REVERSED = true` no config)
- [ ] Repetir para o motor direito
- [ ] Atualizar `MOTOR_LEFT_REVERSED` e `MOTOR_RIGHT_REVERSED` no `config.hpp`

---

## 5. Verificar tensão de referência dos DRV8825 (corrente do motor)

- [ ] Medir a tensão no potenciômetro de cada DRV8825 (VREF)
- [ ] A corrente máxima do motor NEMA17 é tipicamente 1.0-1.5A
- [ ] VREF ideal ≈ 0.7V a 1.2V (depende do resistor sense)
- [ ] Fórmula: I_max = VREF × 2 (para DRV8825 com resistor sense de 0.1Ω)

---

## 6. Nivelamento físico

- [ ] Verificar se o MPU6050 está montado nivelado (paralelo ao chão)
- [ ] Se não estiver nivelado, anotar o offset mecânico
- [ ] Verificar se as rodas estão bem fixas e alinhadas
- [ ] Verificar se o centro de massa está acima do eixo das rodas (ou ligeiramente atrás)

---

## 7. Bateria

- [ ] Bateria Makita BL1016 (10.8V/12V) — verificar tensão nominal
- [ ] Verificar se há divisor de tensão para monitoramento pela ADC do ESP32-C3
- [ ] Se houver, anotar os resistores do divisor (R1, R2) para configurar no firmware

---

## 8. Teste de LEDC PWM (pré-balanceamento)

Após gravar o firmware mk2 pela primeira vez:

- [ ] Conectar via Serial (115200 baud)
- [ ] Digitar `motor left 500` — o motor esquerdo deve girar a 500 steps/s
- [ ] Digitar `motor right 500` — o motor direito deve girar a 500 steps/s
- [ ] Digitar `motor left 0` — o motor esquerdo deve parar
- [ ] Verificar se o sentido está correto (usar `motor left -500` para inverter)

---

## Configuração da RoboCORE (preencher após medir)

```cpp
// Em include/config.hpp
WHEEL_DIAMETER_MM =  // <-- preencher
STEPS_PER_REV =      // <-- preencher (200 × microstepping)
MOTOR_LEFT_REVERSED  = false  // <-- ajustar se necessário
MOTOR_RIGHT_REVERSED = false  // <-- ajustar se necessário
```

---
