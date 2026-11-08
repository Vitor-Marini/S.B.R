# Tutorial de Uso — S.B.R mk2

> ⚠️ Na fase de bring-up (WiFi/BLE desligados) siga **[COMO_RODAR.md](COMO_RODAR.md)**. Este tutorial descreve o uso completo com dashboard e controle.

Guia prático para configurar, conectar e operar o robô equilibrista.

---

## 1. Pré-requisitos de Hardware

Antes de ligar o robô, complete o checklist em `todos/hardware_checklist.md`:
- [ ] Medir diâmetro das rodas
- [ ] Verificar microstepping dos DRV8825
- [ ] Testar sentido de rotação dos motores
- [ ] Verificar VREF (corrente dos motores)

---

## 2. Gravar Firmware

Conecte o ESP32-C3 via USB e execute:

```bash
# Compilar e gravar firmware
pio run --target upload

# Gravar sistema de arquivos (dashboard web)
pio run --target uploadfs
```

Após gravar, abra o Monitor Serial:

```bash
pio device monitor -b 115200
```

Você deve ver:

```
=== S.B.R mk2 ===
MPU6050 inicializado com DLPF 44Hz
Calibrando giroscópio...
Gyro offsets: X=0.00 Y=0.00 Z=0.00
Calibração concluída.
WiFi AP: 192.168.4.1
Web server iniciado.
BLE UART: SBR_CONTROL pronto

S.B.R mk2 — Terminal Interativo
Digite 'help' para comandos disponíveis.
sbr>
```

---

## 3. Formas de Conectar

### 3.1 Terminal Serial (via USB)

- **Baud rate**: 115200
- **Comandos**: digite `help` para lista completa
- **Latência**: <1ms (mais responsivo)

### 3.2 WiFi — Dashboard Web

O robô cria uma rede WiFi própria:

| Campo | Valor |
|-------|-------|
| SSID | `SBR_ROBOT` |
| Senha | `12345678` |
| Endereço | `http://192.168.4.1` |

Conecte seu celular/notebook a essa rede e abra o navegador no endereço acima.

### 3.3 Bluetooth (BLE UART)

- **Nome do dispositivo**: `SBR_CONTROL`
- **Serviço**: Nordic UART Service (NUS)
- **UUID RX**: `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
- **UUID TX**: `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`

Use um app de terminal BLE (ex: "Serial Bluetooth Terminal" no Android ou "LightBlue" no iOS).

---

## 4. Tour pela Página Web

Abra `http://192.168.4.1` no navegador.

### 4.1 Barra de Status
Mostra o estado atual do robô com cores:

| Cor | Estado | Significado |
|-----|--------|-------------|
| Cinza | IDLE | Parado, esperando comando |
| Laranja | STARTING | Rampa de inicialização (2s) |
| Verde | ACTIVE | Equilíbrio ativo |
| Azul | MOVING | Andando (ACTIVE + velocidade) |
| Vermelho | FAULT | Ângulo excedeu limite, motores parados |

### 4.2 Cartões de Telemetria
- **Pitch**: ângulo atual em graus
- **Vel**: velocidade estimada (m/s)
- **Alvo V**: velocidade alvo (m/s) — definida pelo joystick
- **Alvo T**: curva alvo (°/s) — definida pelo joystick
- **Output**: torque do PID de equilíbrio (-100 a +100)

### 4.3 Gráfico de Ângulo
Mostra os últimos 80 pontos do pitch em tempo real (Chart.js).

### 4.4 Botões de Controle
- **INICIAR**: começa o balanceamento (transição IDLE → STARTING → ACTIVE)
- **PARAR**: para o balanceamento (volta a IDLE)
- **CALIBRAR**: recalibra o giroscópio (robô deve estar parado)

### 4.5 Joystick Virtual
- **Desktop**: clique e arraste com o mouse
- **Mobile**: toque e arraste com o dedo
- **Eixo Y** (cima/baixo): velocidade (frente/trás, até ±1.0 m/s)
- **Eixo X** (esquerda/direita): curva (até ±180°/s)
- Ao soltar, velocidade e curva voltam a zero

### 4.6 Sliders PID
Ajuste Kp, Ki, Kd e setpoint em tempo real. As alterações são salvas na flash automaticamente.

### 4.7 Exportar CSV
Baixa os dados de ângulo coletados durante a sessão.

---

## 5. Operação Passo a Passo

### Primeira inicialização (teste de motores)

```bash
sbr> motor left 500    # motor esquerdo gira
sbr> motor right 500   # motor direito gira
sbr> motor left 0      # para
sbr> motor both 1000   # ambos giram
```

Verifique o sentido. Se estiver errado, ajuste `leftReversed`/`rightReversed` no config.

### Iniciar balanceamento

1. Coloque o robô sobre as rodas, segurando-o na vertical
2. Digite `start` (ou clique INICIAR no dashboard)
3. O robô fará uma rampa de 2s (STARTING) e entrará em ACTIVE
4. **Solte o robô lentamente** — ele deve se equilibrar

### Ajuste fino (tuning)

Se o robô oscilar ou cair:

```bash
sbr> tune kd 1.5    # aumenta amortecimento
sbr> tune kp 30     # mais agressivo
sbr> tune ki 0.3    # menos integral
sbr> tune sp 1.0    # inclina setpoint (anda para frente)
```

**Regra geral**:
- **Oscila rápido**: aumenta Kd (amortecimento)
- **Oscila lento** (vai e volta): diminui Kp
- **Cai para um lado**: aumenta Kp + verifica setpoint
- **Erro estacionário** (inclinado sem oscilar): aumenta Ki

### Andar (joystick)

1. Robô em ACTIVE
2. Arraste o joystick para cima → robô anda para frente
3. Arraste para os lados → robô curva
4. Solte → robô para

### Comandos úteis

```bash
sbr> status    # ver todos parâmetros
sbr> angle     # ler ângulo atual
sbr> save      # salvar configuração na flash
sbr> vel 0.5   # andar a 0.5 m/s (sem joystick)
sbr> turn -30  # curvar 30°/s à esquerda
sbr> reset     # zerar odometria e integrais
```

---

## 6. Controle via BLE

Conecte ao dispositivo `SBR_CONTROL` pelo app de terminal BLE.

| Comando | Exemplo | Efeito |
|---------|---------|--------|
| `start` | `start` | Iniciar balanceamento |
| `stop` | `stop` | Parar balanceamento |
| `f 0.5` | `f 0.5` | Andar 0.5 m/s para frente |
| `v -0.3` | `v -0.3` | Andar 0.3 m/s para trás |
| `t 45` | `t 45` | Curva 45°/s à direita |
| `turn -30` | `turn -30` | Curva 30°/s à esquerda |
| `s` | `s` | Parar (zera velocidade e curva) |

---

## 7. Recuperação de Falhas

Se o robô cair (ângulo > 45°):

1. Estado muda para **FAULT** (barra vermelha)
2. Motores desligam imediatamente
3. Coloque o robô na vertical novamente
4. Após 3 segundos com ângulo < 15°, ele volta automaticamente a **IDLE**
5. Digite `start` para tentar novamente

**Se o robô cair repetidamente**: reduza Kp, aumente Kd, e verifique se o
centro de massa está sobre o eixo das rodas.

---

## 8. Dicas de Segurança

- **Sempre segure o robô ao ligá-lo** — ele pode sair disparado
- Use uma **bateria carregada** (tensão baixa = torque fraco)
- Teste em **superfície plana e antiderrapante**
- Comece com **Kp baixo (10-15)** e aumente gradualmente
- Se o robô vibrar/tremer: **aumente Kd** ou **reduza Kp**
- Mantenha o **USB conectado** nos primeiros testes para ver logs
- O BLE e WiFi funcionam simultaneamente, mas podem sofrer interferência
  se usados ao mesmo tempo

---

## 9. Problemas Comuns

| Problema | Causa provável | Solução |
|----------|---------------|---------|
| Motor não gira | Direção errada | `tune left_reversed` ou `right_reversed` |
| Robô cai para frente | Setpoint incorreto | Ajuste `tune sp` (valor negativo se cai para frente) |
| Robô oscila rápido | Kd baixo demais | Aumente Kd |
| Robô oscila lento | Kp alto demais | Reduza Kp |
| WiFi não aparece | LittleFS não gravado | Execute `pio run --target uploadfs` |
| BLE não conecta | Múltiplos dispositivos | Desligue BLE de outros dispositivos |
| Angulo preso em 0 | MPU6050 não comunicando | Verifique fiação I2C (GPIO 8/9) |
| Reset loop | Bateria fraca | Carregue a bateria |
