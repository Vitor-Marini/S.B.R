# Como rodar o S.B.R mk2 (bring-up, Fase 3)

Guia curto do dia a dia: gravar, abrir a ponte, ver gráficos, dar start e ajustar.
Nesta fase **WiFi, BLE e DS4 estão desligados** (`ENABLE_RADIOS 0` em `src/main.cpp`), então tudo é pela USB.
O plano e o histórico dos testes estão em `todos/plano_refatoracao.md`.

Todos os comandos abaixo rodam a partir desta pasta:

```bash
cd "/home/vitorsynkar/personal/dev/S.B.R/firmware/robot/S.B.R mk2"
```

---

## 1. Gravar o firmware

A porta USB só pode ter **um dono**: feche a ponte (Ctrl+C) antes de gravar.

```bash
~/.platformio/penv/bin/pio run -t upload --upload-port /dev/ttyACM0
```

A configuração (ganhos, setpoint etc.) fica em `/config.json` na flash e **sobrevive** à gravação.

---

## 2. Abrir a ponte serial

A ponte é o único programa que abre a USB. Por ela, ao mesmo tempo:
- **este terminal** vira o terminal do robô (o que você digitar vai pro CLI);
- o **PlotJuggler** recebe os gráficos (UDP 9870);
- outros programas (ex.: o Claude) se conectam pela TCP 5555.

```bash
~/.platformio/penv/bin/python tools/serial_bridge.py
```

Deve aparecer `[conectado em /dev/ttyACM0]`. Se o cabo cair, ela reconecta sozinha.

---

## 3. Ver os gráficos no PlotJuggler

1. Instalar uma vez: `sudo snap install plotjuggler`
2. Abrir `plotjuggler` → **Streaming** → **UDP Server** → porta **9870**, protocolo **JSON** → **Start**.
3. No terminal da ponte, digite `stream` (liga/desliga o envio a 50 Hz).
4. Arraste as curvas da lista da esquerda para o gráfico. Botão direito → *Split* cria mais gráficos.

| Curva | O que é |
|-------|---------|
| `pitch` | ângulo filtrado (°). Positivo = inclinado para a frente |
| `acc` | ângulo só do acelerômetro (°). Ruidoso com o motor ligado, é normal |
| `gyroint` | gyro integrado (°), diagnóstico. Deve acompanhar `acc` ao inclinar devagar |
| `rate` | velocidade angular (°/s) |
| `speed` | velocidade comandada da roda (m/s) |
| `accel` | saída do PD = **controle** (m/s² pedidos à roda) |
| `err` | **erro** de ângulo: pitch − (setpoint + voff) (°) |
| `voff` | offset de ângulo pedido pela malha de velocidade (°) |
| `state` | estado do robô (tabela abaixo) |
| `tick` | taxa do loop de controle (deve ficar em 200) |
| `temp` | temperatura do MPU (°C) |

| `state` | Nome | Significado |
|---------|------|-------------|
| 0 | INIT | procurando o MPU |
| 1 | CALIBRATE | calibrando o gyro (deixe parado) |
| 2 | IDLE | parado, motores sem pulso |
| 3 | STARTING | esperando você segurar no ponto de equilíbrio |
| 4 | ACTIVE | equilibrando |
| 6 | FAULT | passou de 45°, motores parados |

---

## 4. Rodar um teste de equilíbrio

1. **Superfície com grip** (mouse pad, borracha). Na mesa lisa a roda patina e ele cai.
2. Ligue a **bateria dos motores** com o robô em IDLE.
3. Robô com as **rodas no chão**, você segurando por cima, com a mão solta, no ponto de equilíbrio.
   - **Não dê `start` com ele no suporte**: se ele já estiver no ângulo do setpoint, entra em ACTIVE na hora e gira as rodas no ar.
4. No terminal da ponte: `start`.
   - Fica em STARTING até passar 0,3 s a menos de 3° do setpoint, e aí vai para ACTIVE.
   - Se não chegar lá em 10 s, volta para IDLE.
5. Alivie a mão aos poucos, sem soltar de vez.
6. Para parar: `stop`.
7. Se cair (> 45°) → FAULT. Coloque em pé: depois de 3 s ele volta para **IDLE** (não volta a equilibrar sozinho, de propósito). Para tentar de novo, `start`.

---

## 5. Comandos do terminal

| Comando | O que faz |
|---------|-----------|
| `start` / `stop` | liga / desliga o equilíbrio |
| `state` | estado atual |
| `status` | config completa + telemetria |
| `angle` | pitch, ângulo do acelerômetro, gyro integrado, taxa do loop (deve ser 200 Hz) |
| `raw` | acelerômetro, gyro e temperatura brutos |
| `regs` | registradores do MPU (esperado: `75=68 6B=01 19=00 1A=03 1B=08 1C=10`) |
| `cal` | recalibra o gyro (robô parado, só em IDLE) |
| `stream` | liga/desliga os gráficos |
| `motor left\|right\|both <passos/s>` | teste de motor, só em IDLE (rodas no ar!) |
| `tune <param> <valor>` | ajusta e **salva** na flash (tabela abaixo) |

### Parâmetros do `tune`

| Param | Atual | Unidade / efeito |
|-------|-------|------------------|
| `kp` | 0.35 | (m/s²) por grau. Força para voltar ao setpoint. Mínimo físico ≈ 0,17 |
| `kd` | 0.03 | (m/s²) por °/s. Amortece. **Baixar piorou** (testado) |
| `sp` | 5.0 | setpoint (°): ângulo do ponto de equilíbrio |
| `vmax` | 0.5 | limite de velocidade da roda (m/s) |
| `vkp` | 4 | malha de velocidade: ° de inclinação por m/s de erro. Amortece o "vai e volta" depois de um empurrão |
| `vki` | 0 | integral da velocidade: faz voltar ao lugar. Comece em 0 |
| `alpha` | 0.995 | filtro: quanto confia no gyro (0,98 = mais acelerômetro) |
| `spr` | 200 | passos por volta: 200 em full step, **3200 em 1/16** |
| `accel` | 25000 | trava de aceleração do motor (passos/s²). **400000 em 1/16** |
| `abx`, `abz` | 0 | bias do acelerômetro (m/s²) |

### Como ajustar o PD

Mude **uma coisa por vez** e grave o teste (com o `stream` ligado).
- Oscila devagar (~1–2 Hz) e cai → aumente o `kd` (ou diminua um pouco o `kp`).
- Mole, tomba devagar sem reagir → aumente o `kp`.
- Treme rápido (3–5 Hz) → em full step é o motor; veja a seção 6.
- Anda sempre para um lado → ajuste o `sp` aos poucos (±0,5°) para o lado contrário. A malha de velocidade (Fase 4) vai resolver isso de vez.

---

## 6. Depois de trocar para microstepping 1/16

1. Nos dois DRV8825: **M2 → 3,3 V** (M0 e M1 em GND/soltos).
2. No terminal: `tune spr 3200` e `tune accel 400000`.
3. Teste com as rodas no ar: `motor both 3200`, uma volta por segundo, suave e silencioso. Depois `motor both 0`.
4. Repita o teste de equilíbrio com os mesmos `kp`/`kd`. Como os ganhos estão em m/s, não mudam.

---

## 7. Problemas comuns

| Sintoma | Causa provável |
|---------|----------------|
| `pio upload` falha / "port busy" | ponte aberta: Ctrl+C nela |
| `/dev/ttyACM0` some ao inclinar | cabo USB ruim/curto (já aconteceu) |
| `i2cWriteReadNonStop returned Error -1` no log | leitura do MPU falhou: fio I2C solto/longo ou ruído dos motores. Prenda e encurte os fios SDA/SCL |
| `angle` mostra `Tick` bem abaixo de 200 Hz | algo travando o loop |
| `gyroint` não acompanha `acc` ao inclinar | gyro ruim (o primeiro MPU tinha esse defeito, ver plano #1) |
