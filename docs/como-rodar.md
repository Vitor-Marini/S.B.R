# Como rodar o S.B.R

Guia do dia a dia: gravar, abrir a ponte, ver gráficos, dar start e ajustar.
O robô fala pela USB **ou** pelo WiFi (UDP, mesmo protocolo). Hardware em [hardware.md](hardware.md);
o histórico de diagnóstico em [diario.md](diario.md); a teoria de controle em [controle.html](controle.html).

Todos os comandos abaixo rodam a partir da **raiz do repositório**:

```bash
cd ~/personal/dev/S.B.R
```

---

## 1. Gravar o firmware

A porta USB só pode ter **um dono**: feche a ponte em modo USB (Ctrl+C) antes de gravar.
A ponte em modo WiFi (`udp:`) não usa a USB e pode ficar aberta (a ESP reinicia: deixe o robô parado).

```bash
~/.platformio/penv/bin/pio run -d firmware -t upload
```

A configuração (ganhos, setpoint etc.) fica em `/config.json` na flash e **sobrevive** à gravação.

---

## 1b. Atalho: tudo de uma vez (WiFi + PlotJuggler + ponte)

```bash
tools/sbr.sh            # via WiFi (sbr.local); tools/sbr.sh usb = pelo cabo
python3 tools/gamepad.py   # em outro terminal: DS4 conectado no PC (USB ou Bluetooth)
```

Abre o PlotJuggler com o layout `tools/sbr_layout.xml` (abas *Equilibrio* e *Saude*) já ouvindo
a UDP 9870, liga o `stream` e deixa este terminal como terminal do robô.
A rede WiFi é salva uma vez pela USB com `wifi <rede> <senha>` (2,4 GHz, nome sem espaços).
O DS4 **não** conecta direto na ESP32-C3 (ela só tem Bluetooth LE; o DS4 é Bluetooth Clássico).

DS4: analógico esquerdo = frente/trás, direito = virar · △ start · ○ para de andar · ✕ **stop** (desliga
o equilíbrio: segure o robô) · L1/R1 velocidade · **Options** = modo motores (só em IDLE, rodas no ar:
cada analógico vertical gira uma roda, até 1 volta/s).

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
3. No terminal da ponte, digite `stream on` (envio a 50 Hz; `stream off` desliga). O `tools/sbr.sh` já faz isso.
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
| `vtgt` / `wtgt` | velocidade (m/s) e curva (°/s) **pedidas** pelos comandos (`drive`, teclado, DS4) |
| `yrate` | taxa de giro (°/s, + = esquerda) |
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
   **Cabo USB folgado** (volta grande, sem arrastar) e mãos fora: cabo puxando ou mão encostando seguram o robô e escondem o comportamento real (já enganou o diagnóstico).
   **Não mande `stop` com ele em pé**: os motores param e ele cai. Segure antes.
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

## 4b. Controlar pelo PC (teclado)

Com a ponte rodando e o robô em ACTIVE, em outro terminal:

```bash
python3 tools/teleop.py
```

Setas/WASD aceleram e viram, **espaço para**, `q` sai. O programa manda `drive <m/s> <°/s>` a 20 Hz.
Se ele fechar ou travar, o robô para sozinho em 0,5 s (watchdog).

**Qualquer programa pode controlar o robô do mesmo jeito** (câmera, IA, script): abrir TCP `localhost:5555`
e mandar linhas `drive v w` repetidamente (10–20 Hz). `+v` = frente, `+w` = virar à esquerda.
A velocidade é limitada a `vmax/2` e a curva a ±180 °/s.

---

## 5. Comandos do terminal

| Comando | O que faz |
|---------|-----------|
| `start` / `stop` | liga / desliga o equilíbrio |
| `drive <m/s> <°/s>` | andar/virar (+ = frente / esquerda). Vale por 0,5 s: repita a 10–20 Hz |
| `motors <L> <R>` | teste remoto dos motores em passos/s (só IDLE). Vale por 0,5 s. "L" do firmware = roda **esquerda** |
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
| `sp` | 6.8 | setpoint (°): ângulo do ponto de equilíbrio |
| `vmax` | 0.8 | limite de velocidade da roda (m/s) |
| `vkp` | 4 | malha de velocidade: ° de inclinação por m/s de erro. Amortece o "vai e volta" depois de um empurrão |
| `vki` | 1 | integral da velocidade (testar 2 com as rodas firmes): vence a resistência à rolagem ao andar, tira a deriva e resiste a puxão (limitado a ±2°) |
| `ykp` | 0.001 | curva: (m/s de diferença entre rodas) por °/s de erro |
| `yki` | 0.01 | integral da curva |
| `alpha` | 0.995 | filtro: quanto confia no gyro (0,98 = mais acelerômetro) |
| `spr` | 3200 | passos por volta: 200 em full step, **3200 em 1/16 (atual)** |
| `accel` | 400000 | trava de aceleração do motor (passos/s²). 25000 em full step |
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
