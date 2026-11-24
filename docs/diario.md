# Diário de desenvolvimento — S.B.R

> Diagnóstico e histórico dos testes, em ordem. Começou como o plano de refatoração do firmware mk2
> (2026-10-31). Caminhos antigos (`firmware/robot/S.B.R mk2/...`) hoje são `firmware/`, `tools/` e `docs/`.

## Plano de refatoração (original)

Diagnóstico feito em 2026-10-31 lendo o código + testes no robô via serial (`/dev/ttyACM0`).
Não é reescrita: estado/config/CLI servem. É **consertar e enxugar** a cadeia IMU → PID → motor.

---

## Diagnóstico (por que nunca equilibrou)

Ordem = impacto.

| # | Problema | Evidência | Onde |
|---|----------|-----------|------|
| 1 | **O gyro do MPU6050 está com defeito (bloqueia o equilíbrio).** Lê ~1/5 da rotação real no Y (razões 0,17–0,27 entre movimentos) e ~1/9 no Z; quase não vê movimento rápido; bias Y = 126°/s (spec ±20); o bias muda depois de mexer (−0,4 → −1,25°/s); \|acel\| = 1,52 g parado. Registradores conferidos (`regs`): config 100% correta → é o chip (clone/danificado). ~~Hipótese anterior de eixo errado: estava errada~~, o eixo de queda é acel X/Z + gyro Y, o mesmo que o Mahony usava. | `raw`, `angle`, `regs` (Fase 2) | hardware |
| 2 | **LEDC 10-bit não gera < ~76 Hz.** Pedido abaixo disso falha silencioso e o motor fica na frequência anterior. Perto do equilíbrio (velocidades baixas) a roda continua girando na velocidade velha. | Boot: `E ledc: requested frequency ... can not be achieved` | `motor.cpp:52,60-66` |
| 3 | **Full step.** Ruído/vibração forte a 200 passos/s (ressonância), suave a 500. Resolução grosseira + zona morta enorme (76 Hz = 0,38 volta/s). | Teste de motor | jumpers M0-M2 dos DRV8825 |
| 4 | **Rampa em módulo, DIR inverte na hora.** De +500 pra −500 o motor inverte a 500 passos/s → perde passo. | Código | `motor.cpp:88-111` |
| 5 | **Odometria quebrada.** `(int32_t)(|v|*dt)` nunca é negativo e quase sempre trunca pra 0 → malha de velocidade recebe lixo e empurra o offset de ângulo pra um lado. | Código | `motor.cpp:131-132` |
| 6 | **Sem taxa fixa.** `supervisor.run()` roda a cada `loop()`, dt via `millis()` (resolução 1 ms), `dt==0` vira 0,01 (10× maior). Corrompe filtro e termo D. | Código | `main.cpp:71`, todos os `dt` |
| 7 | **Unidades misturadas.** `pitchRate`/`yawRate` em rad/s usados como °/s (STARTING usa `kd * rad/s`; malha de curva compara alvo °/s com yaw rad/s). | Código | `supervisor.cpp:106`, `velocity.cpp:46` |
| 8 | **D derivando erro + filtro 0,8** → atraso e ruído. Deveria usar o gyro direto. | Código | `balance.cpp:33-34` |
| 9 | **Duas malhas de yaw brigando** (heading-hold do balance + turnOffset do velCtrl) e malha de velocidade ligada desde o início. | Código | `balance.cpp:49-63`, `velocity.cpp` |
| 10 | **FAULT nunca sai.** `_runFault` usa `imu.getPitch()` sem `imu.read()` → ângulo congelado >45° pra sempre. | Código | `supervisor.cpp:135-150` |
| 11 | Yaw deriva ~0,75°/s: a calibração do gyro roda **com os motores vibrando** (ver #12) em todo boot. | Teste | `imu_manager.cpp:33` |
| 12 | Motores giram a 1 kHz durante INIT/CALIBRATE (duty 512 no `begin`, set_freq(1) falha). | Código + boot | `motor.cpp:37,48,52` |
| 13 | WiFi AP + BLE UART + scan NimBLE no mesmo núcleo único do C3 durante o equilíbrio. | Código | `main.cpp` |
| 14 | Defaults inconsistentes: código 200 passos/rev / roda 74 mm, docs 3200 / 80 mm. | Código vs `context.md` | `config.hpp` |
| 15 | ~~Bias do gyro Y~~ → incorporado ao #1. | — | — |

Confirmado OK: os dois motores giram suave a 500 passos/s; com o mesmo comando, as duas rodas giram no mesmo sentido (flags `reversed` corretas). O MPU6050 responde.

---

## Fase 0 — Hardware (você)

- [ ] **Microstepping 1/16 nos dois DRV8825**: M2 → 3,3 V (M0, M1 em GND/soltos). `steps_per_rev = 3200`. Resolve ressonância + zona morta. (1/8: M0+M1 → 3,3 V, 1600.)
- [ ] Vref ≈ corrente do motor / 2 (ex.: 1,2 A → 0,6 V).
- [ ] MPU bem preso (fita dupla face/parafuso, sem folga) — vibração vira ruído no ângulo.

**Teste:** `motor left 500` gira ~0,16 volta/s suave e silencioso.

---

## Fase 1 — Camada de motor (`motor.cpp`)

> ✅ Feita em 2026-10-31 e testada no robô (full step): 30 passos/s gira devagar, inversão ±500 limpa, velocidade com sinal no `status`. `tune spr`/`tune accel` adicionados.

- Timer LEDC **12-bit** (faixa ~20 Hz – 19 kHz a 80 MHz); 50% de duty = **2048** (não 512). Abaixo de ~20 passos/s → duty 0 (parado), nunca `set_freq` inválido.
- Canais começam com **duty 0** no `begin()` (hoje giram a 1 kHz durante a calibração do gyro).
- **Rampa em velocidade com sinal**; DIR só muda quando a velocidade rampada cruza zero.
- `ledc_set_freq` / `set_duty` só quando o valor muda.
- **Odometria em float com sinal**: `_pos += v * dt`.
- `max_accel` vira só trava de segurança alta (×16 com 1/16: ~400000 passos/s²) — quem define a aceleração passa a ser o controlador (Fase 3).
- Corrigir defaults em `config.hpp` **e** apagar o `/config.json` gravado (ele tem `steps_per_rev: 200`, `max_accel: 25000` e sobrescreve os defaults): `pio run -t uploadfs`, ou adicionar `spr`/`accel` ao `tune`.

**Teste:** `motor left 30` gira devagar e contínuo; `motor left 2000` → `motor left -2000` inverte sem engasgar; `status` mostra L spd negativo.

---

## Fase 2 — Loop fixo + IMU

> ✅ Código feito em 2026-10-31: tick fixo 200 Hz medido (~1,1 ms/ciclo), I2C 400 kHz, Mahony removido, complementar 1 eixo (`tune abx/abz/alpha`), comandos `raw`, `regs`, `angle` com Acc/Gyro-int/Tick. Acelerômetro OK. **Teste de gyro reprovado → trocar o MPU6050 (ver #1) antes da Fase 3.**
> ✅ 2026-11-08: MPU antigo testado frio (34 °C): razão gyro/acel = 0,22 constante → não era calor, era sensibilidade errada do chip. **MPU trocado**: razão 1,07 / 1,02, bias Y ~1°/s, deriva 0,02°/s, \|acel\| ≈ 1 g. Mesma orientação (frente = +). Fase 2 aprovada. Novos: `stream` (Teleplot 50 Hz) e temperatura no `raw`.

- Um tick de **200 Hz** por `micros()` em `main.cpp`; um `dt` fixo (0,005) para IMU e PID. `Wire.setClock(400000)`.
- Comando `raw` no CLI (ax ay az gx gy gz) → descobrir qual eixo é o de queda com a placa deitada.
- Trocar Mahony por **filtro complementar de 1 eixo** no eixo certo: `ang = 0.98*(ang + gyro°/s*dt) + 0.02*atan2(a_frente, a_vertical)`. Tudo em **graus e °/s**. (Mahony 3D é desnecessário pra equilibrar e é onde o eixo se perdeu.)
- Calibrar gyro só com o robô parado e motores parados.

**Teste:** inclinar ±20° → leitura ±20° (sinal: frente = positivo); deitado de volta → volta ao valor em <1 s; parado → deriva <0,1°/s.

---

## Fase 3 — Equilíbrio só com PD (rádios desligados)

> 🟡 2026-11-08: código feito e gravado (PD→aceleração integrada, kp/kd em m/s², START espera ficar perto do setpoint, FAULT→IDLE corrigido, rádios off via `ENABLE_RADIOS 0`). Sentido conferido. setpoint 5,0°, vmax 0,5.
> 4 tentativas em **full step**: oscilação rápida e crescente (3,4–3,7 Hz) e queda em 2–4 s. kd 0,015 piorou (1,8 Hz, pêndulo sem amortecimento) → kd 0,03 fica. alpha 0,995 não mudou a oscilação rápida. Gyro bateu em ±500°/s na queda.
> **Bloqueio: microstepping 1/16** (passo de 1,2 mm, ressonância em ~0,23 m/s e zona morta na inversão, tudo dentro da faixa do equilíbrio). Depois: `tune spr 3200`, `tune accel 400000` e repetir com kp 0,35 / kd 0,03 / alpha 0,995.
> Tentativa 5 (mouse pad, com grip): 19,7 s em ACTIVE sem FAULT, mas ainda oscilando a 4,7 Hz (±10–20°). **No fim uma roda saiu do eixo** → checar fixação das rodas (folga roda/eixo também gera oscilação). Erro I2C `Error -1` apareceu com motor ligado.
> Tentativa 6 (bateria nova, 25 s ACTIVE): ciclo-limite estável de 4 Hz, ±8°, sem saturar. A velocidade comandada segue o pitch com ~20 ms de atraso (o que o PD prevê); para oscilar a 4 Hz faltam ~40 ms de atraso **entre o comando e a roda**. 29% do tempo abaixo de 60 passos/s, 195 inversões em 25 s → provável latch do LEDC (nova freq só vale no fim do período lento) + full step. Ganhos não resolvem atraso. Bateria fraca derrubou os motores numa tentativa anterior (monitor de bateria sobe de prioridade).
> Fix no firmware (compilado, falta gravar): timer estacionado em 2 kHz quando a velocidade zera; `>tick` no stream. **Microstepping 1/16 agora é obrigatório.** Conferir também Vref e folga das rodas.
> Ferramentas: `tools/serial_bridge.py` (ponte serial → TCP 5555 + PlotJuggler UDP 9870 JSON).

- Desligar WiFi/BLE/DS4 no bring-up (flag em `cfg` ou `#if`).
- `balance.compute()`: PD vira **aceleração** e é integrado em velocidade: `v += (Kp*(ang - setpoint) + Kd*gyroRate) * dt`, clamp em v, v (passos/s) nos dois motores. Gyro em °/s, sem derivar erro. Sem yaw, sem velCtrl.
  - Por quê: stepper impõe velocidade, mas o que levanta o pêndulo é aceleração. PD direto em velocidade é instável com qualquer ganho (falta termo proporcional ao ângulo); integrado, fica `L·s² + Kd·s + (Kp − g)` → estável com Kp acima do limiar e Kd > 0.
- `v = 0` ao entrar em IDLE, FAULT e STARTING.
- Sinal: inclinou pra frente → rodas vão pra frente (testar com a mão antes de soltar).
- `setpoint` = ângulo lido segurando o robô no ponto de equilíbrio.
- Consertar FAULT (`imu.read()` antes de checar) e remover `STATE_MOVING` (duplicado de ACTIVE).
- STARTING usa a mesma função do balance (sem cópia com unidade errada).

**Sintonia:** Kd=0, sobe Kp até oscilar rápido; aí sobe Kd até amortecer; Kp ×0,8.
**Teste:** fica em pé 30 s sozinho (pode derivar devagar pra um lado — normal sem malha de velocidade).

---

## Fase 4 — Malha de velocidade

> 🟡 2026-11-08: **o robô equilibra** (depois de firmar as rodas; mesmos ganhos da Fase 3). Malha de velocidade feita: estimativa = velocidade comandada (stepper), PI → offset de ângulo ±5°, `tune vkp/vki`, stream com `err`/`voff`/`tick` (tick = 200 Hz em ACTIVE ✅). Empurrões leves: recupera em ~1,5 s; um empurrão forte saturou `vmax` 0,5 e caiu → `vmax` 0,8. Deriva pra trás (−0,08 m/s) resolvida com `sp` 5,4 + `vki` 1 (média ≈ 0).
> Aberto: tremor parado subiu de 0,6° (gravação de 10 s logo após firmar rodas) para 1,2–1,7°. A/B: malha de velocidade off → 1,17°; sem o fix do timer → 1,68° (fix mantido no código). Suspeitas físicas: carga da bateria, rodas afrouxando de novo, piso.

- velCtrl lê odometria float da Fase 1; saída = offset de ângulo (±5°), ganho bem menor que o atual.
- **Teste:** fica parado no lugar 60 s; empurrãozinho → volta.

## Fase 5 — Curva, controle, web (um de cada vez)

> 🟡 2026-11-08: API única de movimento `drive <m/s> <°/s>` (+ = frente/esquerda) com **watchdog de 0,5 s** dentro de `setTarget/setTurnRate` (vale p/ CLI, web, BLE, DS4). Alvo de velocidade limitado a vmax/2. Malha de curva PI em m/s de diferença entre rodas (`ykp` 0,001, `yki` 0,01; 0,003 amplificava o tremor). Sinais verificados: giro p/ esquerda = gyro Z+; motor "L" do firmware = roda DIREITA. Teste: +30°/s por 2 s → +60,5° ✅. `tools/teleop.py` (teclado → ponte TCP). Qualquer programa (câmera/IA) controla mandando `drive v w` na TCP 5555.
> Andar p/ frente/trás "não funcionava": robô estava **preso pelo cabo/mão** (segurava +1° de setpoint sem andar). Com cabo folgado anda; deriva p/ trás era `sp` baixo → `sp` 6,8 (voff ≈ 0). Degrau no alvo dava tranco >0,3 m/s → rampa de 0,3 m/s² no alvo. `vkp` 15 instável (fica 4). teleop virou segurar-para-mover.
> Pendente: estimar velocidade pela taxa de passos aplicada (abaixo de 20 passos/s a roda não gira, mas o integral conta).
> Próximo: WiFi/web com a mesma API (medir `tick`), trocar senha do AP; DS4 quando chegar.

- **Uma** malha de yaw só (taxa de curva via gyro Z em °/s; apagar heading-hold do `balance.cpp`).
- Religar DS4 → testar; depois WiFi/web → testar. Logar `dt` máximo do loop; se passar de 6 ms com rádio ligado, rádio é o culpado.

---

## O que sai do código

`MahonyFilter`, `STATE_MOVING`, heading-hold duplicado em `balance.cpp`, cópia do PD em `_runStarting`, `_rampMotor` em módulo, `ComplementaryFilter` genérico (vira 3 linhas no IMU).

---

## 2026-11-10 — microstepping 1/16 e o "não anda pra frente"

- 1/16 ligado (M2 em nível alto): motor suave e silencioso; parado o tremor caiu de 1,5° → **0,08°**. `spr` 3200, `accel` 400000 (padrões do código atualizados).
- `drive` continua sem andar: com `vki` 5 a malha de velocidade pediu +3°, o corpo inclinou +3° (gyroint confirma) e a velocidade ficou ~0, sem cair. Com rodas travadas (IDLE) solto na mesa ele **cai em <1 s** → centro de massa acima do eixo (hipótese de CM baixo descartada).
- **Causa mais provável: MPU mal fixado** (o usuário confirmou que a placa oscila). Se a placa gira em relação ao chassi, o PD equilibra o ângulo da placa, não o do robô: explica o setpoint mudando entre sessões (5,0→6,8), inclinações "seguradas" sem andar, e o reset por mau contato no VCC/GND.
- Próximo: fixar o MPU rígido (parafuso/espaçador ou cola dura, fios com alívio de tensão, longe dos drivers), `cal` parado, remedir `sp`, repetir `drive 0.1`. Só depois mexer em vki/clamp do integral.
- Ponte: modo `auto` acha a ESP por `/dev/serial/by-id` (a USB voltou como ttyACM1 e um `stop` se perdeu).

### 2026-11-10 (tarde) — por que não anda pra frente/trás

- Feedforward de velocidade (roda = equilíbrio + alvo) + integral congelado ao andar: implementado. Resultado: o equilíbrio cancela o feedforward e o robô fica parado 0,5–2° inclinado.
- Roda travada (IDLE) cai rápido **na mesa e no mouse pad** → não há "apoio" do pneu (rodas de PLA). Hipótese do pneu descartada.
- Teste de rigidez (chassi seguro): placa do MPU flexiona até ~0,8° sob o dedo — pouco. **O chassi deforma** (confirmado pelo usuário): o corpo/IMU inclina sem o centro de massa sair de cima do eixo → inclinar não gera aceleração, então nem a malha de velocidade nem o feedforward fazem andar.
- Bloqueio de hardware: chassi rígido (bateria, motores e MPU numa estrutura que não flexione). Depois: `cal` quente, remedir `sp`, `drive 0.1`. Peso no topo é opcional (deixa o pêndulo mais lento e tolerante).
- Gyro deriva ~0,14°/s quando o MPU esquenta (33 → 46 °C, perto dos drivers): recalibrar quente ou afastar.

---

## 2026-11-21 — hardware novo (chassi rígido, IMU sobre os motores, bateria no topo)

- MPU saudável depois da remontagem (regs ok, 0 falhas I2C). Parâmetros do CAD: CG do corpo 37 mm acima do eixo,
  L efetivo = I/(m·d) ≈ 0,10 m. Setpoint remedido: **−0,9°** (robô na vertical, bate com o CAD).
- "Acelera e cai": os **dois motores ficaram invertidos** na remontagem → `tune lrev 1`, `tune rrev 1`
  (comando novo). Frente = lado da chave liga/desliga (referência física; o teste "pra frente" de memória enganou).
- "Gira sem parar": nessa montagem o motor "L" é a roda **esquerda** (medido girando no lugar: `motors 1600 -1600`
  → horário) → sinal da curva invertido em `balance.cpp`; modo motores do gamepad corrigido.
- Resultado: parado com desvio de **0,10°**; curva segue 30 °/s exatos. Andar ainda não: com o integral da
  velocidade **congelado ao andar** (mudança minha de 2026-11-10) sobrava só o P (0,4°), pouco pra vencer a
  resistência à rolagem com CG tão baixo. `vkp` 10 andou devagar mas oscilou → voltou a 4.
- Correção: integral sempre ativo, limitado a ±2° no termo (`I_MAX_DEG`); `vki` 2.
- Depois do teste com `vkp` 10 (sacudiu forte), parado passou a oscilar a **3,4 Hz, ±3–4°** — igual com `vki` 0
  e 1, então não é o integral. Causa: **as duas rodas com folga no eixo** (mesmo sintoma da folga de 2026-11-08).
  Pendente: rodas novas presas direto no eixo; depois testar `drive` com o integral corrigido (`vki` 1, depois 2).

### 2026-11-21 — rodas novas presas no eixo: sintonia

- Parado: desvio de **0,15°**, sem oscilação (a folga das rodas era a causa). Malha de equilíbrio sem mudanças.
- Setpoint ajustado pela compensação da malha de velocidade: **−1,3°**.
- **Primeira vez andando:** `drive 0.1` com o integral sempre ativo. `vki` 1: anda devagar (chega a 0,035 m/s em 5 s).
  `vki` 3: acelera até 0,18 m/s (passa do alvo) e andou 31 cm até bater na borda do mouse pad (o integral encheu
  contra a borda e ele caiu — comportamento esperado de um integral travado). Fica `vki` **2**.
- Próximo: espaço maior (chão) pra testar 0,1–0,2 m/s, ré, empurrão e o DS4.

### 2026-11-21 — comportamento parado, empurrão e joystick

Queixas: cada start se comporta diferente e deriva; empurrado, volta pro lugar e passa; joystick demora a parar;
girar faz andar. Causas: (1) gyro calibrado no boot com o robô na mão (bias ~0,6 °/s); (2) integral da velocidade
zerado a cada start (reaprende o setpoint); (3) bug meu: integral só acumulava com erro < 3 cm/s → deriva maior que
isso nunca era corrigida; (4) integral = memória de posição → volta pro lugar após empurrão.
Mudanças (validadas no simulador do controle.html antes do robô):
- parado: malha de velocidade só P (sem memória de posição); integral só andando, esvazia em 0,5 s ao parar;
- **auto-ajuste do setpoint**: com velocidade estável (aceleração filtrada < 0,15 m/s²), o setpoint segue a inclinação
  pedida pelo P (τ = 1 s) e é salvo ao parar. Simulado: erro de 1° aprendido em ~2 s com 35 cm de deriva no 1º start;
- zero do gyro atualizado sozinho em IDLE quando parado (< 3 °/s, τ = 5 s);
- rampas: velocidade 0,6 m/s², curva 240 °/s²; DS4 limitado a 30 °/s (a 60 °/s ele ainda sai andando/cai — aberto).
- `gxz` (vazamento do giro no gyro de pitch) medido girando à mão: gy ≈ 0,0068·gz → `gxz` −0,007. Não resolveu a
  curva a 60 °/s; a causa principal ali ainda é desconhecida.
- **Auto-ajuste do setpoint removido**: sem travas, aprendia errado a cada queda/segurada (rodas no limite = "velocidade
  estável") e salvava ao cair — levou o setpoint a −8,8°. Com setpoint e gyro certos, 20 s parado ficou em ±5 mm.
  Lição: o simulador não tinha quedas nem robô segurado; testar uma mudança por vez no robô.
- Empurrão: freia e fica onde parou ✅. Andar no chão com `vki` 3 funcionou (arranque ~1,5 s no mouse pad).
- △ não engatava depois de uma batida: gyro com zero errado de ~16 °/s (recalibrado com o robô na mão após a queda).
  Correções: calibração descartada se o gyro variar > 4 °/s; zero corrigido em IDLE quando o acelerômetro diz
  "parado" (corrige erros grandes); △ funciona a partir de FAULT; integral esvazia em 0,2 s; `gamepad.py` mostra o
  estado do robô; ponte grava logs em `logs/`.
- Log de 19:40 (analisar_log.py): 7 starts engataram (inclusive de FAULT); deriva parado ~6 cm/s (setpoint ~0,3°
  errado); trechos a 0,8 m/s sem comando = robô levantado ligado. Mudanças: **FAULT se as rodas ficarem no limite
  por 1 s**; **auto-ajuste do setpoint de volta, com travas** (2 s calmo, |v| < 0,3 m/s, |rate| < 20 °/s,
  |pitch − sp| < 3°, ±1,5° do valor do start, τ 2,5 s; salvo só num `stop`, descartado em FAULT). Simulado com
  setpoint errado (0,4° e 1° corrigidos; 2° fora do limite), empurrão, robô segurado a 2° e 5° (FAULT, setpoint
  intacto), queda (setpoint volta ao do start) e andar.

## 2026-11-22: travadas de 100 ms com o USB conectado

Medindo o maior intervalo entre ciclos (novo no `angle`), apareciam buracos de ~100 ms a cada poucos
segundos com o cabo USB ligado e ninguém lendo a serial: a escrita na USB esperava espaço até o timeout
padrão (100 ms) e travava o loop de controle. `Serial.setTxTimeoutMs(1)` faz desistir em ~2 ms.
**Não usar 0:** no HWCDC do core o contador de tentativas dá a volta e o write trava o loop pra sempre
(com o cabo ligado e a serial fechada). Foi o que fez a página web abrir sem receber nenhum dado.
Pode ter atrapalhado testes antigos feitos no cabo sem a ponte aberta.

## 2026-11-22: giro com feedforward, `turn <graus>` e segurar posição parado

- **Giro lento:** a curva era só PI sobre o erro (`ykp` 0,001), então o PI tinha que "descobrir" a diferença
  entre rodas. Agora há feedforward `ω·bitola/2` (bitola 150 mm) e o PI só corrige o resto. Limite da página
  e do `gamepad.py` subiu de 30 para 45 °/s.
- **`turn <graus>`** (+ = esquerda): integra o gyro Z, taxa `min(√(a·falta), 4·falta, 45 °/s)`, termina com
  falta < 1° ("giro ok"), desiste em 10 s; `drive`/`stop` cancelam. Botões ↺90/180/↻90 na página, setas ←→ no DS4.
  Testado: chega perto do ângulo pedido, mas o robô se deslocava girando → resolvido pelo item abaixo.
- **Segurar posição parado:** âncora quando a rampa zera e a roda < 3 cm/s; volta a até 8 cm/s (`HOLD_KP` 1,5/s);
  passou de 50 cm (carregado), a âncora vem junto. Acabou o "recuo" ao soltar o analógico e o deslocamento no giro.
- **Oscilação lenta (±17 cm, ~15 s)** logo depois: o auto-ajuste do setpoint (τ 2,5 s) lia o `voff` da volta à
  posição como erro de setpoint e empurrava o setpoint (+0,16° → −0,64° em 6 s): duas malhas integrando na mesma
  escala de tempo. τ do auto-ajuste → 10 s: sobra ±3 cm / ~5 s, setpoint firme em ~1,25°.
- "Anda sempre pra frente, mudar o setpoint não muda nada" = sintoma de `drive` chegando (andando, o integral
  come o erro de setpoint). Suspeita: analógico do DS4 com deriva; a página agora mostra os analógicos crus.

## 2026-11-22 (noite): move, zero do gyro corrigido equilibrando, trajeto 2D

- **`move <cm>`** + `tools/sbr.py` (SDK pelo WebSocket, sem a ponte). Frear "no ponto" passava 20–40 cm (o robô
  freia a ~0,15 m/s² e antes acelera pra inclinar pra trás). Virou **trajetória de referência** (posição + velocidade,
  trapezoidal a 0,15 m/s²); no move o integral de velocidade fica de fora (somava com a posição: 0,36 m/s com alvo
  de 0,2; com `vki` 0, 0,24 m/s). `move ok` só quando assenta (senão o `turn` seguinte começava com o robô voltando
  → arco: era o "infinito" no demo). Demo: move 50 → 49,6 cm, mas 15 s (atrasa ~1 s, passa 15 cm, vai e volta ±10 cm).
- **Gyro Z**: parado sem comando o yaw andava ~1,7 °/s (rodas: 0°) → zero do Z corrigido equilibrando sem comando
  de giro (τ 5 s). Depois: 0,8° em 17 s.
- **Gyro Y**: pitch ficava ~1,1° acima do acelerômetro equilibrando (zero do gyro × τ do filtro) → o setpoint
  "certo" mudava a cada sessão. Filtro complementar com integral (τ 20 s): diferença → ~0,1°. Setpoint real
  (acelerômetro com o robô equilibrado, 2 gravações): **−1,7°**. O suporte segura em −1,5°.
- Auto-ajuste rápido (τ 3 s) só com a posição saturada; lento (τ 10 s) perto do lugar. `HOLD_KP` 1,5 → 0,7.
- Página: trajeto 2D ao vivo com rumo das rodas (`head`, novo no stream) e do gyro; analógicos crus.
- **Limite da cascata**: a malha de velocidade tem ~1 s de atraso (inclinar antes de andar/frear); toda malha de
  posição em cima (parado, move, empurrão) ou oscila ou demora. Parado: ±2,5 cm. Giro 180°: gyro 179°, rodas 189°
  (patinou ou bitola ≠ 150 mm). Próximo grande passo: **LQR** (branch `feature/lqr-control`).

## 2026-11-22 (madrugada): LQR vira o padrão

- `tools/lqr.py`: modelo com aceleração da roda como entrada (stepper) e L medido 0,09 m; simulação com atraso,
  ruído, perturbação calibrada pelo tremor real e limites. A cascata simulada reproduz o robô (recua antes de sair,
  passa 8–15 cm) — por isso deu pra confiar no LQR simulado.
- Firmware: `ctl lqr|pid`, `a = kt·θ + kw·ω + kx·(x−x_ref) + kv·(v−v_ref) + a_ref`, θ_ref = a_ref/g; ganhos "suave"
  (`lkt` 0,744, `lkw` 0,0707, `lkx` 27,67, `lkv` 14,04), salvos como padrão.
- Tropeços: (1) termo de posição saturava entre 11 e 30 cm (referência só vinha junto a 30 cm, teto 3 m/s²) →
  empurrão virava vai-e-vem crescente; (2) pedia 11 m/s² nos empurrões (a roda não acompanha) → teto 4 m/s²;
  (3) com referência a 8 cm e ganhos fracos, 1,5° de setpoint/piso fazia andar pra trás a 0,1 m/s → limite = mín(0,3 m,
  2,5/kx). Ganhos mais fracos ("S6") aguentam mais atraso, mas o suave com essas correções ficou melhor.
- Resultado: parado ±0,3–1 cm (cascata ±2,5–4); ~30 empurrões e direção pelo analógico com 1 queda (pancada que levou
  a roda a 0,8 m/s, o limite). **Demo**: move 50 → 48,0 cm em 4,8 s; turn 180 → 179,0°; move 50 → 48,8 cm; turn 180 →
  179,1°; terminou a ~10 cm do início (cascata: 15 s por move e falhava). Rodas contam ~5 % a mais no giro que o gyro
  (bitola real ≠ 150 mm?).
- Aberto: rejuntes devagar às vezes derrubam (roda patina ou perde passo? Vref nunca medido); limite de 0,8 m/s.

## 2026-11-23: giro calibrado

- `turn ±1800` com fita: gyro errou ~3° em 5 voltas nos dois sentidos (`gzs` 1,002). Rodas contaram 4,9 % (esq.,
  com rejunte) e 8,7 % (dir., sem rejunte) a mais com bitola 150 mm → `track` 0,160 (média). Rodas não estão
  frouxas: a diferença é escorregamento desigual dos pneus no giro no lugar. `turn` para a 0,3° (antes 1°).
- Demo depois: move 48,8 / 48,9 cm em 4,8 s, turn 179,7° / 179,7°. **Medido com fita: terminou a ~0,5 cm do
  início e < 1° torto.** Na volta do meio o robô estava visivelmente fora da linha (deslocou no 1º giro); o 2º giro,
  simétrico, compensou.
