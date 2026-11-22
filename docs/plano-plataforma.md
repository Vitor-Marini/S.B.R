# Plano: o S.B.R como plataforma (câmera externa, missões e IA)

Objetivo: usar o robô como base para outros projetos. **O robô só executa** (equilíbrio, velocidade, giro,
segurar posição); **o cérebro fica fora**, num programa no PC que vê o robô por uma câmera externa com tags
ArUco/AprilTag e manda comandos pelo WiFi. Sem SLAM e sem sensor novo no robô.

## Por que dá certo com o que já existe

- API única de movimento `drive <m/s> <°/s>` com **watchdog de 0,5 s**: se o programa do PC travar, o robô para.
- `turn <graus>` (gyro Z) e **segurar posição parado** (odometria dos steppers).
- WebSocket `/ws` no robô (mesmo protocolo do terminal) e TCP 5555 na ponte; `stream` a 50 Hz com `state`,
  `speed`, `yaw`, `yrate`...

## Arquitetura em camadas

```
PC   4. IA (LLM, tool use)   "vá até a tag 5 e volte"        decide O QUÊ (segundos)
     3. Missões              goto 5 → espera 2 s → goto 1
     2. Navegação            goto(x, y): v e ω a 10–20 Hz     decide COMO
     1. Localização          câmera + tags → (x, y, θ) do robô
        ─────────────── WiFi: drive / turn / stream ───────────────
Robô 0. equilíbrio, velocidade, giro, segurar posição          (pronto)
```

Cada camada só fala com a de baixo e é testável sozinha. **Regra:** a IA nunca fica na malha de controle
(responde em segundos; a navegação precisa de comando a cada 50–100 ms). A IA escolhe missões; a navegação dirige.

## Camada 1: localização

- **Tags no chão** em posições medidas (ex.: 4 cantos da área) → homografia pixel → cm no plano do chão.
- **Tag no topo do robô** (6–8 cm) → posição (x, y) e rumo θ (a tag tem orientação).
- OpenCV `cv2.aruco` (já lê `DICT_APRILTAG_36h11`, mais robusta que as 4x4); nada além do opencv.
- Câmera: webcam alta olhando pra baixo (mais simples); alternativa: celular como câmera IP. Kinect fica pra
  depois (profundidade = obstáculos, mas drivers chatos no Linux).
- **Paralaxe:** a tag do topo está a ~20–30 cm do chão; com câmera inclinada isso desloca a posição. Resolver
  com a homografia calibrada na altura do topo, ou câmera calibrada (ChArUco) + `solvePnP`.
- Inclinação do equilíbrio (±1–2°) mexe a tag poucos mm: desprezível.

## Camada 2: navegação

Go-to-goal de robô diferencial:
```python
erro = wrap(atan2(dy, dx) - θ)
w = K_ROT * erro
v = K_LIN * dist * max(0, cos(erro))   # só anda apontado
drive(v, w)                            # 10–20 Hz; chegou (dist < 3 cm) → drive 0 0 e segura posição
```
Depois: lista de waypoints, `goto_tag(id)`, chegar com rumo final, obstáculos marcados por tags + A* em grade.

**Fusão câmera + odometria** (fase 6): câmera = absoluta, lenta, às vezes perde a tag; odometria = rápida,
suave, acumula erro. Filtro complementar (ou EKF), mesma ideia do filtro do pitch.

## Camada 3: missões

```yaml
- goto_tag: 3
- wait: 2
- turn: 180
- goto_tag: 5
- repeat: 3
```
Cada passo: condição de sucesso, timeout, e **aborta se o robô cair em FAULT** (`state` no stream) ou a câmera
perder o robô por mais de ~0,5 s (manda `drive 0 0`).

## Camada 4: IA

LLM com tool use: `goto_tag(id)`, `turn(graus)`, `onde_estou()`, `listar_tags()`, `esperar(s)`. O pedido em
português vira uma sequência de chamadas que o executor da camada 3 cumpre. As ferramentas são o limite do que
a IA pode fazer.

## Ideias de missão

Patrulha entre tags · estacionar na "garagem" alinhado · seguir uma tag (boné/caixa) mantendo distância ·
pista desenhada na tela sobre a imagem da câmera · entrega de objeto leve da tag A à B · pega-pega.

## Fases (cada uma testada antes da próxima)

| Fase | Entrega | Critério de aceite |
|---|---|---|
| 1 | `tools/sbr.py` (SDK: conecta no `/ws`, `drive`, `turn`, estado, callback do stream) + `move <cm>` no firmware | script anda 50 cm, gira 90°, volta, sem a ponte |
| 2 | câmera + tags: mostra robô e tags com (x, y, θ), sem mover nada | posição medida com erro < 3 cm na área |
| 3 | `goto(x, y)` em malha fechada pela câmera | chega a 5 tags seguidas com erro < 5 cm |
| 4 | executor de missões (YAML), abort por FAULT/perda da câmera | patrulha 3 voltas sozinho |
| 5 | IA com tool use sobre as missões | pedido em português vira missão cumprida |
| 6 | fusão câmera + odometria, obstáculos, Kinect | segue andando com a câmera tapada 1 s |

## Material

Webcam (ou celular), suporte alto pra câmera, tags impressas (gerar PDF com `cv2.aruco` no tamanho certo),
área de piso com luz uniforme.

## Riscos

- Latência câmera + WiFi (~100–200 ms): manter velocidades baixas (≤ 0,3 m/s) e ganhos modestos na fase 3.
- Reflexo/brilho na tag do topo; tag pequena demais pra altura da câmera.
- O robô ainda balança ±3 cm parado: tolerância de chegada não pode ser menor que isso.
