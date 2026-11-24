# Plano: controle por estado (LQR)

> Historical plan (Portuguese), kept as written. Current docs: [labs/03-lqr](../../labs/03-lqr/).

Por quê: hoje é uma cascata (equilíbrio PD → velocidade PI → posição P). Cada malha espera a de dentro, e a de
velocidade tem ~1 s de atraso (o robô inclina antes de andar e antes de frear). Resultado medido em 2026-11-22:
parado ±2,5 cm, `move` passa ~15 cm e vai e volta ±10 cm, toda malha de posição rápida oscila. Com LQR, um único
cálculo usa os 4 estados juntos e "sabe" que precisa inclinar antes: para rápido sem passar do ponto.

## Ideia

`aceleração da roda = −(k1·pitch + k2·pitch_rate + k3·posição + k4·velocidade)` (mais o giro, separado)
Os ganhos saem de um modelo do robô (pêndulo invertido sobre rodas) e de pesos Q/R que dizem o que importa mais
(ficar em pé, ficar no lugar, não acelerar demais). Stepper ajuda: a roda faz a aceleração mandada (entrada = aceleração).

## Passos

1. Medir: massa, altura do centro de massa (pendurar/balança), raio da roda. Registrar em `docs/hardware.md`.
2. Modelo linear (x, ẋ, θ, θ̇) com entrada = aceleração da roda; LQR no PC (`tools/lqr.py`, numpy/scipy).
3. Simular junto com o atraso do loop, o ruído medido do pitch (±0,2°, ±4 °/s) e a saturação dos motores.
4. Firmware: modo `ctl lqr` ao lado do atual (troca ao vivo; o PD+cascata continua como fallback), ganhos por `tune`.
5. Testar: parado (meta ±1 cm), empurrão, `move 50` (meta: sem passar, < 5 s), andar pelo analógico.
6. Integrador de posição/rampa de referência pra `move` e `drive` em cima do LQR.

## Riscos

Modelo errado (centro de massa mal medido) → ganhos ruins: começar com Q conservador. Ruído do pitch_rate
multiplicado por ganho alto → tremor: filtrar ou reduzir peso. Hardware improvisado (folgas) limita o mínimo.
