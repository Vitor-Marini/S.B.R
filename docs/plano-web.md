# Plano: página web de controle servida pelo robô

Objetivo: usar o robô sem PC. O robô serve uma página (celular) com joystick, DS4, gráficos ao vivo,
console e ajustes. As ferramentas do PC (ponte, PlotJuggler, logs, gamepad.py) continuam funcionando.

## Decisões (2026-11-22)

- **Rede:** WiFi de casa quando disponível (`http://sbr.local` ou o IP); senão o robô cria a própria rede
  (`SBR`, com senha definida pelo usuário) em `http://192.168.4.1`.
- **Aparelho:** celular primeiro (joystick na tela, DS4 pareado no celular via Gamepad API).
- **Sem PIN** (rede de casa confiável; a rede própria do robô tem senha). Risco aceito: qualquer um na rede
  de casa pode abrir a página.

## Arquitetura

- ESPAsyncWebServer + WebSocket (`/ws`). Um WebSocket por cliente carrega o **mesmo protocolo de texto**
  do terminal: entra `drive v w`, `start`, `tune ...` (vai pra `cli.run`); sai o `stream` (um JSON por quadro,
  50 Hz) e o texto do terminal (`term`).
- Página **embutida no firmware** (gzip, ~10 KB), não no LittleFS: `uploadfs` apagaria o `/config.json`.
- Gráficos em canvas na própria página (como o simulador do controle.html): funciona sem internet.
- Joystick e DS4 mandam `drive` a 20 Hz só enquanto há comando (watchdog de 0,5 s no robô, como hoje).

## Fases (cada uma testada no robô antes da próxima)

| Fase | Entrega | Critério de aceite |
|---|---|---|
| 1 | servidor web + WebSocket ligado ao terminal (**feito**) | `tick` em 200 Hz com a página aberta e o stream ligado |
| 2 | página: estado, start/stop/calibrar, joystick, gráficos, console (**feito, testado 2026-11-22**) | controlar pelo celular sem PC |
| 3 | DS4 (Gamepad API) e teclado (**feito junto com a fase 2**) | dirigir com o DS4 pareado no celular |
| 4 | painel de ajustes (kp, kd, vkp, vki, sp, vmax) | mudar `vkp` pela página |
| 5 | rede própria do robô quando não acha o WiFi de casa | usar fora de casa |

## Riscos

- Núcleo único: servir a página compete com o loop de controle. Se o `tick` cair na fase 1, mover o controle
  para uma tarefa FreeRTOS de prioridade alta.
- Flash: o servidor web soma ~100–150 KB (hoje 65% usada).
- `sbr.local` não resolve em alguns Android: a página/terminal mostram o IP.
- Gamepad API: navegadores recentes podem exigir contexto seguro (https), e a página é `http://`. Testar no
  celular antes de desenhar a fase 3 em cima dela.
- Fase 5: `web.begin()` hoje só roda quando o WiFi de casa conecta; no modo rede própria tem que chamar também.

## Fase 1: medido (2026-11-22, IDLE, PC a −44 dBm)

| | tick | pior run | maior intervalo |
|---|---|---|---|
| sem página | 200 Hz | ~0,9–1,7 ms | 5–10 ms |
| WebSocket aberto, stream + `drive` a 20 Hz | 200 Hz | ~1,1–2 ms | 6–12 ms |

Medido só pelo WiFi (serial fechada): stream chega a 50 quadros/s, nada descartado. A saída é acumulada
enquanto há mensagem a caminho (a lib só manda a próxima depois da confirmação) e o TCP vai sem Nagle.
O stream sai em `cli.update()`, fora do `supervisor.run()`: o aumento do "pior run" é a tarefa do servidor
(AsyncTCP) interrompendo o loop no meio do controle. Cabe no orçamento de 5 ms; falta medir equilibrando com o celular.
Lib: ESPAsyncWebServer 3.6.0 + AsyncTCP 3.6.1 (linha ESP32Async, com trava interna para `textAll` de outra tarefa).
