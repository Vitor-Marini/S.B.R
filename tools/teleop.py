#!/usr/bin/env python3
"""Controle do robô pelo teclado, via ponte serial (tools/serial_bridge.py precisa estar rodando).

Teclas (segure para mover, solte para parar): ↑/w frente, ↓/s trás,
        ←/a vira à esquerda, →/d à direita, +/- muda a velocidade, espaço = parar, q = sair.
O terminal não avisa quando a tecla é solta: o movimento dura HOLD s depois da última
repetição da tecla (o auto-repeat do teclado demora ~0,5 s para começar).

Envia `drive <m/s> <°/s>` a 20 Hz só enquanto uma tecla está segurada (e um zero ao soltar). Se este programa morrer, o robô para sozinho em 0,5 s
(watchdog no firmware). Qualquer outro programa (câmera, IA...) controla o robô do mesmo
jeito: abrir TCP localhost:5555 e mandar linhas `drive v w` repetidamente.

Uso: python3 tools/teleop.py
"""
import curses
import socket
import time

LEVELS = [0.1, 0.2, 0.3, 0.4]  # m/s (o firmware limita a vmax/2)
W_CMD = 60.0                    # °/s
HOLD = 0.6                      # s sem repetição da tecla = soltou
PERIOD = 0.05                   # 20 Hz


def main(scr):
    sock = socket.create_connection(("127.0.0.1", 5555))
    curses.curs_set(0)
    scr.nodelay(True)
    level = 1
    v_dir = w_dir = 0
    v_t = w_t = 0.0
    last = 0.0
    sent_nonzero = False
    try:
        while True:
            now = time.time()
            k = scr.getch()
            while k != -1:
                if k in (ord("q"), ord("Q")):
                    return
                if k in (curses.KEY_UP, ord("w")):
                    v_dir, v_t = 1, now
                elif k in (curses.KEY_DOWN, ord("s")):
                    v_dir, v_t = -1, now
                elif k in (curses.KEY_LEFT, ord("a")):
                    w_dir, w_t = 1, now
                elif k in (curses.KEY_RIGHT, ord("d")):
                    w_dir, w_t = -1, now
                elif k in (ord("+"), ord("=")):
                    level = min(level + 1, len(LEVELS) - 1)
                elif k == ord("-"):
                    level = max(level - 1, 0)
                elif k == ord(" "):
                    v_dir = w_dir = 0
                k = scr.getch()
            if now - v_t > HOLD:
                v_dir = 0
            if now - w_t > HOLD:
                w_dir = 0
            v, w = v_dir * LEVELS[level], w_dir * W_CMD

            if now - last >= PERIOD:
                last = now
                # só manda com tecla segurada (+ um zero ao soltar): não atropela outros controles;
                # o watchdog do robô para sozinho
                if v or w or sent_nonzero:
                    sock.sendall(f"drive {v:.2f} {w:.0f}\n".encode())
                sent_nonzero = bool(v or w)
                try:
                    sock.recv(65536, socket.MSG_DONTWAIT)  # descarta o que a ponte manda
                except BlockingIOError:
                    pass
                scr.erase()
                scr.addstr(0, 0, "S.B.R teleop  (segure setas/WASD, +/- velocidade, espaço = parar, q = sair)")
                scr.addstr(2, 0, f"nível:      {LEVELS[level]:.1f} m/s")
                scr.addstr(3, 0, f"velocidade: {v:+.2f} m/s")
                scr.addstr(4, 0, f"curva:      {w:+.0f} °/s  (+ = esquerda)")
                scr.addstr(6, 0, "lembre: o robô precisa estar em ACTIVE (start)")
                scr.refresh()
            time.sleep(0.005)
    finally:
        sock.sendall(b"drive 0 0\n")
        sock.close()


if __name__ == "__main__":
    curses.wrapper(main)
