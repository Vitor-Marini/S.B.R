#!/usr/bin/env python3
"""Controle do robô por gamepad (DS4/PS4 ou qualquer outro) conectado no PC.

A ESP32-C3 só tem Bluetooth LE e o DS4 usa Bluetooth Clássico: por isso o controle
conecta no PC (Bluetooth ou cabo USB) e este programa manda `drive` pela ponte,
igual ao teleop do teclado. Só biblioteca padrão (lê /dev/input/jsX direto).

  Analógico esquerdo (vertical)   → frente/trás
  Analógico direito (horizontal)  → virar
  △ triângulo → start (segure o robô no ponto de equilíbrio)
  ○ círculo   → para de andar (drive 0 0; NÃO desliga o equilíbrio)
  ✕ xis       → stop: DESLIGA o equilíbrio (o robô cai: segure antes)
  □ quadrado  → calibra o gyro (só em IDLE, robô parado no suporte/deitado)
  L1 / R1     → diminui / aumenta a velocidade máxima
  Options     → liga/desliga o MODO MOTORES (só em IDLE, rodas no ar):
                analógico esquerdo (vertical) = roda esquerda, direito = roda direita

Uso:
  python3 tools/gamepad.py            # controla (ponte precisa estar rodando)
  python3 tools/gamepad.py --show     # só mostra eixos/botões, p/ conferir o mapeamento
"""
import glob
import re
import os
import socket
import struct
import sys
import time

# mapeamento do DS4 no driver hid-playstation do Linux (confira com --show)
AXIS_LY, AXIS_RX, AXIS_RY = 1, 3, 4
BTN_CROSS, BTN_CIRCLE, BTN_TRIANGLE, BTN_SQUARE, BTN_L1, BTN_R1, BTN_OPTIONS = 0, 1, 2, 3, 4, 5, 9
STATE_NAMES = {0: "INIT", 1: "CALIBRANDO (deixe parado)", 2: "IDLE", 3: "STARTING (segure na vertical)",
               4: "ACTIVE (equilibrando)", 6: "FAULT (caiu)"}
MOTOR_MAX = 3200.0  # passos/s no analógico todo (1 volta/s em 1/16)

LEVELS = [0.1, 0.2, 0.3, 0.4, 0.5, 0.65]  # m/s no analógico todo pra frente (firmware limita a vmax/2)
W_MAX = 30.0                   # °/s no analógico todo pro lado (60 °/s ainda desestabiliza; ver diário)
DEADZONE = 0.12
PERIOD = 0.05                  # 20 Hz

JS_EVENT = struct.Struct("<IhBB")  # tempo, valor, tipo, número (linux/joystick.h)
JS_BUTTON, JS_AXIS, JS_INIT = 0x01, 0x02, 0x80


def open_joystick():
    devs = sorted(glob.glob("/dev/input/js*"))
    if not devs:
        sys.exit("Nenhum gamepad em /dev/input/js*. Conecte o controle no PC (USB ou Bluetooth).")
    fd = os.open(devs[0], os.O_RDONLY | os.O_NONBLOCK)
    print(f"gamepad: {devs[0]}")
    return fd


def read_events(fd):
    """Devolve a lista de (tipo, número, valor) pendentes, sem bloquear."""
    out = []
    while True:
        try:
            data = os.read(fd, JS_EVENT.size * 32)
        except BlockingIOError:
            return out
        if not data:
            return out
        for i in range(0, len(data) - JS_EVENT.size + 1, JS_EVENT.size):
            _, value, etype, number = JS_EVENT.unpack_from(data, i)
            out.append((etype & ~JS_INIT, number, value))


def shape(raw):
    """Eixo bruto (-32767..32767) → -1..1 com zona morta."""
    x = raw / 32767.0
    if abs(x) < DEADZONE:
        return 0.0
    return (abs(x) - DEADZONE) / (1 - DEADZONE) * (1 if x > 0 else -1)


def show(fd):
    print("Mexa os analógicos e aperte os botões (Ctrl+C sai).")
    while True:
        for etype, number, value in read_events(fd):
            print(f"{'eixo' if etype == JS_AXIS else 'botão'} {number}: {value}")
        time.sleep(0.01)


def main():
    fd = open_joystick()
    if "--show" in sys.argv:
        return show(fd)

    sock = socket.create_connection(("127.0.0.1", 5555))
    sock.setblocking(False)
    axes, level, last, sent_nonzero, motor_mode, robot_state = {}, 1, 0.0, False, False, None
    print("Controlando. △ start, ○ para de andar, ✕ stop, L1/R1 velocidade, Options modo motores. Ctrl+C sai.")
    try:
        while True:
            for etype, number, value in read_events(fd):
                if etype == JS_AXIS:
                    axes[number] = value
                elif etype == JS_BUTTON and value == 1:
                    if number == BTN_TRIANGLE:
                        sock.send(b"start\n")
                    elif number == BTN_SQUARE:
                        # recalibra o zero do gyro: só em IDLE, com o robô parado (no suporte/deitado)
                        if robot_state in (2, None):
                            sock.send(b"cal\n")
                            print("\rcalibrando: deixe o robô parado ~2 s        ")
                        else:
                            print("\rcalibrar só em IDLE: dê stop (✕) segurando o robô antes  ")
                        continue
                    elif number == BTN_CROSS:
                        sock.send(b"drive 0 0\nstop\n")
                        print("\rSTOP: equilíbrio desligado                ")
                    elif number == BTN_OPTIONS:
                        motor_mode = not motor_mode
                        axes.clear()
                        sock.send(b"motors 0 0\ndrive 0 0\n")
                        print("\rMODO MOTORES (só em IDLE, rodas no ar)    " if motor_mode
                              else "\rmodo andar (drive)                        ")
                        continue
                    elif number == BTN_CIRCLE:
                        axes.clear()
                        sock.send(b"drive 0 0\n")
                    elif number == BTN_L1:
                        level = max(level - 1, 0)
                    elif number == BTN_R1:
                        level = min(level + 1, len(LEVELS) - 1)
                    print(f"\rvelocidade máx {LEVELS[level]:.1f} m/s   ", end="", flush=True)

            now = time.time()
            if now - last >= PERIOD:
                last = now
                if motor_mode:
                    # firmware "L" (GPIO0) é a roda ESQUERDA: analógico esquerdo → L, direito → R
                    left = -shape(axes.get(AXIS_LY, 0)) * MOTOR_MAX
                    right = -shape(axes.get(AXIS_RY, 0)) * MOTOR_MAX
                    active, msg = bool(left or right), f"motors {left:.0f} {right:.0f}\n"
                else:
                    v = -shape(axes.get(AXIS_LY, 0)) * LEVELS[level]   # pra cima = negativo no Linux
                    w = -shape(axes.get(AXIS_RX, 0)) * W_MAX           # pra direita = virar à direita (w < 0)
                    active, msg = bool(v or w), f"drive {v:.2f} {w:.0f}\n"
                # igual ao teleop: só manda com o analógico mexido (+ um zero ao soltar),
                # pra não atropelar outros controles; o watchdog do robô para sozinho
                if active or sent_nonzero:
                    sock.send(msg.encode())
                sent_nonzero = active
                try:
                    # mostra quando o estado do robô muda (ex.: △ não engatou porque está em FAULT)
                    for m in re.finditer(rb">state:(\d+)", sock.recv(65536)):
                        st = int(m.group(1))
                        if st != robot_state:
                            robot_state = st
                            print(f"\rrobô: {STATE_NAMES.get(st, st)}                    ")
                except BlockingIOError:
                    pass
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        sock.send(b"drive 0 0\nmotors 0 0\n")
        sock.close()


def _selftest():
    assert shape(0) == 0 and shape(3000) == 0, "zona morta"
    assert abs(shape(32767) - 1) < 1e-6 and abs(shape(-32767) + 1) < 1e-6, "fundo de escala"
    assert 0 < shape(16000) < 1
    ev = JS_EVENT.pack(123, -32767, JS_AXIS | JS_INIT, AXIS_LY)
    r, w = os.pipe()
    os.set_blocking(r, False)
    os.write(w, ev)
    assert read_events(r) == [(JS_AXIS, AXIS_LY, -32767)], "parse do evento"
    print("selftest ok")


if __name__ == "__main__":
    _selftest() if "--selftest" in sys.argv else main()
