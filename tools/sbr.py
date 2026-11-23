#!/usr/bin/env python3
"""SDK do S.B.R: controla o robô direto pelo WiFi (WebSocket /ws do robô), sem a ponte serial.

Uso como biblioteca (docs/plano-plataforma.md, fase 1):

    from sbr import Robot
    r = Robot("sbr.local")          # ou o IP
    r.wait_state(r.ACTIVE)          # dê △ (start) e espere equilibrar
    r.move(50); r.turn(180)         # bloqueiam até "move ok"/"giro ok"; devolvem cm/graus feitos
    r.drive(0.2, 0)                 # andar livre: repetir a 10–20 Hz (watchdog de 0,5 s no robô)
    print(r.frame["pitch"])         # último quadro do stream (50 Hz)
    r.close()

Linha de comando:
    python3 tools/sbr.py [host] demo   → anda 50 cm, gira 180°, volta 50 cm, gira 180° (robô equilibrando)
    python3 tools/sbr.py [host] <comando do terminal...>   → manda e mostra as respostas por 2 s
    python3 tools/sbr.py selftest
Só biblioteca padrão.
"""
import base64
import os
import queue
import re
import socket
import struct
import sys
import threading
import time


class Robot:
    INIT, CALIBRATE, IDLE, STARTING, ACTIVE, MOVING, FAULT = range(7)

    def __init__(self, host="sbr.local", port=80, on_frame=None):
        self.frame = {}           # último quadro completo do stream
        self.on_frame = on_frame  # callback(frame) a cada quadro (roda na thread de leitura)
        self.lines = queue.Queue()  # linhas de texto do terminal (respostas, avisos)
        self._cur, self._buf = {}, ""
        self._sock = socket.create_connection((host, port), timeout=5)
        _handshake(self._sock, host)
        self._sock.settimeout(None)
        self._lock = threading.Lock()
        self._alive = True
        threading.Thread(target=self._reader, daemon=True).start()
        self.send("stream on")

    # --- comandos
    def send(self, line):
        with self._lock:
            self._sock.sendall(_ws_frame(line.encode()))

    def drive(self, v, w=0.0):
        """m/s (+ frente), °/s (+ esquerda). Vale 0,5 s: repetir enquanto quiser andar."""
        self.send(f"drive {v:.3f} {w:.1f}")

    def halt(self):
        self.send("drive 0 0")

    def start(self):
        self.send("start")

    def stop(self):
        self.send("drive 0 0")
        self.send("stop")

    def calibrate(self):
        self.send("cal")

    def move(self, cm, timeout=20):
        """Anda cm (+ frente) e para no alvo. Devolve os cm andados; erro se o robô recusar/desistir."""
        return self._until(f"move {cm:.1f}", r"move (ok|desistiu): (-?[\d.]+)", "move:", timeout)

    def turn(self, deg, timeout=15):
        """Gira no lugar (+ esquerda). Devolve os graus girados (gyro Z)."""
        return self._until(f"turn {deg:.1f}", r"giro (ok|desistiu).*?: (-?[\d.]+)", "turn:", timeout)

    def wait_state(self, state, timeout=60):
        t = time.time()
        while self.frame.get("state") != state:
            if time.time() - t > timeout:
                raise TimeoutError(f"estado {state} não chegou (agora: {self.frame.get('state')})")
            time.sleep(0.05)

    def close(self):
        self._alive = False
        try:
            self.halt()
            self._sock.close()
        except OSError:
            pass

    # --- interno
    def _until(self, cmd, ok_re, refuse_prefix, timeout):
        while not self.lines.empty():  # descarta respostas antigas
            self.lines.get_nowait()
        self.send(cmd)
        t = time.time()
        while time.time() - t < timeout:
            try:
                line = self.lines.get(timeout=0.2)
            except queue.Empty:
                continue
            if line.startswith(refuse_prefix):
                raise RuntimeError(line)
            m = re.search(ok_re, line)
            if m:
                if m.group(1) != "ok":
                    raise RuntimeError(line)
                return float(m.group(2))
        self.halt()
        raise TimeoutError(f"sem resposta para '{cmd}'")

    def _reader(self):
        try:
            for text in _ws_messages(self._sock, self._lock):
                self._feed(text)
        except OSError:
            pass
        if self._alive:
            self.lines.put("!! conexão com o robô caiu")

    def _feed(self, text):
        self._buf += text
        *lines, self._buf = self._buf.split("\n")
        for line in lines:
            while line.startswith("sbr> "):
                line = line[5:]
            m = re.match(r">(\w+):(\S+)", line)
            if m:
                try:
                    self._cur[m.group(1)] = float(m.group(2))
                except ValueError:
                    continue
                if m.group(1) == "temp":  # último campo do quadro
                    self.frame, self._cur = self._cur, {}
                    if self.on_frame:
                        self.on_frame(self.frame)
            elif line.strip():
                self.lines.put(line.strip())


# --- WebSocket mínimo (RFC 6455): cliente manda frames mascarados, servidor manda sem máscara
def _handshake(sock, host):
    key = base64.b64encode(os.urandom(16)).decode()
    sock.sendall((f"GET /ws HTTP/1.1\r\nHost: {host}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                  f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
    resp = b""
    while b"\r\n\r\n" not in resp:
        chunk = sock.recv(1024)
        if not chunk:
            raise ConnectionError("robô fechou a conexão no handshake")
        resp += chunk
    if b" 101 " not in resp.split(b"\r\n", 1)[0]:
        raise ConnectionError("handshake recusado: " + resp.split(b"\r\n", 1)[0].decode(errors="replace"))


def _ws_frame(payload, opcode=1):
    mask = os.urandom(4)
    n = len(payload)
    head = bytes([0x80 | opcode])
    if n < 126:
        head += bytes([0x80 | n])
    elif n < 65536:
        head += bytes([0x80 | 126]) + struct.pack(">H", n)
    else:
        head += bytes([0x80 | 127]) + struct.pack(">Q", n)
    return head + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(payload))


def _recv_exact(sock, n):
    data = b""
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise ConnectionError("conexão fechada")
        data += chunk
    return data


def _ws_messages(sock, lock):
    """Gera as mensagens de texto do servidor; responde ping, junta fragmentos."""
    parts = b""
    while True:
        b0, b1 = _recv_exact(sock, 2)
        op, n = b0 & 0x0F, b1 & 0x7F
        if n == 126:
            n = struct.unpack(">H", _recv_exact(sock, 2))[0]
        elif n == 127:
            n = struct.unpack(">Q", _recv_exact(sock, 8))[0]
        mask = _recv_exact(sock, 4) if b1 & 0x80 else None
        data = _recv_exact(sock, n)
        if mask:
            data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        if op == 8:
            return
        if op == 9:
            with lock:
                sock.sendall(_ws_frame(data, 10))
            continue
        if op in (0, 1):
            parts += data
            if b0 & 0x80:
                yield parts.decode(errors="replace")
                parts = b""


def demo(r):
    print("esperando o robô equilibrar (dê △ / start)...")
    r.wait_state(r.ACTIVE)
    time.sleep(1)
    for step in [("move", 50), ("turn", 180), ("move", 50), ("turn", 180)]:
        print(f"{step[0]} {step[1]} → ", end="", flush=True)
        print(f"{getattr(r, step[0])(step[1]):.1f}")
        time.sleep(1)  # deixa a posição assentar


def _selftest():
    r = Robot.__new__(Robot)
    r.frame, r.on_frame, r.lines, r._cur, r._buf = {}, None, queue.Queue(), {}, ""
    r._feed("sbr> >pitch:1.5\n>sta")
    r._feed("te:4\n>temp:35\nmove ok: 49.8 cm\n")
    assert r.frame == {"pitch": 1.5, "state": 4.0, "temp": 35.0}, r.frame
    assert r.lines.get_nowait() == "move ok: 49.8 cm"
    f = _ws_frame(b"drive 0 0")
    assert f[0] == 0x81 and f[1] == 0x80 | 9
    unmasked = bytes(b ^ f[2 + i % 4] for i, b in enumerate(f[6:]))
    assert unmasked == b"drive 0 0"
    assert re.search(r"giro (ok|desistiu).*?: (-?[\d.]+)", "giro desistiu (travado/no suporte?): 12.0°").group(2) == "12.0"
    a, b = socket.socketpair()
    a.sendall(bytes([0x01, 3]) + b"abc" + bytes([0x80, 3]) + b"def" + bytes([0x89, 0]))
    gen = _ws_messages(b, threading.Lock())
    assert next(gen) == "abcdef"
    print("selftest ok")


if __name__ == "__main__":
    args = sys.argv[1:]
    if args == ["selftest"]:
        _selftest()
        sys.exit()
    host = args.pop(0) if args and args[0] not in ("demo",) and "." in args[0] else "sbr.local"
    robot = Robot(host)
    try:
        if args == ["demo"]:
            demo(robot)
        elif args:
            robot.send(" ".join(args))
            t = time.time()
            while time.time() - t < 2:
                try:
                    print(robot.lines.get(timeout=0.2))
                except queue.Empty:
                    pass
        else:
            print(__doc__)
    finally:
        robot.close()
