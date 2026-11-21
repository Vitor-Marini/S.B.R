#!/usr/bin/env python3
"""Ponte serial: um único dono da porta, vários leitores ao mesmo tempo.

- Este terminal: mostra tudo que o robô imprime e envia o que você digitar.
- TCP localhost:5555: outros clientes (ex.: o Claude) recebem todas as linhas
  e o que enviarem vai para a serial.
- UDP localhost:9870: linhas '>nome:valor' (comando `stream`) viram JSON
  para o PlotJuggler (Streaming → UDP Server, porta 9870, protocolo JSON).

Uso: ~/.platformio/penv/bin/python tools/serial_bridge.py [auto | /dev/ttyACMx]
     ~/.platformio/penv/bin/python tools/serial_bridge.py udp:sbr.local   (sem cabo, via WiFi)
Reconecta sozinho se o cabo cair. Grava todo o `stream` em logs/AAAAMMDD-HHMMSS.csv (--no-log desliga).
"""
import csv
import glob
import json
import os
import socket
import sys
import threading
import time

import serial

ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
PORT = ARGS[0] if ARGS else "auto"
AUTO_STREAM = "--stream" in sys.argv  # liga o stream do robô se ele estiver desligado


def resolve_port():
    """auto: acha a ESP pelo nome fixo (a USB pode voltar como ttyACM1 depois de cair)."""
    if PORT != "auto":
        return PORT
    found = sorted(glob.glob("/dev/serial/by-id/usb-Espressif*")) or sorted(glob.glob("/dev/ttyACM*"))
    if not found:
        raise serial.SerialException("robô não encontrado na USB")
    return found[0]


TCP_PORT, UDP_PORT = 5555, 9870
last_stream = 0.0  # hora da última linha '>nome:valor' recebida
frame = {}         # quadro do stream sendo montado


def flush_frame():
    if frame:
        udp.sendto(json.dumps(frame).encode(), ("127.0.0.1", UDP_PORT))
        log_frame(frame)
        frame.clear()


# log automático: um CSV por sessão em logs/ (na raiz do repo), com tudo que vem do `stream`
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs")
log_file = log_writer = None


def log_frame(f):
    global log_file, log_writer
    if "--no-log" in sys.argv:
        return
    if log_writer is None:
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, time.strftime("%Y%m%d-%H%M%S") + ".csv")
        log_file = open(path, "w", newline="")
        log_writer = csv.DictWriter(log_file, fieldnames=["t"] + list(f), extrasaction="ignore")
        log_writer.writeheader()
        print(f"[gravando log em {os.path.relpath(path)}]", flush=True)
    log_writer.writerow({"t": f"{time.time():.3f}", **f})
    log_file.flush()

class UdpLink:
    """Robô via WiFi (UDP 5555) com a mesma interface read/write da serial."""

    def __init__(self, host):
        self.addr = (socket.gethostbyname(host), 5555)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1)
        self.last_hello = 0.0

    def write(self, data):
        self.sock.sendto(data, self.addr)

    def read(self, n):
        # o robô responde para quem mandou o último pacote: "\n" periódico mantém o registro
        if time.time() - self.last_hello > 2:
            self.last_hello = time.time()
            self.write(b"\n")
        try:
            return self.sock.recv(65536)
        except socket.timeout:
            return b""


ser = None
clients = []
lock = threading.Lock()
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_serial(data: bytes):
    try:
        if ser:
            ser.write(data)
    except (serial.SerialException, OSError):
        pass  # loop de leitura cuida da reconexão


def serve_client(conn):
    with lock:
        clients.append(conn)
    try:
        while data := conn.recv(1024):
            send_serial(data)
    except OSError:
        pass
    finally:
        with lock:
            clients.remove(conn)
        conn.close()


def _selftest():
    """Cliente que nunca lê não trava a ponte e continua conectado; cliente bom segue recebendo."""
    slow_a, slow_b = socket.socketpair()
    good_a, good_b = socket.socketpair()
    for s in (slow_a, good_a):
        threading.Thread(target=serve_client, args=(s,), daemon=True).start()
    time.sleep(0.1)
    t0 = time.time()
    got = 0
    for _ in range(4000):
        handle_line(b">x:1.0" + b" " * 500 + b"\n")
        got += len(good_b.recv(4096))
    time.sleep(0.1)
    assert time.time() - t0 < 5, "ponte travou"
    assert slow_a in clients, "cliente que só escreve foi derrubado"
    sys.argv.append("--no-log")  # o resto do teste não deve criar arquivo em logs/
    assert good_a in clients and got > 1_000_000, "cliente bom parou de receber"
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    global UDP_PORT
    UDP_PORT = rx.getsockname()[1]
    frame.clear()  # sobra do teste anterior (linhas ">x:")
    for l in (b">pitch:6.5\n", b"sbr> >speed:0.1\n", b">temp:30\n"):
        handle_line(l)
    rx.settimeout(1)
    assert json.loads(rx.recv(4096)) == {"pitch": 6.5, "speed": 0.1, "temp": 30.0}, "quadro agrupado"
    print("selftest ok")


def tcp_server():
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", TCP_PORT))
    srv.listen()
    while True:
        conn, _ = srv.accept()
        threading.Thread(target=serve_client, args=(conn,), daemon=True).start()


def stdin_reader():
    for line in sys.stdin:
        send_serial(line.encode())


def handle_line(raw: bytes):
    with lock:
        for c in list(clients):
            try:
                # sem bloquear: cliente que não lê (só manda `drive`) perde linhas, mas continua
                # conectado e comandando; nunca trava a ponte nem os outros clientes
                c.send(raw, socket.MSG_DONTWAIT)
            except BlockingIOError:
                pass
            except OSError:
                try:
                    c.shutdown(socket.SHUT_RDWR)  # cliente morto: serve_client sai e o remove
                except OSError:
                    pass
    line = raw.decode(errors="replace").rstrip()
    while line.startswith("sbr> "):  # prompt do CLI grudado na linha
        line = line[5:]
    if line.startswith(">") and ":" in line:
        global last_stream
        last_stream = time.time()
        name, _, value = line[1:].partition(":")
        try:
            v = float(value)
        except ValueError:
            return
        # um JSON por quadro (todos os sinais juntos): o PlotJuggler se perde com um pacote por valor.
        # O quadro fecha em `temp` (último campo do stream) ou quando um nome se repete.
        if name in frame:
            flush_frame()
        frame[name] = v
        if name == "temp":
            flush_frame()
        return  # não polui o terminal com o stream
    print(line, flush=True)


def main():
    global ser
    threading.Thread(target=tcp_server, daemon=True).start()
    threading.Thread(target=stdin_reader, daemon=True).start()
    print(f"ponte: {PORT} | TCP {TCP_PORT} | PlotJuggler UDP {UDP_PORT}", flush=True)
    while True:
        try:
            if PORT.startswith("udp:"):
                ser = UdpLink(PORT[4:])
            else:
                ser = serial.Serial(resolve_port(), 115200, timeout=0.1)
            print(f"[conectado em {getattr(ser, 'port', PORT)}]", flush=True)
            buf = b""
            connected_at = time.time()
            asked = False
            while True:
                # firmware antigo só tem `stream` liga/desliga: só pede se nada chegou desde a conexão
                if AUTO_STREAM and not asked and time.time() - connected_at > 1.5:
                    asked = True
                    if last_stream < connected_at:
                        send_serial(b"stream on\n")
                buf += ser.read(4096)
                *lines, buf = buf.split(b"\n")
                for l in lines:
                    handle_line(l + b"\n")
        except (serial.SerialException, OSError):
            if ser:
                print("[serial caiu, tentando reconectar...]", flush=True)
            ser = None
            time.sleep(1)


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        _selftest()
        sys.exit()
    try:
        main()
    except KeyboardInterrupt:
        pass
