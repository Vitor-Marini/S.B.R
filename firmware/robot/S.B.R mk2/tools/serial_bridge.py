#!/usr/bin/env python3
"""Ponte serial: um único dono da porta, vários leitores ao mesmo tempo.

- Este terminal: mostra tudo que o robô imprime e envia o que você digitar.
- TCP localhost:5555: outros clientes (ex.: o Claude) recebem todas as linhas
  e o que enviarem vai para a serial.
- UDP localhost:9870: linhas '>nome:valor' (comando `stream`) viram JSON
  para o PlotJuggler (Streaming → UDP Server, porta 9870, protocolo JSON).

Uso: ~/.platformio/penv/bin/python tools/serial_bridge.py [/dev/ttyACM0]
Reconecta sozinho se o cabo cair.
"""
import json
import socket
import sys
import threading
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
TCP_PORT, UDP_PORT = 5555, 9870

ser = None
clients = []
lock = threading.Lock()
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_serial(data: bytes):
    try:
        if ser:
            ser.write(data)
    except serial.SerialException:
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
                c.sendall(raw)
            except OSError:
                pass
    line = raw.decode(errors="replace").rstrip()
    while line.startswith("sbr> "):  # prompt do CLI grudado na linha
        line = line[5:]
    if line.startswith(">") and ":" in line:
        name, _, value = line[1:].partition(":")
        try:
            udp.sendto(json.dumps({name: float(value)}).encode(), ("127.0.0.1", UDP_PORT))
        except ValueError:
            pass
        return  # não polui o terminal com o stream
    print(line, flush=True)


def main():
    global ser
    threading.Thread(target=tcp_server, daemon=True).start()
    threading.Thread(target=stdin_reader, daemon=True).start()
    print(f"ponte: {PORT} | TCP {TCP_PORT} | PlotJuggler UDP {UDP_PORT}", flush=True)
    while True:
        try:
            ser = serial.Serial(PORT, 115200, timeout=0.1)
            print(f"[conectado em {PORT}]", flush=True)
            buf = b""
            while True:
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
    try:
        main()
    except KeyboardInterrupt:
        pass
