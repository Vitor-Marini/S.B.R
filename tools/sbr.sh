#!/usr/bin/env bash
# Abre o PlotJuggler (layout do robô, UDP 9870 já ouvindo) e a ponte neste terminal,
# com o stream ligado. Digite comandos do robô aqui; Ctrl+C fecha a ponte.
#
#   tools/sbr.sh          # via WiFi (sbr.local)
#   tools/sbr.sh usb      # via cabo USB
#   tools/sbr.sh 192.168.0.50   # via WiFi pelo IP, se sbr.local não resolver
#
# Controle pelo DS4 em outro terminal: python3 tools/gamepad.py
set -euo pipefail
cd "$(dirname "$0")"

case "${1:-wifi}" in
    wifi) LINK="udp:sbr.local" ;;
    usb)  LINK="auto" ;;
    *)    LINK="udp:$1" ;;
esac

if ! pgrep -x plotjuggler >/dev/null; then
    # cópia na pasta do snap: o lançador do PlotJuggler quebra caminhos com espaço
    LAYOUT="$HOME/snap/plotjuggler/common/sbr_layout.xml"
    mkdir -p "$(dirname "$LAYOUT")"
    cp sbr_layout.xml "$LAYOUT"
    plotjuggler --nosplash --layout "$LAYOUT" --start_streamer DataStreamUDP >/dev/null 2>&1 &
fi

exec ~/.platformio/penv/bin/python serial_bridge.py "$LINK" --stream
