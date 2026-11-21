#!/usr/bin/env python3
"""Analisa um log gravado pela ponte (logs/*.csv) e resume a sessão.

Uso:
  python3 tools/analisar_log.py                 # o log mais recente
  python3 tools/analisar_log.py logs/ARQ.csv    # um log específico

Mostra: linha do tempo dos estados, qualidade do equilíbrio parado, cada `start` (engatou?),
cada comando de andar (tempo pra arrancar e pra parar, distância), cada curva (girou o pedido?
saiu do lugar?) e o que aconteceu no segundo antes de cada queda.
"""
import csv
import glob
import os
import statistics as st
import sys

STATES = {0: "INIT", 1: "CALIBRANDO", 2: "IDLE", 3: "STARTING", 4: "ACTIVE", 5: "MOVING", 6: "FAULT"}
HZ = 50  # o stream manda 50 quadros/s


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                rows.append({k: float(v) for k, v in r.items()})
            except (TypeError, ValueError):
                continue  # linha cortada (ponte fechada no meio da escrita)
    t0 = rows[0]["t"]
    for r in rows:
        r["t"] -= t0
    return rows


def segments(rows, key):
    """Trechos contíguos com o mesmo valor (ou mesmo "tem/não tem") de `key`."""
    out, cur = [], None
    for i, r in enumerate(rows):
        v = r[key]
        if cur is None or v != cur[0]:
            cur = [v, i, i]
            out.append(cur)
        else:
            cur[2] = i
    return out


def moving_segments(rows, key):
    """Trechos em que `key` != 0 (comando ativo), juntando ruído de 1 quadro."""
    out, start = [], None
    for i, r in enumerate(rows):
        on = abs(r[key]) > 1e-6
        if on and start is None:
            start = i
        if not on and start is not None:
            out.append((start, i - 1))
            start = None
    if start is not None:
        out.append((start, len(rows) - 1))
    return out


def dist(rows, a, b):
    return sum(rows[i]["speed"] / HZ for i in range(a, b + 1))


def first(rows, a, b, cond):
    for i in range(a, b + 1):
        if cond(rows[i]):
            return i
    return None


def analyze(rows):
    out = []
    p = out.append
    dur = rows[-1]["t"]
    p(f"Duração {dur:.1f} s, {len(rows)} quadros")

    segs = segments(rows, "state")
    p("Estados: " + " → ".join(f"{STATES.get(int(s), int(s))}({(rows[b]['t'] - rows[a]['t']):.1f}s)"
                              for s, a, b in segs))

    low = [r for r in rows if r["tick"] < 195]
    p(f"Loop: tick {min(r['tick'] for r in rows):.0f}..{max(r['tick'] for r in rows):.0f} Hz"
      + (f" ({len(low)} quadros < 195 Hz)" if low else ""))

    act = [r for r in rows if r["state"] == 4]
    if not act:
        p("Nunca entrou em ACTIVE: nada de equilíbrio pra analisar.")
        return out
    # "parado" = ACTIVE, sem comando há 2 s (fora da frenagem) e longe (1 s) de uma queda
    busy = set()
    for i, r in enumerate(rows):
        if r["vtgt"] or r["wtgt"]:
            busy.update(range(i, i + 2 * HZ))
        if r["state"] == 6:
            busy.update(range(i - HZ, i + 1))
    still = [r for i, r in enumerate(rows) if r["state"] == 4 and i not in busy]
    p(f"\nEquilíbrio (ACTIVE {len(act) / HZ:.1f} s, parado {len(still) / HZ:.1f} s)")
    if len(still) > HZ:
        p(f"  parado: desvio do pitch {st.pstdev([r['pitch'] for r in still]):.2f}°, "
          f"|rotação| máx {max(abs(r['rate']) for r in still):.0f}°/s, "
          f"velocidade média {st.mean(r['speed'] for r in still):+.3f} m/s (deriva)")
        voff = st.mean(r["voff"] for r in still)
        if abs(voff) > 0.2:
            p(f"  a malha de velocidade compensa {voff:+.2f}° em média: some isso ao setpoint (tune sp <atual {voff:+.2f}>)")

    # cada start
    for k, (s, a, b) in enumerate(segs):
        if int(s) == 3:
            nxt = int(segs[k + 1][0]) if k + 1 < len(segs) else None
            ok = "engatou (ACTIVE)" if nxt == 4 else f"NÃO engatou (foi pra {STATES.get(nxt, nxt)})"
            pit = [rows[i]["pitch"] for i in range(a, b + 1)]
            p(f"  start em t={rows[a]['t']:.1f}s: {ok} em {(rows[b]['t'] - rows[a]['t']):.1f}s; "
              f"pitch durante {min(pit):+.1f}..{max(pit):+.1f}°")

    # comandos de andar
    drives = moving_segments(rows, "vtgt")
    if drives:
        p("\nAndar")
    for a, b in drives:
        tgt = max((rows[i]["vtgt"] for i in range(a, b + 1)), key=abs)
        go = first(rows, a, b, lambda r: abs(r["speed"]) >= 0.8 * abs(tgt) and r["speed"] * tgt > 0)
        end = min(b + 5 * HZ, len(rows) - 1)
        stop = first(rows, b + 1, end, lambda r: abs(r["speed"]) < 0.02) if b + 1 <= end else None
        err = st.mean(abs(rows[i]["vtgt"] - rows[i]["speed"]) for i in range(a, b + 1))
        p(f"  t={rows[a]['t']:.1f}s alvo {tgt:+.2f} m/s por {(b - a + 1) / HZ:.1f}s: "
          + (f"arrancou (80%) em {(rows[go]['t'] - rows[a]['t']):.1f}s" if go is not None else "NÃO chegou a 80% do alvo")
          + f", andou {dist(rows, a, b):+.2f} m, erro médio {err:.3f} m/s; "
          + (f"parou {(rows[stop]['t'] - rows[b]['t']):.1f}s depois de soltar, +{abs(dist(rows, b + 1, stop)):.2f} m"
             if stop is not None else "não parou em 5 s"))

    # curvas
    turns = [(a, b) for a, b in moving_segments(rows, "wtgt") if b - a > HZ // 2]
    if turns:
        p("\nCurvas")
    for a, b in turns:
        tgt = max((rows[i]["wtgt"] for i in range(a, b + 1)), key=abs)
        mid = range(a + (b - a) // 3, b + 1)
        moving = any(abs(rows[i]["vtgt"]) > 1e-6 for i in range(a, b + 1))
        p(f"  t={rows[a]['t']:.1f}s alvo {tgt:+.0f}°/s por {(b - a + 1) / HZ:.1f}s: girou {st.mean(rows[i]['yrate'] for i in mid):+.0f}°/s"
          + ("" if moving else f", saiu do lugar {dist(rows, a, b):+.2f} m"))

    # quedas
    falls = [(a, b) for s, a, b in segs if int(s) == 6]
    if falls:
        p("\nQuedas (FAULT)")
    for a, _ in falls:
        w = [rows[i] for i in range(max(0, a - HZ), a)]
        if w:
            p(f"  t={rows[a]['t']:.1f}s: no segundo antes → pitch {min(r['pitch'] for r in w):+.1f}..{max(r['pitch'] for r in w):+.1f}°, "
              f"velocidade {min(r['speed'] for r in w):+.2f}..{max(r['speed'] for r in w):+.2f} m/s"
              + (" (no limite!)" if max(abs(r['speed']) for r in w) > 0.75 else "")
              + f", comando v={w[-1]['vtgt']:+.2f} w={w[-1]['wtgt']:+.0f}")
    return out


def _selftest():
    rows, t = [], 0.0
    def add(n, **kw):
        nonlocal t
        for _ in range(n):
            r = dict(t=t, pitch=-1.3, acc=-1.3, gyroint=0, rate=0, yaw=0, speed=0, accel=0, err=0, voff=0,
                     vtgt=0, yrate=0, wtgt=0, state=2, tick=200, temp=35)
            r.update(kw)
            rows.append(r)
            t += 1 / HZ
    add(50)
    add(20, state=3)
    add(100, state=4)
    add(25, state=4, vtgt=0.1, speed=0.0)
    add(75, state=4, vtgt=0.1, speed=0.1)
    add(10, state=4, speed=0.05)
    add(50, state=4)
    add(30, state=4, speed=0.8, pitch=30)
    add(20, state=6)
    text = "\n".join(analyze(rows))
    assert "engatou (ACTIVE)" in text, text
    assert "arrancou (80%) em 0.5s" in text, text
    assert "parou 0.2s depois" in text, text
    assert "(no limite!)" in text, text
    print("selftest ok")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        _selftest()
        sys.exit()
    logs = sys.argv[1:] or sorted(glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs", "*.csv")))[-1:]
    if not logs:
        sys.exit("Nenhum log em logs/. A ponte grava um quando recebe o stream.")
    print(f"== {logs[-1]} ==")
    print("\n".join(analyze(load(logs[-1]))))
