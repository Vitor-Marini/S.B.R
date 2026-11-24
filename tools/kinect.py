#!/usr/bin/env python3
"""Kinect v1 (Xbox 360) pelo libfreenect_sync via ctypes: imagem colorida + profundidade em mm.

Fase 2 da plataforma (docs/plano-plataforma.md), passo 1: ver o que o Kinect vê.
Precisa do libfreenect0.5 (já instalado: apt libfreenect-dev) e do Kinect na fonte + USB.

    python3 tools/kinect.py           → ao vivo: profundidade com o robô marcado + mapa visto de cima com o trajeto;
                                        clique na profundidade = distância no ponto; c limpa o trajeto; q sai
    python3 tools/kinect.py snap DIR  → salva DIR/cor.png e DIR/prof.png (pra conferir sem tela)
    python3 tools/kinect.py tilt GRAUS → motor de inclinação (−27 a +27; negativo = olhar pra baixo)
    python3 tools/kinect.py chao DIR   → acha o plano do chão e o robô (bolha acima do chão); salva DIR/chao.png
    python3 tools/kinect.py selftest
"""
import ctypes
import os
import sys

import numpy as np

W, H = 640, 480
DEPTH_MM = 5    # freenect_depth_format: FREENECT_DEPTH_MM (registrado na imagem colorida, em mm; 0 = sem leitura)
VIDEO_RGB = 0   # freenect_video_format: FREENECT_VIDEO_RGB

_lib = None


def _load():
    global _lib
    if _lib is None:
        _lib = ctypes.CDLL("libfreenect_sync.so.0.5")
        for fn in (_lib.freenect_sync_get_depth, _lib.freenect_sync_get_video):
            fn.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_uint32), ctypes.c_int, ctypes.c_int]
            fn.restype = ctypes.c_int
    return _lib


def depth_mm(index=0):
    """Profundidade 480×640 uint16 em mm (0 = sem leitura: perto demais, longe demais, reflexo, sombra)."""
    buf, ts = ctypes.c_void_p(), ctypes.c_uint32()
    if _load().freenect_sync_get_depth(ctypes.byref(buf), ctypes.byref(ts), index, DEPTH_MM):
        raise RuntimeError("Kinect não respondeu (fonte ligada? USB? `lsusb | grep 045e`)")
    return np.ctypeslib.as_array(ctypes.cast(buf, ctypes.POINTER(ctypes.c_uint16)), (H, W)).copy()


def rgb(index=0):
    buf, ts = ctypes.c_void_p(), ctypes.c_uint32()
    if _load().freenect_sync_get_video(ctypes.byref(buf), ctypes.byref(ts), index, VIDEO_RGB):
        raise RuntimeError("Kinect não respondeu")
    return np.ctypeslib.as_array(ctypes.cast(buf, ctypes.POINTER(ctypes.c_uint8)), (H, W, 3)).copy()


def colorize(d, near=500, far=4000):
    """Profundidade → imagem colorida (perto = quente); sem leitura = preto."""
    import cv2
    x = np.clip((d.astype(np.float32) - near) / (far - near), 0, 1)
    img = cv2.applyColorMap((255 - x * 255).astype(np.uint8), cv2.COLORMAP_JET)
    img[d == 0] = 0
    return img


def tilt(deg, index=0):
    """Motor de inclinação do Kinect: −27° (olhando pra baixo) a +27°."""
    _load().freenect_sync_set_tilt_degs.argtypes = [ctypes.c_int, ctypes.c_int]
    if _lib.freenect_sync_set_tilt_degs(int(max(-27, min(27, deg))), index):
        raise RuntimeError("motor do Kinect não respondeu")


# Lente da câmera colorida (a profundidade DEPTH_MM já vem alinhada a ela): valores típicos do Kinect v1.
# shortcut: sem calibração da lente deste Kinect (erro de poucos %), calibrar com tabuleiro se a posição precisar de mm
FX = FY = 525.0
CX, CY = 319.5, 239.5


def points(d):
    """Profundidade (mm) → pontos 3D da câmera (m): x pra direita, y pra baixo, z pra frente. Também os pixels (v, u)."""
    v, u = np.nonzero(d)
    z = d[v, u] / 1000.0
    return np.column_stack(((u - CX) * z / FX, (v - CY) * z / FY, z)), v, u


def fit_floor(P, iters=400, tol=0.015, seed=0):
    """Plano do chão por RANSAC: sorteia 3 pontos, fica com o plano que mais pontos aceitam (a < tol), refina.
    Devolve (n, c) com altura(p) = n·p + c, positiva acima do chão (do lado da câmera)."""
    rng = np.random.default_rng(seed)
    best, best_in = None, None
    for _ in range(iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs(P @ n - n @ a) < tol
        if best_in is None or inl.sum() > best_in.sum():
            best, best_in = (n, -n @ a), inl
    Q = P[best_in]
    m = Q.mean(0)
    n = np.linalg.svd(Q - m)[2][-1]          # normal = direção de menor espalhamento dos pontos do chão
    c = -n @ m
    if c < 0:                                # câmera (origem) tem que ficar acima: altura(0) = c > 0
        n, c = -n, -c
    return n, c


def floor_frame(n, c):
    """Eixos no chão: origem embaixo da câmera, x = pra frente (eixo z da câmera projetado), y = esquerda."""
    o = -c * n                               # ponto do chão embaixo da câmera
    fwd = np.array([0, 0, 1.0]) - n * n[2]
    ex = fwd / np.linalg.norm(fwd)
    ey = np.cross(n, ex)                     # n pra cima, x pra frente → y pra esquerda
    return o, ex, ey


def find_robot(d, n, c, hmin=0.03, hmax=0.45, min_px=150, max_size=0.35, top_min=0.14, near=None, gate=0.15):
    """Maior bolha entre hmin e hmax acima do chão com no máximo max_size m no chão. Com near=(x, y): só a mais
    perto dessa posição, até gate m (rastreio: não pula pros seus pés ou pro pé de um móvel). Devolve (x, y no chão em m, pixels, altura máx, máscara)."""
    import cv2
    P, v, u = points(d)
    h = P @ n + c
    # o ruído da profundidade do Kinect v1 cresce com z² (~1 cm a 2 m, ~3 cm a 3,5 m): de longe, o próprio chão
    # passaria de 3 cm, colaria no robô e a bolha ficaria grande demais. O limiar acompanha.
    sel = (h > np.maximum(hmin, 0.0025 * P[:, 2] ** 2 * 4)) & (h < hmax)
    mask = np.zeros(d.shape, np.uint8)
    mask[v[sel], u[sel]] = 255
    k, lab, stats, _ = cv2.connectedComponentsWithStats(cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)))
    o, ex, ey = floor_frame(n, c)
    labs = lab[v, u]
    best = None
    for i in range(1, k):
        blob = labs == i
        zc = float(P[blob, 2].mean()) if blob.any() else 1.0
        if stats[i, cv2.CC_STAT_AREA] < max(30, min_px / zc ** 2):   # de longe o robô ocupa menos pixels (∝ 1/z²)
            continue
        q = P[blob] - np.outer(P[blob] @ n + c, n) - o   # pontos da bolha projetados no chão
        fx, fy = q @ ex, q @ ey
        # tamanho no chão de robô (o rodapé da parede ou um móvel é uma bolha maior que isso)
        # tamanho e altura de robô (o rodapé, um móvel, um pé ou um chinelo não passam)
        if max(np.ptp(fx), np.ptp(fy)) > max_size or h[blob].max() < top_min:
            continue
        x, y = float(fx.mean()), float(fy.mean())
        score = -np.hypot(x - near[0], y - near[1]) if near else blob.sum()
        if near and -score > gate:
            continue
        if best is None or score > best[0]:
            best = (score, (x, y, int(blob.sum()), float(h[blob].max()), lab == i))
    return best[1] if best else None


MAP_PX = 200                      # px por metro no mapa
MAP_X = (0.3, 3.3)                # m à frente da câmera (vertical do mapa)
MAP_Y = (-1.5, 1.5)               # m pro lado (horizontal; esquerda = esquerda)


def to_map(x, y):
    return int((MAP_Y[1] - y) * MAP_PX), int((MAP_X[1] - x) * MAP_PX)


def draw_map(trail):
    """Mapa do chão visto de cima: grade de 10 cm (linha forte a cada 50), câmera embaixo, trajeto e o robô."""
    import cv2
    w, h = to_map(MAP_X[0], MAP_Y[0])
    img = np.full((h, w, 3), 24, np.uint8)
    for i in range(int(MAP_Y[0] * 10), int(MAP_Y[1] * 10) + 1):
        u = to_map(0, i / 10)[0]
        cv2.line(img, (u, 0), (u, h), (70, 70, 70) if i % 5 == 0 else (40, 40, 40), 1)
    for i in range(int(MAP_X[0] * 10), int(MAP_X[1] * 10) + 1):
        v = to_map(i / 10, 0)[1]
        cv2.line(img, (0, v), (w, v), (70, 70, 70) if i % 5 == 0 else (40, 40, 40), 1)
        if i % 5 == 0:
            cv2.putText(img, f"{i / 10:.1f} m", (4, v - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
    cv2.circle(img, to_map(MAP_X[0], 0), 6, (255, 200, 0), -1)          # lado da câmera
    for p, q in zip(trail, trail[1:]):
        if p is not None and q is not None:
            cv2.line(img, to_map(*p), to_map(*q), (0, 200, 0), 2)
    if trail and trail[-1] is not None:
        cv2.circle(img, to_map(*trail[-1]), 10, (0, 255, 0), 2)
        x, y = trail[-1]
        cv2.putText(img, f"x {x:.2f}  y {y:+.2f} m", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return img


def live():
    """Ao vivo: chão medido uma vez no começo (o Kinect não se mexe), robô procurado a cada quadro."""
    import cv2
    import time
    d = depth_mm()
    P, _, _ = points(d)
    n, c = fit_floor(P[:: max(1, len(P) // 20000)])
    print(f"chão: câmera a {c:.2f} m, {np.degrees(np.arccos(abs(n[1]))):.0f}° pra baixo")
    trail, click, t0, frames, fps, seen, last = [], {}, time.time(), 0, 0, 0.0, None
    cv2.namedWindow("profundidade")
    cv2.setMouseCallback("profundidade", lambda ev, x, y, *_: ev == cv2.EVENT_LBUTTONDOWN and click.update(p=(x, y)))
    while True:
        d = depth_mm()
        img = colorize(d)
        # segue a última posição (até 15 cm); perdido por mais de 1 s, pega o candidato mais perto de onde ele
        # sumiu, em qualquer distância (ele costuma voltar por onde saiu)
        tracking = trail and trail[-1] is not None and time.time() - seen < 1.0
        r = find_robot(d, n, c, near=last, gate=0.15 if tracking else 99.0)
        if r:
            seen, last = time.time(), r[:2]
            x, y, px, hm, mask = r
            cnt, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(img, cnt, -1, (255, 255, 255), 2)
            last = trail[-1] if trail else None
            if last is None or np.hypot(x - last[0], y - last[1]) > 0.005:   # 5 mm: não desenha o tremor
                trail = (trail + [(x, y)])[-2000:]
        elif trail and trail[-1] is not None and time.time() - seen > 1.0:
            trail.append(None)                                               # perdeu: não liga os pontos
        if "p" in click:
            u, v = click["p"]
            cv2.circle(img, (u, v), 4, (255, 255, 255), 1)
            cv2.putText(img, f"{d[v, u]} mm", (u + 6, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        frames += 1
        if time.time() - t0 > 1:
            fps, frames, t0 = frames / (time.time() - t0), 0, time.time()
        cv2.putText(img, f"{fps:.0f} quadros/s" + ("" if r else "  sem robo"), (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("profundidade", img)
        cv2.imshow("mapa (visto de cima)", draw_map(trail))
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k == ord("c"):
            trail = []


def stop():
    if _lib is not None:
        _lib.freenect_sync_stop()


def main():
    import cv2
    args = sys.argv[1:]
    if args[:1] == ["selftest"]:
        _load()
        d = np.array([[0, 500, 4000]], np.uint16)
        c = colorize(d)
        assert (c[0, 0] == 0).all() and not (c[0, 1] == c[0, 2]).all(), "colorize"
        # chão sintético: câmera a 1,5 m olhando 30° pra baixo, caixa de 25 cm a 2 m à frente e 0,3 m à esquerda
        th, Hc = np.radians(30), 1.5
        up = np.array([0, -np.cos(th), -np.sin(th)])
        vv, uu = np.mgrid[0:H, 0:W]
        r = np.stack(((uu - CX) / FX, (vv - CY) / FY, np.ones((H, W))), -1)
        den = r @ up
        o, ex, ey = floor_frame(up, Hc)
        def depth_at(hgt):
            t = np.where(den < -1e-6, (hgt - Hc) / den, 0)
            pts = r * t[..., None]
            return t, (pts - o) @ ex, (pts - o) @ ey
        t0, _, _ = depth_at(0.0)
        t1, x1, y1 = depth_at(0.25)
        box = (np.abs(x1 - 2.0) < 0.1) & (np.abs(y1 - 0.3) < 0.1) & (t1 > 0)
        box |= (np.abs(x1 - 1.2) < 0.1) & (np.abs(y1 + 0.4) < 0.1) & (t1 > 0)   # outra "coisa" do tamanho do robô
        dz = np.where(box, t1, t0)
        dsyn = np.where((dz > 0.5) & (dz < 5), dz * 1000, 0).astype(np.uint16)
        Ps = points(dsyn)[0]
        n2, c2 = fit_floor(Ps[::20])
        assert abs(c2 - Hc) < 0.02, f"altura da câmera {c2}"
        rr = find_robot(dsyn, n2, c2)
        rr = find_robot(dsyn, n2, c2, near=(1.95, 0.32))          # rastreio: segue a do lado, não a maior
        assert rr and abs(rr[0] - 2.0) < 0.05 and abs(rr[1] - 0.3) < 0.05, f"robô em {rr and rr[:2]}"
        assert find_robot(dsyn, n2, c2, near=(0.6, 0.0)) is None, "longe de tudo: nada"
        print("selftest ok (libfreenect_sync carregou; chão e bolha sintéticos achados)")
        return
    try:
        if args[:1] == ["tilt"]:
            tilt(float(args[1]))
            import time
            time.sleep(2)  # o motor leva ~1–2 s
            print(f"inclinação: {args[1]}°")
            return
        if args[:1] == ["chao"]:
            out = args[1] if len(args) > 1 else "."
            d = depth_mm()
            P, v, u = points(d)
            n, c = fit_floor(P[:: max(1, len(P) // 20000)])
            tilt_deg = np.degrees(np.arccos(abs(n[1])))
            print(f"chão: câmera a {c:.2f} m de altura, olhando {tilt_deg:.0f}° abaixo da horizontal")
            r = find_robot(d, n, c)
            h = np.zeros(d.shape, np.float32)
            h[v, u] = P @ n + c
            img = cv2.cvtColor(np.clip(128 + h * 600, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
            img[d == 0] = 0
            img[(d > 0) & (h > 0.03)] = (0, 140, 255)          # acima do chão: laranja
            if r:
                x, y, px, hm, m = r
                img[m] = (0, 255, 0)                            # o robô: verde
                print(f"robô: {x:.2f} m à frente, {y:+.2f} m pro lado (esq +), {px} pixels, {hm*100:.0f} cm de altura")
            else:
                print("robô: nenhuma bolha acima do chão")
            cv2.imwrite(os.path.join(out, "chao.png"), img)
            return
        if args[:1] == ["snap"]:
            out = args[1] if len(args) > 1 else "."
            d, c = depth_mm(), rgb()
            cv2.imwrite(os.path.join(out, "cor.png"), cv2.cvtColor(c, cv2.COLOR_RGB2BGR))
            cv2.imwrite(os.path.join(out, "prof.png"), colorize(d))
            ok = d[d > 0]
            print(f"salvo em {out}: profundidade válida em {100 * ok.size / d.size:.0f}% dos pixels, "
                  f"{ok.min() if ok.size else 0}–{ok.max() if ok.size else 0} mm")
            return
        live()
    finally:
        stop()


if __name__ == "__main__":
    main()
