#!/usr/bin/env python3
"""goto pelo Kinect (fase 3, docs/plano-plataforma.md): clique no mapa e o robô vai até lá.

Posição: Kinect (tools/kinect.py, absoluta, em metros no chão). Rumo: gyro do robô (`yaw` do stream), alinhado
ao mapa pelo próprio movimento: andando reto, a direção em que o Kinect vê o robô se mover É o rumo dele no mapa.

    python3 tools/navegar.py [host]   (robô equilibrando; Kinect ligado e sem outro programa usando)
      clique no robô (janela profundidade) → escolhe quem seguir
      clique no mapa                       → vai até lá (se o rumo ainda não é conhecido, anda 10 cm reto antes)
      clique e arraste no mapa             → vai até lá e termina virado pra direção do arraste
      mapa: cinza = o que a câmera vê, vermelho = obstáculo, linha amarela = trajeto planejado (vermelha se bate)
      x → cancela (para e segura a posição)   c → limpa o trajeto   q → sai
    python3 tools/navegar.py selftest
"""
import math
import sys
import time

import numpy as np

import kinect
import sbr

V_MAX = 0.15          # m/s
W_MAX = 45.0          # °/s
K_LIN = 0.8           # 1/s: velocidade = K_LIN × distância (devagar perto do alvo)
K_ROT = 2.0           # (°/s) por °: giro = K_ROT × erro de rumo
TURN_ONLY = 35.0      # °: acima disso de erro de rumo, só gira (não anda torto)
ARRIVE = 0.04         # m: chegou
ALIGN_DIST = 0.10     # m: anda reto isso pra descobrir o rumo no mapa
FACE_TOL = 3.0        # °: virou pra direção pedida
MAX_RANGE = 2.3       # m à frente da câmera: medido, começa a falhar ~2,5 m (perto da borda)
LOST_GIVEUP = 0.5     # s sem achar o robô no Kinect: desiste do alvo
MARGIN = 0.20         # m: alvo longe da borda do que a câmera vê (o robô inteiro tem que caber na imagem)


def wrap(a):
    """Ângulo em graus pra (−180, 180]."""
    return (a + 180.0) % 360.0 - 180.0


def control(pos, heading, goal):
    """Go-to-goal de robô diferencial. pos/goal em m no mapa, heading em ° (0 = eixo x do mapa, + = esquerda).
    Devolve (v m/s, w °/s, distância)."""
    dx, dy = goal[0] - pos[0], goal[1] - pos[1]
    dist = math.hypot(dx, dy)
    err = wrap(math.degrees(math.atan2(dy, dx)) - heading)
    w = max(-W_MAX, min(W_MAX, K_ROT * err))
    v = 0.0 if abs(err) > TURN_ONLY else min(V_MAX, K_LIN * dist) * math.cos(math.radians(err))
    return v, w, dist


class Navigator:
    """Estados: parado → alinhando (anda reto ALIGN_DIST) → indo → parado."""

    def __init__(self, robot):
        self.robot = robot
        self.goal = None
        self.face = None          # ° no mapa pra terminar virado (None = tanto faz)
        self.offset = None        # rumo no mapa = yaw do gyro + offset
        self.state = "parado"
        self._align_from = None
        self._seg = None          # trecho reto pra refinar o offset (posição, yaw)
        self._lost = None         # desde quando o Kinect não acha o robô
        self.why = ""             # por que parou (mostrado no mapa)

    def heading(self):
        yaw = self.robot.frame.get("yaw")
        return None if yaw is None or self.offset is None else wrap(yaw + self.offset)

    def go(self, goal, face=None):
        self.goal, self.face, self._lost, self.why = goal, face, None, ""
        self.state = "indo" if self.offset is not None else "alinhando"
        self._align_from = None

    def cancel(self):
        self.goal, self.face, self.state = None, None, "parado"
        self.robot.halt()

    def step(self, pos):
        """Chamado a cada quadro do Kinect com a posição do robô (ou None se perdido)."""
        if self.state == "parado":
            return
        if self.robot.frame.get("state") != sbr.Robot.ACTIVE:
            # caiu/parou: desiste do alvo e esquece o rumo (o robô foi levantado, talvez recalibrado: yaw zera)
            self.cancel()
            self.offset = None
            return
        if pos is None:
            # perdeu o robô no Kinect: para na hora; 0,5 s sem achar, desiste do alvo (não volta a andar sozinho
            # quando "reencontrar", que pode ser outra coisa)
            self.robot.halt()
            self._lost = self._lost or time.time()
            if time.time() - self._lost > LOST_GIVEUP:
                self.cancel()
                self.why = "parou: perdeu o robo"
            return
        self._lost = None
        yaw = self.robot.frame["yaw"]
        if self.state == "alinhando":
            if self._align_from is None:
                self._align_from = pos
            dx, dy = pos[0] - self._align_from[0], pos[1] - self._align_from[1]
            if math.hypot(dx, dy) < ALIGN_DIST:
                self.robot.drive(0.08, 0)
                return
            self.offset = wrap(math.degrees(math.atan2(dy, dx)) - yaw)
            self.state = "indo"
        if self.state == "virando":
            err = wrap(self.face - wrap(yaw + self.offset))
            if abs(err) < FACE_TOL:
                self.cancel()
            else:
                self.robot.drive(0, max(-W_MAX, min(W_MAX, K_ROT * err)))
            return
        v, w, dist = control(pos, wrap(yaw + self.offset), self.goal)
        if dist < ARRIVE:
            if self.face is None:
                self.cancel()
            else:
                self.robot.halt()
                self.state = "virando"
            return
        self.robot.drive(v, w)
        # andando reto: refina o offset com o que o Kinect vê (corrige a deriva lenta do gyro)
        if abs(w) < 5 and v > 0.05:
            if self._seg is None:
                self._seg = (pos, yaw)
            elif math.hypot(pos[0] - self._seg[0][0], pos[1] - self._seg[0][1]) > 0.10:
                seen = math.degrees(math.atan2(pos[1] - self._seg[0][1], pos[0] - self._seg[0][0]))
                new = wrap(seen - (yaw + self._seg[1]) / 2)
                self.offset = wrap(self.offset + 0.3 * wrap(new - self.offset))
                self._seg = None
        else:
            self._seg = None


def camera_footprint(n, c, o, ex, ey, max_depth=4.0):
    """Contorno do chão que a câmera vê: raios das bordas da imagem até o chão (cortados em max_depth)."""
    W, H = kinect.W, kinect.H
    border = [(u, 0) for u in range(0, W, 20)] + [(W - 1, v) for v in range(0, H, 20)] + \
             [(u, H - 1) for u in range(W - 1, 0, -20)] + [(0, v) for v in range(H - 1, 0, -20)]
    pts = []
    for u, v in border:
        r = np.array([(u - kinect.CX) / kinect.FX, (v - kinect.CY) / kinect.FY, 1.0])
        den = n @ r
        t = -c / den if den < 0 else max_depth
        q = r * min(t, max_depth)
        q = q - (q @ n + c) * n - o
        pts.append((float(q @ ex), float(q @ ey)))
    return pts


def obstacles(d, n, c, o, ex, ey, robot_mask=None, hmin=0.03, hmax=1.0):
    """Pontos no chão (x, y em m) de tudo entre hmin e hmax acima do chão, menos o robô."""
    P, v, u = kinect.points(d)
    h = P @ n + c
    sel = (h > hmin) & (h < hmax)
    if robot_mask is not None:
        sel &= ~robot_mask[v, u]
    q = P[sel] - np.outer(h[sel], n) - o
    return np.column_stack((q @ ex, q @ ey))


def blocked(obs_img, a, b, robot_r=0.10):
    """O segmento a→b no mapa passa por obstáculo (com folga do raio do robô)?"""
    import cv2
    line = np.zeros(obs_img.shape[:2], np.uint8)
    cv2.line(line, kinect.to_map(*a), kinect.to_map(*b), 255, max(1, int(2 * robot_r * kinect.MAP_PX)))
    return bool(((line > 0) & (obs_img > 0)).any())


def main(host):
    import cv2
    robot = sbr.Robot(host)
    nav = Navigator(robot)
    d = kinect.depth_mm()
    P, _, _ = kinect.points(d)
    n, c = kinect.fit_floor(P[:: max(1, len(P) // 20000)])
    o, ex, ey = kinect.floor_frame(n, c)
    print(f"chão: câmera a {c:.2f} m, {np.degrees(np.arccos(abs(n[1]))):.0f}° pra baixo. Clique no robô na profundidade.")
    fov = np.array([kinect.to_map(*p) for p in camera_footprint(n, c, o, ex, ey)], np.int32)

    def trackable(pt, margin=MARGIN):
        """Dentro do que a câmera vê, longe da borda, e não longe demais."""
        u, v = kinect.to_map(*pt)
        return pt[0] <= MAX_RANGE and cv2.pointPolygonTest(fov, (float(u), float(v)), True) >= margin * kinect.MAP_PX
    msg, msg_t = "", 0.0
    obs_img, frame_i = None, 0
    clicks, drag = {}, {}

    def on_map(ev, x, y, *_):
        if ev == cv2.EVENT_LBUTTONDOWN:
            drag["a"] = (x, y)
        elif ev == cv2.EVENT_LBUTTONUP and "a" in drag:
            clicks["alvo"] = (drag.pop("a"), (x, y))
    cv2.namedWindow("profundidade")
    cv2.namedWindow("mapa (visto de cima)")
    cv2.setMouseCallback("profundidade", lambda ev, x, y, *_: ev == cv2.EVENT_LBUTTONDOWN and clicks.update(robo=(x, y)))
    cv2.setMouseCallback("mapa (visto de cima)", on_map)
    trail, last, seen = [], None, 0.0
    try:
        while True:
            d = kinect.depth_mm()
            if "robo" in clicks:                         # o ponto do chão embaixo do clique vira a última posição
                u, v = clicks.pop("robo")
                if d[v, u]:
                    z = d[v, u] / 1000.0
                    p = np.array([(u - kinect.CX) * z / kinect.FX, (v - kinect.CY) * z / kinect.FY, z])
                    q = p - (p @ n + c) * n - o
                    last, seen, trail = (float(q @ ex), float(q @ ey)), time.time(), []
            tracking = last is not None and time.time() - seen < 1.0
            r = kinect.find_robot(d, n, c, near=last, gate=0.15 if tracking else 99.0) if last else None
            img = kinect.colorize(d)
            if r:
                seen, last = time.time(), r[:2]
                cnt, _ = cv2.findContours(r[4].astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(img, cnt, -1, (255, 255, 255), 2)
                if not trail or trail[-1] is None or math.hypot(last[0] - trail[-1][0], last[1] - trail[-1][1]) > 0.005:
                    trail = (trail + [last])[-2000:]
            elif trail and trail[-1] is not None and time.time() - seen > 1.0:
                trail.append(None)
            if "alvo" in clicks:
                (mu, mv), (eu, ev_) = clicks.pop("alvo")
                g = (kinect.MAP_X[1] - mv / kinect.MAP_PX, kinect.MAP_Y[1] - mu / kinect.MAP_PX)
                if last and not trackable(g):
                    msg, msg_t = "fora da area rastreavel", time.time()
                elif last:
                    # arrastou mais de 15 px: direção do arraste (no mapa, pra cima = +x, pra esquerda = +y)
                    face = math.degrees(math.atan2(mu - eu, mv - ev_)) if math.hypot(eu - mu, ev_ - mv) > 15 else None
                    nav.go(g, face)
            frame_i += 1
            if frame_i % 6 == 1:     # obstáculos ~5 vezes por segundo (é a parte cara)
                obs = obstacles(d, n, c, o, ex, ey, r[4] if r else None)
                mw, mh = kinect.to_map(kinect.MAP_X[0], kinect.MAP_Y[0])
                obs_img = np.zeros((mh, mw), np.uint8)
                if len(obs):
                    uv = np.array([kinect.to_map(x, y) for x, y in obs[:: max(1, len(obs) // 20000)]])
                    ok = (uv[:, 0] >= 0) & (uv[:, 0] < mw) & (uv[:, 1] >= 0) & (uv[:, 1] < mh)
                    obs_img[uv[ok, 1], uv[ok, 0]] = 255
                    obs_img = cv2.dilate(obs_img, np.ones((3, 3), np.uint8))
            if nav.state != "parado" and r and not trackable(r[:2], margin=0.05):
                nav.cancel()                                     # saindo da área rastreável: para antes de perder
                msg, msg_t = "parou: saindo da area", time.time()
            nav.step(r[:2] if r else None)
            m = kinect.draw_map(trail)
            fov_layer = np.zeros_like(m)
            cv2.fillPoly(fov_layer, [fov], (60, 60, 60))
            m = cv2.add(m, fov_layer)                              # área que a câmera vê, mais clara
            cv2.polylines(m, [fov], True, (140, 140, 140), 1)
            yl = kinect.to_map(MAX_RANGE, 0)[1]
            cv2.line(m, (0, yl), (m.shape[1], yl), (0, 140, 255), 1)    # limite de alcance
            if nav.why:
                msg, msg_t, nav.why = nav.why, time.time(), ""
            if time.time() - msg_t < 2:
                cv2.putText(m, msg, (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if obs_img is not None:
                m[obs_img > 0] = (40, 40, 220)                     # obstáculos em vermelho
            if nav.goal:
                if last:
                    bad = obs_img is not None and blocked(obs_img, last, nav.goal)
                    cv2.line(m, kinect.to_map(*last), kinect.to_map(*nav.goal), (0, 0, 255) if bad else (0, 220, 255), 1)
                cv2.drawMarker(m, kinect.to_map(*nav.goal), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
                if nav.face is not None:
                    g = nav.goal
                    tip = (g[0] + 0.15 * math.cos(math.radians(nav.face)), g[1] + 0.15 * math.sin(math.radians(nav.face)))
                    cv2.arrowedLine(m, kinect.to_map(*g), kinect.to_map(*tip), (0, 0, 255), 2, tipLength=0.3)
            hd = nav.heading()
            if last and hd is not None:
                tip = (last[0] + 0.15 * math.cos(math.radians(hd)), last[1] + 0.15 * math.sin(math.radians(hd)))
                cv2.arrowedLine(m, kinect.to_map(*last), kinect.to_map(*tip), (255, 255, 255), 2, tipLength=0.3)
            cv2.putText(m, nav.state if last else "clique no robo", (8, m.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
            cv2.imshow("profundidade", img)
            cv2.imshow("mapa (visto de cima)", m)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord("x"):
                nav.cancel()
            if k == ord("c"):
                trail = []
    finally:
        robot.halt()
        robot.close()
        kinect.stop()


def _selftest():
    assert wrap(190) == -170 and wrap(-190) == 170 and wrap(180) == -180
    v, w, dist = control((0, 0), 0, (1, 0))          # alvo em frente: anda reto
    assert v > 0 and w == 0 and abs(dist - 1) < 1e-9
    v, w, _ = control((0, 0), 0, (0, 1))             # alvo à esquerda: só gira pra esquerda (+)
    assert v == 0 and w > 0
    v, w, _ = control((0, 0), 90, (1, 0))            # virado pra esquerda, alvo em frente do mapa: gira à direita
    assert w < 0
    v, w, _ = control((0, 0), 0, (-1, 0.01))         # alvo atrás: gira, não anda de ré
    assert v == 0 and abs(w) == W_MAX

    class FakeRobot:                                  # robô simulado no plano: gyro com offset de 30° do mapa
        ACTIVE = 4
        def __init__(s): s.x = s.y = 0.0; s.th = 30.0; s.v = s.w = 0.0; s.frame = {"state": 4, "yaw": 0.0}
        def drive(s, v, w): s.v, s.w = v, w
        def halt(s): s.v = s.w = 0.0
        def tick(s, dt):
            s.th += s.w * dt
            s.x += s.v * dt * math.cos(math.radians(s.th)); s.y += s.v * dt * math.sin(math.radians(s.th))
            s.frame["yaw"] = wrap(s.th - 30.0)
    fr = FakeRobot()
    nav = Navigator(fr)
    nav.go((1.0, 0.5))
    for _ in range(3000):
        nav.step((fr.x, fr.y))
        fr.tick(1 / 30)
        if nav.state == "parado":
            break
    assert nav.state == "parado" and math.hypot(fr.x - 1.0, fr.y - 0.5) < ARRIVE + 0.01, (fr.x, fr.y, nav.state)
    assert abs(wrap(nav.offset - 30)) < 2, nav.offset
    fr2 = FakeRobot()                                  # alvo com direção: termina virado pra lá
    nav2 = Navigator(fr2)
    nav2.go((0.6, -0.3), face=180.0)
    for _ in range(3000):
        nav2.step((fr2.x, fr2.y))
        fr2.tick(1 / 30)
        if nav2.state == "parado":
            break
    assert nav2.state == "parado" and abs(wrap(fr2.th - 180)) < FACE_TOL + 1, (fr2.th, nav2.state)
    fr2.frame["state"] = 6                             # caiu: desiste e esquece o rumo
    nav2.go((1, 1))
    nav2.step((fr2.x, fr2.y))
    assert nav2.state == "parado" and nav2.offset is None
    fr3 = FakeRobot()                                  # perdeu o robô no Kinect: para e, depois de 0,5 s, desiste
    nav3 = Navigator(fr3)
    nav3.go((1.0, 0.0))
    nav3.step((0.0, 0.0)); fr3.v = 0.1
    nav3.step(None)
    assert fr3.v == 0 and nav3.state != "parado"
    nav3._lost -= LOST_GIVEUP + 0.1
    nav3.step(None)
    assert nav3.state == "parado" and nav3.why
    import cv2
    w_, h_ = kinect.to_map(kinect.MAP_X[0], kinect.MAP_Y[0])
    oi = np.zeros((h_, w_), np.uint8)
    cv2.circle(oi, kinect.to_map(1.5, 0.0), 5, 255, -1)
    assert blocked(oi, (1.0, 0.0), (2.0, 0.0)) and not blocked(oi, (1.0, 0.8), (2.0, 0.8))
    print("selftest ok")


if __name__ == "__main__":
    if sys.argv[1:] == ["selftest"]:
        _selftest()
    else:
        main(sys.argv[1] if len(sys.argv) > 1 else "sbr.local")
