#!/usr/bin/env python3
"""goto pelo Kinect (fase 3, docs/plano-plataforma.md): clique no mapa e o robô vai até lá.

Posição: Kinect (tools/kinect.py, absoluta, em metros no chão). Rumo: gyro do robô (`yaw` do stream), alinhado
ao mapa pelo próprio movimento: andando reto, a direção em que o Kinect vê o robô se mover É o rumo dele no mapa.

    python3 tools/navegar.py [host]   (robô equilibrando; Kinect ligado e sem outro programa usando)
      clique no robô (janela profundidade) → escolhe quem seguir
      clique no mapa                       → vai até lá (se o rumo ainda não é conhecido, anda 10 cm reto antes)
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
        self.offset = None        # rumo no mapa = yaw do gyro + offset
        self.state = "parado"
        self._align_from = None
        self._seg = None          # trecho reto pra refinar o offset (posição, yaw)

    def heading(self):
        yaw = self.robot.frame.get("yaw")
        return None if yaw is None or self.offset is None else wrap(yaw + self.offset)

    def go(self, goal):
        self.goal = goal
        self.state = "indo" if self.offset is not None else "alinhando"
        self._align_from = None

    def cancel(self):
        self.goal, self.state = None, "parado"
        self.robot.halt()

    def step(self, pos):
        """Chamado a cada quadro do Kinect com a posição do robô (ou None se perdido)."""
        if self.state == "parado":
            return
        if pos is None or self.robot.frame.get("state") != sbr.Robot.ACTIVE:
            self.robot.halt()        # perdeu o robô ou ele não está equilibrando: espera parado
            return
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
        v, w, dist = control(pos, wrap(yaw + self.offset), self.goal)
        if dist < ARRIVE:
            self.cancel()
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


def main(host):
    import cv2
    robot = sbr.Robot(host)
    nav = Navigator(robot)
    d = kinect.depth_mm()
    P, _, _ = kinect.points(d)
    n, c = kinect.fit_floor(P[:: max(1, len(P) // 20000)])
    o, ex, ey = kinect.floor_frame(n, c)
    print(f"chão: câmera a {c:.2f} m, {np.degrees(np.arccos(abs(n[1]))):.0f}° pra baixo. Clique no robô na profundidade.")
    clicks = {}
    cv2.namedWindow("profundidade")
    cv2.namedWindow("mapa (visto de cima)")
    cv2.setMouseCallback("profundidade", lambda ev, x, y, *_: ev == cv2.EVENT_LBUTTONDOWN and clicks.update(robo=(x, y)))
    cv2.setMouseCallback("mapa (visto de cima)", lambda ev, x, y, *_: ev == cv2.EVENT_LBUTTONDOWN and clicks.update(alvo=(x, y)))
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
                mu, mv = clicks.pop("alvo")
                if last:
                    nav.go((kinect.MAP_X[1] - mv / kinect.MAP_PX, kinect.MAP_Y[1] - mu / kinect.MAP_PX))
            nav.step(r[:2] if r else None)
            m = kinect.draw_map(trail)
            if nav.goal:
                cv2.drawMarker(m, kinect.to_map(*nav.goal), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
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
    print("selftest ok")


if __name__ == "__main__":
    if sys.argv[1:] == ["selftest"]:
        _selftest()
    else:
        main(sys.argv[1] if len(sys.argv) > 1 else "sbr.local")
