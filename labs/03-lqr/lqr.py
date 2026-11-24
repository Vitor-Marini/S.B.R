#!/usr/bin/env python3
"""LQR do S.B.R: modelo, ganhos e simulação comparando com a cascata atual (labs/03-lqr).

Modelo (stepper: a roda faz a aceleração mandada, então a entrada é a aceleração `a` da roda):
    θ̈ = (g·sen θ − a·cos θ) / L        pêndulo sobre o eixo; L = I/(m·d) medido ≈ 0,09 m (teste do pêndulo)
    ẍ = a
Estados s = [x, v, θ, ω] (m, m/s, rad, rad/s), θ+ = caindo pra frente (mesma convenção do firmware).
LQR: a = −K·s, ou seja, UMA soma de 4 termos — o PD de equilíbrio de hoje + posição + velocidade juntos,
em vez da posição/velocidade mexerem no setpoint (cascata, que espera a malha de dentro).

Uso: python3 labs/03-lqr/lqr.py            → ganhos + simulação (cascata x LQR) de parado, empurrão e move 50 cm
     python3 labs/03-lqr/lqr.py --plot     → também salva lqr.png (precisa de matplotlib)
"""
import sys

import numpy as np
from scipy.linalg import expm, solve_discrete_are

G = 9.81
L_MODEL = 0.09        # m (medido: 0,084–0,099)
DT = 0.005            # loop de 200 Hz
V_MAX = 0.8           # m/s (cfg.maxVelocity)
DIST = 2.5           # rad/s²: com ele a cascata simulada balança ±3–4 cm parada, como a real
A_MAX = 6.0           # m/s²: limite conservador sem saber a corrente (Vref) dos drivers
DEG = np.pi / 180

# Pesos pela regra de Bryson: 1/(desvio máximo aceitável)². O que importa mais pesa mais.
# "suave": quase o mesmo resultado simulado com ganhos ~2× os do PD atual (os de 3°/3 m/s² davam ~6×: ruído)
MAX_X, MAX_V, MAX_TH, MAX_W, MAX_A = 0.05, 0.4, 8 * DEG, 100 * DEG, 1.5


def model(L):
    A = np.array([[0, 1, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1], [0, 0, G / L, 0]], float)
    B = np.array([[0], [1], [0], [-1 / L]], float)
    return A, B


def discretize(A, B, dt):
    # retentor de ordem zero: exp da matriz aumentada [[A, B], [0, 0]]·dt
    n, m = A.shape[0], B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n], M[:n, n:] = A, B
    E = expm(M * dt)
    return E[:n, :n], E[:n, n:]


def lqr_gains(L=L_MODEL, dt=DT):
    Ad, Bd = discretize(*model(L), dt)
    Q = np.diag([1 / MAX_X**2, 1 / MAX_V**2, 1 / MAX_TH**2, 1 / MAX_W**2])
    R = np.array([[1 / MAX_A**2]])
    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    return K[0], np.linalg.eigvals(Ad - Bd @ K)


# ----------------------------------------------------------------- controladores (passo de 5 ms)
class Cascade:
    """Cópia do firmware atual: balance.cpp (PD → aceleração integrada) + velocity.cpp (PI de velocidade
    → offset do setpoint, posição parada, move por trajetória de referência). Sem auto-ajuste do setpoint."""
    KP, KD = 0.35, 0.03            # m/s² por °, por °/s
    VKP, VKI = 4.0, 3.0            # ° por m/s
    HKP, HOLD_MAX_V, HOLD_ARM = 0.7, 0.08, 0.03
    MVA, MVS = 0.1, 0.15

    def __init__(self):
        self.speed = self.wheel = 0.0
        self.vi = 0.0
        self.hold_armed, self.hold_pos = False, 0.0
        self.move = None  # [goal, done, ref_p, ref_v]

    def start_move(self, goal):
        self.move = [goal, 0.0, 0.0, 0.0]
        self.vi = 0.0

    def step(self, pitch_deg, rate_dps):
        v = self.wheel
        target, move_hold = 0.0, 0.0
        if self.move:
            m = self.move
            m[1] += v * DT
            g = abs(m[0])
            m[3] = min(m[3] + self.MVA * DT, self.MVS, np.sqrt(2 * self.MVA * max(g - m[2], 0)))
            m[2] = min(m[2] + max(m[3], 0.005) * DT, g)
            target = np.sign(m[0]) * m[3]
            move_hold = np.clip(-self.HKP * (m[1] - np.sign(m[0]) * m[2]), -self.HOLD_MAX_V, self.HOLD_MAX_V)
            if m[2] >= g:
                self.hold_armed, self.hold_pos = True, m[1] - m[0]
                self.move, target, move_hold = None, 0.0, 0.0
        driving = target != 0.0
        hold_v = 0.0
        if driving:
            self.hold_armed = False
        else:
            if not self.hold_armed and abs(v) < self.HOLD_ARM:
                self.hold_armed, self.hold_pos = True, 0.0
            if self.hold_armed:
                self.hold_pos = np.clip(self.hold_pos + v * DT, -0.5, 0.5)
                hold_v = np.clip(-self.HKP * self.hold_pos, -self.HOLD_MAX_V, self.HOLD_MAX_V)
        err_v = target + hold_v + move_hold - v
        if driving and not self.move:
            self.vi += err_v * DT
        else:
            self.vi -= self.vi * DT / 0.2
        offset = np.clip(self.VKP * err_v + self.VKI * self.vi, -5, 5)
        accel = self.KP * (pitch_deg - offset) + self.KD * rate_dps
        self.speed += accel * DT
        self.wheel = np.clip(self.speed + target, -V_MAX, V_MAX)
        self.speed = self.wheel - target
        return self.wheel


class LQR:
    """a = a_ref − K·(s − s_ref). No move a referência inclui θ_ref = a_ref/g (o quanto precisa inclinar)."""

    def __init__(self, K):
        self.K = K
        self.wheel = 0.0
        self.x = 0.0          # odometria (o firmware tem: passos das rodas)
        self.ref = None       # [goal, p, v, a]

    def start_move(self, goal):
        self.ref = [goal, 0.0, 0.0, 0.0]

    def step(self, pitch_deg, rate_dps):
        self.x += self.wheel * DT
        xr = vr = ar = 0.0
        goal = 0.0
        if self.ref:
            goal, p, vr_, _ = self.ref
            g = abs(goal)
            vn = min(vr_ + Cascade.MVA * DT, Cascade.MVS, np.sqrt(2 * Cascade.MVA * max(g - p, 0)))
            ar_ = (vn - vr_) / DT
            p = min(p + max(vn, 0.005) * DT, g)
            self.ref = [goal, p, vn, ar_]
            s = np.sign(goal)
            xr, vr, ar = s * p, s * vn, s * ar_
            if p >= g:
                self.ref, xr, vr, ar = None, goal, 0.0, 0.0
                self.goal_hold = goal
        xr = xr if self.ref else getattr(self, "goal_hold", 0.0)
        e = np.array([self.x - xr, self.wheel - vr, pitch_deg * DEG - ar / G, rate_dps * DEG])
        a = np.clip(ar - self.K @ e, -A_MAX, A_MAX)
        self.wheel = float(np.clip(self.wheel + a * DT, -V_MAX, V_MAX))
        return self.wheel


# ----------------------------------------------------------------- planta
def simulate(ctrl, T, L_true=L_MODEL, th0=0.0, w0=0.0, bias_deg=0.0, noise=True, move_at=None, seed=1, dist=None):
    rng = np.random.default_rng(seed)
    th, w, x = th0 * DEG, w0 * DEG, 0.0
    prev_wheel, delayed = 0.0, (th0, w0)
    out = []
    for i in range(int(T / DT)):
        if move_at is not None and abs(i * DT - move_at[0]) < DT / 2:
            ctrl.start_move(move_at[1])
        # sensor com 1 ciclo de atraso; ruído medido no suporte (gyro ~0,3 °/s, pitch ~0,05°) + erro de setpoint
        meas = delayed
        delayed = (th / DEG + bias_deg + (rng.normal(0, 0.05) if noise else 0),
                   w / DEG + (rng.normal(0, 0.3) if noise else 0))
        wheel = ctrl.step(*meas)
        a = (wheel - prev_wheel) / DT
        prev_wheel = wheel
        # perturbação (vibração, folga, piso): ruído de aceleração angular, calibrado pra cascata dar o tremor medido
        d = rng.normal(0, DIST if dist is None else dist) if noise else 0.0
        w += ((G * np.sin(th) - a * np.cos(th)) / L_true + d) * DT
        th += w * DT
        x += wheel * DT
        out.append((i * DT, x, wheel, th / DEG, a))
        if abs(th) > 45 * DEG:
            break
    return np.array(out)


def summary(name, r, goal=None, t_from=0.0):
    t, x, v, th, a = r.T
    fell = abs(th[-1]) > 45
    sel = t >= t_from
    s = f"  {name:8s} "
    if fell:
        return s + f"CAIU em {t[-1]:.1f} s"
    if goal is None:
        s += f"x {x[sel].min()*100:+5.1f}..{x[sel].max()*100:+5.1f} cm  pitch ±{th[sel].std():.2f}°  a ±{a[sel].std():.2f} m/s²"
    else:
        over = (x.max() - goal) * 100 if goal > 0 else (goal - x.min()) * 100
        far = np.where(abs(x - goal) > 0.03)[0]
        settle = t[far[-1]] if len(far) else 0.0
        s += f"passou {max(over, 0):4.1f} cm  dentro de 3 cm a partir de {settle:4.1f} s  fim {abs(x[-1]-goal)*100:.1f} cm"
    return s


def compare(K, L_true=L_MODEL):
    for title, kw, goal, t_from in [
        ("parado 30 s, ruído + setpoint 0,3° errado", dict(T=30, bias_deg=0.3), None, 5),
        ("empurrão (30 °/s pra frente), sem ruído", dict(T=10, w0=30, noise=False), None, 0),
        ("move 50 cm aos 1 s (perfil mva 0,1 / mvs 0,15)", dict(T=20, move_at=(1.0, 0.5)), 0.5, 0),
    ]:
        print(title)
        for name, c in (("cascata", Cascade()), ("LQR", LQR(K))):
            print(summary(name, simulate(c, L_true=L_true, **kw), goal, t_from))


def main():
    K, poles = lqr_gains()
    print(f"L do modelo: {L_MODEL} m   loop {1/DT:.0f} Hz   pesos: x {MAX_X} m, v {MAX_V} m/s, θ {MAX_TH/DEG:.0f}°, "
          f"ω {MAX_W/DEG:.0f} °/s, a {MAX_A} m/s²")
    k = -K  # a = k·s: todos positivos (à frente/rápido/inclinado pra frente → acelera pra frente)
    print("Ganhos (a = k_x·x + k_v·v + k_θ·θ + k_ω·ω), nas unidades do firmware:")
    print(f"  pitch: {k[2]*DEG:.3f} m/s² por °      (PD de hoje: kp 0,35)")
    print(f"  giro:  {k[3]*DEG:.4f} m/s² por °/s   (PD de hoje: kd 0,03)")
    print(f"  posição: {k[0]:.1f} m/s² por m     velocidade: {k[1]:.1f} m/s² por m/s")
    print("  (posição à frente → acelera pra FRENTE: é o jeito de inclinar pra trás e então voltar)")
    tc = [-DT / np.log(abs(p)) for p in poles]
    print("  constantes de tempo da malha fechada: " + ", ".join(f"{x:.2f} s" for x in sorted(tc)))
    print()
    compare(K)
    print("\nRobustez: ganhos calculados com L = 0,09 m, robô real com outro L")
    for L_true in (0.075, 0.11):
        print(f"L real = {L_true} m")
        r = simulate(LQR(K), T=20, L_true=L_true, move_at=(1.0, 0.5))
        print(summary("LQR", r, 0.5))
    if "--plot" in sys.argv:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
        for name, c in (("cascata", Cascade()), ("LQR", LQR(K))):
            t, x, v, th, a = simulate(c, T=20, move_at=(1.0, 0.5)).T
            ax[0].plot(t, x * 100, label=name)
            ax[1].plot(t, v, label=name)
            ax[2].plot(t, th, label=name)
        ax[0].axhline(50, color="gray", lw=0.5)
        for a_, lab in zip(ax, ("posição (cm)", "velocidade (m/s)", "pitch (°)")):
            a_.set_ylabel(lab)
            a_.legend()
        ax[2].set_xlabel("s")
        fig.suptitle("move 50 cm: cascata x LQR (simulado)")
        fig.savefig("lqr.png", dpi=110)
        print("salvo lqr.png")


def _selftest():
    K, poles = lqr_gains()
    assert all(abs(p) < 1 for p in poles), "malha fechada instável"
    assert K[2] < 0, f"sinal: pitch+ (caindo pra frente) → acelera pra frente ({K})"
    r = simulate(LQR(K), T=5, th0=3.0, noise=False)
    assert abs(r[-1, 3]) < 0.1 and abs(r[-1, 1]) < 0.05, "não estabilizou de 3°"
    print("selftest ok")


if __name__ == "__main__":
    _selftest() if "selftest" in sys.argv else main()
