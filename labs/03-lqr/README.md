# Lab 03 — LQR (Linear Quadratic Regulator)

> **In one sentence:** instead of stacking loops, look at the whole state (tilt, tilt rate, position, speed) at once
> and compute the best gains for it from a physical model.

**Status:** ✅ **default controller** (`ctl lqr`).

```
a = kθ·θ + kω·ω + kx·(x − x_ref) + kv·(v − v_ref) + a_ref        θ_ref = a_ref / g
```

## In the code

| Where | What |
|---|---|
| [`lqr.py`](lqr.py) | model (`θ'' = (g·θ − a)/L`), discretization, Riccati solution, simulation of cascade vs LQR |
| `firmware/src/velocity.cpp` → `_updateLqr()` | reference (x, v, a) and position/speed terms |
| `firmware/src/balance.cpp` | tilt terms + the 4 m/s² acceleration cap |

```bash
python3 labs/03-lqr/lqr.py            # gains + simulated comparison
python3 labs/03-lqr/lqr.py --plot     # also saves lqr.png
```

## Key results (measured)

| | PD + cascade | LQR |
|---|---|---|
| standing still | ±2.5–4 cm | ±0.3–1 cm |
| `move 50` | overshoots 3–15 cm, 6–25 s | ~1 cm, 4.8 s |
| demo (move/turn/move/turn) | failed | passed, back within 0.5 cm |

The only physical number the model needs is **L = 0.09 m** (pendulum test; CAD said 0.10).
Original plan: [docs/history/plan-lqr.md](../../docs/history/plan-lqr.md).

*Full write-up coming next (a Portuguese version already exists in the study guide).*
