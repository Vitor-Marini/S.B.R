# Lab 01 — PID and cascade control

> **In one sentence:** a PD on the tilt keeps the robot up; PI and P loops stacked on top (speed, position) make it
> stop, drive and hold its place. It works, but each loop waits for the one inside it.

**Status:** ✅ works, available with `ctl pid`. Replaced by [LQR](../03-lqr/) as the default.

## In the code

| Where | What |
|---|---|
| `firmware/src/balance.cpp` | PD: `accel = kp·err + kd·rate`, integrated into wheel speed |
| `firmware/src/velocity.cpp` → `_updateCascade()` | speed PI → angle offset (`voff`), position hold, `move` profile, setpoint auto-trim |
| [`simulator.html`](simulator.html) | interactive explanation + simulator of the cascade (open in a browser; text in Portuguese) |

## Key results (measured)

| | PD + cascade |
|---|---|
| standing still | ±2.5–4 cm, slow ~5 s wave |
| `move 50` | overshoots 3–15 cm, 6–25 s to settle |
| setpoint 1.5° off | drives away (needs the auto-trim) |

## Lessons

- Two integrators on the same time scale (auto-trim τ 2.5 s + position hold) fought and made a ±17 cm wave.
- The speed loop has ~1 s of delay (the robot must lean before it accelerates): any position loop on top
  either rings or is slow. That's what motivated [LQR](../03-lqr/).

*Full write-up (intuition, theory, diagrams) coming next.* History: [journal](../../docs/journal.md).
