# Lab 05 — Web control

> **In one sentence:** the robot serves its own control page; a WebSocket carries the same text protocol as the
> serial terminal, so the phone, a DS4 and programs all speak one language.

**Status:** ✅ in use. Open `http://sbr.local`.

## In the code

| Where | What |
|---|---|
| `firmware/src/web.cpp` | ESPAsyncWebServer, `/ws`; received lines queued and run in the main loop (never mid-control) |
| `firmware/include/web_page.h` | the page (HTML/CSS/JS in a C++ string): joystick, Gamepad API, charts, 2D trail, console |
| `firmware/src/net.cpp` | `Tee`: everything `term` prints goes to USB, UDP and the page |
| `tools/sbr.py` | Python SDK over the same WebSocket |

## Lessons

- `Serial.setTxTimeoutMs(0)` hangs the loop forever on the ESP32-C3 (HWCDC) when USB is connected and nobody reads:
  use 1. It's why the page first loaded with 0 frames.
- Output is coalesced while a message is in flight + TCP no-delay → 50 frames/s, loop still at 200 Hz.

Original plan: [docs/history/plan-web.md](../../docs/history/plan-web.md). *Full write-up coming next.*
