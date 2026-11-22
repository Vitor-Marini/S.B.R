#ifndef VELOCITY_HPP
#define VELOCITY_HPP

#include <Arduino.h>

class VelocityController {
public:
    void begin();
    void setTarget(float ms);
    void setTurnRate(float degS);
    void turnBy(float deg);  // gira X graus no lugar (+ = esquerda) pelo gyro Z; drive/stop cancelam
    float getTarget() { return _targetVelocity; }
    float getRampedTarget() { return _rampedTarget; }
    float spAtStart() { return _spStart; }   // setpoint quando o ACTIVE começou
    bool trimmed() { return _trimmed; }      // o auto-ajuste mexeu no setpoint nesta sessão
    float getTurnRate() { return _targetTurnRate; }
    float getEstimated() { return _estimatedVelocity; }

    void update();
    float getAngleOffset();
    float getTurnOffset();  // m/s de diferença entre rodas
    void reset();

private:
    float _targetVelocity = 0.0f;
    float _targetTurnRate = 0.0f;
    float _angleOffset = 0.0f;
    float _turnOffset = 0.0f;

    float _velIntegral = 0.0f;
    float _turnIntegral = 0.0f;
    float _estimatedVelocity = 0.0f;
    uint32_t _lastCmdMs = 0;
    float _rampedTarget = 0.0f;
    float _rampedTurn = 0.0f;
    uint32_t _calmTicks = 0;
    float _spStart = 0.0f;
    bool _trimmed = false;
    bool _holdArmed = false;
    float _holdPos = 0.0f;                            // m andados desde a âncora (parado)
    static constexpr float HOLD_KP = 1.5f;            // 1/s: 2 cm fora → pede 3 cm/s de volta
    static constexpr float HOLD_MAX_V = 0.08f;        // m/s máx que a posição pede
    static constexpr float HOLD_SLIP_M = 0.50f;       // m: empurrão volta pro lugar; além disso aceita o lugar novo
    static constexpr float HOLD_ARM_SPEED = 0.03f;    // m/s: ancora quando a roda quase para
    bool _turnActive = false;
    float _turnGoal = 0.0f, _turnDone = 0.0f;  // ° pedidos / ° girados (gyro Z integrado)
    uint32_t _turnStartMs = 0;
    static constexpr float TRACK_M = 0.150f;          // bitola centro a centro (docs/hardware.md)
    static constexpr float TURN_CMD_RATE = 45.0f;     // °/s máx do turn
    static constexpr float TURN_KP = 4.0f;            // 1/s: perto do alvo, taxa = 4 × graus que faltam
    static constexpr uint32_t TURN_TIMEOUT_MS = 10000; // no suporte o robô não gira: desiste
    static constexpr float TRIM_MAX_SPEED = 0.3f;    // m/s (deriva de até ~1,2° de erro de setpoint)
    static constexpr float TRIM_MAX_RATE = 20.0f;    // °/s
    static constexpr float TRIM_MAX_ERR_DEG = 3.0f;  // ° de pitch longe do setpoint
    static constexpr uint32_t TRIM_CALM_TICKS = 400; // 2 s calmo antes de começar
    static constexpr float TRIM_TAU_S = 10.0f;   // bem mais lento que a malha de posição (2,5 s brigava com ela: oscilação de ~15 s)
    static constexpr float TRIM_RANGE_DEG = 1.5f;    // máximo que se afasta do setpoint do start
    static constexpr float TARGET_ACCEL = 0.6f;  // m/s² (rampa de arrancada e frenagem)
    static constexpr float TURN_ACCEL = 240.0f;  // °/s² (0 → 60 °/s em 0,25 s)
    static constexpr float I_DECAY_S = 0.2f;     // s: integral se esvazia ao parar de andar (mais lento = demora a parar)
    static constexpr float I_MAX_DEG = 2.0f;     // máximo de inclinação que o integral pode pedir
    static const uint32_t CMD_TIMEOUT_MS = 500;
};

extern VelocityController velCtrl;
#endif
