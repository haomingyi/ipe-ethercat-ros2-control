# Hardware Test Safety Checklist

## Before every power-on

- Secure the joint body so reaction torque cannot move it.
- Clear people, tools, and loose objects from the flange workspace.
- Prepare a reachable emergency stop or DC power disconnect.
- Verify supply voltage, current limit, polarity, and protective earth.
- Use one dedicated Ethernet interface and start only one EtherCAT master.
- Confirm whether the center bore contains cables before allowing multi-turn motion.

## Recommended sequence

1. Start with monitoring only and verify slave identity, OP state, WKC, and PDO health.
2. Clear a latched fault only after reading the status word and error code.
3. Enable at a zero command and confirm stable feedback.
4. Start with a small CSP angle or a low, short CSV/CST command.
5. Watch position, velocity, torque, supply current, temperature, and physical motion.
6. Return the command to zero, deactivate the controller, then stop the master.

## Mode-specific risk

- **CSP:** an incorrect target can command an immediate large displacement.
- **CSV:** a nonzero command continues rotating until it is explicitly zeroed.
- **CST:** constant torque can accelerate an unloaded joint even when the value looks small.

In an emergency, cut drive power. Do not wait for a software shutdown sequence.
