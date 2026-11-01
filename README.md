# Navibot motor controller

This first controller version targets a classic Arduino Nano with an ATmega328P,
an L298N dual H-bridge, and two N20 motors. It uses one rising-edge encoder
signal per motor. The target sign chooses direction; encoder feedback measures
speed magnitude only.

## Pins

| Function | Left | Right |
| --- | --- | --- |
| L298N enable/PWM | D10 / ENA | D6 / ENB |
| Direction input 1 | A4 / IN1 | D7 / IN3 |
| Direction input 2 | A3 / IN2 | D8 / IN4 |
| Encoder signal | D2 / INT0 | D3 / INT1 |

The left motor preserves the original sketch's pins exactly: ENA D10, IN1 A4,
IN2 A3, encoder D2. The right motor uses the added pins shown above. Remove the
ENA and ENB jumpers on the L298N module so the Nano can control speed with PWM.
Supply motor power from a suitable external supply and join its ground to Nano
ground. Do not power the motors from the Nano's 5 V pin.

## Binary protocol implementation

The sketch listens on `Serial` at 115200 baud. It accepts only frames addressed
to `NODE_ID` and only valid request messages. Other node IDs are ignored to
avoid multiple nodes replying on the shared RS-485 bus.

To make the not-yet-specified payload concrete, the current implementation uses
these provisional formats:

- Request payload: left signed int16 RPM, then right signed int16 RPM, each
  little-endian, whole RPM. Payload length is 4.
- Reply message type `0x81`, with payload: status (0 ACK / 1 NACK), reason code
  (0 for ACK), signed int16 left measured RPM, signed int16 right measured RPM.
- Reply sequence echoes the request sequence. The node stores the next expected
  sequence value in memory, but does not reject out-of-order or duplicate
  sequence values until sender retry behavior is defined.
- CRC-16/CCITT-FALSE covers offsets 2 through the payload end; sync and CRC
  bytes are excluded. CRC is transmitted low byte first.
- Maximum payload is 16 bytes. `COMMAND_TIMEOUT_MS` is provisionally 1000 ms.

The code sends replies through UART. `RS485_DIRECTION_PIN` is `0xFF` until the
transceiver control pin and polarity are provided, so the sketch cannot yet
switch the transceiver into transmit mode. Set that constant when the hardware
detail is known. No text diagnostics are sent on the same UART.

Until commands are sent, both motors start stopped. For a temporary bench run,
set `START_LEFT_RPM` and/or `START_RIGHT_RPM` near the top of the sketch.

The function rejects invalid motor indexes, non-finite values, and targets
outside `MAX_TARGET_RPM`. It returns `true` only when it accepts the request.
After a fault, set both targets to zero and then call `clearFault()` to re-arm.

## Operating and fault cases

- **Startup:** all bridge outputs are set low; both target speeds start at zero.
- **Normal run:** each motor has an independent 100 ms RPM sample and PID state.
- **Zero target:** PWM and direction outputs are disabled for that motor.
- **Direction reversal:** the motor coasts for at least one sample interval
  before the opposite direction is applied.
- **Invalid command:** addressed command receives a NACK; targets and command
  watchdog are unchanged.
- **Valid command:** both targets update together and an ACK includes current
  measured RPM for both motors.
- **Command timeout:** if no valid addressed command arrives within
  `COMMAND_TIMEOUT_MS`, both targets are set to zero.
- **Encoder stall:** if a commanded motor produces no encoder edge for 2 seconds,
  the controller latches a fault and disables both motors.
- **Latched fault:** both motors stay disabled. An all-zero valid request clears
  the fault; nonzero requests receive a NACK until it is cleared.

## Values to verify before powered operation

1. `LEFT_COUNTS_PER_REV` and `RIGHT_COUNTS_PER_REV` must be the measured number
   of **rising edges counted on this one input** per output-shaft revolution.
   The provisional 1000 value is inherited from the supplied sketch, not
   confirmed for these motors. Count edges over several shaft turns and divide
   by the number of turns; set each motor separately if they differ.
2. `MAX_TARGET_RPM` is a software limit, not a guarantee the motor can reach
   that speed through the L298N under load. The listing says 135 RPM but does not
   provide a dependable stall-current figure.
3. Check the exact motor stall current and L298N module thermal rating. The
   L298N has no current measurement in this firmware, so a software stall fault
   cannot protect against overcurrent during the 2-second detection window.
4. Confirm encoder output voltage and pin order. Connect encoder VCC to its
   specified logic voltage, encoder ground to Nano ground, and each signal to
   its assigned input. Avoid feeding a signal above the Nano's 5 V logic rail.
5. The initial gains are P-only (`KP = 1.70`, `KI = KD = 0`). Tune first with
   wheels clear of the ground and a conservative target. L298N voltage loss,
   load, and gear friction can require different gains and lower speed limits.

This version does not yet include a hardware emergency-stop input, current
sensing, supply-voltage monitoring, or a hardware watchdog. The L298N module
and software are suitable for a controlled prototype, not a certified
industrial safety function.
