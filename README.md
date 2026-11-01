# Navibot motor controller

This first controller version targets a classic Arduino Nano with an ATmega328P,
an L298N dual H-bridge, and two N20 motors. It uses one rising-edge encoder
signal per motor. The target sign chooses direction; encoder feedback measures
speed magnitude only.

## Pins

| Function | Left | Right |
| --- | --- | --- |
| L298N enable/PWM | D10 / ENA | D9 / ENB |
| Direction input 1 | A4 / IN1 | A1 / IN3 |
| Direction input 2 | A3 / IN2 | A2 / IN4 |
| Encoder signal | D2 / INT0 | D3 / INT1 |

The left motor preserves the original sketch's pins exactly: ENA D10, IN1 A4,
IN2 A3, encoder D2. The right motor uses the pins shown above: ENB D9, IN3 A1,
IN4 A2, encoder D3. D6 is reserved for RS-485 direction control. Remove the ENA
and ENB jumpers on the L298N module so the Nano can control speed with PWM.
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

The code sends replies through UART and controls the RS-485 transceiver using
D6. Tie the transceiver's DE and active-low `/RE` pins together and connect them
to D6: LOW selects receive mode, and HIGH selects transmit mode for replies.
The sketch switches back to receive mode after UART transmission completes.
No text diagnostics are sent on the same UART.

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

## USB serial keyboard teleop

`teleop.py` sends the binary protocol above through a USB-to-RS-485 adapter. It
sends each selected RPM setpoint to both node IDs `0x01` and `0x02`, and repeats
both commands every 100 ms. It sends to one node and waits briefly for its reply
before addressing the next node, to give the half-duplex RS-485 bus time to turn
around. The terminal shows each node's ACK/NACK or `no reply` status. It requires
Python 3.10 or newer and `pyserial`:

```sh
python3 -m pip install -r requirements.txt
python3 teleop.py --port /dev/cu.usbserial-XXXX --rpm 30
```

On Windows, open PowerShell or Command Prompt and use the COM port shown in
Device Manager, for example:

```powershell
python teleop.py --port COM3 --rpm 30
```

On Linux, the port is often `/dev/ttyUSB0` or `/dev/ttyACM0`. On macOS, use the
device listed under `/dev/cu.*`. The script uses 115200 baud by default, matching
the sketch; override it with `--baud` if needed. Run it in an interactive
Windows, macOS, or Linux terminal that supports single-key input.

| Key | Command sent to the two motor firmware |
| --- | --- |
| W | Forward: left +RPM, right +RPM |
| S | Reverse: left -RPM, right -RPM |
| A | Pivot left: left -RPM, right +RPM |
| D | Pivot right: left +RPM, right -RPM |
| Space | Stop |
| X or Ctrl-C | Send stop and exit |

The two firmware boards must have different node IDs. The sketch defaults to
`0x01`; before compiling for the second board, change the `NAVIBOT_NODE_ID`
default near the protocol constants to `0x02`, then upload that build to the
second board. Upload a build set to `0x01` to the first board. Do not put two
boards with the same ID on one bus, since their replies would collide.

This firmware only accepts two wheel speeds per node. It cannot command lateral
strafe or provide the wheel mixing for a three- or four-wheel omni base; that
requires a matching multi-wheel firmware protocol and motor controller. Verify
the robot's wheel orientation and motor signs with the wheels raised before
running it on the floor. The USB-RS485 adapter is the bus master and should
provide automatic transmit/receive direction control; D6 controls the
transceiver on the Nano node.
