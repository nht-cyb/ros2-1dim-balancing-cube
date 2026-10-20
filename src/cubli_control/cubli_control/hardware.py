"""ESP32 serial protocol and Nidec 24H motor model (no ROS imports; shared by the
driver node, the fake ESP32 and the tests).

Line protocol, NMEA style: `$<TYPE>,<field>,...*<XOR checksum in hex>\\n`

  ESP32 -> PC  $S,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<enc>*HH   sensor frame (200 Hz)
                 accel in m/s^2, gyro in rad/s, both in the MPU6050's own axes;
                 enc = signed wheel encoder count
               $X,<t_us>,<angle>,<rate>,<wheel_speed>,<torque>,<mode>*HH
                 on-board estimate (on-board mode only, right after each $S): tilt [rad]
                 before angle_offset, tilt rate [rad/s], wheel speed [rad/s], the
                 LQR torque it applied [N m] and its mode (MODE_* below); while
                 calibrating, tilt and rate are 0
               $I,<text>*HH                                         info / log line
  PC -> ESP32  $T,<effort>*HH     motor effort in [-1, 1] (sign = direction)
               $B,<ms>*HH         brake pulse for <ms> milliseconds (jump-up)
               $E,<0|1>*HH        disable / enable the motor
               $M,<0|1>*HH        0 = passthrough (PC closes the loop), 1 = on-board balance
               $K,<k1>,<k2>,<k3>*HH   LQR gain, u = -K [tilt, rate, wheel speed]
               $P,<name>,<value>*HH   on-board parameter, see ONBOARD_PARAMS
               $A,<ax>,<ay>,<az>*HH   body axes from sensor axes, e.g. $A,+x,-z,+y

The ESP32 stops the motor if no $T arrives for 100 ms. In on-board mode it runs the
estimator and the LQR itself at 200 Hz: while it balances it ignores $T; otherwise it
applies $T as usual (spin-up). It engages within engage_angle, or within catch_angle for
jump_timeout seconds after a $B, and gives up like the PC controller does. Switching to
mode 1 (or sending $A) restarts the gyro calibration, so the cube must lie still. If no
valid line arrives from the PC for 2 s it stops balancing and the motor.
"""

from dataclasses import dataclass
import math


def checksum(body: str) -> str:
    value = 0
    for ch in body:
        value ^= ord(ch)
    return f'{value:02X}'


def encode(kind: str, *fields) -> bytes:
    body = ','.join([kind] + [str(f) for f in fields])
    return f'${body}*{checksum(body)}\n'.encode('ascii')


def parse(line):
    """Return (kind, [fields]) for a valid line, or None for a corrupt/partial one."""
    if isinstance(line, bytes):
        try:
            line = line.decode('ascii')
        except UnicodeDecodeError:
            return None
    line = line.strip()
    if not line.startswith('$') or '*' not in line:
        return None
    body, _, check = line[1:].rpartition('*')
    if checksum(body) != check.upper():
        return None
    kind, *fields = body.split(',')
    return kind, fields


@dataclass
class SensorFrame:
    t_us: int
    accel: tuple   # (ax, ay, az) m/s^2, sensor axes
    gyro: tuple    # (gx, gy, gz) rad/s, sensor axes
    encoder: int

    @classmethod
    def from_fields(cls, fields):
        if len(fields) != 8:
            raise ValueError(f'expected 8 fields, got {len(fields)}')
        v = [float(f) for f in fields[1:7]]
        return cls(int(fields[0]), tuple(v[0:3]), tuple(v[3:6]), int(fields[7]))

    def encode(self) -> bytes:
        a, g = self.accel, self.gyro
        return encode('S', self.t_us, f'{a[0]:.3f}', f'{a[1]:.3f}', f'{a[2]:.3f}',
                      f'{g[0]:.4f}', f'{g[1]:.4f}', f'{g[2]:.4f}', self.encoder)


MODE_PASSTHROUGH, MODE_CALIBRATING, MODE_IDLE, MODE_BALANCE = 0, 1, 2, 3
MODE_NAMES = {MODE_PASSTHROUGH: 'PASSTHROUGH', MODE_CALIBRATING: 'CALIBRATING',
              MODE_IDLE: 'IDLE', MODE_BALANCE: 'BALANCE'}

# $P names the firmware understands, with its defaults (units: rad, N m, rad/s, s, counts).
ONBOARD_PARAMS = {
    'engage': 0.12, 'disengage': 0.16, 'catch': 0.4, 'jump_timeout': 1.5,
    'max_torque': 0.08, 'stall_torque': 0.10, 'no_load_speed': 600.0, 'max_effort': 1.0,
    'angle_offset': 0.0, 'tau': 0.5, 'imu_offset': 0.106, 'cpr': 400, 'enc_sign': 1,
}


@dataclass
class EstimateFrame:
    t_us: int
    angle: float        # [rad], before angle_offset (like the estimator node's output)
    rate: float         # [rad/s]
    wheel_speed: float  # [rad/s]
    torque: float       # [N m] applied by the on-board LQR (0 when not balancing)
    mode: int           # MODE_*

    @classmethod
    def from_fields(cls, fields):
        if len(fields) != 6:
            raise ValueError(f'expected 6 fields, got {len(fields)}')
        v = [float(f) for f in fields[1:5]]
        return cls(int(fields[0]), *v, int(fields[5]))

    def encode(self) -> bytes:
        return encode('X', self.t_us, f'{self.angle:.5f}', f'{self.rate:.4f}',
                      f'{self.wheel_speed:.2f}', f'{self.torque:.5f}', self.mode)


def onboard_setup(k, params: dict, axes) -> list:
    """Commands that configure and start on-board balance (send after $E,1)."""
    unknown = set(params) - set(ONBOARD_PARAMS)
    if unknown:
        raise ValueError(f'unknown on-board parameters: {sorted(unknown)}')
    k = [float(v) for v in k]
    if len(k) != 3:
        raise ValueError('the on-board LQR takes exactly 3 gains')
    lines = [encode('A', *axes)]
    lines += [encode('P', name, f'{float(value):.6g}') for name, value in params.items()]
    lines.append(encode('K', *(f'{v:.6g}' for v in k)))
    lines.append(encode('M', 1))
    return lines


def remap_axes(vector, axes):
    """Map sensor-frame vector to the body frame.

    axes: three strings like ['+x', '-z', '+y'] giving, for body x, y, z, which sensor
    axis (and sign) it equals. Body x must be the balancing edge axis.
    """
    index = {'x': 0, 'y': 1, 'z': 2}
    out = []
    for spec in axes:
        sign = -1.0 if spec[0] == '-' else 1.0
        out.append(sign * vector[index[spec[-1]]])
    return tuple(out)


@dataclass(frozen=True)
class NidecMotor:
    """Nidec 24H with its built-in driver, modelled as a voltage-driven DC motor.

    torque = stall_torque * (effort - speed / no_load_speed)

    Both constants are placeholders until you measure your motor (see README).
    """
    stall_torque: float = 0.10      # [N m] at full effort, wheel stopped
    no_load_speed: float = 600.0    # [rad/s] at full effort, no load

    def effort_for(self, torque: float, speed: float, max_effort: float = 1.0) -> float:
        effort = torque / self.stall_torque + speed / self.no_load_speed
        return max(-max_effort, min(max_effort, effort))

    def torque_for(self, effort: float, speed: float) -> float:
        return self.stall_torque * (effort - speed / self.no_load_speed)


def counts_to_rad(counts: int, cpr: int) -> float:
    return counts * 2.0 * math.pi / cpr
