"""Part 3: serial protocol, axis remapping and motor model (no ROS or board needed)."""

import os
import re

import pytest

from cubli_control.hardware import (
    MODE_BALANCE, ONBOARD_PARAMS, EstimateFrame, NidecMotor, SensorFrame, checksum,
    counts_to_rad, encode, onboard_setup, parse, remap_axes)

SKETCH = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'firmware', 'cubli_esp32',
                      'cubli_esp32.ino')


def test_encode_matches_nmea_style_checksum():
    assert encode('T', '0.250') == f'$T,0.250*{checksum("T,0.250")}\n'.encode()
    assert parse(encode('B', 300)) == ('B', ['300'])


def test_corrupt_or_partial_lines_are_rejected():
    good = encode('T', '0.250').decode().strip()
    assert parse(good) is not None
    assert parse(good.replace('0.250', '0.350')) is None   # payload changed
    assert parse(good[:-3]) is None                         # no checksum
    assert parse(b'\xff\xfe garbage') is None
    assert parse('') is None


def test_sensor_frame_round_trip():
    frame = SensorFrame(123456, (0.1, -6.937, 6.937), (0.0123, -0.0005, 0.0), -4242)
    kind, fields = parse(frame.encode())
    back = SensorFrame.from_fields(fields)
    assert kind == 'S'
    assert back.t_us == 123456 and back.encoder == -4242
    assert back.accel == pytest.approx(frame.accel, abs=1e-3)
    assert back.gyro == pytest.approx(frame.gyro, abs=1e-4)


def test_sensor_frame_wrong_field_count():
    with pytest.raises(ValueError):
        SensorFrame.from_fields(['1', '2', '3'])


def test_firmware_frame_format_parses():
    # Same printf format as the ESP32 sketch.
    body = 'S,%lu,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%ld' % (
        4000000000, 0.0, -6.937, 6.937, 0.0012, 0.0, -0.0003, -17)
    kind, fields = parse(f'${body}*{checksum(body)}\r\n')
    assert kind == 'S' and SensorFrame.from_fields(fields).encoder == -17


def test_remap_axes():
    v = (1.0, 2.0, 3.0)
    assert remap_axes(v, ['+x', '+y', '+z']) == (1.0, 2.0, 3.0)
    assert remap_axes(v, ['-y', '+z', '-x']) == (-2.0, 3.0, -1.0)


def test_motor_model_round_trip_and_saturation():
    motor = NidecMotor(stall_torque=0.1, no_load_speed=600.0)
    effort = motor.effort_for(0.05, 120.0)
    assert effort == pytest.approx(0.7)
    assert motor.torque_for(effort, 120.0) == pytest.approx(0.05)
    assert motor.effort_for(1.0, 0.0) == 1.0
    assert motor.effort_for(-1.0, 0.0, max_effort=0.5) == -0.5


def test_counts_to_rad():
    assert counts_to_rad(400, 400) == pytest.approx(6.283185307)


def test_onboard_command_messages():
    assert parse(encode('M', 1)) == ('M', ['1'])
    assert parse(encode('K', '-12.5', '-1.25', '-0.001')) == ('K', ['-12.5', '-1.25', '-0.001'])
    assert parse(encode('P', 'engage', '0.12')) == ('P', ['engage', '0.12'])
    assert parse(encode('A', '+x', '-z', '+y')) == ('A', ['+x', '-z', '+y'])


def test_onboard_setup_sequence():
    lines = onboard_setup([-3.1, -0.42, -0.00123456789], {'engage': 0.12, 'cpr': 400},
                          ['+x', '-z', '+y'])
    msgs = [parse(line) for line in lines]
    assert msgs[0] == ('A', ['+x', '-z', '+y'])          # axes first: they restart calibration
    assert ('P', ['engage', '0.12']) in msgs and ('P', ['cpr', '400']) in msgs
    assert msgs[-2] == ('K', ['-3.1', '-0.42', '-0.00123457'])
    assert msgs[-1] == ('M', ['1'])                      # mode switch last
    assert all(len(line) < 96 for line in lines)         # fits the firmware's line buffer


def test_onboard_setup_rejects_bad_input():
    with pytest.raises(ValueError):
        onboard_setup([1.0, 2.0, 3.0], {'not_a_param': 1.0}, ['+x', '+y', '+z'])
    with pytest.raises(ValueError):
        onboard_setup([1.0, 2.0, 3.0, 4.0], {}, ['+x', '+y', '+z'])   # delay-aware gain


def test_estimate_frame_round_trip():
    frame = EstimateFrame(4000000123, 0.01234, -0.5678, 123.45, -0.01234, MODE_BALANCE)
    kind, fields = parse(frame.encode())
    back = EstimateFrame.from_fields(fields)
    assert kind == 'X' and back.t_us == 4000000123 and back.mode == MODE_BALANCE
    assert (back.angle, back.rate, back.wheel_speed, back.torque) == pytest.approx(
        (0.01234, -0.5678, 123.45, -0.01234))
    with pytest.raises(ValueError):
        EstimateFrame.from_fields(fields[:-1])


def test_firmware_estimate_format_parses():
    # Same printf format as the ESP32 sketch.
    body = 'X,%lu,%.5f,%.4f,%.2f,%.5f,%d' % (17, -0.01, 0.2, -55.5, 0.08, 3)
    kind, fields = parse(f'${body}*{checksum(body)}\r\n')
    assert kind == 'X' and EstimateFrame.from_fields(fields).wheel_speed == -55.5


def test_firmware_knows_every_onboard_param():
    with open(SKETCH) as f:
        sketch = f.read()
    names = set(re.findall(r'strcmp\(name, "(\w+)"\)', sketch))
    assert names == set(ONBOARD_PARAMS)
