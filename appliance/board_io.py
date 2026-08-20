"""Serialized Board writes — call only from the ArmPi_mini main thread."""

from __future__ import annotations

from typing import Any, List, Tuple


def write_batched_pwm(board: Any, duration_s: float, packet: List[Tuple[int, int]]) -> None:
    if board is None or not hasattr(board, "pwm_servo_set_position"):
        return
    rows = [[int(sid), int(pulse)] for sid, pulse in packet]
    board.pwm_servo_set_position(max(duration_s, 0.01), rows)


def cartesian_to_joints(ak: Any, x_cm: float, y_cm: float, z_cm: float, pitch_deg: float) -> List[float]:
    """Use existing ArmIK; return [j1..j4] degrees or raise."""
    if ak is None or not hasattr(ak, "setPitchRange"):
        raise RuntimeError("ArmIK not available")
    result = ak.setPitchRange((x_cm, y_cm, z_cm), pitch_deg, pitch_deg - 30, pitch_deg + 30)
    if not result:
        raise RuntimeError("IK failed")
    servos, _alpha, *_rest = result
    # servos keys 3..6 → elbow, shoulder, lift, base
    order = (6, 5, 4, 3)
    pulses = [float(servos.get(i, 1500)) for i in order]
    return [((p - 500.0) * 180.0 / 2000.0) - 90.0 for p in pulses]
