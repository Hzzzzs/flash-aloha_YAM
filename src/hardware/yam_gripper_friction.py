"""Velocity-based manual assistance for YAM Ultra 2 linear_4310 grippers.

Units are SDK motor-coordinate radians and Nm, NOT normalized opening or ARX
gripper metres. Optional filtered acceleration assistance shares the torque cap.
"""
from dataclasses import asdict, dataclass, fields
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class GripperFriction:
    coulomb_nm: float = 0.0
    viscous_nm_s_rad: float = 0.0
    transition_rad_s: float = 0.2
    deadband_rad_s: float = 0.02
    max_torque_nm: float = 0.03
    max_speed_rad_s: float = 8.0
    end_margin: float = 0.05
    acceleration_enabled: bool = False
    acceleration_gain_nm_s2_rad: float = 0.0
    acceleration_deadband_rad_s2: float = 0.5
    acceleration_filter_s: float = 0.04

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name == 'acceleration_enabled':
                if not isinstance(value, bool):
                    raise ValueError('acceleration_enabled must be boolean')
                continue
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f'{field.name} must be a finite number')
        if not (0 <= self.coulomb_nm <= 0.06 and 0 <= self.viscous_nm_s_rad <= 0.02
                and 0.02 <= self.transition_rad_s <= 2.0
                and 0.005 <= self.deadband_rad_s <= 0.2
                and 0 < self.max_torque_nm <= 0.06
                and 0.1 <= self.max_speed_rad_s <= 8.0
                and 0.02 <= self.end_margin <= 0.2
                and 0 <= self.acceleration_gain_nm_s2_rad <= 0.002
                and 0.1 <= self.acceleration_deadband_rad_s2 <= 20
                and 0.02 <= self.acceleration_filter_s <= 0.2):
            raise ValueError('Gripper friction parameter outside supported tuning limits')

    def torque(self, opening, normalized_velocity, motor_velocity, acceleration=0.0):
        if not all(math.isfinite(v) for v in (opening, normalized_velocity, motor_velocity, acceleration)):
            raise ValueError('Non-finite gripper feedback')
        if not 0 <= opening <= 1:
            return 0.0
        if abs(motor_velocity) > self.max_speed_rad_s:
            raise RuntimeError('Gripper assist overspeed; assistance stopped')
        speed = max(0.0, abs(motor_velocity) - self.deadband_rad_s)
        velocity = math.copysign(speed, motor_velocity)
        torque = (self.coulomb_nm * math.tanh(velocity / self.transition_rad_s)
                  + self.viscous_nm_s_rad * velocity)
        # Assist speeding up only; do not amplify deceleration or push at rest.
        # This also keeps endpoint taper direction consistent with motor torque.
        if self.acceleration_enabled and speed > 0 and acceleration * velocity > 0:
            accel = math.copysign(max(0., abs(acceleration) - self.acceleration_deadband_rad_s2), acceleration)
            torque += self.acceleration_gain_nm_s2_rad * accel
        torque = max(-self.max_torque_nm, min(self.max_torque_nm, torque))
        # Taper only toward the approached stop; still allow retreat from it.
        distance = 1 - opening if normalized_velocity > 0 else opening
        return torque * min(1.0, max(0.0, distance / self.end_margin))


class AccelerationEstimator:
    """Differentiate fresh SDK publications, then low-pass using their actual dt."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.previous = None
        self.filtered = 0.0

    def update(self, velocity, timestamp, filter_s):
        if not all(math.isfinite(v) for v in (velocity, timestamp, filter_s)) or filter_s <= 0:
            raise ValueError('Invalid acceleration feedback/filter')
        if self.previous is None:
            self.previous = (velocity, timestamp)
            return 0.0
        last_velocity, last_timestamp = self.previous
        dt = timestamp - last_timestamp
        if dt < 0 or dt > 0.1:
            self.reset()
            self.previous = (velocity, timestamp)
            return 0.0
        if dt < 0.001:
            # Duplicate/too-close publications must not create derivative spikes.
            return self.filtered
        self.previous = (velocity, timestamp)
        raw = (velocity - last_velocity) / dt
        alpha = -math.expm1(-dt / filter_s)
        self.filtered += alpha * (raw - self.filtered)
        return self.filtered


def load_config(path):
    document = json.loads(Path(path).read_text())
    if (set(document) != {'schema_version', 'robot', 'gripper', 'arms'}
            or document['schema_version'] not in (1, 2) or document['robot'] != 'yam_ultra_2'
            or document['gripper'] != 'linear_4310'):
        raise ValueError('Expected YAM Ultra 2 / linear_4310 assist config v1 or v2')
    arms = document['arms']
    if not isinstance(arms, dict) or not arms or set(arms) - {'left', 'right'}:
        raise ValueError('Friction config must name left and/or right arms')
    return {role: GripperFriction(**parameters) for role, parameters in arms.items()}


def save_config(path, role, parameters):
    if role not in ('left', 'right'):
        raise ValueError('Expected left or right')
    path = Path(path)
    arms = load_config(path) if path.exists() else {}
    arms[role] = parameters
    document = dict(schema_version=2, robot='yam_ultra_2', gripper='linear_4310',
                    arms={name: asdict(value) for name, value in arms.items()})
    path.parent.mkdir(parents=True, exist_ok=True)
    from recording.recording_utils import atomic_write_json
    atomic_write_json(path, document)
