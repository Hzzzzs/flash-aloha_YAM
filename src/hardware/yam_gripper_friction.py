"""Velocity-based manual assistance for YAM Ultra 2 linear_4310 grippers.

Units are SDK motor-coordinate radians and Nm, NOT normalized opening or ARX
gripper metres. No constant bias or acceleration feedback is applied.
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

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f'{field.name} must be a finite number')
        if not (0 <= self.coulomb_nm <= 0.06 and 0 <= self.viscous_nm_s_rad <= 0.02
                and 0.02 <= self.transition_rad_s <= 2.0
                and 0.005 <= self.deadband_rad_s <= 0.2
                and 0 < self.max_torque_nm <= 0.06
                and 0.1 <= self.max_speed_rad_s <= 8.0
                and 0.02 <= self.end_margin <= 0.2):
            raise ValueError('Gripper friction parameter outside supported tuning limits')

    def torque(self, opening, normalized_velocity, motor_velocity):
        if not all(math.isfinite(v) for v in (opening, normalized_velocity, motor_velocity)):
            raise ValueError('Non-finite gripper feedback')
        if not 0 <= opening <= 1:
            return 0.0
        if abs(motor_velocity) > self.max_speed_rad_s:
            raise RuntimeError('Gripper assist overspeed; assistance stopped')
        speed = max(0.0, abs(motor_velocity) - self.deadband_rad_s)
        velocity = math.copysign(speed, motor_velocity)
        torque = (self.coulomb_nm * math.tanh(velocity / self.transition_rad_s)
                  + self.viscous_nm_s_rad * velocity)
        torque = max(-self.max_torque_nm, min(self.max_torque_nm, torque))
        # Taper only toward the approached stop; still allow retreat from it.
        distance = 1 - opening if normalized_velocity > 0 else opening
        return torque * min(1.0, max(0.0, distance / self.end_margin))


def load_config(path):
    document = json.loads(Path(path).read_text())
    if (set(document) != {'schema_version', 'robot', 'gripper', 'arms'}
            or document['schema_version'] != 1 or document['robot'] != 'yam_ultra_2'
            or document['gripper'] != 'linear_4310'):
        raise ValueError('Expected YAM Ultra 2 / linear_4310 friction config v1')
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
    document = dict(schema_version=1, robot='yam_ultra_2', gripper='linear_4310',
                    arms={name: asdict(value) for name, value in arms.items()})
    path.parent.mkdir(parents=True, exist_ok=True)
    from recording.recording_utils import atomic_write_json
    atomic_write_json(path, document)
