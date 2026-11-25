"""Build inert adapters from model/role configuration; never connect on import."""
from collections.abc import Mapping

from contracts.robot_contract import normalize_robot_contract


def normalize_hardware_config(config, *, base_dir=None):
    if not isinstance(config, Mapping) or set(config) - {'type', 'arms'} or not config.get('arms'):
        raise ValueError('Hardware config requires type and arms')
    backend = config.get('type')
    if backend not in ('arx', 'yam'):
        raise ValueError('Hardware type must be arx or yam')
    if not isinstance(config['arms'], Mapping) or set(config['arms']) - {'left', 'right'}:
        raise ValueError('Hardware arms must name left and/or right')
    arms, ports = {}, set()
    for role in ('left', 'right'):
        if role not in config['arms']:
            continue
        arm = dict(config['arms'][role])
        allowed = {'model', 'can_interface', 'command_timeout_s', 'max_feedback_age_s'}
        if backend == 'yam':
            allowed |= {'i2rt_root', 'gripper_force_n'}
        if set(arm) - allowed:
            raise ValueError(f'Unknown hardware options for {role}')
        model = arm.get('model')
        supported = ('L5', 'Trossen_L5') if backend == 'arx' else ('yam_ultra_2',)
        if model not in supported:
            raise ValueError(f'{role}: explicitly choose one of {supported}')
        if backend == 'yam' and 'gripper_force_n' in arm:
            force = arm['gripper_force_n']
            if isinstance(force, bool) or not isinstance(force, (int, float)) or not 0 < force <= 25:
                raise ValueError(f'{role}: gripper_force_n must be in (0, 25]')
        port = arm.get('can_interface')
        if not isinstance(port, str) or not port.strip() or port in ports:
            raise ValueError('Each arm needs a distinct CAN interface')
        ports.add(port)
        arms[role] = arm
    return {'type': backend, 'arms': arms}


def hardware_robot_contract(config):
    return normalize_robot_contract({'robot_contract_version': 1,
        'arms': {role: {'model': arm['model']} for role, arm in config['arms'].items()}})


def build_arms(config, *, base_dir=None):
    config = normalize_hardware_config(config, base_dir=base_dir)
    # Validate every arm's configuration before connecting any hardware.
    if config['type'] == 'arx':
        from hardware.backends.arx import ARXAdapter as Adapter
    else:
        from hardware.backends.yam import YAMAdapter as Adapter
    return {role: Adapter(role=role, **arm) for role, arm in config['arms'].items()}
