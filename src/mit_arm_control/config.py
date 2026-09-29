"""Explicit robot configuration; no guessed hardware safety limits."""
import json
import math
from pathlib import Path

# SDK 轴顺序固定；gripper 对应用户所称 J7。
NAMES = ("J1", "J2", "J3", "J4", "J5", "J6", "gripper")
# 电机协议量程 (位置 rad, 速度 rad/s, 力矩 N·m)，不等于机械安全限位。
MODELS = {"4310_48V": (12.5, 50.0, 10.0), "4340_48V": (12.5, 20.0, 28.0)}
LIMIT_KEYS = ("q_min", "q_max", "dq_max", "tau_max", "kp_max", "kd_max",
              "q_rate", "dq_rate", "kp_rate", "kd_rate", "tau_rate", "temperature_max")


def finite(x):
    return type(x) in (int, float) and math.isfinite(x)


def load_config(path, *, no_gripper=False):
    c = json.loads(Path(path).read_text())
    if c.get("version") != 1:
        raise ValueError("configuration version must be 1")
    if no_gripper:
        c['joints'] = [j for j in c.get('joints', []) if j.get('name') != 'gripper']
    expected = NAMES[:-1] if no_gripper else NAMES
    if [j.get("name") for j in c.get("joints", [])] != list(expected):
        raise ValueError("joints must be ordered " + ", ".join(expected))
    for j in c["joints"]:
        if j.get("model") != ("4340_48V" if j["name"] in ("J2", "J3") else "4310_48V"):
            raise ValueError(f"{j['name']}: unexpected robot model")
    for key, default in (("rate_hz", 1000), ("command_timeout", .05),
                         ("feedback_timeout", .05), ("stop_timeout", 2.0)):
        c.setdefault(key, default)
        if not finite(c[key]) or c[key] <= 0:
            raise ValueError(f"{key} must be positive and finite")
    if c["rate_hz"] > 2000 or c["stop_timeout"] > 5:
        raise ValueError("rate_hz <= 2000 and stop_timeout <= 5 required")
    c.setdefault("max_unobserved", 4)
    if type(c["max_unobserved"]) is not int or c["max_unobserved"] < 1:
        raise ValueError("max_unobserved must be a positive integer")
    return c


def readiness(c, hardware=False, ranges=None):
    # 汇总所有缺项供界面展示；未知方向、零位或限制不会被猜测填充。
    errors, ids = [], set()
    for j in c["joints"]:
        name = j["name"]
        for field in ("can_id", "master_id"):
            v = j.get(field)
            hi = 15 if field == "can_id" else 0x7FE
            if type(v) is not int or not 1 <= v <= hi:
                errors.append(f"{name}.{field}: missing/invalid")
            key = (j.get("bus"), v)
            if key in ids:
                errors.append(f"{name}: duplicate bus ID {v}")
            ids.add(key)
        if not isinstance(j.get("bus"), str) or not j["bus"]:
            errors.append(f"{name}.bus missing")
        lim = j.get("limits", {})
        if any(not finite(lim.get(k)) for k in LIMIT_KEYS):
            errors.append(f"{name}: incomplete limits")
            continue
        # 实机离线检查不知道电机量程；arm 读回后再检查能否容纳机械限制。
        actual_range = ranges.get(name) if ranges is not None else (
            None if hardware else MODELS[j["model"]]
        )
        if ranges is not None and actual_range is None:
            errors.append(f"{name}: missing runtime protocol range")
        if not (lim["q_min"] <= 0 <= lim["q_max"] and lim["q_min"] < lim["q_max"]):
            errors.append(f"{name}: position limits must include zero")
        if actual_range is not None:
            if (len(actual_range) != 3
                    or any(not finite(value) or value <= 0 for value in actual_range)):
                errors.append(f"{name}: invalid protocol range")
            else:
                p, v, t = actual_range
                if not (-p+.1 <= lim["q_min"] and lim["q_max"] <= p-.1
                        and lim["dq_max"] <= v and lim["tau_max"] <= t):
                    errors.append(f"{name}: configured limits exceed protocol range or wrap margin")
        if not (0 < lim["dq_max"] and 0 < lim["tau_max"]
                and 0 < lim["kp_max"] <= 500 and 0 < lim["kd_max"] <= 5
                and 0 < lim["temperature_max"] <= 100):
            errors.append(f"{name}: invalid physical/gain limits")
        if any(lim[k] <= 0 for k in ("q_rate", "dq_rate", "kp_rate", "kd_rate", "tau_rate")):
            errors.append(f"{name}: command rate limits must be positive")
    return errors


def select_joints(config, names):
    if not names or len(names) != len(set(names)):
        raise ValueError('observation requires unique joint names')
    by_name = {joint['name']: joint for joint in config['joints']}
    joints, ids = [], set()
    for name in names:
        if name not in by_name:
            raise ValueError(f'unknown joint: {name}')
        joint = by_name[name]
        if not isinstance(joint.get('bus'), str) or not joint['bus']:
            raise ValueError(f'{name}: bus required')
        for key, maximum in (('can_id', 15), ('master_id', 0x7FE)):
            value = joint.get(key)
            if type(value) is not int or not 1 <= value <= maximum:
                raise ValueError(f'{name}: invalid {key}')
            identity = (joint['bus'], value)
            if identity in ids:
                raise ValueError(f'{name}: duplicate bus ID')
            ids.add(identity)
        # 观察模式与运动模式均直接使用电机坐标。
        joints.append(dict(joint))
    return joints
