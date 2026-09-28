"""Pure DM MIT codec. See docs/PROVENANCE.md; no implicit clipping."""
from dataclasses import dataclass
import math
import struct
from .config import MODELS

READ, WRITE, STATUS = 0x33, 0x55, 0xCC
DISABLE, ENABLE, ZERO = 0xFD, 0xFC, 0xFE
FIELDS = ("q_des", "dq_des", "kp", "kd", "tau_ff")


@dataclass(frozen=True)
class Frame:
    bus: str
    can_id: int
    data: bytes
    fd: bool = True
    flags: int = 1


def quantize(x, lo, hi, bits):
    if not math.isfinite(x) or not lo <= x <= hi:
        raise ValueError(f"out of protocol range: {x}")
    return int((x-lo)*((1 << bits)-1)/(hi-lo))


def unquantize(x, lo, hi, bits):
    return x*(hi-lo)/((1 << bits)-1)+lo


def pack_mit(j, target):
    p, v, t = MODELS[j["model"]]
    s, z = j["direction"], j["zero_joint"]
    q = quantize(s*(target["q_des"]-z), -p, p, 16)
    dq = quantize(s*target["dq_des"], -v, v, 12)
    kp = quantize(target["kp"], 0, 500, 12)
    kd = quantize(target["kd"], 0, 5, 12)
    tau = quantize(s*target["tau_ff"], -t, t, 12)
    return Frame(j["bus"], j["can_id"], bytes((q >> 8, q & 255, dq >> 4,
                 (dq & 15) << 4 | kp >> 8, kp & 255, kd >> 4,
                 (kd & 15) << 4 | tau >> 8, tau & 255)))


def special(j, code):
    return Frame(j["bus"], j["can_id"], b'\xff'*7+bytes([code]))


def register_request(j, op, rid=0, value=0):
    payload = struct.pack('<HBB', j["can_id"], op, rid)
    if op != STATUS:
        payload += struct.pack('<I' if rid in (7, 8, 9, 10, 13, 14, 15, 16, 35, 36) else '<f', value)
    return Frame(j["bus"], 0x7FF, payload)


def decode(j, frame):
    """Return parameter or feedback; never let a parameter refresh liveness."""
    if frame.bus != j["bus"] or frame.can_id != j["master_id"]:
        return None
    d = frame.data
    if len(d) == 8 and int.from_bytes(d[:2], 'little') == j["can_id"] and d[2] in (READ, WRITE):
        rid = d[3]
        value = struct.unpack('<I' if rid in (7, 8, 9, 10, 13, 14, 15, 16, 35, 36) else '<f', d[4:])[0]
        return ("parameter", (d[2], rid, value))
    if len(d) != 8 or d[0] & 15 != j["can_id"]:
        return None
    p, v, t = MODELS[j["model"]]
    q = unquantize(d[1] << 8 | d[2], -p, p, 16)
    dq = unquantize(d[3] << 4 | d[4] >> 4, -v, v, 12)
    tau = unquantize((d[4] & 15) << 8 | d[5], -t, t, 12)
    return ("feedback", dict(q=j["direction"]*q+j["zero_joint"],
            dq=j["direction"]*dq, tau=j["direction"]*tau,
            status=d[0] >> 4, mos_temperature=d[6], rotor_temperature=d[7]))
