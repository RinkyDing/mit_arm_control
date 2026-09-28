import argparse
import json
import os
from pathlib import Path
from .config import load_config, readiness
from .service import run, hardware_locks
from .backend import SocketCAN
from .protocol import READ


def main():
    parser = argparse.ArgumentParser(description='MIT arm controller: simulation by default; never auto-arms')
    parser.add_argument('operation', choices=('serve', 'check-config', 'diagnose'))
    parser.add_argument('--config', required=True)
    parser.add_argument('--socket', default='/tmp/mit-arm-control.sock')
    parser.add_argument('--hardware', action='store_true', help='explicitly permit real CAN access')
    parser.add_argument('--final-state', default='final-state.json', help='durable shutdown report')
    args = parser.parse_args()
    c = load_config(args.config)
    # 纯配置检查不打开 CAN，适合先查看仍缺哪些参数。
    if args.operation == 'check-config':
        errors = readiness(c, args.hardware)
        print(json.dumps({'ready': not errors, 'errors': errors}, ensure_ascii=False, indent=2))
        return int(bool(errors))
    if args.operation == 'diagnose':
        if not args.hardware:
            parser.error('diagnose is read-only hardware access; pass --hardware explicitly')
        joints = [j for j in c['joints'] if type(j.get('can_id')) is int
                  and type(j.get('master_id')) is int and j.get('bus')]
        if not joints:
            parser.error('no fully specified IDs to query')
        # 只读诊断不构造控制器，也不写模式、归零、使能或失能。
        locks = hardware_locks(dict(c, joints=joints), commissioned=False)
        bus = None
        try:
            bus = SocketCAN(joints)
            for j in joints:
                values = {rid: bus.register(j, READ, rid) for rid in (10, 21, 22, 23)}
                print(json.dumps({'joint': j['name'], 'registers': values}))
        finally:
            if bus is not None:
                bus.close()
            for fd in locks:
                os.close(fd)
        return 0
    run(c, args.socket, args.hardware, args.final_state)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
