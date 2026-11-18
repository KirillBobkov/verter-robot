"""Track BMS sample age independently of publisher heartbeats."""

import copy
import math
import time


class BatteryState:
    def __init__(self, robot_id, clock=time.monotonic):
        self.robot_id, self.clock = robot_id, clock
        self.received = None
        self.message = None

    def accept(self, message):
        if not isinstance(message, dict) or message.get('v') != 1 or message.get('robot_id') != self.robot_id:
            raise ValueError('Invalid BMS envelope')
        age = message.get('sample_age_s')
        if age is not None and (type(age) not in (int, float) or not math.isfinite(age) or age < 0):
            raise ValueError('Invalid BMS sample age')
        if not isinstance(message.get('connected'), bool) or not isinstance(message.get('stale'), bool):
            raise ValueError('Invalid BMS status')
        self.message = copy.deepcopy(message)
        self.received = self.clock()

    def snapshot(self):
        if self.message is None:
            return {
                'status': 'waiting',
                'connected': False,
                'stale': True,
                'sample_age_s': None,
                'heartbeat_age_s': None,
                'sample': None,
                'device': None,
                'error': None,
            }
        out = copy.deepcopy(self.message)
        elapsed = max(0, self.clock() - self.received)
        out['heartbeat_age_s'] = round(elapsed, 3)
        if out.get('sample_age_s') is not None:
            out['sample_age_s'] = round(out['sample_age_s'] + elapsed, 3)
        lost = elapsed > 8
        out['stale'] = bool(
            out['stale'] or lost or out.get('sample') is None or out['sample_age_s'] is None or out['sample_age_s'] > 8
        )
        if lost:
            out['connected'] = False
            out['status'] = 'publisher_lost'
        return out
