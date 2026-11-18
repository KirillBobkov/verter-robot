"""Volatile manual velocity lease. No ROS imports; the sink is injected by the ROS thread."""

import math
import threading
import time
import uuid


class Motion:
    MAX_LINEAR = 0.5
    MAX_ANGULAR = 0.5
    LEASE_S = 1.0

    def __init__(self, enabled=False, clock=time.monotonic):
        self.enabled, self.clock = enabled, clock
        self.lock = threading.RLock()
        self.ready = False
        self.owner = None
        self.sequence = -1
        self.linear = self.angular = 0.0
        self.deadline = 0.0
        self.next_publish = 0.0
        self.pending_stop = False
        self.retired = {}
        self.reason = 'idle'
        self.accept_after = time.time()

    def _stop(self, reason):
        if self.owner:
            self.retired[self.owner] = self.clock() + 60
        self.owner = None
        self.linear = self.angular = 0.0
        self.deadline = 0.0
        self.pending_stop = True
        self.reason = reason
        self.accept_after = time.time()

    def snapshot(self):
        with self.lock:
            return {
                'enabled': self.enabled,
                'ready': self.ready,
                'active': self.owner is not None,
                'session_id': self.owner,
                'seq': self.sequence,
                'linear_x': self.linear,
                'angular_z': self.angular,
                'remaining_s': max(0, self.deadline - self.clock()),
                'reason': self.reason,
                'max_linear': self.MAX_LINEAR,
                'max_angular': self.MAX_ANGULAR,
                'lease_s': self.LEASE_S,
            }

    def command(self, operation, args, issued_at=None):
        with self.lock:
            now = self.clock()
            self.retired = {key: expiry for key, expiry in self.retired.items() if expiry > now}
            if operation == 'status':
                if args:
                    raise ValueError('unexpected_args')
                return self.snapshot()
            if not self.enabled:
                raise ValueError('manual_motion_disabled')
            if not self.ready:
                raise ValueError('ros_publisher_not_ready')
            if operation == 'stop':
                if args:
                    raise ValueError('unexpected_args')
                self._stop('operator_stop')
                return self.snapshot()
            if issued_at is not None and issued_at <= self.accept_after:
                raise ValueError('command_precedes_stop_or_start')
            fields = {'session_id', 'seq'} | ({'linear_x', 'angular_z'} if operation == 'set' else set())
            if operation not in ('set', 'keepalive') or set(args) != fields:
                raise ValueError('invalid_motion_args')
            session, sequence = args['session_id'], args['seq']
            uuid.UUID(session)
            if type(sequence) is not int or sequence < 0:
                raise ValueError('invalid_sequence')
            if session in self.retired:
                raise ValueError('session_closed')
            if self.owner and now >= self.deadline:
                self._stop('lease_expired')
                raise ValueError('lease_expired')
            if self.owner and self.owner != session:
                raise ValueError('manual_control_busy')
            if self.owner and sequence <= self.sequence:
                raise ValueError('stale_sequence')
            if operation == 'keepalive' and self.owner != session:
                raise ValueError('no_active_session')
            if operation == 'set':
                linear, angular = args['linear_x'], args['angular_z']
                if any(type(v) not in (int, float) or not math.isfinite(v) for v in (linear, angular)):
                    raise ValueError('invalid_velocity')
                if abs(linear) > self.MAX_LINEAR or abs(angular) > self.MAX_ANGULAR:
                    raise ValueError('velocity_limit')
                self.linear, self.angular = linear, angular
                if linear == 0 and angular == 0:
                    self.retired[session] = now + 60
                    self._stop('zero_target')
                    return self.snapshot()
            self.owner, self.sequence = session, sequence
            self.deadline = now + self.LEASE_S
            self.reason = 'manual'
            return self.snapshot()

    def tick(self, sink, connected=True):
        with self.lock:
            now = self.clock()
            if self.owner and (not connected or now >= self.deadline):
                self._stop('lease_expired' if connected else 'nats_disconnected')
            if self.pending_stop:
                sink(0.0, 0.0)
                self.pending_stop = False
                self.next_publish = now + 0.05
            elif self.owner and now >= self.next_publish:
                sink(self.linear, self.angular)
                self.next_publish = now + 0.05

    def shutdown(self, sink):
        with self.lock:
            if self.owner or self.pending_stop:
                self._stop('shutdown')
                self.tick(sink)
            self.ready = False


def validate_request(raw, now=None):
    import json

    if len(raw) > 4096:
        raise ValueError('request_too_large')
    body = json.loads(raw)
    if not isinstance(body, dict) or set(body) != {'v', 'id', 'issued_at', 'ttl_s', 'args'}:
        raise ValueError('invalid_envelope')
    if type(body['v']) is not int or body['v'] != 1:
        raise ValueError('unsupported_version')
    uuid.UUID(body['id'])
    now = time.time() if now is None else now
    issued, ttl = body['issued_at'], body['ttl_s']
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in (issued, ttl)):
        raise ValueError('invalid_time')
    if not 0 < ttl <= 1 or issued > now + 0.5 or now > issued + ttl:
        raise ValueError('expired_or_invalid_time')
    if not isinstance(body['args'], dict):
        raise ValueError('invalid_args')
    return body
