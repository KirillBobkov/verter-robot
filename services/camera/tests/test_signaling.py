"""Offline tests: no SDK device constructed and no ROS context initialized."""

import asyncio, json, time, unittest, uuid
from types import SimpleNamespace
from unittest.mock import AsyncMock
from server import CameraService, validate, encode


def body(op='status', args=None):
    return {
        'v': 1,
        'id': str(uuid.uuid4()),
        'issued_at': time.time(),
        'ttl_s': 30,
        'action': 'camera.' + op,
        'args': args or {},
    }


class TestValidation(unittest.TestCase):
    def test_reject_motion_and_expired(self):
        b = body()
        b['action'] = 'cmd_vel.set'
        with self.assertRaises(ValueError):
            validate(encode(b), 'status')
        b = body()
        b['issued_at'] -= 60
        with self.assertRaises(ValueError):
            validate(encode(b), 'status')

    def test_bad_number_and_payload(self):
        for data in [b'[]', b'null', b'x' * 65537]:
            with self.assertRaises(ValueError):
                validate(data, 'status')
        b = body()
        b['ttl_s'] = True
        with self.assertRaises(ValueError):
            validate(encode(b), 'status')


class TestService(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.capture = SimpleNamespace(
            status=lambda: {'state': 'streaming', 'streams': {}, 'stale': False}, image=lambda _: object()
        )
        self.service = CameraService(
            {'robot_id': 'test', 'max_sessions': 2, 'video_fps': 10, 'ice_servers': []}, self.capture
        )

    async def test_status_no_peers(self):
        result = await self.service.execute('status', {})
        self.assertEqual(result['sessions'], 0)

    async def test_duplicate_and_conflict(self):
        self.service.execute = AsyncMock(return_value={'session_id': 'example'})
        b = body('offer')
        m = SimpleNamespace(
            subject='test.camera.command.offer', data=encode(b), reply='_INBOX.test', respond=AsyncMock()
        )
        await asyncio.gather(self.service.dispatch(m), self.service.dispatch(m))
        self.service.execute.assert_awaited_once()
        self.assertEqual(m.respond.await_args_list[0], m.respond.await_args_list[1])
        b['args'] = {'changed': True}
        m.data = encode(b)
        await self.service.dispatch(m)
        self.assertEqual(json.loads(m.respond.await_args.args[0])['error'], 'id_conflict')

    async def test_bad_offer_leaves_no_session(self):
        with self.assertRaises(Exception):
            await self.service.execute('offer', {'type': 'offer', 'stream': 'color', 'sdp': 'm=video invalid'})
        self.assertEqual(len(self.service.peers), 0)

    async def test_limit_and_unknown_stream(self):
        self.service.peers = {'a': {}, 'b': {}}
        with self.assertRaisesRegex(ValueError, 'session_limit'):
            await self.service.execute('offer', {'type': 'offer', 'stream': 'color', 'sdp': 'm=video invalid'})
        with self.assertRaisesRegex(ValueError, 'invalid_offer'):
            await self.service.execute('offer', {'type': 'offer', 'stream': 'bogus', 'sdp': 'm=video invalid'})


if __name__ == '__main__':
    unittest.main()
