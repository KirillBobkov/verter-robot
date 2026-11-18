import copy
from pathlib import Path
import tempfile
import unittest

from robot_nats.modes import Modes


class Telemetry:
    def __init__(self, nodes=None):
        self.nodes = nodes or ['esp32_ctrl']

    def snapshot(self):
        return {
            'sources': {'graph': {'stale': False, 'value': {'nodes': self.nodes, 'cmd_vel_publishers': ['esp32_ctrl']}}}
        }


class FakeSupervisor:
    def __init__(self):
        self.calls = []
        self.fail = False
        self.result = {'ok': True, 'mode': 'idle', 'map': None, 'groups': {'hardware': 100}, 'last_exit': {}}

    def call(self, request, timeout):
        self.calls.append(copy.deepcopy(request))
        if self.fail:
            raise OSError('supervisor offline')
        if request['command'] == 'mode':
            self.result['mode'] = request['mode']
            self.result['map'] = request.get('map')
            if request['mode'] != 'idle':
                self.result['groups']['mode'] = 123
            else:
                self.result['groups'].pop('mode', None)
        return copy.deepcopy(self.result)


class ModeTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.client = FakeSupervisor()
        self.config = {'allow_mode_changes': True, 'maps': {}}
        self.modes = Modes(self.config, Telemetry(), self.client)
        await self.modes.start()

    async def asyncTearDown(self):
        await self.modes.stop()

    async def test_supervisor_state_is_shared_and_shutdown_never_stops_mode(self):
        self.client.result['mode'] = 'mapping'  # A mode started from CLI.
        self.client.result['groups']['mode'] = 123
        await self.modes.refresh()
        self.assertEqual(self.modes.state()['actual'], 'mapping')
        self.assertEqual(self.modes.state()['owner'], 'supervisor')
        before = list(self.client.calls)
        await self.modes.stop()
        self.assertEqual(self.client.calls, before)
        self.assertEqual(self.client.result['mode'], 'mapping')

    async def test_mapping_accepted_then_ready_despite_esp_publisher(self):
        state = await self.modes.switch({'mode': 'mapping'}, request_id='operator-request')
        self.assertEqual(state['operation']['status'], 'running')
        self.assertEqual(state['desired'], 'mapping')
        await self.modes.task
        state = self.modes.state()
        self.assertEqual(state['actual'], 'mapping')
        self.assertEqual(state['operation']['status'], 'succeeded')
        self.assertEqual(state['operation']['id'], 'operator-request')
        self.assertEqual([c['command'] for c in self.client.calls].count('mode'), 1)
        self.assertFalse(state['localization_verified'])

    async def test_duplicate_target_and_discard_validation(self):
        await self.modes.switch({'mode': 'mapping'})
        await self.modes.task
        count = len(self.client.calls)
        await self.modes.switch({'mode': 'mapping'})
        self.assertEqual([c['command'] for c in self.client.calls[count:]], ['status'])
        with self.assertRaises(ValueError):
            await self.modes.switch({'mode': 'idle'})
        await self.modes.switch({'mode': 'idle', 'discard': True})
        await self.modes.task
        self.assertEqual(self.modes.state()['actual'], 'idle')

    async def test_disabled_offline_and_conflicting_nodes_do_not_send_mode(self):
        self.config['allow_mode_changes'] = False
        with self.assertRaises(ValueError):
            await self.modes.switch({'mode': 'mapping'})
        self.config['allow_mode_changes'] = True
        self.client.fail = True
        with self.assertRaises(ValueError):
            await self.modes.switch({'mode': 'mapping'})
        self.assertTrue(self.modes.state()['stale'])
        self.client.fail = False
        self.modes.telemetry.nodes = ['slam_toolbox']
        with self.assertRaises(ValueError):
            await self.modes.switch({'mode': 'mapping'})
        self.assertFalse(any(c['command'] == 'mode' for c in self.client.calls))

    async def test_navigation_allowlist_and_no_localization_alias(self):
        for args in [
            {'mode': 'localization'},
            {'mode': 'navigation', 'map': '/tmp/injected'},
            {'mode': 'mapping', 'map': 'x'},
            {'mode': 'idle', 'discard': 'yes'},
        ]:
            with self.assertRaises(ValueError):
                await self.modes.switch(args)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.yaml'
            path.write_text('test fixture only')
            self.config['maps']['test'] = str(path)
            await self.modes.switch({'mode': 'navigation', 'map': 'test'})
            await self.modes.task
            self.assertEqual(self.modes.state()['map'], 'test')
            self.assertEqual(self.client.calls[-1]['map'], str(path))

    async def test_failure_is_reported_and_no_stop_or_retry_is_sent(self):
        state = await self.modes.switch({'mode': 'mapping'})
        self.client.fail = True
        await self.modes.task
        self.assertEqual(self.modes.state()['operation']['status'], 'failed')
        await self.modes.refresh()
        self.assertEqual(sum(c['command'] == 'mode' for c in self.client.calls), 1)
