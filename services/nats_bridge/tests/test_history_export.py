import csv
import io
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, patch

from aiohttp.test_utils import TestClient, TestServer

from robot_nats.history import History
from robot_nats.history_export import COLUMNS
from robot_nats.protocol import encode
from robot_nats.web import make_app


class HistoryExportTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.history = History(Path(self.directory.name) / 'history.sqlite')
        self.now = time.time()
        sample = {
            'soc_pct': 94,
            'voltage_V': 28.735,
            'current_A': -1.2,
            'cells_V': {'1': 4.1},
            'charging_enabled': True,
            'protocol': 'unicode тест, "quoted"\nnext',
            'future_field': 7,
        }
        with self.history.connect() as db:
            db.executemany(
                'INSERT INTO observations VALUES (?, ?, ?, ?, ?)',
                [
                    (
                        self.now - 1000 + i,
                        self.now - 1001 + i,
                        str(i),
                        'discharging' if i < 251 else 'unknown',
                        json.dumps(sample) if i < 251 else None,
                    )
                    for i in range(252)
                ],
            )
        self.requests = []

        async def request(subject, raw, timeout):
            self.requests.append((subject, json.loads(raw)))
            body = json.loads(raw)
            self.assertEqual(subject, 'test_robot.battery.command.history')
            result = self.history.query(body['args'])
            return SimpleNamespace(data=encode({'ok': True, 'result': result}))

        self.nc = SimpleNamespace(is_connected=True, close=AsyncMock(), request=AsyncMock(side_effect=request))
        config = {
            'robot_id': 'test_robot',
            'nats_url': 'mock',
            'client_user': 'mock',
            'client_password': 'mock',
            'web_allowed_hosts': ['127.0.0.1'],
        }
        with patch('robot_nats.web.nats.connect', new=AsyncMock(return_value=self.nc)):
            self.client = TestClient(TestServer(await make_app(config)))
        await self.client.start_server()

    async def asyncTearDown(self):
        await self.client.close()
        self.directory.cleanup()

    async def test_csv_stream_is_paginated_exact_and_timestamped(self):
        response = await self.client.get('/api/battery/export?format=csv&period_s=3600')
        self.assertEqual(response.status, 200)
        raw = await response.read()
        self.assertTrue(raw.startswith(b'\xef\xbb\xbf'))
        rows = list(csv.DictReader(io.StringIO(raw.decode('utf-8-sig'))))
        self.assertEqual(len(rows), 252)
        self.assertEqual(len(self.requests), 2)
        self.assertEqual(rows[0]['soc_pct'], '94')
        self.assertEqual(rows[0]['charging_state'], 'discharging')
        self.assertTrue(rows[0]['received_at_utc'].endswith('Z'))
        self.assertAlmostEqual(float(rows[0]['received_at_unix_s']), self.now - 1000)
        self.assertAlmostEqual(float(rows[0]['sample_at_unix_s']), self.now - 1001)
        self.assertEqual(rows[0]['cell_1_V'], '4.1')
        self.assertEqual(rows[0]['cell_32_V'], '')
        self.assertEqual(rows[0]['charging_enabled'], '1')
        self.assertIn('\n', rows[0]['protocol'])
        self.assertEqual(json.loads(rows[0]['sample_json'])['future_field'], 7)
        self.assertEqual(rows[-1]['charging_state'], 'unknown')
        self.assertEqual(rows[-1]['voltage_V'], '')
        self.assertIn('.csv', response.headers['Content-Disposition'])
        self.assertEqual(self.requests[1][1]['args']['after'], self.now - 751)
        self.assertEqual(self.requests[0][1]['args']['to'], self.requests[1][1]['args']['to'])

    async def test_txt_and_empty_export(self):
        response = await self.client.get('/api/battery/export?format=txt&period_s=3600')
        rows = list(csv.reader(io.StringIO(await response.text()), delimiter='\t'))
        self.assertEqual(rows[0], COLUMNS)
        self.assertEqual(len(rows), 253)
        self.assertIn('.txt', response.headers['Content-Disposition'])
        response = await self.client.get('/api/battery/export?format=csv&period_s=30')
        self.assertEqual(len(list(csv.reader(io.StringIO((await response.read()).decode('utf-8-sig'))))), 1)

    async def test_bad_export_parameters_and_origin_do_not_request_data(self):
        for query in ['format=pdf', 'period_s=nan', 'period_s=foo', 'unknown=1', 'period_s=1&from=1']:
            self.assertEqual((await self.client.get('/api/battery/export?' + query)).status, 400)
        self.assertEqual((await self.client.get('/api/battery/export', headers={'Origin': 'http://other'})).status, 403)
        self.assertEqual(self.requests, [])

    def test_raw_pages_validate_cursor_and_preserve_source(self):
        args = {'raw': True, 'from': self.now - 1000, 'to': self.now, 'limit': 10}
        result = self.history.query(args)
        self.assertEqual(len(result['rows']), 10)
        self.assertEqual(result['rows'][0][4]['soc_pct'], 94)
        second = self.history.query(dict(args, after=result['next_after']))
        self.assertEqual(second['rows'][0][2], '10')
        for extra in [{'limit': 1000}, {'raw': False}, {'after': float('nan')}, {'after': self.now + 1}]:
            with self.assertRaises(ValueError):
                self.history.query(dict(args, **extra))
