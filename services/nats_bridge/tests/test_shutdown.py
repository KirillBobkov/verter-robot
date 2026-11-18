import asyncio
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock

from robot_nats.service import Service


class ShutdownTests(unittest.IsolatedAsyncioTestCase):
    async def test_publication_exits_even_if_transport_swallows_cancellation(self):
        started = asyncio.Event()

        async def put(key, value):
            started.set()
            try:
                await asyncio.sleep(100)
            except asyncio.CancelledError:
                return  # Simulate a transport completing instead of propagating cancellation.

        service = Service.__new__(Service)
        service.closing = False
        service.kv = SimpleNamespace(put=put)
        service.nc = SimpleNamespace(is_connected=True, publish=AsyncMock())
        service.snapshot = lambda: {}
        service.metric_snapshot = lambda state: {'service.one': {}, 'service.two': {}}
        service.config = {'robot_id': 'test_robot'}
        task = asyncio.create_task(service.publish_loop())
        await started.wait()
        service.closing = True
        task.cancel()
        await asyncio.wait_for(task, timeout=1)
