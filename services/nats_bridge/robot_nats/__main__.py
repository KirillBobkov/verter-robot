import argparse
import asyncio
import json
import logging
from pathlib import Path
import signal
from aiohttp import web
from .service import Service
from .web import make_app


async def main(config):
    service = Service(config)
    runner = None
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)
    try:
        await service.start()
        if config.get('web_enabled', True):
            runner = web.AppRunner(await make_app(config))
            await runner.setup()
            await web.TCPSite(runner, config.get('web_host', '127.0.0.1'), config['web_port']).start()
        logging.info(
            'NATS API ready; debug HTTP=%s on %s:%s; mode execution=%s',
            config.get('web_enabled', True),
            config.get('web_host', '127.0.0.1'),
            config['web_port'],
            config['allow_mode_changes'],
        )
        await stop.wait()
    finally:
        if runner:
            await runner.cleanup()
        await service.close()


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--config', required=True)
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO)
    asyncio.run(main(json.loads(Path(args.config).read_text())))
