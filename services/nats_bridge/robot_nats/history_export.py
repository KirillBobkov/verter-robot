"""Stream unaggregated BMS records through bounded read-only NATS pages."""

import csv
from datetime import datetime, timezone
import io
import json
import logging
import math
import time

from aiohttp import web
import nats

from .contracts import BATTERY
from .history import RETENTION
from .protocol import encode, envelope

SAMPLE_FIELDS = list(dict.fromkeys([field[0] for field in BATTERY.values()] + ['protocol', 'flags_interpretation']))
CELL_FIELDS = [f'cell_{index}_V' for index in range(1, 33)]
COLUMNS = (
    ['received_at_utc', 'received_at_unix_s', 'sample_at_utc', 'sample_at_unix_s', 'sample_id', 'charging_state']
    + SAMPLE_FIELDS
    + CELL_FIELDS
    + ['sample_json']
)


def utc_timestamp(at):
    if at is None:
        return ''
    return datetime.fromtimestamp(at, timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def export_row(row):
    received_at, sample_at, sample_id, state, sample = row
    values = [utc_timestamp(received_at), received_at, utc_timestamp(sample_at), sample_at, sample_id, state]
    sample = sample or {}
    for field in SAMPLE_FIELDS:
        value = sample.get(field)
        values.append(int(value) if isinstance(value, bool) else value)
    cells = sample.get('cells_V') or {}
    values.extend(cells.get(str(index)) for index in range(1, 33))
    values.append(json.dumps(sample, ensure_ascii=False, allow_nan=False, separators=(',', ':')) if sample else '')
    return values


def mount_history_export(app, config, nc):
    async def fetch_page(args):
        if not nc.is_connected:
            raise web.HTTPServiceUnavailable(text='NATS disconnected')
        body = envelope('battery.history', args)
        body.pop('action')
        try:
            reply = await nc.request(f'{config["robot_id"]}.battery.command.history', encode(body), timeout=15)
        except (nats.errors.TimeoutError, nats.errors.NoRespondersError):
            raise web.HTTPServiceUnavailable(text='BMS history backend unavailable')
        result = json.loads(reply.data)
        if not result.get('ok'):
            raise web.HTTPBadRequest(text=result.get('error', 'History export failed'))
        return result['result']

    async def download(request):
        if set(request.query) - {'format', 'from', 'to', 'period_s'}:
            raise web.HTTPBadRequest(text='Unknown export query parameter')
        file_format = request.query.get('format', 'csv')
        if file_format not in ('csv', 'txt'):
            raise web.HTTPBadRequest(text='format must be csv or txt')
        try:
            args = {key: float(value) for key, value in request.query.items() if key != 'format'}
        except ValueError:
            raise web.HTTPBadRequest(text='Invalid export query parameter')
        if any(not math.isfinite(value) for value in args.values()):
            raise web.HTTPBadRequest(text='Invalid export query parameter')
        if 'period_s' in args and ('from' in args or 'to' in args):
            raise web.HTTPBadRequest(text='period_s cannot be combined with from/to')
        end = args.get('to', time.time())
        start = args.get('from', end - args.get('period_s', RETENTION))
        page_args = {'from': start, 'to': end, 'raw': True, 'limit': 250}
        page = await fetch_page(page_args)
        stamp = datetime.fromtimestamp(end, timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        content_type = 'text/csv' if file_format == 'csv' else 'text/plain'
        response = web.StreamResponse(
            headers={
                'Content-Type': content_type + '; charset=utf-8',
                'Content-Disposition': f'attachment; filename="bms-{stamp}.{file_format}"',
            }
        )
        await response.prepare(request)
        buffer = io.StringIO(newline='')
        writer = csv.writer(buffer, delimiter=',' if file_format == 'csv' else '\t', lineterminator='\r\n')
        if file_format == 'csv':
            await response.write(b'\xef\xbb\xbf')
        writer.writerow(COLUMNS)
        try:
            while True:
                for row in page['rows']:
                    writer.writerow(export_row(row))
                await response.write(buffer.getvalue().encode('utf-8'))
                buffer.seek(0)
                buffer.truncate(0)
                cursor = page['next_after']
                if cursor is None:
                    break
                if cursor <= page_args.get('after', float('-inf')):
                    raise RuntimeError('History pagination did not advance')
                page_args['after'] = cursor
                page = await fetch_page(page_args)
            await response.write_eof()
        except (ConnectionError, ConnectionResetError):
            pass  # Browser cancelled the download; stop requesting pages.
        except Exception:
            logging.exception('BMS export interrupted; closing incomplete download')
            if request.transport:
                request.transport.close()
            raise
        return response

    app.router.add_get('/api/battery/export', download)
