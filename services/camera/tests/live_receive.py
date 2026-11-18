"""Read-only camera acceptance: NATS signaling -> decoded WebRTC frames."""

import asyncio, json, time, uuid
from pathlib import Path
import nats, cv2
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration


async def main():
    root = Path(__file__).resolve().parents[3]
    cfg = json.loads((root / 'config/nats/camera.json').read_text())
    out = root / 'data/diagnostics/camera'
    out.mkdir(parents=True, exist_ok=True)
    nc = await nats.connect(cfg['nats_url'], user=cfg['client_user'], password=cfg['client_password'])

    async def request(op, args={}):
        body = {
            'v': 1,
            'id': str(uuid.uuid4()),
            'issued_at': time.time(),
            'ttl_s': 30,
            'action': 'camera.' + op,
            'args': args,
        }
        result = json.loads(
            (await nc.request(f'{cfg["robot_id"]}.camera.command.{op}', json.dumps(body).encode(), timeout=20)).data
        )
        assert result['ok'], result
        return result['result']

    results = {}
    try:
        for stream in ('color', 'depth'):
            pc = RTCPeerConnection(RTCConfiguration(iceServers=[]))
            pc.addTransceiver('video', direction='recvonly')
            future = asyncio.get_running_loop().create_future()
            sid = None

            @pc.on('track')
            def track(t):
                if not future.done():
                    future.set_result(t)

            try:
                await pc.setLocalDescription(await pc.createOffer())
                offer = await request('offer', {'type': 'offer', 'sdp': pc.localDescription.sdp, 'stream': stream})
                sid = offer['session_id']
                await pc.setRemoteDescription(RTCSessionDescription(sdp=offer['sdp'], type=offer['type']))
                video = await asyncio.wait_for(future, 15)
                stamps = []
                start = time.monotonic()
                for i in range(25):
                    frame = await asyncio.wait_for(video.recv(), 10)
                    stamps.append(frame.pts)
                cv2.imwrite(str(out / (stream + '.jpg')), frame.to_ndarray(format='bgr24'))
                results[stream] = {
                    'frames': 25,
                    'width': frame.width,
                    'height': frame.height,
                    'seconds': time.monotonic() - start,
                    'pts_increasing': all(a < b for a, b in zip(stamps, stamps[1:])),
                }
                assert results[stream]['pts_increasing']
            finally:
                await pc.close()
                if sid:
                    await request('close', {'session_id': sid})
        results['status'] = await request('status')
        assert not results['status']['stale']
        assert results['status']['sessions'] == 0
        (out / 'webrtc_acceptance.json').write_text(json.dumps(results, indent=2))
        print(json.dumps(results, indent=2))
    finally:
        await nc.close()


asyncio.run(main())
