"""Read-only HTTP camera acceptance. Use --url for an SSH forwarded port."""

import argparse, json, time, urllib.request, urllib.error

p = argparse.ArgumentParser()
p.add_argument('--url', default='http://127.0.0.1:8089')
a = p.parse_args()
opened = []
results = {}
try:
    for stream in ('color', 'depth'):
        r = urllib.request.urlopen(a.url + '/api/camera/mjpeg?stream=' + stream, timeout=10)
        opened.append(r)
        assert 'multipart/x-mixed-replace' in r.headers['Content-Type']
        data = r.read(2048)
        assert b'Content-Type: image/jpeg' in data and b'\xff\xd8' in data
        results[stream] = 'JPEG part received'
    for suffix, headers, expected in [
        ('/api/camera/mjpeg', {}, 429),
        ('/api/camera/mjpeg?stream=invalid', {}, 400),
        ('/api/camera/mjpeg', {'Origin': 'http://foreign.invalid'}, 403),
    ]:
        try:
            urllib.request.urlopen(urllib.request.Request(a.url + suffix, headers=headers), timeout=10)
            raise AssertionError('Expected rejection')
        except urllib.error.HTTPError as e:
            assert e.code == expected
            (results.update({str(expected): True}))
finally:
    for r in opened:
        r.close()
print(json.dumps(results, indent=2))
