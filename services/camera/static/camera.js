'use strict';
let httpActive = false;
let pc = null,
    session = null,
    generation = 0,
    lastFrames = 0,
    lastProgress = 0;
const $ = (id) => document.getElementById(id);
const envelope = (op, args = {}) => ({
    v: 1,
    id: crypto.randomUUID
        ? crypto.randomUUID()
        : '10000000-1000-4000-8000-100000000000'.replace(/[018]/g, (c) =>
              (c ^ (crypto.getRandomValues(new Uint8Array(1))[0] & (15 >> (c / 4)))).toString(16)
          ),
    issued_at: Date.now() / 1000,
    ttl_s: 30,
    action: 'camera.' + op,
    args
});
async function api(op, args) {
    const r = await fetch(
        '/api/camera/' + op,
        args === undefined
            ? {}
            : {
                  method: 'POST',
                  headers: { 'Content-Type': 'application/json' },
                  body: JSON.stringify(envelope(op, args))
              }
    );
    if (!r.ok) throw Error('HTTP ' + r.status);
    const result = await r.json();
    if (!result.ok) throw Error(result.error);
    return result.result;
}
function closeSession(id) {
    if (id)
        fetch('/api/camera/close', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(envelope('close', { session_id: id })),
            keepalive: true
        }).catch(() => {});
}
function disconnect() {
    generation++;
    httpActive = false;
    $('mjpeg').onload = null;
    $('mjpeg').onerror = null;
    $('mjpeg').removeAttribute('src');
    $('mjpeg').replaceWith($('mjpeg').cloneNode(false));
    $('mjpeg').hidden = true;
    $('video').hidden = false;
    $('transport').disabled = false;
    $('frames').textContent = 'Кадры ещё не получены';
    if (pc) pc.close();
    pc = null;
    closeSession(session);
    session = null;
    $('video').srcObject = null;
    $('connect').disabled = false;
    $('disconnect').disabled = true;
    $('stream').disabled = false;
    $('connection').textContent = 'Видео отключено';
}
async function connect() {
    disconnect();
    const gen = generation;
    $('connect').disabled = true;
    $('disconnect').disabled = false;
    $('stream').disabled = true;
    $('connection').textContent = 'Соединение…';
    $('transport').disabled = true;
    if ($('transport').value === 'http') {
        httpActive = true;
        $('video').hidden = true;
        $('mjpeg').hidden = false;
        $('mjpeg').onload = () => {
            if (gen === generation) {
                $('connection').textContent = 'HTTP / SSH: изображение получено';
                $('frames').textContent = 'HTTP MJPEG · до 5 кадров/с';
            }
        };
        $('mjpeg').onerror = () => {
            if (gen === generation) {
                disconnect();
                $('connection').textContent = 'HTTP: поток недоступен или завершён. Подключите заново.';
            }
        };
        $('mjpeg').src = '/api/camera/mjpeg?stream=' + encodeURIComponent($('stream').value);
        $('connection').textContent = 'HTTP / SSH: подключение…';
        return;
    }
    try {
        const ice = await api('ice');
        if (gen !== generation) return;
        const peer = new RTCPeerConnection(ice);
        pc = peer;
        peer.addTransceiver('video', { direction: 'recvonly' });
        peer.ontrack = (e) => {
            $('video').srcObject = new MediaStream([e.track]);
            $('video')
                .play()
                .catch(() => {});
        };
        peer.onconnectionstatechange = () => {
            if (gen !== generation) return;
            $('connection').textContent = 'WebRTC: ' + peer.connectionState;
            if (['failed', 'disconnected'].includes(peer.connectionState)) {
                disconnect();
                $('connection').textContent = 'Связь потеряна — подключите видео заново';
            }
        };
        await peer.setLocalDescription(await peer.createOffer());
        await new Promise((resolve, reject) => {
            if (peer.iceGatheringState === 'complete') return resolve();
            const timeout = setTimeout(() => {
                peer.removeEventListener('icegatheringstatechange', check);
                reject(Error('ICE timeout'));
            }, 15000);
            function check() {
                if (peer.iceGatheringState === 'complete') {
                    clearTimeout(timeout);
                    peer.removeEventListener('icegatheringstatechange', check);
                    resolve();
                }
            }
            peer.addEventListener('icegatheringstatechange', check);
        });
        if (gen !== generation) return;
        const answer = await api('offer', { type: 'offer', sdp: peer.localDescription.sdp, stream: $('stream').value });
        if (gen !== generation) {
            closeSession(answer.session_id);
            return;
        }
        session = answer.session_id;
        await peer.setRemoteDescription({ type: answer.type, sdp: answer.sdp });
        lastFrames = 0;
        lastProgress = Date.now();
    } catch (e) {
        if (gen === generation) {
            disconnect();
            $('connection').textContent = 'Ошибка: ' + e.message;
        }
    }
}
$('connect').addEventListener('click', connect);
$('disconnect').addEventListener('click', disconnect);
window.addEventListener('pagehide', disconnect);
let polling = false;
async function status() {
    if (polling) return;
    polling = true;
    try {
        const current = await api('status');
        $('status').textContent = JSON.stringify(current, null, 2);
        if (httpActive) {
            if (current.stale) {
                disconnect();
                $('connection').textContent = 'Нет свежих кадров камеры';
            } else {
                $('frames').textContent = 'HTTP MJPEG · до 5 кадров/с · данные камеры свежие';
            }
        }
        if (pc) {
            const stats = await pc.getStats();
            for (const s of stats.values())
                if (s.type === 'inbound-rtp' && s.kind === 'video') {
                    const count = s.framesDecoded || 0;
                    if (count > lastFrames) {
                        lastProgress = Date.now();
                        lastFrames = count;
                    }
                    $('frames').textContent =
                        `Получено кадров: ${count} · ${s.frameWidth || '—'}×${s.frameHeight || '—'} · ${Date.now() - lastProgress > 3000 ? 'Нет свежих кадров' : 'Изображение обновляется'}`;
                }
        }
    } catch (e) {
        $('status').textContent = 'Нет связи: ' + e.message;
    } finally {
        polling = false;
    }
}
if (['127.0.0.1', 'localhost', '[::1]'].includes(location.hostname)) $('transport').value = 'http';
status();
setInterval(status, 2000);
