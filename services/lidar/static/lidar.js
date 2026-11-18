'use strict';
const canvas = document.getElementById('scan');
const ctx = canvas.getContext('2d');
const connection = document.getElementById('connection');
const notice = document.getElementById('notice');
const stats = document.getElementById('stats');
const radius = document.getElementById('radius');
let scan = null,
    model = null,
    status = null,
    received = 0,
    statusReceived = 0,
    socket = null,
    closing = false;

function connect() {
    const url = new URL('/api/lidar/ws', window.location.href);
    url.protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    socket = new WebSocket(url);
    socket.binaryType = 'arraybuffer';
    socket.onopen = () => {
        connection.textContent = 'Поток подключён';
    };
    socket.onmessage = (event) => {
        try {
            const value = JSON.parse(
                typeof event.data === 'string' ? event.data : new TextDecoder().decode(event.data)
            );
            if (value.v !== 1) return;
            if (value.type === 'scan' && value.target_frame === 'base_footprint' && Array.isArray(value.ranges)) {
                scan = value;
                received = performance.now();
            } else if (value.type === 'model' && value.frame_id === 'base_footprint') {
                model = value;
            } else if (value.type === 'status') {
                status = value;
                statusReceived = performance.now();
            }
        } catch {
            connection.textContent = 'Некорректный кадр';
        }
    };
    socket.onclose = () => {
        connection.textContent = 'Связь потеряна · повтор подключения…';
        scan = null;
        status = null;
        if (!closing) setTimeout(connect, 1500);
    };
    socket.onerror = () => socket.close();
}

function draw() {
    const { width, height } = canvas.getBoundingClientRect();
    const dpr = Math.min(devicePixelRatio || 1, 2);
    if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
        canvas.width = Math.round(width * dpr);
        canvas.height = Math.round(height * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, width, height);
    const scale = (Math.min(width, height) / 2 - 35) / Number(radius.value);
    const project = (x, y) => [width / 2 - y * scale, height / 2 - x * scale];
    const line = (a, b) => {
        ctx.beginPath();
        ctx.moveTo(...a);
        ctx.lineTo(...b);
        ctx.stroke();
    };
    const spacing = Number(radius.value) <= 3 ? 0.5 : Number(radius.value) <= 10 ? 1 : 5;
    ctx.strokeStyle = '#263547';
    ctx.lineWidth = 1;
    for (let m = -Math.ceil(width / scale); m <= Math.ceil(width / scale); m += spacing) {
        line(project(-height / scale, m), project(height / scale, m));
        line(project(m, -width / scale), project(m, width / scale));
    }
    ctx.strokeStyle = '#496078';
    line([width / 2, 0], [width / 2, height]);
    line([0, height / 2], [width, height / 2]);
    ctx.fillStyle = '#9cb5cd';
    ctx.font = '12px system-ui';
    ctx.fillText(`Сетка ${spacing} м`, 15, height - 15);
    const age = scan ? (performance.now() - received) / 1000 + (scan.delivery_age_s || 0) : Infinity;
    const stateFresh = status && (performance.now() - statusReceived) / 1000 + (status.delivery_age_s || 0) < 3;
    const fresh = scan && age <= 1.5 && stateFresh && !status.stale && socket?.readyState === WebSocket.OPEN;
    let count = 0;
    if (fresh) {
        const [qx, qy, qz, qw] = scan.transform.rotation_xyzw;
        const [tx, ty] = scan.transform.translation;
        const a = 1 - 2 * (qy * qy + qz * qz),
            b = 2 * (qx * qy - qz * qw);
        const c = 2 * (qx * qy + qz * qw),
            d = 1 - 2 * (qx * qx + qz * qz);
        ctx.fillStyle = '#55e6c1';
        scan.ranges.forEach((distance, i) => {
            if (distance === null || !Number.isFinite(distance)) return;
            const angle = scan.angle_min + i * scan.angle_increment;
            const x = distance * Math.cos(angle),
                y = distance * Math.sin(angle);
            const [px, py] = project(tx + a * x + b * y, ty + c * x + d * y);
            ctx.fillRect(px - 1.5, py - 1.5, 3, 3);
            count++;
        });
        const [lx, ly] = project(tx, ty);
        ctx.strokeStyle = '#ffd581';
        ctx.beginPath();
        ctx.arc(lx, ly, 5, 0, Math.PI * 2);
        ctx.stroke();
    }
    if (model) {
        for (const polygon of model.polygons) {
            ctx.beginPath();
            polygon.points.forEach(([x, y], i) => {
                const p = project(x, y);
                i ? ctx.lineTo(...p) : ctx.moveTo(...p);
            });
            ctx.closePath();
            ctx.fillStyle = polygon.kind === 'body' ? '#325f8c99' : '#0b101c';
            ctx.strokeStyle = '#87b9f0';
            ctx.lineWidth = 1.5;
            ctx.fill();
            ctx.stroke();
        }
    }
    const [ax, ay] = project(0.36, 0);
    ctx.fillStyle = '#e7f1ff';
    ctx.beginPath();
    ctx.moveTo(ax, ay - 8);
    ctx.lineTo(ax - 5, ay + 3);
    ctx.lineTo(ax + 5, ay + 3);
    ctx.closePath();
    ctx.fill();
    ctx.fillText('ВПЕРЁД', ax + 12, ay);
    notice.textContent = fresh
        ? model
            ? ''
            : 'Ожидание модели робота…'
        : stateFresh && status.error
          ? `Нет свежего скана: ${status.error}`
          : 'Нет свежих данных лидара';
    stats.textContent = fresh
        ? `${count} точек · ${Math.round(age * 1000)} мс · ${scan.frame_id}`
        : 'Точки скрыты до получения свежего скана';
}
window.addEventListener('pagehide', () => {
    closing = true;
    socket?.close();
});
setInterval(draw, 100);
connect();
