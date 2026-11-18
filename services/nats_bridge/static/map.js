/* Map viewer and saved map management. Motion controls are handled by the shared drive widget. */
(() => {
    const el = id => document.getElementById(id);
    const projection = new ol.proj.Projection({code: 'VERTER_MAP', units: 'm'});
    let image = null, displayed = null, latest = null, lastPose = null, model = null;
    let metaBusy = false, poseBusy = false, lastPoseReply = 0, scanBusy = false, lastScan = null, scanReply = 0, serverClock = null, saving = false, saveError = null, saveRequestId = null;
    let metadataReply = null, notifiedSave = null, observedSave = false;
    let preview = null;
    let savedMaps = [], libraryLoading = false, libraryPending = false, deleting = false;
    let catalogScope = null;
    function visibleMetadata() {
        return preview ? preview.metadata : displayed;
    }
    const raster = new ol.source.ImageCanvas({
        projection, ratio: 1, interpolate: false,
        canvasFunction(extent, resolution, pixelRatio, size) {
            const canvas = document.createElement('canvas');
            canvas.width = size[0]; canvas.height = size[1];
            const currentImage = preview ? preview.image : image;
            const metadata = visibleMetadata();
            if (!currentImage || !metadata) return canvas;
            const ctx = canvas.getContext('2d');
            const sx = size[0] / (extent[2] - extent[0]), sy = size[1] / (extent[3] - extent[1]);
            ctx.imageSmoothingEnabled = false;
            ctx.setTransform(sx, 0, 0, -sy, -extent[0] * sx, extent[3] * sy);
            ctx.translate(metadata.origin.x, metadata.origin.y);
            ctx.rotate(metadata.origin.yaw);
            ctx.translate(0, metadata.height * metadata.resolution);
            ctx.scale(metadata.resolution, -metadata.resolution);
            ctx.drawImage(currentImage, 0, 0);
            return canvas;
        }
    });
    const robots = new ol.source.Vector({wrapX: false});
    const laser = new ol.source.Vector({wrapX: false});
    const view = new ol.View({projection, center: [0, 0], resolution: .02, minResolution: .001, maxResolution: 10});
    const map = new ol.Map({target: 'ol-map', view, layers: [
        new ol.layer.Image({source: raster}),
        new ol.layer.Vector({source: laser, style: new ol.style.Style({image: new ol.style.Circle({radius: 2, fill: new ol.style.Fill({color: '#00b889'})})})}),
        new ol.layer.Vector({source: robots, style: feature => {
            if (feature.get('kind') === 'center') return new ol.style.Style({image: new ol.style.Circle({
                radius: 4, fill: new ol.style.Fill({color: '#59d7ff'}), stroke: new ol.style.Stroke({color: '#0b1824', width: 1})
            })});
            return new ol.style.Style({
                fill: new ol.style.Fill({color: feature.get('kind') === 'body' ? '#4798e466' : '#174e8d99'}),
                stroke: new ol.style.Stroke({color: feature.get('kind') === 'arrow' ? '#45caff' : '#176ec1', width: 2})
            });
        }})
    ]});
    function copy(value) {
        return value == null ? null : JSON.parse(JSON.stringify(value));
    }
    function mapState() {
        const available = !preview && !!displayed && !!latest?.available && displayed.map_id === latest.map_id &&
            displayed.revision === latest.revision;
        const stale = !available || !!latest.stale || document.hidden || metadataReply === null ||
            performance.now() - metadataReply > 3500;
        return {available, stale};
    }
    function poseValid() {
        const state = mapState();
        const age = Math.max(0, lastPose?.age_s || 0) + (performance.now() - lastPoseReply) / 1000;
        return state.available && !state.stale && lastPose?.valid === true &&
            lastPose.map_id === displayed.map_id && lastPose.revision === displayed.revision &&
            Number.isFinite(age) && age <= 1.5 &&
            ['x', 'y', 'yaw'].every(key => Number.isFinite(lastPose[key]));
    }
    function scanValid() {
        const state = mapState();
        const age = Math.max(0, lastScan?.age_s || 0) + (performance.now() - scanReply) / 1000;
        return state.available && !state.stale && lastScan?.valid === true &&
            lastScan.map_id === displayed.map_id && lastScan.revision === displayed.revision &&
            Number.isFinite(age) && age <= 1.5 && Array.isArray(lastScan.points) && lastScan.points.length <= 1800 &&
            lastScan.points.every(point => Array.isArray(point) && point.length === 2 && point.every(Number.isFinite));
    }
    function getState() {
        const state = mapState();
        return {
            ...state, mode: latest?.mode || 'unknown', reason: latest?.reason || latest?.error || null,
            loading: metaBusy, metadata: copy(latest), displayed: copy(visibleMetadata()), preview: preview?.name || null,
            pose: {...copy(lastPose), valid: poseValid()},
            scan: {valid: scanValid(), reason: lastScan?.reason || null,
                age_s: lastScan?.age_s, stamp: lastScan?.stamp,
                points_count: lastScan?.points?.length || 0, raw_count: lastScan?.raw_count}
        };
    }
    function notifyState() {
        if (typeof window.dispatchEvent === 'function' && typeof CustomEvent === 'function')
            window.dispatchEvent(new CustomEvent('verter:map-state', {detail: getState()}));
    }
    window.VerterMap = {map, view, getMetadata: () => preview ? null : displayed, getState};
    map.addControl(new ol.control.ScaleLine({units: 'metric'}));
    map.on('pointermove', event => {
        el('cursor-info').textContent = `Курсор: x ${event.coordinate[0].toFixed(2)} м · y ${event.coordinate[1].toFixed(2)} м`;
    });
    map.on('pointerdrag', () => { if (!preview) el('follow').checked = false; });

    async function api(args, signal) {
        const sent = performance.now();
        const response = await fetch('/api/map?' + new URLSearchParams(args), {signal: signal || AbortSignal.timeout(6000)});
        if (!response.ok) throw Error('HTTP ' + response.status);
        const body = await response.json();
        if (!body.ok) throw Error(body.error || 'Нет ответа map API');
        if (Number.isFinite(body.at) && performance.now() - sent < 1000) serverClock = {at: body.at, midpoint: (sent + performance.now()) / 2};
        return body.result;
    }
    function validate(metadata) {
        if (metadata.frame_id !== 'map' || metadata.encoding !== 'zlib-int8-row-major' ||
            !Number.isInteger(metadata.width) || !Number.isInteger(metadata.height) || metadata.width <= 0 || metadata.height <= 0 ||
            metadata.width * metadata.height > 4194304 || metadata.decoded_bytes !== metadata.width * metadata.height ||
            !Number.isFinite(metadata.resolution) || metadata.resolution <= 0 ||
            !metadata.origin || !['x', 'y', 'yaw'].every(key => Number.isFinite(metadata.origin[key])) ||
            !Number.isInteger(metadata.compressed_bytes) || metadata.compressed_bytes < 1 || metadata.compressed_bytes > 4260000 ||
            !Number.isInteger(metadata.chunk_bytes) || metadata.chunk_bytes < 1 || metadata.chunk_bytes > 65536 ||
            metadata.chunks !== Math.ceil(metadata.compressed_bytes / metadata.chunk_bytes) ||
            !Number.isInteger(metadata.revision) || metadata.revision < 1 || typeof metadata.map_id !== 'string' ||
            !/^[0-9a-f]{64}$/.test(metadata.sha256) || !/^[0-9a-f]{64}$/.test(metadata.decoded_sha256))
            throw Error('Некорректные параметры карты');
    }
    function unpack(packed, expected) {
        const parts = []; let length = 0;
        const inflater = new pako.Inflate({chunkSize: 65536});
        inflater.onData = chunk => {
            length += chunk.length;
            if (length > expected) throw Error('Превышен размер карты');
            parts.push(chunk);
        };
        inflater.push(packed, true);
        if (inflater.err || length !== expected) throw Error('Не удалось распаковать карту');
        const bytes = new Uint8Array(length); let offset = 0;
        for (const part of parts) { bytes.set(part, offset); offset += part.length; }
        return bytes;
    }
    async function downloadRaster(metadata, request, signal, progress) {
        validate(metadata);
        const packed = new Uint8Array(metadata.compressed_bytes);
        for (let index = 0; index < metadata.chunks; index++) {
            progress(index + 1, metadata.chunks);
            const chunk = await api({...request, map_id: metadata.map_id, revision: metadata.revision, index}, signal);
            if (chunk.map_id !== metadata.map_id || chunk.revision !== metadata.revision || chunk.index !== index || chunk.chunks !== metadata.chunks)
                throw Error('Части карты относятся к разным версиям');
            const bytes = Uint8Array.from(atob(chunk.data_b64), value => value.charCodeAt(0));
            const offset = index * metadata.chunk_bytes;
            if (bytes.length !== Math.min(metadata.chunk_bytes, metadata.compressed_bytes - offset)) throw Error('Неверный размер части карты');
            packed.set(bytes, offset);
        }
        if (sha256(packed) !== metadata.sha256) throw Error('Не совпала контрольная сумма карты');
        const raw = unpack(packed, metadata.decoded_bytes);
        if (sha256(raw) !== metadata.decoded_sha256) throw Error('Не совпала контрольная сумма клеток');
        const canvas = document.createElement('canvas'); canvas.width = metadata.width; canvas.height = metadata.height;
        const ctx = canvas.getContext('2d'), pixels = ctx.createImageData(metadata.width, metadata.height);
        for (let y = 0; y < metadata.height; y++) for (let x = 0; x < metadata.width; x++) {
            const value = raw[y * metadata.width + x];
            if (value > 100 && value !== 255) throw Error('Некорректная клетка карты');
            const at = ((metadata.height - 1 - y) * metadata.width + x) * 4;
            const gray = value === 255 ? 125 : Math.round(245 - value * 2.2);
            pixels.data[at] = pixels.data[at + 1] = pixels.data[at + 2] = gray; pixels.data[at + 3] = 255;
        }
        ctx.putImageData(pixels, 0, 0);
        return canvas;
    }
    async function load(metadata) {
        const controller = new AbortController(), timeout = setTimeout(() => controller.abort(), 60000);
        try {
            const canvas = await downloadRaster(metadata, {part: 'chunk'}, controller.signal, (part, total) => {
                if (!preview) el('connection').textContent = `Получение карты: ${part}/${total}`;
            });
            const newSession = !displayed || displayed.map_id !== metadata.map_id;
            image = canvas;
            displayed = metadata;
            if (!preview) {
                raster.changed();
                if (newSession) {
                    if (metadata.mode === 'mapping') view.setRotation(0);
                    fit();
                }
            }
        } finally { clearTimeout(timeout); }
    }
    function world(x, y, origin) {
        return [origin.x + x * Math.cos(origin.yaw) - y * Math.sin(origin.yaw),
            origin.y + x * Math.sin(origin.yaw) + y * Math.cos(origin.yaw)];
    }
    function fit() {
        const metadata = visibleMetadata();
        if (!metadata) return;
        const w = metadata.width * metadata.resolution, h = metadata.height * metadata.resolution;
        const corners = [[0,0], [w,0], [w,h], [0,h]].map(([x,y]) => world(x,y,metadata.origin));
        view.fit(ol.extent.boundingExtent(corners), {padding: [30,30,30,30], duration: 0});
        if (!preview) el('follow').checked = false;
    }
    function describeMap(metadata) {
        const origin = metadata.origin;
        return `${metadata.width} × ${metadata.height} клеток · ${metadata.resolution} м/клетку · origin (${origin.x.toFixed(2)}, ${origin.y.toFixed(2)}), ${(origin.yaw * 180 / Math.PI).toFixed(1)}°`;
    }
    function renderLiveMap(error = null) {
        if (preview) return;
        el('fit-map').disabled = !displayed;
        el('map-empty').hidden = !!displayed;
        el('map-empty').textContent = latest?.mode === 'idle'
            ? 'Нет активной карты. Можно просмотреть сохранённую карту, запустить локализацию или начать картографирование.'
            : 'Ожидание карты текущего режима…';
        const connection = el('connection');
        if (error) connection.textContent = 'Нет обновления карты: ' + error;
        else if (!latest?.available) connection.textContent = latest ? `Режим: ${latest.mode} · ${latest.reason}` : 'Подключение…';
        else connection.textContent = latest.stale ? `Карта устарела · ${latest.error || latest.age_s + ' с'}`
            : `Карта получена · версия ${latest.revision} · ${latest.age_s} с`;
        connection.classList.toggle('stale', !!error || !latest?.available || latest.stale);
        el('map-info').textContent = displayed ? describeMap(displayed) : 'Карта пока не получена';
    }
    function renderPreview() {
        const metadata = preview.metadata;
        el('fit-map').disabled = !metadata;
        el('connection').textContent = `Сохранённая карта · ${preview.name}`;
        el('connection').classList.toggle('stale', !!preview.error);
        el('map-empty').hidden = !!metadata;
        el('map-empty').textContent = preview.error || preview.progress;
        el('map-info').textContent = metadata ? describeMap(metadata) : 'Параметры карты пока не получены';
    }
    function drawRobot() {
        robots.clear(); el('fit-robot').disabled = true;
        el('follow').disabled = !poseValid();
        el('follow').title = poseValid() ? 'Следовать за свежим положением робота' : 'Нужно свежее положение робота на отображаемой карте';
        el('fit-robot').title = el('follow').title;
        if (!poseValid()) {
            el('pose-info').textContent = 'Положение робота неизвестно · ' + (lastPose?.reason || 'ожидание свежих map → base_footprint');
            return;
        }
        const pose = lastPose, transform = point => world(point[0], point[1], pose);
        for (const polygon of model?.polygons || []) {
            const points = polygon.points.map(transform); points.push(points[0]);
            robots.addFeature(new ol.Feature({geometry: new ol.geom.Polygon([points]), kind: polygon.kind}));
        }
        robots.addFeature(new ol.Feature({geometry: new ol.geom.Point([pose.x,pose.y]), kind: 'center'}));
        robots.addFeature(new ol.Feature({geometry: new ol.geom.LineString([[0,0],[.36,0],[.28,.05],[.36,0],[.28,-.05]].map(transform)), kind: 'arrow'}));
        el('pose-info').textContent = `Робот: x ${pose.x.toFixed(2)} м · y ${pose.y.toFixed(2)} м · угол ${(pose.yaw*180/Math.PI).toFixed(1)}° · map`;
        el('fit-robot').disabled = false;
        if (el('follow').checked) view.setCenter([pose.x,pose.y]);
    }
    async function pollMetadata() {
        if (metaBusy || document.hidden) return;
        metaBusy = true;
        try {
            latest = await api({part: 'metadata'});
            metadataReply = performance.now();
            if (!latest.available) {
                if (displayed) {
                    image = displayed = null;
                    if (!preview) raster.changed();
                }
            } else if (!displayed || displayed.map_id !== latest.map_id || displayed.revision !== latest.revision) {
                await load(latest);
            }
            renderLiveMap();
            renderSave(latest.save);
            drawRobot();
            drawScan();
            const scope = `${latest.mode}|${latest.saved_map || ''}`;
            if (catalogScope !== null && catalogScope !== scope) refreshLibrary();
            catalogScope = scope;
            renderLibrary();
            const diagnostics = await api({part: 'diagnostics'});
            el('scan-quality').textContent = diagnostics.variable_scan_geometry ? `Геометрия /scan меняется: ${diagnostics.beam_count_min}–${diagnostics.beam_count_max} лучей за 10 с. Это требует проверки входных данных SLAM.` : '';
        } catch (error) {
            latest = {...latest, stale: true}; drawRobot(); drawScan(); renderSave(latest.save);
            renderLiveMap(error.message);
        } finally { metaBusy = false; notifyState(); }
    }
    async function pollPose() {
        if (poseBusy || document.hidden) return;
        poseBusy = true;
        try { lastPose = await api({part: 'pose'}); lastPoseReply = performance.now(); }
        catch (error) { lastPose = {valid: false, reason: error.message}; }
        finally { poseBusy = false; drawRobot(); notifyState(); }
    }
    function drawScan() {
        laser.clear();
        if (!el('show-scan').checked) { el('scan-info').textContent = 'Точки лидара скрыты'; return; }
        if (!scanValid()) {
            el('scan-info').textContent = 'Облако лидара недоступно · ' + (lastScan?.reason || 'ожидание TF');
            return;
        }
        laser.addFeature(new ol.Feature({geometry: new ol.geom.MultiPoint(lastScan.points)}));
        el('scan-info').textContent = `Лидар: ${lastScan.points.length} точек · исходных лучей ${lastScan.raw_count} · возраст ${lastScan.age_s.toFixed(2)} с`;
    }
    async function pollScan() {
        if (scanBusy || document.hidden) return;
        scanBusy = true;
        try { lastScan = await api({part: 'scan'}); scanReply = performance.now(); }
        catch (error) { lastScan = {valid: false, reason: error.message}; }
        finally { scanBusy = false; drawScan(); notifyState(); }
    }
    function renderSave(state) {
        const name = el('map-name');
        const reason = saving || state?.busy ? 'Дождитесь завершения сохранения' :
            !state?.enabled ? 'Сохранение отключено или состояние ещё не получено' :
            latest?.mode !== 'mapping' ? 'Сохранение доступно при картографировании' :
            preview ? 'Вернитесь к текущей карте для сохранения' :
            !mapState().available || mapState().stale ? 'Нужна свежая карта текущей сессии' :
            !name.value || !name.checkValidity() ? 'Введите имя: латиница, цифры, _ или -' : '';
        el('save-map').disabled = !!reason;
        el('save-map').title = reason || 'Сохранить карту и граф SLAM с указанным именем';
        if (saveError && state?.id !== saveRequestId) { el('save-info').textContent = saveError; return; }
        if (!state) return;
        const firstObservation = !observedSave;
        observedSave = true;
        if (state.status === 'running') el('save-info').textContent = `Сохраняется ${state.name}: ${state.stage}`;
        else if (state.status === 'succeeded') {
            el('save-info').textContent = `Сохранено: ${state.name || state.base_path} · карта и граф SLAM`;
            if (state.id && state.id !== notifiedSave) {
                notifiedSave = state.id;
                if ((!firstObservation || state.id === saveRequestId) &&
                    typeof window.dispatchEvent === 'function' && typeof CustomEvent === 'function')
                    window.dispatchEvent(new CustomEvent('verter:map-saved', {
                        detail: {id: state.id, name: state.name, base_path: state.base_path}
                    }));
            }
        }
        else if (['failed','interrupted'].includes(state.status)) el('save-info').textContent = `Сохранение не завершено: ${state.error}${state.grid_saved ? ' · YAML/PGM уже созданы; выберите новое имя для повторной попытки' : ''}`;
        else el('save-info').textContent = reason || 'Карта готова к сохранению. Сначала остановите робота вручную.';
    }
    function uuid() {
        const bytes = crypto.getRandomValues(new Uint8Array(16)); bytes[6]=(bytes[6]&15)|64; bytes[8]=(bytes[8]&63)|128;
        const hex=[...bytes].map(value=>value.toString(16).padStart(2,'0')).join('');
        return `${hex.slice(0,8)}-${hex.slice(8,12)}-${hex.slice(12,16)}-${hex.slice(16,20)}-${hex.slice(20)}`;
    }
    async function mapCommand(action, name, id) {
        if (!serverClock || performance.now() - serverClock.midpoint > 10000)
            throw Error('Нет свежей синхронизации времени; дождитесь обновления карты');
        const response = await fetch('/api/command', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({
                v: 1, id, issued_at: serverClock.at + (performance.now() - serverClock.midpoint) / 1000,
                ttl_s: 30, action, args: {name}
            }),
            signal: AbortSignal.timeout(10000)
        });
        if (!response.ok) throw Error('HTTP ' + response.status);
        const body = await response.json();
        if (!body.ok) throw Error(body.error || 'Нет ответа map API');
        return body.result;
    }
    el('save-map').onclick = async () => {
        const name = el('map-name').value;
        if (!name || !el('map-name').checkValidity()) { el('map-name').reportValidity(); return; }
        saving = true; saveError = null; saveRequestId = uuid(); renderSave(latest?.save);
        try {
            const result = await mapCommand('map.save', name, saveRequestId);
            latest.save = result;
            renderSave(result);
        } catch (error) { saveError = 'Сохранение не подтверждено: '+error.message+'. Проверьте состояние; автоматического повтора нет.'; }
        finally { saving=false; renderSave(latest?.save); }
    };

    function selectedSavedMap() {
        const name = el('saved-map').value;
        return savedMaps.find(item => item.name === name);
    }
    const deleteErrors = {
        map_deletion_disabled: 'Удаление карт отключено в настройках робота.',
        saved_map_not_found: 'Карта отсутствует в каталоге.',
        map_in_use: 'Карта используется текущим режимом.',
        map_configured: 'Карта указана в настройках робота.',
        map_has_waypoints_or_routes: 'У карты есть сохранённые точки или маршруты.',
        map_image_in_use: 'Файл изображения используется другой картой.',
        supervisor_unavailable: 'Состояние режимов робота недоступно.',
        wait_for_current_map_or_mode_operation: 'Дождитесь завершения операции с картой или смены режима.'
    };
    function deleteRestriction(item) {
        if (item?.active || (item && item.name === latest?.saved_map)) return deleteErrors.map_in_use;
        if (item?.configured) return deleteErrors.map_configured;
        if (item?.has_definitions) return deleteErrors.map_has_waypoints_or_routes;
        return '';
    }
    function renderLibrary() {
        const item = selectedSavedMap();
        const samePreview = preview && preview.name === item?.name && !preview.error;
        const blocked = libraryLoading || deleting;
        const restriction = deleteRestriction(item);
        el('saved-map').disabled = blocked || !savedMaps.length;
        el('saved-refresh').disabled = blocked;
        el('saved-preview').disabled = blocked || !item?.grid || !!samePreview;
        el('saved-delete').disabled = blocked || !item || !!restriction;
        el('saved-delete-hint').textContent = restriction;
        el('saved-preview').title = item && !item.grid ? 'У этой карты нет YAML/PGM для просмотра' : '';
    }
    async function refreshLibrary() {
        if (libraryLoading) {
            libraryPending = true;
            return;
        }
        libraryLoading = true;
        renderLibrary();
        try {
            const result = await api({part: 'saved'});
            if (!Array.isArray(result.maps)) throw Error('Некорректный список карт');
            savedMaps = result.maps.filter(item => typeof item?.name === 'string');
            const select = el('saved-map');
            const before = select.value;
            const options = savedMaps.length
                ? savedMaps.map(item => new Option(item.name, item.name))
                : [new Option('Нет сохранённых карт', '')];
            select.replaceChildren(...options);
            if (savedMaps.some(item => item.name === before)) select.value = before;
        } catch (error) {
            if (!savedMaps.length) el('saved-map').replaceChildren(new Option('Список недоступен', ''));
            el('saved-info').textContent = 'Не удалось обновить список карт: ' + error.message;
        } finally {
            libraryLoading = false;
            renderLibrary();
            if (libraryPending) {
                libraryPending = false;
                await refreshLibrary();
            }
        }
    }

    async function showSavedMap() {
        const item = selectedSavedMap();
        if (!item?.grid || el('saved-preview').disabled) return;
        const name = item.name;
        const camera = preview?.camera || {
            mapId: displayed?.map_id,
            center: view.getCenter(),
            resolution: view.getResolution(),
            rotation: view.getRotation()
        };
        preview?.controller?.abort();
        const controller = new AbortController();
        const timeout = setTimeout(() => controller.abort(), 60000);
        const current = {name, camera, controller, progress: 'Загрузка сохранённой карты…'};
        preview = current;
        el('map-stage').dataset.preview = 'true';
        el('map-live').hidden = false;
        el('saved-info').textContent = '';
        view.setRotation(0);
        raster.changed();
        renderPreview();
        renderLibrary();
        renderSave(latest?.save);
        drawRobot();
        drawScan();
        notifyState();
        if (window.matchMedia('(max-width: 900px)').matches)
            el('map-stage').scrollIntoView({block: 'start'});
        el('ol-map').focus({preventScroll: true});
        try {
            const metadata = await api({part: 'saved_metadata', name}, controller.signal);
            const canvas = await downloadRaster(metadata, {part: 'saved_chunk', name}, controller.signal, (part, total) => {
                if (preview !== current) return;
                current.progress = `Получение сохранённой карты ${part}/${total}…`;
                renderPreview();
            });
            if (preview !== current) return;
            current.metadata = metadata;
            current.image = canvas;
            raster.changed();
            fit();
        } catch (error) {
            if (preview === current) current.error = controller.signal.aborted
                ? 'Истекло время загрузки сохранённой карты.'
                : 'Не удалось открыть сохранённую карту: ' + error.message;
        } finally {
            clearTimeout(timeout);
            if (preview === current) {
                current.controller = null;
                renderPreview();
                renderLibrary();
                notifyState();
            }
        }
    }
    function showLiveMap() {
        if (!preview) return;
        const {camera, controller} = preview;
        preview = null;
        controller?.abort();
        delete el('map-stage').dataset.preview;
        el('map-live').hidden = true;
        raster.changed();
        const newSession = displayed && displayed.map_id !== camera.mapId;
        view.setRotation(newSession && displayed.mode === 'mapping' ? 0 : camera.rotation);
        if (newSession) fit();
        else {
            view.setCenter(camera.center);
            view.setResolution(camera.resolution);
        }
        renderLiveMap();
        renderLibrary();
        renderSave(latest?.save);
        drawRobot();
        drawScan();
        notifyState();
    }
    async function deleteSavedMap() {
        const item = selectedSavedMap();
        if (!item || el('saved-delete').disabled) return;
        const name = item.name;
        const graph = item.pose_graph ? ' и её сохранённый граф SLAM' : '';
        if (!window.confirm(`Удалить карту «${name}»${graph}? Это действие нельзя отменить.`)) return;
        deleting = true;
        el('saved-info').textContent = `Удаляется карта «${name}»…`;
        renderLibrary();
        try {
            const result = await mapCommand('map.delete', name, uuid());
            if (result?.deleted !== name) throw Error('Сервер не подтвердил имя удалённой карты');
            if (preview?.name === name) showLiveMap();
            savedMaps = savedMaps.filter(entry => entry.name !== name);
            el('saved-map').selectedOptions[0]?.remove();
            await refreshLibrary();
            el('saved-info').textContent = `Карта «${name}» удалена.`;
            window.dispatchEvent(new CustomEvent('verter:map-deleted', {detail: {name}}));
        } catch (error) {
            await refreshLibrary();
            const reason = deleteErrors[error.message];
            el('saved-info').textContent = reason || `Удаление не подтверждено: ${error.message}. Проверьте список карт.`;
        } finally {
            deleting = false;
            renderLibrary();
        }
    }
    el('saved-refresh').onclick = () => {
        el('saved-info').textContent = '';
        refreshLibrary();
    };
    el('saved-map').onchange = () => {
        el('saved-info').textContent = '';
        renderLibrary();
    };
    el('saved-preview').onclick = showSavedMap;
    el('saved-delete').onclick = deleteSavedMap;
    el('map-live').onclick = () => {
        showLiveMap();
        el('ol-map').focus({preventScroll: true});
    };
    window.addEventListener('verter:map-tool-change', event => {
        if (event.detail?.tab === 'save') refreshLibrary();
    });
    window.addEventListener('verter:map-saved', refreshLibrary);
    el('map-name').value = 'survey_' + new Date().toISOString().replace(/[-:]/g,'').slice(0,15).replace('T','_');
    el('map-name').addEventListener('input', () => renderSave(latest?.save));
    el('show-scan').onchange = drawScan;
    el('fit-map').onclick = fit;
    el('fit-robot').onclick = () => { if (poseValid()) { view.setCenter([lastPose.x,lastPose.y]); view.setResolution(.01); } };
    el('follow').onchange = drawRobot;
    document.addEventListener('visibilitychange', () => {
        lastPose = null; drawRobot();
        lastScan = null; drawScan();
        notifyState();
        if (!document.hidden) { pollMetadata(); pollPose(); pollScan(); }
    });
    api({part: 'model'}).then(value => { model = value.available ? value : null; drawRobot(); }).catch(() => {});
    pollMetadata(); pollPose(); pollScan();
    setInterval(pollMetadata, 1000); setInterval(pollPose, 200); setInterval(pollScan, 200);
    setInterval(() => {
        if (mapState().stale || !poseValid()) drawRobot();
        if (mapState().stale || !scanValid()) drawScan();
        renderSave(latest?.save);
        notifyState();
    }, 500);
})();
