// ============================================================
// GLOBAL STATE
// ============================================================
let mqttClient       = null;
let isConnected      = false;
let macAddress       = "";
let telemetryTopic   = "";
let commandTopic     = "";
let lastPingTime     = null;
let pingInterval     = null;
let lastTelemetryData = null;
let polaritySynced    = false;

// Auto-Discovery
let discoveryClient    = null;
const discoveredDevices = new Map();

// Charts
const MAX_POINTS = 60;
let telemetryChart = null;
let errorChart     = null;

// Autopilot 2D
let autopilotActive    = false;
let autopilotStartX    = 0;
let autopilotStartY    = 0;
let autopilotTargetX   = 300;
let autopilotTargetY   = 100;
let autopilotWaypointSet = false;

// ============================================================
// CANVAS / ROBOT SIMULATOR
// ============================================================
const canvas = document.getElementById('robot-canvas');
const ctx    = canvas.getContext('2d');
const WHEEL_BASE      = 80;
const STEER_MAX_TICKS = 200;
const STEER_MAX_RAD   = Math.PI / 4;

let carX = 300, carY = 150, carTheta = -Math.PI / 2;
let steerAngleRad = 0;
let prevTracPos   = 0;
let pathHistory   = [];

function resetCarPose() {
    const r  = canvas.getBoundingClientRect();
    const w  = (r.width  || 600);
    const h  = (r.height || 280);
    carX = w / 2; carY = h / 2;
    carTheta = -Math.PI / 2;
    steerAngleRad = 0;
    prevTracPos   = 0;
    pathHistory   = [];
    autopilotWaypointSet = false;
    drawCar();
}

function setupCanvas() {
    function resize() {
        const dpr  = window.devicePixelRatio || 1;
        const rect = canvas.getBoundingClientRect();
        canvas.width  = rect.width  * dpr;
        canvas.height = rect.height * dpr;
        ctx.scale(dpr, dpr);
        resetCarPose();
    }
    window.addEventListener('resize', resize);
    resize();

    // Click on canvas → set target coordinates (local frame relative to car)
    canvas.addEventListener('click', (e) => {
        const rect = canvas.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        
        autopilotTargetX = x;
        autopilotTargetY = y;
        autopilotWaypointSet = true;
        
        // Calculate coordinates in meters relative to the car's current pose
        const dx = x - carX;
        const dy = y - carY;
        // y points forward (along theta), x points rightward (perpendicular)
        const x_local_m = (-dx * Math.sin(carTheta) + dy * Math.cos(carTheta)) / 200.0;
        const y_local_m = (dx * Math.cos(carTheta) + dy * Math.sin(carTheta)) / 200.0;
        
        document.getElementById('goto-x').value = x_local_m.toFixed(2);
        document.getElementById('goto-y').value = y_local_m.toFixed(2);
        
        // Automatically switch mode to coordinates in the selector
        const modeSelect = document.getElementById('goto-mode');
        const groupManual = document.getElementById('group-goto-manual');
        const groupCoords = document.getElementById('group-goto-coords');
        modeSelect.value = 'coords';
        groupManual.style.display = 'none';
        groupCoords.style.display = 'grid';
        
        drawCar(); // show waypoint marker
    });
}

function updateCarKinematics(tracPos, steerDeg) {
    // Convert steering degrees to radians
    steerAngleRad = (steerDeg * Math.PI) / 180.0;
    steerAngleRad = Math.max(-STEER_MAX_RAD, Math.min(STEER_MAX_RAD, steerAngleRad));

    // Displacement since last frame (signed, in encoder ticks → scale to px)
    const SCALE = 0.08; // px per tick
    const ds    = (tracPos - prevTracPos) * SCALE;
    prevTracPos = tracPos;

    if (Math.abs(ds) > 0.001) {
        if (Math.abs(steerAngleRad) < 0.01) {
            // Straight line
            carX += ds * Math.cos(carTheta);
            carY += ds * Math.sin(carTheta);
        } else {
            // Ackermann bicycle model
            const R      = WHEEL_BASE / Math.tan(steerAngleRad);
            const dTheta = ds / R;
            carX     += R * (Math.sin(carTheta + dTheta) - Math.sin(carTheta));
            carY     -= R * (Math.cos(carTheta + dTheta) - Math.cos(carTheta));
            carTheta += dTheta;
        }

        // Wrap around canvas edges
        const dpr = window.devicePixelRatio || 1;
        const W   = canvas.width  / dpr;
        const H   = canvas.height / dpr;
        carX = ((carX % W) + W) % W;
        carY = ((carY % H) + H) % H;

        pathHistory.push({ x: carX, y: carY });
        if (pathHistory.length > 500) pathHistory.shift();
    }
    drawCar();
}

function drawCar() {
    const dpr = window.devicePixelRatio || 1;
    const W   = canvas.width  / dpr;
    const H   = canvas.height / dpr;
    ctx.clearRect(0, 0, W, H);

    // Grid
    ctx.strokeStyle = 'rgba(255,255,255,0.03)';
    ctx.lineWidth   = 1;
    for (let x = 0; x < W; x += 40) { ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,H); ctx.stroke(); }
    for (let y = 0; y < H; y += 40) { ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke(); }

    // Autopilot waypoint marker
    if (autopilotWaypointSet) {
        const tx = autopilotTargetX;
        const ty = autopilotTargetY;
        // Pulsing ring
        ctx.beginPath();
        ctx.arc(tx, ty, 14, 0, Math.PI * 2);
        ctx.strokeStyle = autopilotActive ? 'rgba(16,185,129,0.9)' : 'rgba(249,115,22,0.8)';
        ctx.lineWidth   = 2;
        ctx.setLineDash([4, 4]);
        ctx.stroke();
        ctx.setLineDash([]);
        // Center dot
        ctx.beginPath();
        ctx.arc(tx, ty, 4, 0, Math.PI * 2);
        ctx.fillStyle = autopilotActive ? '#10b981' : '#f97316';
        ctx.fill();
        // Crosshair lines
        ctx.strokeStyle = autopilotActive ? 'rgba(16,185,129,0.5)' : 'rgba(249,115,22,0.5)';
        ctx.lineWidth   = 1;
        ctx.beginPath(); ctx.moveTo(tx - 18, ty); ctx.lineTo(tx + 18, ty); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(tx, ty - 18); ctx.lineTo(tx, ty + 18); ctx.stroke();
        // Label
        ctx.fillStyle   = autopilotActive ? '#10b981' : '#f97316';
        ctx.font        = 'bold 10px Outfit, sans-serif';
        ctx.fillText(`(${Math.round(tx)}, ${Math.round(ty)})`, tx + 16, ty - 6);

        // Line from car to waypoint
        ctx.beginPath();
        ctx.moveTo(carX, carY);
        ctx.lineTo(tx, ty);
        ctx.strokeStyle = autopilotActive ? 'rgba(16,185,129,0.25)' : 'rgba(249,115,22,0.15)';
        ctx.lineWidth   = 1;
        ctx.setLineDash([6, 6]);
        ctx.stroke();
        ctx.setLineDash([]);
    }

    // Trail
    if (pathHistory.length > 1) {
        ctx.beginPath();
        ctx.strokeStyle = 'rgba(14,165,233,0.4)';
        ctx.lineWidth   = 2;
        ctx.setLineDash([2, 5]);
        ctx.moveTo(pathHistory[0].x, pathHistory[0].y);
        pathHistory.forEach(p => ctx.lineTo(p.x, p.y));
        ctx.stroke();
        ctx.setLineDash([]);
    }

    // Car body
    ctx.save();
    ctx.translate(carX, carY);
    ctx.rotate(carTheta);

    const CW = 22, CH = 40;

    ctx.shadowBlur  = 14;
    ctx.shadowColor = 'rgba(14,165,233,0.5)';
    ctx.fillStyle   = 'rgba(30,41,59,0.95)';
    ctx.strokeStyle = '#0ea5e9';
    ctx.lineWidth   = 2;
    ctx.beginPath();
    ctx.roundRect(-CW, -CH, CW * 2, CH * 2, 5);
    ctx.fill();
    ctx.stroke();
    ctx.shadowBlur = 0;

    // Direction arrow
    ctx.fillStyle = '#f8fafc';
    ctx.beginPath();
    ctx.moveTo(0, -CH + 6);
    ctx.lineTo(-5, -CH + 16);
    ctx.lineTo(5,  -CH + 16);
    ctx.closePath();
    ctx.fill();

    // Rear wheels (fixed)
    [[-CW - 5, CH - 8], [CW + 5, CH - 8]].forEach(([wx, wy]) => {
        ctx.save(); ctx.translate(wx, wy);
        ctx.fillStyle = '#1e293b'; ctx.strokeStyle = '#64748b'; ctx.lineWidth = 1.5;
        ctx.fillRect(-4, -9, 8, 18); ctx.strokeRect(-4, -9, 8, 18);
        ctx.restore();
    });

    // Front wheels (steerable)
    [[-CW - 5, -CH + 8], [CW + 5, -CH + 8]].forEach(([wx, wy]) => {
        ctx.save(); ctx.translate(wx, wy);
        ctx.rotate(steerAngleRad);
        ctx.fillStyle = '#1e293b'; ctx.strokeStyle = '#0ea5e9'; ctx.lineWidth = 1.5;
        ctx.fillRect(-4, -9, 8, 18); ctx.strokeRect(-4, -9, 8, 18);
        ctx.strokeStyle = '#38bdf8'; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(0, -9); ctx.lineTo(0, 9); ctx.stroke();
        ctx.restore();
    });

    ctx.restore();
}

// ============================================================
// CHARTS
// ============================================================
function initChart() {
    Chart.defaults.color       = '#94a3b8';
    Chart.defaults.font.family = "'Outfit', sans-serif";

    // ---- Telemetry Chart (speed + angle vs time) ----
    telemetryChart = new Chart(
        document.getElementById('telemetry-chart').getContext('2d'), {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                { label: 'Velocidad Objetivo', data: [], borderColor: '#0ea5e9', borderWidth: 2, borderDash: [5,5], fill: false, tension: 0.3, pointRadius: 0 },
                { label: 'Velocidad Real',     data: [], borderColor: '#10b981', borderWidth: 2, fill: false, tension: 0.3, pointRadius: 0 },
                { label: 'Ángulo Objetivo',    data: [], borderColor: '#f97316', borderWidth: 2, borderDash: [5,5], fill: false, tension: 0.3, pointRadius: 0, yAxisID: 'y2' },
                { label: 'Ángulo Real',        data: [], borderColor: '#8b5cf6', borderWidth: 2, fill: false, tension: 0.3, pointRadius: 0, yAxisID: 'y2' }
            ]
        },
        options: {
            responsive: true, maintainAspectRatio: false,
            plugins: { legend: { position: 'top', labels: { boxWidth: 12, padding: 8, font: { size: 10 } } } },
            scales: {
                x:  { grid: { color: 'rgba(255,255,255,0.05)' }, ticks: { maxRotation: 0, autoSkip: true, maxTicksLimit: 8 } },
                y:  { grid: { color: 'rgba(255,255,255,0.05)' }, title: { display: true, text: 'ticks/s', font: { size: 9 } } },
                y2: { position: 'right', grid: { drawOnChartArea: false }, title: { display: true, text: 'ticks (ángulo)', font: { size: 9 } } }
            },
            animation: { duration: 0 }
        }
    });

    // ---- Error / Stability Chart ----
    errorChart = new Chart(
        document.getElementById('error-chart').getContext('2d'), {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                { label: 'Error Tracción',  data: [], borderColor: '#f97316', borderWidth: 2, fill: { target: 'origin', above: 'rgba(249,115,22,0.08)' }, tension: 0.3, pointRadius: 0 },
                { label: 'Error Dirección', data: [], borderColor: '#8b5cf6', borderWidth: 2, fill: { target: 'origin', above: 'rgba(139,92,246,0.08)'  }, tension: 0.3, pointRadius: 0 },
                { label: 'ΔU Tracción',     data: [], borderColor: '#0ea5e9', borderWidth: 1, borderDash: [3,3], fill: false, tension: 0.3, pointRadius: 0 }
            ]
        },
        options: {
            responsive: true, maintainAspectRatio: false,
            plugins: { legend: { position: 'top', labels: { boxWidth: 12, padding: 8, font: { size: 10 } } } },
            scales: {
                x: { grid: { color: 'rgba(255,255,255,0.05)' }, ticks: { maxRotation: 0, autoSkip: true, maxTicksLimit: 8 } },
                y: { grid: { color: 'rgba(255,255,255,0.05)' }, title: { display: true, text: 'Error de Control (Fuzzy)', font: { size: 9 } } }
            },
            animation: { duration: 0 }
        }
    });
}

function pushChart(targetSpeed, actualSpeed, targetAngle, actualAngle, tracErr, steerErr, tracDu) {
    const t = new Date().toLocaleTimeString([], { hour:'2-digit', minute:'2-digit', second:'2-digit' });

    function pushToChart(chart, label, values) {
        chart.data.labels.push(label);
        chart.data.datasets.forEach((ds, i) => ds.data.push(values[i]));
        if (chart.data.labels.length > MAX_POINTS) {
            chart.data.labels.shift();
            chart.data.datasets.forEach(ds => ds.data.shift());
        }
        chart.update('none'); // 'none' = no animation, maximum speed
    }

    pushToChart(telemetryChart, t, [targetSpeed, actualSpeed, targetAngle, actualAngle]);
    pushToChart(errorChart,     t, [tracErr, steerErr, tracDu]);
}

// ============================================================
// MQTT
// ============================================================
function connectMQTT() {
    polaritySynced = false;
    macAddress = document.getElementById('mac-input').value
        .replace(/[^a-fA-F0-9]/g, '').toUpperCase();
    if (macAddress.length !== 12) {
        alert("La MAC debe tener 12 caracteres hexadecimales (ej: 24:0A:C4:08:E9:D4).");
        return;
    }

    const btn = document.getElementById('connect-btn');
    btn.disabled = true;
    btn.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i> Conectando...';

    const base      = "esp32/robot_fuzzy/" + macAddress;
    telemetryTopic  = base + "/telemetry";
    commandTopic    = base + "/command";
    document.getElementById('base-topic-display').innerText = base + "/...";
    lastPingTime = null;

    const useSSL = window.location.protocol === "https:";
    const port   = useSSL ? 8884 : 8000;
    const randId = "WebAck_" + Math.random().toString(16).substr(2, 8);

    mqttClient = new Paho.MQTT.Client("broker.hivemq.com", port, randId);
    mqttClient.onConnectionLost  = onConnectionLost;
    mqttClient.onMessageArrived  = onMessageArrived;
    mqttClient.connect({ useSSL, timeout: 10, onSuccess: onConnectSuccess, onFailure: onConnectFailure });
}

function onConnectSuccess() {
    isConnected = true;
    document.getElementById('conn-badge').className = "badge badge-connected";
    document.getElementById('conn-text').innerText  = "ESPERANDO ESP32...";
    const btn = document.getElementById('connect-btn');
    btn.disabled = false;
    btn.className = "btn btn-danger";
    btn.innerHTML = '<i class="fa-solid fa-unplug"></i> Desconectar';
    mqttClient.subscribe(telemetryTopic);
    startHeartbeatTimer();
}

function onConnectFailure(err) {
    alert("No se pudo conectar al broker MQTT. Verifica tu conexión.");
    const btn = document.getElementById('connect-btn');
    btn.disabled = false; btn.className = "btn btn-primary";
    btn.innerHTML = '<i class="fa-solid fa-plug"></i> Conectar';
    isConnected = false;
}

function onConnectionLost(res) {
    isConnected = false;
    polaritySynced = false;
    document.getElementById('conn-badge').className = "badge badge-disconnected";
    document.getElementById('conn-text').innerText  = "DESCONECTADO";
    const btn = document.getElementById('connect-btn');
    btn.disabled = false; btn.className = "btn btn-primary";
    btn.innerHTML = '<i class="fa-solid fa-plug"></i> Conectar';
    stopHeartbeatTimer();
    if (autopilotActive) stopAutopilot(false);
}

function disconnectMQTT() {
    if (mqttClient && isConnected) mqttClient.disconnect();
}

function publishCommand(obj) {
    if (!mqttClient || !isConnected) return;
    const msg = new Paho.MQTT.Message(JSON.stringify(obj));
    msg.destinationName = commandTopic;
    mqttClient.send(msg);
}

// ============================================================
// TELEMETRY HANDLER
// ============================================================
function onMessageArrived(message) {
    // Skip discovery messages
    if (message.destinationName !== telemetryTopic) return;

    try {
        const d = JSON.parse(message.payloadString);
        lastTelemetryData = d;

        // Heartbeat
        lastPingTime = new Date();
        document.getElementById('last-ping').innerText = lastPingTime.toLocaleTimeString();
        document.getElementById('conn-badge').className = "badge badge-connected";
        document.getElementById('conn-text').innerText  = "ESP32 ONLINE";

        // --- Traction numeric telemetry ---
        const measSpeed = (typeof d.trac.speed === 'number') ? d.trac.speed : 0;
        document.getElementById('num-target-speed').innerText = d.trac.target.toFixed(1);
        document.getElementById('num-speed-trac').innerText   = measSpeed.toFixed(1);
        document.getElementById('num-pwm-trac').innerText     = Math.round(d.trac.pwm);
        document.getElementById('num-pos-trac').innerText     = d.trac.pos;

        // --- Steering numeric telemetry (firmware sends degrees) ---
        const steerTargetDeg = (typeof d.steer.target === 'number') ? d.steer.target : 0;
        const steerPosDeg    = (typeof d.steer.pos    === 'number') ? d.steer.pos    : 0;
        const steerErrDeg    = (typeof d.steer.err    === 'number') ? d.steer.err    : 0;
        document.getElementById('num-target-angle').innerText = steerTargetDeg.toFixed(1);
        document.getElementById('num-pos-steer').innerText    = steerPosDeg.toFixed(1);
        document.getElementById('num-pwm-steer').innerText    = Math.round(d.steer.pwm);
        document.getElementById('num-err-steer').innerText    = steerErrDeg.toFixed(1);

        // --- Steering hardware limits indicator ---
        if (d.limits) {
            const limCW  = document.getElementById('lim-cw');
            const limCCW = document.getElementById('lim-ccw');
            const limPos = document.getElementById('lim-pos-bar');
            if (limCW)  limCW.style.color  = d.limits.at_cw  === true || d.limits.at_cw  === 'true'  ? '#ef4444' : '#475569';
            if (limCCW) limCCW.style.color = d.limits.at_ccw === true || d.limits.at_ccw === 'true'  ? '#ef4444' : '#475569';
            if (limPos) {
                // Map steer pos to percentage bar: -826 = 0%, 0 = 50%, +826 = 100%
                const pct = ((d.limits.steer_pos + 826) / 1652 * 100).toFixed(1);
                limPos.style.width = Math.max(0, Math.min(100, pct)) + '%';
                limPos.style.background = (d.limits.at_cw === true || d.limits.at_cw === 'true' || d.limits.at_ccw === true || d.limits.at_ccw === 'true') ? '#ef4444' : '#3b82f6';
            }
        }

        // --- Diagnostics panel update ---
        if (d.diag) {
            const diagPanel = document.getElementById('diag-status');
            if (diagPanel) {
                if (d.diag.active) {
                    diagPanel.innerHTML = `<span style="color:#f97316">&#9654; DIAGNÓSTICO ACTIVO — Motor: <strong>${d.diag.motor.toUpperCase()}</strong>, PWM: <strong>${d.diag.pwm}</strong></span>`;
                } else {
                    diagPanel.innerHTML = `<span style="color:#475569">Sin diagnóstico activo</span>`;
                }
            }
        }

        // --- Charts (target speed, actual speed, steering angle) ---
        const tracErr  = d.trac.err  || 0;
        const steerErr = steerErrDeg;
        const tracDu   = d.trac.du   || 0;
        pushChart(d.trac.target, measSpeed, steerTargetDeg, steerPosDeg, tracErr, steerErr, tracDu);

        // --- Fuzzy inspector bars (Traction only now — steering uses PD) ---
        document.getElementById('val-fuzzy-err-trac').innerText  = (d.trac.target - measSpeed).toFixed(1);
        document.getElementById('val-fuzzy-derr-trac').innerText = (d.trac.derr || 0).toFixed(2);
        // Steering PD — show error magnitude in fuzzy bar format
        document.getElementById('val-fuzzy-err-steer').innerText  = steerErrDeg.toFixed(1);
        document.getElementById('val-fuzzy-derr-steer').innerText = '(PD Clásico)';
        // Simulate bar fill from steering error magnitude
        const steerNormErr = Math.min(1.0, Math.abs(steerErrDeg) / 180.0);
        setBar('bar-se-2', 'txt-se-2', 1 - steerNormErr); // ZE rises as error shrinks
        if (steerErrDeg > 0) {
            setBar('bar-se-3', 'txt-se-3', steerNormErr * 0.6);
            setBar('bar-se-4', 'txt-se-4', steerNormErr * 0.4);
            ['bar-se-0','bar-se-1'].forEach(id => { const el=document.getElementById(id); if(el) el.style.width='0%'; });
            ['txt-se-0','txt-se-1'].forEach(id => { const el=document.getElementById(id); if(el) el.innerText='0%'; });
        } else {
            setBar('bar-se-0', 'txt-se-0', steerNormErr * 0.4);
            setBar('bar-se-1', 'txt-se-1', steerNormErr * 0.6);
            ['bar-se-3','bar-se-4'].forEach(id => { const el=document.getElementById(id); if(el) el.style.width='0%'; });
            ['txt-se-3','txt-se-4'].forEach(id => { const el=document.getElementById(id); if(el) el.innerText='0%'; });
        }
        if (d.fuzzy && d.fuzzy.trac_mu_e) {
            d.fuzzy.trac_mu_e.forEach((v,i)  => setBar('bar-te-'+i,  'txt-te-'+i,  v));
        }
        if (d.fuzzy && d.fuzzy.trac_mu_de) {
            d.fuzzy.trac_mu_de.forEach((v,i) => setBar('bar-tde-'+i, 'txt-tde-'+i, v));
        }

        // --- 2D Ackermann Simulator (steer in degrees) ---
        updateCarKinematics(d.trac.pos, steerPosDeg);

        // --- Precision Arrival (ESP32-Side) Feedback ---
        if (d.arrival) {
            updateArrivalFeedback(d.arrival);
        }

        // --- Dynamic Polarity + Swap (NVS) Calibration Sync ---
        if (d.polarity && !polaritySynced) {
            document.getElementById('cal-t-mot').checked = (d.polarity.t_mot === -1);
            document.getElementById('cal-t-enc').checked = (d.polarity.t_enc === -1);
            document.getElementById('cal-s-mot').checked = (d.polarity.s_mot === -1);
            document.getElementById('cal-s-enc').checked = (d.polarity.s_enc === -1);
            const swapEl = document.getElementById('cal-swap-hw');
            if (swapEl) swapEl.checked = (d.polarity.swap === 1);
            polaritySynced = true;
        }

    } catch(e) { console.error("Telemetry parse error:", e, message.payloadString); }
}

function setBar(barId, txtId, val) {
    const v = Math.max(0, Math.min(1, val || 0));
    document.getElementById(barId).style.width = (v * 100) + '%';
    document.getElementById(txtId).innerText   = Math.round(v * 100) + '%';
}

// ============================================================
// HEARTBEAT
// ============================================================
function startHeartbeatTimer() {
    stopHeartbeatTimer();
    pingInterval = setInterval(() => {
        if (lastPingTime && (new Date() - lastPingTime) / 1000 > 5) {
            document.getElementById('conn-badge').className = "badge badge-disconnected";
            document.getElementById('conn-text').innerText  = "ESP32 OFFLINE";
        }
    }, 1000);
}
function stopHeartbeatTimer() {
    if (pingInterval) { clearInterval(pingInterval); pingInterval = null; }
}

// ============================================================
// THROTTLED DRIVE
// ============================================================
let sliderTimeout = null;
function throttledDrive(speed, angle) {
    if (sliderTimeout) clearTimeout(sliderTimeout);
    sliderTimeout = setTimeout(() => {
        publishCommand({ cmd: "drive", speed: parseFloat(speed), angle: parseFloat(angle) });
    }, 60);
}

// ============================================================
// AUTO-DISCOVERY
// ============================================================
function initDiscovery() {
    const useSSL = window.location.protocol === "https:";
    const port   = useSSL ? 8884 : 8000;
    const randId = "WebDiscover_" + Math.random().toString(16).substr(2, 8);

    discoveryClient = new Paho.MQTT.Client("broker.hivemq.com", port, randId);
    discoveryClient.onConnectionLost = () => setTimeout(initDiscovery, 8000);
    discoveryClient.onMessageArrived = (msg) => {
        try {
            const d = JSON.parse(msg.payloadString);
            if (d.mac) addDiscoveredDevice(d.mac, d.ip);
        } catch(e) {}
    };
    discoveryClient.connect({
        useSSL, timeout: 8,
        onSuccess: () => discoveryClient.subscribe("esp32/robot_fuzzy/discovery"),
        onFailure: () => setTimeout(initDiscovery, 8000)
    });
}

function addDiscoveredDevice(mac, ip) {
    const listEl = document.getElementById('detected-list');
    const secEl  = document.getElementById('discovery-section');
    if (!listEl || !secEl) return;

    const cleanMac = mac.replace(/[^a-fA-F0-9]/g, '').toUpperCase();
    if (cleanMac.length !== 12) return;

    const fmtMac = cleanMac.match(/.{1,2}/g).join(':');
    const now    = new Date();

    if (discoveredDevices.has(cleanMac)) {
        discoveredDevices.get(cleanMac).lastSeen = now;
        return;
    }

    secEl.style.display = 'block';
    const btn = document.createElement('button');
    btn.id        = `btn-dev-${cleanMac}`;
    btn.className = 'btn';
    btn.style.cssText = 'background:rgba(16,185,129,0.1);border:1px solid rgba(16,185,129,0.3);color:var(--color-green);padding:6px 12px;font-size:0.8rem;border-radius:6px;cursor:pointer;display:inline-flex;align-items:center;gap:6px;transition:all 0.2s;';
    btn.innerHTML = `<i class="fa-solid fa-microchip"></i> ${fmtMac} <span style="font-size:0.7rem;opacity:0.7;padding:1px 5px;background:rgba(255,255,255,0.08);border-radius:3px;">${ip||''}</span>`;
    btn.onclick   = () => {
        document.getElementById('mac-input').value = fmtMac;
        if (isConnected) { disconnectMQTT(); setTimeout(connectMQTT, 300); }
        else connectMQTT();
    };
    listEl.appendChild(btn);
    discoveredDevices.set(cleanMac, { lastSeen: now, element: btn });
}

// ============================================================
// PRECISION ARRIVAL FEEDBACK (ESP32-SIDE)
// ============================================================
function updateArrivalFeedback(arrival) {
    const txtState = document.getElementById('txt-goto-state');
    const txtTarget = document.getElementById('txt-goto-target');
    const txtErr = document.getElementById('txt-goto-err-dist');
    const btnSend = document.getElementById('btn-send-goto');
    const btnAbort = document.getElementById('btn-abort-goto');

    if (arrival.active) {
        btnSend.style.display = 'none';
        btnAbort.style.display = 'inline-flex';
        
        let targetText = "";
        const mode = document.getElementById('goto-mode').value;
        if (mode === 'coords') {
            const x = document.getElementById('goto-x').value;
            const y = document.getElementById('goto-y').value;
            targetText = `Punto (${x}m, ${y}m) → Calc: ${arrival.target_dist.toFixed(1)}cm @ ${arrival.target_angle.toFixed(1)}°`;
        } else {
            targetText = `${arrival.target_dist.toFixed(1)} cm a ${arrival.target_angle.toFixed(1)}°`;
        }
        txtTarget.innerText = targetText;

        if (arrival.phase === 1) {
            txtState.innerText = "FASE 1: ORIENTANDO DIRECCIÓN";
            txtState.style.color = "var(--color-orange)";
            txtErr.innerText = "Esperando alineación...";
            txtErr.style.color = "var(--color-orange)";
        } else if (arrival.phase === 2) {
            txtState.innerText = "FASE 2: TRASLACIÓN DE PRECISIÓN";
            txtState.style.color = "var(--color-blue)";
            txtErr.innerText = `${arrival.remaining_dist.toFixed(1)} cm restantes`;
            txtErr.style.color = "var(--color-blue)";
        }
    } else {
        btnSend.style.display = 'inline-flex';
        btnAbort.style.display = 'none';
        
        txtState.innerText = "INACTIVO / EN DESTINO";
        txtState.style.color = "var(--text-secondary)";
        txtTarget.innerText = "--";
        txtErr.innerText = "--";
        txtErr.style.color = "var(--text-secondary)";
    }
}

// ============================================================
// DOM EVENTS
// ============================================================
document.addEventListener("DOMContentLoaded", () => {
    initChart();
    setupCanvas();
    initDiscovery();

    // Cleanup stale discovered devices every 15s
    setInterval(() => {
        const now = new Date();
        discoveredDevices.forEach((dev, mac) => {
            if ((now - dev.lastSeen) / 1000 > 45) {
                dev.element.remove();
                discoveredDevices.delete(mac);
            }
        });
        if (discoveredDevices.size === 0) {
            const s = document.getElementById('discovery-section');
            if (s) s.style.display = 'none';
        }
    }, 15000);

    // Connect button
    document.getElementById('connect-btn').addEventListener('click', () => {
        isConnected ? disconnectMQTT() : connectMQTT();
    });

    // Speed & Angle sliders
    const slSpeed = document.getElementById('slider-speed');
    const slAngle = document.getElementById('slider-angle');
    const lbSpeed = document.getElementById('val-target-speed');
    const lbAngle = document.getElementById('val-target-angle');

    slSpeed.addEventListener('input', e => {
        lbSpeed.innerText = e.target.value;
        const continuous = document.getElementById('continuous-send');
        if (!continuous || continuous.checked) {
            throttledDrive(slSpeed.value, slAngle.value);
        }
    });
    slAngle.addEventListener('input', e => {
        lbAngle.innerText = e.target.value;
        const continuous = document.getElementById('continuous-send');
        if (!continuous || continuous.checked) {
            throttledDrive(slSpeed.value, slAngle.value);
        }
    });

    // Manual send buttons
    document.getElementById('send-speed-btn').addEventListener('click', () => {
        publishCommand({ cmd: "drive", speed: parseFloat(slSpeed.value), angle: parseFloat(slAngle.value) });
    });
    document.getElementById('send-angle-btn').addEventListener('click', () => {
        publishCommand({ cmd: "drive", speed: parseFloat(slSpeed.value), angle: parseFloat(slAngle.value) });
    });

    // Precision positioning Mode Switcher
    const modeSelect = document.getElementById('goto-mode');
    const groupManual = document.getElementById('group-goto-manual');
    const groupCoords = document.getElementById('group-goto-coords');
    
    modeSelect.addEventListener('change', () => {
        if (modeSelect.value === 'coords') {
            groupManual.style.display = 'none';
            groupCoords.style.display = 'grid';
        } else {
            groupManual.style.display = 'grid';
            groupCoords.style.display = 'none';
        }
    });

    // Send Goto Positioning Command
    document.getElementById('btn-send-goto').addEventListener('click', () => {
        if (!isConnected) { alert("Conéctate primero al ESP32."); return; }
        
        const mode = modeSelect.value;
        if (mode === 'coords') {
            const x = parseFloat(document.getElementById('goto-x').value) || 0.0;
            const y = parseFloat(document.getElementById('goto-y').value) || 0.0;
            publishCommand({ cmd: "goto_pt", x: x, y: y });
            
            // For the 2D simulator visualization
            // Convert clicked relative coordinates back to global canvas space
            // Scale: 1m = 200px
            const dx = -x * 200.0 * Math.sin(carTheta) + y * 200.0 * Math.cos(carTheta);
            const dy = x * 200.0 * Math.cos(carTheta) + y * 200.0 * Math.sin(carTheta);
            
            autopilotTargetX = carX + dx;
            autopilotTargetY = carY + dy;
            autopilotWaypointSet = true;
            drawCar();
        } else {
            const dist = parseFloat(document.getElementById('goto-dist').value) || 0.0;
            const angle = parseFloat(document.getElementById('goto-angle').value) || 0.0;
            publishCommand({ cmd: "goto", dist: dist, angle: angle });
            
            // For the 2D simulator visualization
            // In manual mode, we just show a straight line representation
            const ticksPerCm = 25.0;
            const pxDist = (dist * ticksPerCm) * 0.08;
            const radAngle = (angle * Math.PI) / 180.0;
            const totalTheta = carTheta + radAngle;
            
            autopilotTargetX = carX + pxDist * Math.cos(totalTheta);
            autopilotTargetY = carY + pxDist * Math.sin(totalTheta);
            autopilotWaypointSet = true;
            drawCar();
        }
    });

    // Abort Precision Control
    document.getElementById('btn-abort-goto').addEventListener('click', () => {
        publishCommand({ cmd: "stop" });
        autopilotWaypointSet = false;
        drawCar();
    });

    // Preset buttons
    document.querySelectorAll('.preset-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            const s = btn.getAttribute('data-speed');
            const a = btn.getAttribute('data-angle');
            slSpeed.value = s; slAngle.value = a;
            lbSpeed.innerText = s; lbAngle.innerText = a;
            publishCommand({ cmd: "drive", speed: parseFloat(s), angle: parseFloat(a) });
            autopilotWaypointSet = false;
            drawCar();
        });
    });

    // Emergency stop
    document.getElementById('btn-estop').addEventListener('click', () => {
        slSpeed.value = 0; slAngle.value = 0;
        lbSpeed.innerText = 0; lbAngle.innerText = 0;
        publishCommand({ cmd: "stop" });
        autopilotWaypointSet = false;
        drawCar();
    });

    // Reset encoders
    document.getElementById('btn-reset').addEventListener('click', () => {
        publishCommand({ cmd: "reset" });
        prevTracPos = 0; resetCarPose();
    });

    // Clear trail
    document.getElementById('clear-trail-btn').addEventListener('click', () => {
        pathHistory = []; drawCar();
    });

    // Fuzzy tuning
    document.getElementById('send-tuning-btn').addEventListener('click', () => {
        const ge   = parseFloat(document.getElementById('gain-ge').value);
        const gde  = parseFloat(document.getElementById('gain-gde').value);
        const gu   = parseFloat(document.getElementById('gain-gu').value);
        const motor = document.getElementById('tune-motor').value;
        publishCommand({ cmd: "tune", motor, ge, gde, gu });
        const label = motor === 'trac' ? 'Tracción' : 'Dirección';
        alert(`Parámetros enviados → Motor ${label}\nGe=${ge}  Gde=${gde}  Gu=${gu}`);
    });

    // Send Polarity Calibration Command (incluye swap hardware)
    document.getElementById('send-polarity-btn').addEventListener('click', () => {
        if (!isConnected) { alert("Conéctate primero al ESP32."); return; }
        
        const t_mot = document.getElementById('cal-t-mot').checked ? -1 : 1;
        const t_enc = document.getElementById('cal-t-enc').checked ? -1 : 1;
        const s_mot = document.getElementById('cal-s-mot').checked ? -1 : 1;
        const s_enc = document.getElementById('cal-s-enc').checked ? -1 : 1;
        const swapEl = document.getElementById('cal-swap-hw');
        const swap   = (swapEl && swapEl.checked) ? 1 : 0;
        
        polaritySynced = false;
        publishCommand({ cmd: "polarity", t_mot, t_enc, s_mot, s_enc, swap });
        alert(`Configuración de hardware enviada al ESP32:\n\n` +
              `⚙️ Tracción → Motor: ${t_mot === -1 ? 'Invertido' : 'Normal'}, Encoder: ${t_enc === -1 ? 'Invertido' : 'Normal'}\n` +
              `🔄 Dirección → Motor: ${s_mot === -1 ? 'Invertido' : 'Normal'}, Encoder: ${s_enc === -1 ? 'Invertido' : 'Normal'}\n` +
              `🔀 Swap Cables: ${swap === 1 ? 'ACTIVADO (motores cruzados)' : 'Desactivado'}\n\n` +
              `Se guardará permanentemente en la memoria NVS.`);
    });

    // Diagnostics raw PWM
    document.getElementById('btn-diag-run').addEventListener('click', () => {
        if (!isConnected) { alert("Conéctate primero."); return; }
        const motor = document.getElementById('diag-motor').value;
        const pwm   = parseInt(document.getElementById('diag-pwm').value) || 0;
        publishCommand({ cmd: "diagnostics", motor, pwm });
    });
    document.getElementById('btn-diag-stop').addEventListener('click', () => {
        publishCommand({ cmd: "stop" });
    });
});
