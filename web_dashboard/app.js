// --- Global State ---
let mqttClient = null;
let isConnected = false;
let macAddress = "";
let telemetryTopic = "";
let commandTopic = "";
let lastPingTime = null;
let pingInterval = null;

// Auto-Discovery variables
let discoveryClient = null;
const discoveredDevices = new Map(); // MAC -> { ip, lastSeen, element }

// Chart
const maxDataPoints = 40;
let telemetryChart = null;

// --- Ackermann Robot State ---
const canvas = document.getElementById('robot-canvas');
const ctx = canvas.getContext('2d');
const WHEEL_BASE = 80;  // px between front and rear axles
const WHEEL_TRACK = 40; // px between left/right wheels
const STEER_MAX_TICKS = 200;
const STEER_MAX_RAD = Math.PI / 4; // 45 degrees max physical steering angle

let carX, carY, carTheta; // car pose (center of rear axle)
let steerAngleRad = 0;    // current steering wheel angle in radians
let odometer = 0;         // accumulated distance
let pathHistory = [];

function resetCarPose() {
    const w = canvas.getBoundingClientRect().width || 600;
    const h = canvas.getBoundingClientRect().height || 280;
    carX = w / 2; carY = h / 2; carTheta = -Math.PI / 2;
    steerAngleRad = 0; odometer = 0; pathHistory = [];
    drawCar();
}

// --- Chart.js init ---
function initChart() {
    const chartCtx = document.getElementById('telemetry-chart').getContext('2d');
    Chart.defaults.color = '#94a3b8';
    Chart.defaults.font.family = "'Outfit', sans-serif";
    telemetryChart = new Chart(chartCtx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                { label: 'Target Speed', data: [], borderColor: '#0ea5e9', borderWidth: 2, borderDash: [5,5], fill: false, tension: 0.3, pointRadius: 0 },
                { label: 'Actual Speed', data: [], borderColor: '#10b981', borderWidth: 2, fill: false, tension: 0.3, pointRadius: 0 },
                { label: 'Target Angle', data: [], borderColor: '#f97316', borderWidth: 2, borderDash: [5,5], fill: false, tension: 0.3, pointRadius: 0, yAxisID: 'y2' },
                { label: 'Actual Angle', data: [], borderColor: '#8b5cf6', borderWidth: 2, fill: false, tension: 0.3, pointRadius: 0, yAxisID: 'y2' }
            ]
        },
        options: {
            responsive: true, maintainAspectRatio: false,
            plugins: { legend: { position: 'top', labels: { boxWidth: 12, padding: 8, font: { size: 10 } } } },
            scales: {
                x: { grid: { color: 'rgba(255,255,255,0.05)' }, ticks: { maxRotation: 0, autoSkip: true, maxTicksLimit: 10 } },
                y: { grid: { color: 'rgba(255,255,255,0.05)' }, title: { display: true, text: 'Velocidad (ticks/s)', font: { size: 9 } } },
                y2: { position: 'right', grid: { drawOnChartArea: false }, title: { display: true, text: 'Ángulo (ticks)', font: { size: 9 } } }
            },
            animation: { duration: 0 }
        }
    });
}

function updateChart(targetSpeed, actualSpeed, targetAngle, actualAngle) {
    const t = new Date().toLocaleTimeString([], { hour:'2-digit', minute:'2-digit', second:'2-digit' });
    telemetryChart.data.labels.push(t);
    telemetryChart.data.datasets[0].data.push(targetSpeed);
    telemetryChart.data.datasets[1].data.push(actualSpeed);
    telemetryChart.data.datasets[2].data.push(targetAngle);
    telemetryChart.data.datasets[3].data.push(actualAngle);
    if (telemetryChart.data.labels.length > maxDataPoints) {
        telemetryChart.data.labels.shift();
        telemetryChart.data.datasets.forEach(d => d.data.shift());
    }
    telemetryChart.update();
}

// --- Ackermann Canvas Simulator ---
function setupCanvas() {
    function resize() {
        const dpr = window.devicePixelRatio || 1;
        const rect = canvas.getBoundingClientRect();
        canvas.width  = rect.width  * dpr;
        canvas.height = rect.height * dpr;
        ctx.scale(dpr, dpr);
        resetCarPose();
    }
    window.addEventListener('resize', resize);
    resize();
}

function updateCarKinematics(tracPos, steerTicks, dt) {
    // Convert steering ticks → radians
    steerAngleRad = (steerTicks / STEER_MAX_TICKS) * STEER_MAX_RAD;

    // Estimate speed from odometer diff (simplified: use tracPos delta)
    // We drive using steerTicks directly, speed is visual-only
    const speed = 0.3; // pixels per update tick (visual scale)
    if (Math.abs(tracPos) > odometer) {
        const ds = Math.min(Math.abs(tracPos) - odometer, 5);
        const dir = tracPos >= 0 ? 1 : -1;
        odometer = Math.abs(tracPos);

        // Ackermann bicycle model
        if (Math.abs(steerAngleRad) < 0.01) {
            carX += dir * ds * Math.cos(carTheta);
            carY += dir * ds * Math.sin(carTheta);
        } else {
            const R = WHEEL_BASE / Math.tan(steerAngleRad);
            const dTheta = dir * ds / R;
            carX += R * (Math.sin(carTheta + dTheta) - Math.sin(carTheta));
            carY -= R * (Math.cos(carTheta + dTheta) - Math.cos(carTheta));
            carTheta += dTheta;
        }

        // Wrap position
        const w = canvas.width / (window.devicePixelRatio || 1);
        const h = canvas.height / (window.devicePixelRatio || 1);
        carX = ((carX % w) + w) % w;
        carY = ((carY % h) + h) % h;

        pathHistory.push({ x: carX, y: carY });
        if (pathHistory.length > 400) pathHistory.shift();
    }
    drawCar();
}

function drawCar() {
    const dpr = window.devicePixelRatio || 1;
    const W = canvas.width / dpr;
    const H = canvas.height / dpr;
    ctx.clearRect(0, 0, W, H);

    // Grid
    ctx.strokeStyle = 'rgba(255,255,255,0.03)';
    ctx.lineWidth = 1;
    for (let x = 0; x < W; x += 40) { ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,H); ctx.stroke(); }
    for (let y = 0; y < H; y += 40) { ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke(); }

    // Trail
    if (pathHistory.length > 1) {
        ctx.beginPath();
        ctx.strokeStyle = 'rgba(14,165,233,0.35)';
        ctx.lineWidth = 2;
        ctx.setLineDash([2,5]);
        ctx.moveTo(pathHistory[0].x, pathHistory[0].y);
        pathHistory.forEach(p => ctx.lineTo(p.x, p.y));
        ctx.stroke();
        ctx.setLineDash([]);
    }

    // --- Draw Car Body ---
    ctx.save();
    ctx.translate(carX, carY);
    ctx.rotate(carTheta);

    const CW = 28, CH = 50; // car half-width, half-length

    // Body
    ctx.shadowBlur = 12;
    ctx.shadowColor = 'rgba(14,165,233,0.4)';
    ctx.fillStyle = 'rgba(30,41,59,0.92)';
    ctx.strokeStyle = '#0ea5e9';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.roundRect(-CW, -CH, CW*2, CH*2, 6);
    ctx.fill();
    ctx.stroke();
    ctx.shadowBlur = 0;

    // Direction arrow
    ctx.fillStyle = '#f8fafc';
    ctx.beginPath();
    ctx.moveTo(0, -CH + 8);
    ctx.lineTo(-6, -CH + 20);
    ctx.lineTo(6, -CH + 20);
    ctx.closePath();
    ctx.fill();

    // --- Rear Wheels (fixed) ---
    const rearY = CH - 10;
    [[-CW - 6, rearY], [CW + 6, rearY]].forEach(([wx, wy]) => {
        ctx.save();
        ctx.translate(wx, wy);
        ctx.fillStyle = '#1e293b';
        ctx.strokeStyle = '#64748b';
        ctx.lineWidth = 1.5;
        ctx.fillRect(-5, -10, 10, 20);
        ctx.strokeRect(-5, -10, 10, 20);
        ctx.restore();
    });

    // --- Front Wheels (steerable) ---
    const frontY = -CH + 10;
    [[-CW - 6, frontY], [CW + 6, frontY]].forEach(([wx, wy]) => {
        ctx.save();
        ctx.translate(wx, wy);
        ctx.rotate(steerAngleRad); // rotate wheels with steering
        ctx.fillStyle = '#1e293b';
        ctx.strokeStyle = '#0ea5e9'; // front wheels highlighted in blue
        ctx.lineWidth = 1.5;
        ctx.fillRect(-5, -10, 10, 20);
        ctx.strokeRect(-5, -10, 10, 20);
        // Tread mark
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(0, -10); ctx.lineTo(0, 10); ctx.stroke();
        ctx.restore();
    });

    ctx.restore(); // car
}

// --- MQTT ---
function connectMQTT() {
    // Clean and normalize MAC address (strip colons, dashes, spaces and uppercase it)
    macAddress = document.getElementById('mac-input').value.replace(/[^a-fA-F0-9]/g, '').trim().toUpperCase();
    if (macAddress.length !== 12) { 
        alert("La MAC debe tener 12 caracteres hexadecimales (ej: 24:0A:C4:08:E9:D4 o 240AC408E9D4)."); 
        return; 
    }

    const btn = document.getElementById('connect-btn');
    btn.disabled = true;
    btn.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i> Conectando...';

    const base = "esp32/robot_fuzzy/" + macAddress;
    telemetryTopic = base + "/telemetry";
    commandTopic   = base + "/command";
    document.getElementById('base-topic-display').innerText = base + "/...";

    // Reset ping time on new connection to avoid immediate offline bug
    lastPingTime = null;

    const useSSL = window.location.protocol === "https:";
    const port   = useSSL ? 8884 : 8000;
    const randId = "WebAck_" + Math.random().toString(16).substr(2, 8);

    mqttClient = new Paho.MQTT.Client("broker.hivemq.com", port, randId);
    mqttClient.onConnectionLost = onConnectionLost;
    mqttClient.onMessageArrived = onMessageArrived;
    mqttClient.connect({ useSSL, timeout: 5, onSuccess: onConnectSuccess, onFailure: onConnectFailure });
}

function onConnectSuccess() {
    isConnected = true;
    const badge = document.getElementById('conn-badge');
    badge.className = "badge badge-connected";
    document.getElementById('conn-text').innerText = "ESPERANDO ESP32...";
    const btn = document.getElementById('connect-btn');
    btn.disabled = false; btn.className = "btn btn-danger";
    btn.innerHTML = '<i class="fa-solid fa-unplug"></i> Desconectar';
    mqttClient.subscribe(telemetryTopic);
    startHeartbeatTimer();
}

function onConnectFailure(err) {
    alert("No se pudo conectar al broker MQTT.");
    const btn = document.getElementById('connect-btn');
    btn.disabled = false; btn.className = "btn btn-primary";
    btn.innerHTML = '<i class="fa-solid fa-plug"></i> Conectar';
    isConnected = false;
}

function onConnectionLost(res) {
    isConnected = false;
    document.getElementById('conn-badge').className = "badge badge-disconnected";
    document.getElementById('conn-text').innerText = "DESCONECTADO";
    const btn = document.getElementById('connect-btn');
    btn.disabled = false; btn.className = "btn btn-primary";
    btn.innerHTML = '<i class="fa-solid fa-plug"></i> Conectar';
    stopHeartbeatTimer();
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

// --- MQTT Message Handler ---
function onMessageArrived(message) {
    try {
        const d = JSON.parse(message.payloadString);
        lastPingTime = new Date();
        document.getElementById('last-ping').innerText = lastPingTime.toLocaleTimeString();

        if (d.connected) {
            document.getElementById('conn-badge').className = "badge badge-connected";
            document.getElementById('conn-text').innerText = "ESP32 ONLINE";
        }

        // Traction telemetry
        const measuredSpeed = d.trac.speed;
        document.getElementById('num-target-speed').innerText = d.trac.target.toFixed(1);
        document.getElementById('num-speed-trac').innerText   = measuredSpeed.toFixed(1);
        document.getElementById('num-pwm-trac').innerText     = Math.round(d.trac.pwm);
        document.getElementById('num-pos-trac').innerText     = d.trac.pos;

        // Steering telemetry
        document.getElementById('num-target-angle').innerText = d.steer.target;
        document.getElementById('num-pos-steer').innerText    = d.steer.pos;
        document.getElementById('num-pwm-steer').innerText    = Math.round(d.steer.pwm);
        document.getElementById('num-err-steer').innerText    = Math.round(d.steer.err);

        // Chart update
        updateChart(d.trac.target, measuredSpeed, d.steer.target, d.steer.pos);

        // Fuzzy bars — Traction
        document.getElementById('val-fuzzy-err-trac').innerText  = d.trac.err.toFixed(2);
        document.getElementById('val-fuzzy-derr-trac').innerText = d.trac.derr.toFixed(2);
        d.fuzzy.trac_mu_e.forEach((v,i)  => { setBar('bar-te-'+i, 'txt-te-'+i, v); });
        d.fuzzy.trac_mu_de.forEach((v,i) => { setBar('bar-tde-'+i,'txt-tde-'+i, v); });

        // Fuzzy bars — Steering
        document.getElementById('val-fuzzy-err-steer').innerText  = d.steer.err.toFixed(2);
        document.getElementById('val-fuzzy-derr-steer').innerText = d.steer.derr.toFixed(2);
        d.fuzzy.steer_mu_e.forEach((v,i)  => { setBar('bar-se-'+i, 'txt-se-'+i, v); });
        d.fuzzy.steer_mu_de.forEach((v,i) => { setBar('bar-sde-'+i,'txt-sde-'+i, v); });

        // 2D Ackermann simulation
        updateCarKinematics(d.trac.pos, d.steer.pos, 0.25);

    } catch(e) { console.error("Telemetry parse error:", e); }
}

function setBar(barId, txtId, val) {
    document.getElementById(barId).style.width = (val * 100) + '%';
    document.getElementById(txtId).innerText = Math.round(val * 100) + '%';
}

// --- Heartbeat ---
function startHeartbeatTimer() {
    stopHeartbeatTimer();
    pingInterval = setInterval(() => {
        if (lastPingTime && (new Date() - lastPingTime) / 1000 > 4) {
            document.getElementById('conn-badge').className = "badge badge-disconnected";
            document.getElementById('conn-text').innerText = "ESP32 OFFLINE";
        }
    }, 1000);
}
function stopHeartbeatTimer() {
    if (pingInterval) { clearInterval(pingInterval); pingInterval = null; }
}

// --- Throttled command publisher ---
let sliderTimeout = null;
function throttledDrive(speed, angle) {
    if (sliderTimeout) clearTimeout(sliderTimeout);
    sliderTimeout = setTimeout(() => {
        publishCommand({ cmd: "drive", speed: parseFloat(speed), angle: parseFloat(angle) });
    }, 50);
}

// --- Auto-Discovery Client ---
function initDiscovery() {
    const useSSL = window.location.protocol === "https:";
    const port   = useSSL ? 8884 : 8000;
    const randId = "WebDiscover_" + Math.random().toString(16).substr(2, 8);

    discoveryClient = new Paho.MQTT.Client("broker.hivemq.com", port, randId);
    discoveryClient.onConnectionLost = (res) => {
        console.log("Discovery connection lost, retrying...", res);
        setTimeout(initDiscovery, 5000);
    };
    discoveryClient.onMessageArrived = (message) => {
        if (message.destinationName === "esp32/robot_fuzzy/discovery") {
            try {
                const d = JSON.parse(message.payloadString);
                if (d.mac) {
                    addDiscoveredDevice(d.mac, d.ip, d.status);
                }
            } catch (e) { console.error("Discovery parsing error:", e); }
        }
    };
    discoveryClient.connect({
        useSSL,
        timeout: 5,
        onSuccess: () => {
            console.log("Discovery connected. Listening for ESP32 pings...");
            discoveryClient.subscribe("esp32/robot_fuzzy/discovery");
        },
        onFailure: (err) => {
            console.error("Discovery connection failed, retrying...", err);
            setTimeout(initDiscovery, 5000);
        }
    });
}

function addDiscoveredDevice(mac, ip, status) {
    const listContainer = document.getElementById('detected-list');
    const section = document.getElementById('discovery-section');
    if (!listContainer || !section) return;

    const cleanMac = mac.replace(/[^a-fA-F0-9]/g, '').toUpperCase();
    if (cleanMac.length !== 12) return;

    const now = new Date();
    const formattedMac = cleanMac.match(/.{1,2}/g).join(':');

    if (discoveredDevices.has(cleanMac)) {
        const dev = discoveredDevices.get(cleanMac);
        dev.lastSeen = now;
        dev.ip = ip;
        const btn = document.getElementById(`btn-dev-${cleanMac}`);
        if (btn) {
            btn.innerHTML = `<i class="fa-solid fa-microchip"></i> ${formattedMac} <span style="font-size: 0.75rem; opacity: 0.75; font-weight: 400; padding: 2px 6px; background: rgba(255,255,255,0.08); border-radius: 4px; margin-left: 5px;">IP: ${ip || 'S/IP'}</span>`;
        }
    } else {
        section.style.display = 'block';

        const btn = document.createElement('button');
        btn.id = `btn-dev-${cleanMac}`;
        btn.className = 'btn';
        btn.style.background = 'rgba(16, 185, 129, 0.1)';
        btn.style.border = '1px solid rgba(16, 185, 129, 0.3)';
        btn.style.color = 'var(--color-green)';
        btn.style.padding = '6px 12px';
        btn.style.fontSize = '0.8rem';
        btn.style.borderRadius = '6px';
        btn.style.cursor = 'pointer';
        btn.style.display = 'inline-flex';
        btn.style.alignItems = 'center';
        btn.style.gap = '6px';
        btn.style.transition = 'all 0.2s';
        btn.innerHTML = `<i class="fa-solid fa-microchip" style="animation: pulseIcon 1.5s infinite ease-in-out;"></i> ${formattedMac} <span style="font-size: 0.75rem; opacity: 0.75; font-weight: 400; padding: 2px 6px; background: rgba(255,255,255,0.08); border-radius: 4px; margin-left: 5px;">IP: ${ip || 'S/IP'}</span>`;

        btn.addEventListener('mouseover', () => {
            btn.style.background = 'rgba(16, 185, 129, 0.2)';
            btn.style.boxShadow = '0 0 10px rgba(16, 185, 129, 0.3)';
            btn.style.borderColor = 'rgba(16, 185, 129, 0.6)';
        });
        btn.addEventListener('mouseout', () => {
            btn.style.background = 'rgba(16, 185, 129, 0.1)';
            btn.style.boxShadow = 'none';
            btn.style.borderColor = 'rgba(16, 185, 129, 0.3)';
        });

        btn.addEventListener('click', () => {
            document.getElementById('mac-input').value = formattedMac;
            // Highlight feedback
            const macInput = document.getElementById('mac-input');
            macInput.style.borderColor = 'var(--color-green)';
            setTimeout(() => { macInput.style.borderColor = ''; }, 1000);
            
            // Auto connect
            if (isConnected) {
                disconnectMQTT();
                setTimeout(connectMQTT, 250);
            } else {
                connectMQTT();
            }
        });

        listContainer.appendChild(btn);
        discoveredDevices.set(cleanMac, { ip, lastSeen: now, element: btn });
    }
}

// --- DOM Events ---
document.addEventListener("DOMContentLoaded", () => {
    initChart();
    setupCanvas();

    // Start auto-discovery background client
    initDiscovery();

    // Setup periodic cleanup for discovered devices (every 5 seconds)
    setInterval(() => {
        const now = new Date();
        discoveredDevices.forEach((dev, mac) => {
            if ((now - dev.lastSeen) / 1000 > 30) {
                dev.element.remove();
                discoveredDevices.delete(mac);
            }
        });
        const section = document.getElementById('discovery-section');
        if (section && discoveredDevices.size === 0) {
            section.style.display = 'none';
        }
    }, 5000);

    document.getElementById('connect-btn').addEventListener('click', () => {
        isConnected ? disconnectMQTT() : connectMQTT();
    });

    const slSpeed = document.getElementById('slider-speed');
    const slAngle = document.getElementById('slider-angle');
    const lbSpeed = document.getElementById('val-target-speed');
    const lbAngle = document.getElementById('val-target-angle');

    slSpeed.addEventListener('input', e => {
        lbSpeed.innerText = e.target.value;
        throttledDrive(slSpeed.value, slAngle.value);
    });
    slAngle.addEventListener('input', e => {
        lbAngle.innerText = e.target.value;
        throttledDrive(slSpeed.value, slAngle.value);
    });

    document.querySelectorAll('.preset-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            const s = btn.getAttribute('data-speed');
            const a = btn.getAttribute('data-angle');
            slSpeed.value = s; slAngle.value = a;
            lbSpeed.innerText = s; lbAngle.innerText = a;
            publishCommand({ cmd: "drive", speed: parseFloat(s), angle: parseFloat(a) });
        });
    });

    document.getElementById('btn-estop').addEventListener('click', () => {
        slSpeed.value = 0; slAngle.value = 0;
        lbSpeed.innerText = 0; lbAngle.innerText = 0;
        publishCommand({ cmd: "stop" });
    });

    document.getElementById('btn-reset').addEventListener('click', () => {
        publishCommand({ cmd: "reset" });
        odometer = 0; resetCarPose();
    });

    document.getElementById('clear-trail-btn').addEventListener('click', () => {
        pathHistory = []; drawCar();
    });

    document.getElementById('send-tuning-btn').addEventListener('click', () => {
        const ge  = parseFloat(document.getElementById('gain-ge').value);
        const gde = parseFloat(document.getElementById('gain-gde').value);
        const gu  = parseFloat(document.getElementById('gain-gu').value);
        const motor = document.getElementById('tune-motor').value;
        publishCommand({ cmd: "tune", motor, ge, gde, gu });
        alert(`Parámetros enviados al motor de ${motor === 'trac' ? 'Tracción' : 'Dirección'}\nGe=${ge}, Gde=${gde}, Gu=${gu}`);
    });
});
