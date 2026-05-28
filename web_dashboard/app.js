// --- Global State ---
let mqttClient = null;
let isConnected = false;
let macAddress = "";
let telemetryTopic = "";
let commandTopic = "";

// Keep alive / Heartbeat variables
let lastPingTime = null;
let pingInterval = null;

// Telemetry History for Chart
const maxDataPoints = 40;
let telemetryChart = null;

// --- Robot Kinematics Simulator Variables ---
const canvas = document.getElementById('robot-canvas');
const ctx = canvas.getContext('2d');
let robotX = canvas.width / 2;
let robotY = canvas.height / 2;
let robotTheta = -Math.PI / 2; // Point up initially

// Robot physical parameters (in pixels for simulation)
const wheelBase = 45; // Distance between wheels in pixels
const wheelRadius = 12;
let leftWheelAngle = 0;
let rightWheelAngle = 0;
let lastLeftPos = 0;
let lastRightPos = 0;
let pathHistory = [];

// --- Initialize Chart.js ---
function initChart() {
    const chartCtx = document.getElementById('telemetry-chart').getContext('2d');
    
    // Set custom grid color and fonts
    Chart.defaults.color = '#94a3b8';
    Chart.defaults.font.family = "'Outfit', sans-serif";

    telemetryChart = new Chart(chartCtx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                {
                    label: 'Target L',
                    data: [],
                    borderColor: '#0ea5e9',
                    borderWidth: 2,
                    borderDash: [5, 5],
                    fill: false,
                    tension: 0.3,
                    pointRadius: 0
                },
                {
                    label: 'Speed L',
                    data: [],
                    borderColor: '#0ea5e9',
                    borderWidth: 2,
                    fill: false,
                    tension: 0.3,
                    pointRadius: 0
                },
                {
                    label: 'Target R',
                    data: [],
                    borderColor: '#10b981',
                    borderWidth: 2,
                    borderDash: [5, 5],
                    fill: false,
                    tension: 0.3,
                    pointRadius: 0
                },
                {
                    label: 'Speed R',
                    data: [],
                    borderColor: '#10b981',
                    borderWidth: 2,
                    fill: false,
                    tension: 0.3,
                    pointRadius: 0
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            plugins: {
                legend: {
                    position: 'top',
                    labels: { boxWidth: 12, padding: 8, font: { size: 10 } }
                }
            },
            scales: {
                x: {
                    grid: { color: 'rgba(255, 255, 255, 0.05)' },
                    ticks: { maxRotation: 0, autoSkip: true, maxTicksLimit: 10 }
                },
                y: {
                    grid: { color: 'rgba(255, 255, 255, 0.05)' },
                    title: { display: true, text: 'Velocidad (ticks/s)', font: { size: 10 } }
                }
            },
            animation: { duration: 0 } // Disable animations for real-time performance
        }
    });
}

// --- Update Chart with New Telemetry ---
function updateChart(targetL, speedL, targetR, speedR) {
    const now = new Date();
    const timeStr = now.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });
    
    telemetryChart.data.labels.push(timeStr);
    telemetryChart.data.datasets[0].data.push(targetL);
    telemetryChart.data.datasets[1].data.push(speedL);
    telemetryChart.data.datasets[2].data.push(targetR);
    telemetryChart.data.datasets[3].data.push(speedR);
    
    if (telemetryChart.data.labels.length > maxDataPoints) {
        telemetryChart.data.labels.shift();
        telemetryChart.data.datasets[0].data.shift();
        telemetryChart.data.datasets[1].data.shift();
        telemetryChart.data.datasets[2].data.shift();
        telemetryChart.data.datasets[3].data.shift();
    }
    
    telemetryChart.update();
}

// --- 2D Robot Kinematics Draw Loop ---
function setupCanvas() {
    // Resize handler
    function resize() {
        const dpr = window.devicePixelRatio || 1;
        const rect = canvas.getBoundingClientRect();
        canvas.width = rect.width * dpr;
        canvas.height = rect.height * dpr;
        ctx.scale(dpr, dpr);
        drawRobot();
    }
    window.addEventListener('resize', resize);
    resize();
}

function updateRobotKinematics(leftPos, rightPos) {
    // Determine ticks differential
    const dLeft = leftPos - lastLeftPos;
    const dRight = rightPos - lastRightPos;
    
    lastLeftPos = leftPos;
    lastRightPos = rightPos;
    
    // Scale tick counts to physical displacement (in pixels)
    // Assume 1 tick = 0.15 pixels of movement
    const tickScale = 0.15;
    const sLeft = dLeft * tickScale;
    const sRight = dRight * tickScale;
    
    // Update wheel rotation angles for visualization (in radians)
    // 1 tick = ~0.05 radians of wheel rotation
    leftWheelAngle += dLeft * 0.05;
    rightWheelAngle += dRight * 0.05;
    
    // Differential Drive Kinematics
    const sAvg = (sLeft + sRight) / 2;
    const dTheta = (sRight - sLeft) / wheelBase;
    
    // Update robot pose
    robotX += sAvg * Math.cos(robotTheta);
    robotY += sAvg * Math.sin(robotTheta);
    robotTheta += dTheta;
    
    // Boundary wrapping to keep robot on screen
    const width = canvas.width / (window.devicePixelRatio || 1);
    const height = canvas.height / (window.devicePixelRatio || 1);
    if (robotX < 0) robotX = width;
    if (robotX > width) robotX = 0;
    if (robotY < 0) robotY = height;
    if (robotY > height) robotY = 0;
    
    // Save position to history
    pathHistory.push({x: robotX, y: robotY});
    if (pathHistory.length > 300) {
        pathHistory.shift();
    }
    
    drawRobot();
}

function drawRobot() {
    const width = canvas.width / (window.devicePixelRatio || 1);
    const height = canvas.height / (window.devicePixelRatio || 1);
    
    ctx.clearRect(0, 0, width, height);
    
    // 1. Draw Grid Lines
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.03)';
    ctx.lineWidth = 1;
    const gridSize = 40;
    for (let x = 0; x < width; x += gridSize) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();
    }
    for (let y = 0; y < height; y += gridSize) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
    }
    
    // 2. Draw Trajectory Path
    if (pathHistory.length > 1) {
        ctx.beginPath();
        ctx.strokeStyle = 'rgba(14, 165, 233, 0.4)';
        ctx.lineWidth = 2;
        ctx.setLineDash([2, 4]);
        ctx.moveTo(pathHistory[0].x, pathHistory[0].y);
        for (let i = 1; i < pathHistory.length; i++) {
            ctx.lineTo(pathHistory[i].x, pathHistory[i].y);
        }
        ctx.stroke();
        ctx.setLineDash([]); // reset
    }
    
    // 3. Draw Robot Chassis
    ctx.save();
    ctx.translate(robotX, robotY);
    ctx.rotate(robotTheta);
    
    // Body circle
    ctx.beginPath();
    ctx.arc(0, 0, 20, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(30, 41, 59, 0.9)';
    ctx.strokeStyle = '#0ea5e9';
    ctx.lineWidth = 2;
    ctx.shadowBlur = 10;
    ctx.shadowColor = 'rgba(14, 165, 233, 0.5)';
    ctx.fill();
    ctx.stroke();
    ctx.shadowBlur = 0; // reset shadow
    
    // Forward direction indicator (triangular arrow)
    ctx.beginPath();
    ctx.moveTo(12, 0);
    ctx.lineTo(-4, -6);
    ctx.lineTo(-4, 6);
    ctx.closePath();
    ctx.fillStyle = '#f8fafc';
    ctx.fill();
    
    // 4. Draw Left Wheel (Top side relative to heading)
    ctx.save();
    ctx.translate(0, -wheelBase/2);
    // Draw wheel box
    ctx.fillStyle = '#1e293b';
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 1.5;
    ctx.fillRect(-8, -4, 16, 8);
    ctx.strokeRect(-8, -4, 16, 8);
    // Draw tread line based on rotation to show movement
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 2;
    ctx.beginPath();
    const lxOffset = Math.sin(leftWheelAngle) * 5;
    ctx.moveTo(lxOffset, -4);
    ctx.lineTo(lxOffset, 4);
    ctx.stroke();
    ctx.restore();
    
    // 5. Draw Right Wheel (Bottom side relative to heading)
    ctx.save();
    ctx.translate(0, wheelBase/2);
    ctx.fillStyle = '#1e293b';
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 1.5;
    ctx.fillRect(-8, -4, 16, 8);
    ctx.strokeRect(-8, -4, 16, 8);
    // Draw tread line based on rotation to show movement
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 2;
    ctx.beginPath();
    const rxOffset = Math.sin(rightWheelAngle) * 5;
    ctx.moveTo(rxOffset, -4);
    ctx.lineTo(rxOffset, 4);
    ctx.stroke();
    ctx.restore();
    
    ctx.restore();
}

// --- MQTT Connection Management ---
function connectMQTT() {
    macAddress = document.getElementById('mac-input').value.trim().toUpperCase();
    if (macAddress.length !== 12) {
        alert("La dirección MAC debe tener exactamente 12 caracteres hexadecimales (sin dos puntos).");
        return;
    }
    
    const connectBtn = document.getElementById('connect-btn');
    connectBtn.disabled = true;
    connectBtn.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i> Conectando...';

    // Construct Topics
    const baseTopic = "esp32/robot_fuzzy/" + macAddress;
    telemetryTopic = baseTopic + "/telemetry";
    commandTopic = baseTopic + "/command";
    
    document.getElementById('base-topic-display').innerText = baseTopic + "/...";

    // Select port and SSL based on page protocol
    const useSSL = window.location.protocol === "https:";
    const port = useSSL ? 8884 : 8000;
    const host = "broker.hivemq.com";
    const randId = "WebClient_" + Math.random().toString(16).substr(2, 8);

    mqttClient = new Paho.MQTT.Client(host, port, randId);
    
    // Set callbacks
    mqttClient.onConnectionLost = onConnectionLost;
    mqttClient.onMessageArrived = onMessageArrived;

    const options = {
        useSSL: useSSL,
        timeout: 5,
        onSuccess: onConnectSuccess,
        onFailure: onConnectFailure
    };

    console.log(`Connecting to ${host}:${port} using SSL=${useSSL}...`);
    mqttClient.connect(options);
}

function onConnectSuccess() {
    console.log("MQTT Connected!");
    isConnected = true;
    
    // Update Badge
    const badge = document.getElementById('conn-badge');
    badge.className = "badge badge-connected";
    document.getElementById('conn-text').innerText = "CONECTADO AL BROKER";
    
    const connectBtn = document.getElementById('connect-btn');
    connectBtn.disabled = false;
    connectBtn.className = "btn btn-danger";
    connectBtn.innerHTML = '<i class="fa-solid fa-unplug"></i> Desconectar';
    
    // Subscribe to telemetry
    mqttClient.subscribe(telemetryTopic);
    console.log("Subscribed to " + telemetryTopic);

    // Start watching for ESP32 connection timeouts (heartbeat)
    startHeartbeatTimer();
}

function onConnectFailure(err) {
    console.error("MQTT Connection failed:", err);
    alert("No se pudo conectar al broker MQTT. Revisa tu conexión a internet.");
    
    const connectBtn = document.getElementById('connect-btn');
    connectBtn.disabled = false;
    connectBtn.className = "btn btn-primary";
    connectBtn.innerHTML = '<i class="fa-solid fa-plug"></i> Conectar';
    
    isConnected = false;
}

function onConnectionLost(responseObject) {
    console.warn("MQTT Connection lost:", responseObject.errorMessage);
    
    // Update UI
    const badge = document.getElementById('conn-badge');
    badge.className = "badge badge-disconnected";
    document.getElementById('conn-text').innerText = "DESCONECTADO";
    
    const connectBtn = document.getElementById('connect-btn');
    connectBtn.disabled = false;
    connectBtn.className = "btn btn-primary";
    connectBtn.innerHTML = '<i class="fa-solid fa-plug"></i> Conectar';
    
    isConnected = false;
    stopHeartbeatTimer();
}

function disconnectMQTT() {
    if (mqttClient && isConnected) {
        mqttClient.disconnect();
    }
}

// --- Send Commands to ESP32 ---
function publishCommand(commandObj) {
    if (!mqttClient || !isConnected) return;
    
    const messageStr = JSON.stringify(commandObj);
    const message = new Paho.MQTT.Message(messageStr);
    message.destinationName = commandTopic;
    mqttClient.send(message);
}

// --- MQTT On Message Arrived ---
function onMessageArrived(message) {
    const payload = message.payloadString;
    
    try {
        const data = JSON.parse(payload);
        
        // 1. Update Connection state & Last Seen
        lastPingTime = new Date();
        document.getElementById('last-ping').innerText = lastPingTime.toLocaleTimeString();
        
        const badge = document.getElementById('conn-badge');
        if (data.connected) {
            badge.className = "badge badge-connected";
            document.getElementById('conn-text').innerText = "ESP32 ONLINE";
        }

        // 2. Numerical Values
        document.getElementById('num-target-left').innerText = data.left.target.toFixed(1);
        document.getElementById('num-speed-left').innerText = data.left.speed.toFixed(1);
        document.getElementById('num-pwm-left').innerText = Math.round(data.left.pwm);
        document.getElementById('num-pos-left').innerText = data.left.pos;

        document.getElementById('num-target-right').innerText = data.right.target.toFixed(1);
        document.getElementById('num-speed-right').innerText = data.right.speed.toFixed(1);
        document.getElementById('num-pwm-right').innerText = Math.round(data.right.pwm);
        document.getElementById('num-pos-right').innerText = data.right.pos;

        // 3. Update Chart
        updateChart(
            data.left.target, 
            data.left.speed, 
            data.right.target, 
            data.right.speed
        );

        // 4. Update Fuzzy Inspector Bars
        // Left
        document.getElementById('val-fuzzy-err-left').innerText = data.left.err.toFixed(2);
        document.getElementById('val-fuzzy-derr-left').innerText = data.left.derr.toFixed(2);
        for(let i=0; i<5; i++) {
            const val = data.fuzzy.left_mu_e[i];
            document.getElementById(`bar-le-${i}`).style.width = (val * 100) + '%';
            document.getElementById(`txt-le-${i}`).innerText = Math.round(val * 100) + '%';
        }
        for(let i=0; i<3; i++) {
            const val = data.fuzzy.left_mu_de[i];
            document.getElementById(`bar-lde-${i}`).style.width = (val * 100) + '%';
            document.getElementById(`txt-lde-${i}`).innerText = Math.round(val * 100) + '%';
        }

        // Right
        document.getElementById('val-fuzzy-err-right').innerText = data.right.err.toFixed(2);
        document.getElementById('val-fuzzy-derr-right').innerText = data.right.derr.toFixed(2);
        for(let i=0; i<5; i++) {
            const val = data.fuzzy.right_mu_e[i];
            document.getElementById(`bar-re-${i}`).style.width = (val * 100) + '%';
            document.getElementById(`txt-re-${i}`).innerText = Math.round(val * 100) + '%';
        }
        for(let i=0; i<3; i++) {
            const val = data.fuzzy.right_mu_de[i];
            document.getElementById(`bar-rde-${i}`).style.width = (val * 100) + '%';
            document.getElementById(`txt-rde-${i}`).innerText = Math.round(val * 100) + '%';
        }

        // 5. Update 2D Simulation Kinematics
        updateRobotKinematics(data.left.pos, data.right.pos);

    } catch (e) {
        console.error("Error parsing telemetry JSON:", e);
    }
}

// --- Heartbeat / Timeout Detection ---
function startHeartbeatTimer() {
    stopHeartbeatTimer();
    pingInterval = setInterval(() => {
        if (lastPingTime) {
            const elapsedSeconds = (new Date() - lastPingTime) / 1000;
            if (elapsedSeconds > 4.0) { // If no message for 4 seconds, mark ESP32 offline
                const badge = document.getElementById('conn-badge');
                badge.className = "badge badge-disconnected";
                document.getElementById('conn-text').innerText = "ESP32 OFFLINE";
            }
        }
    }, 1000);
}

function stopHeartbeatTimer() {
    if (pingInterval) {
        clearInterval(pingInterval);
        pingInterval = null;
    }
}

// --- Throttled command publisher for sliders ---
let sliderPublishTimeout = null;
function throttledSpeedPublish(left, right) {
    if (sliderPublishTimeout) clearTimeout(sliderPublishTimeout);
    sliderPublishTimeout = setTimeout(() => {
        publishCommand({ cmd: "drive", left: parseFloat(left), right: parseFloat(right) });
    }, 50); // 50ms throttle delay
}

// --- DOM Event Listeners ---
document.addEventListener("DOMContentLoaded", () => {
    initChart();
    setupCanvas();
    
    // Connect / Disconnect button
    const connectBtn = document.getElementById('connect-btn');
    connectBtn.addEventListener('click', () => {
        if (!isConnected) {
            connectMQTT();
        } else {
            disconnectMQTT();
        }
    });

    // Speed Sliders input events
    const sliderLeft = document.getElementById('slider-left');
    const sliderRight = document.getElementById('slider-right');
    const labelLeft = document.getElementById('val-target-left');
    const labelRight = document.getElementById('val-target-right');

    sliderLeft.addEventListener('input', (e) => {
        labelLeft.innerText = e.target.value;
        throttledSpeedPublish(sliderLeft.value, sliderRight.value);
    });

    sliderRight.addEventListener('input', (e) => {
        labelRight.innerText = e.target.value;
        throttledSpeedPublish(sliderLeft.value, sliderRight.value);
    });

    // Preset Speed Buttons
    document.querySelectorAll('.preset-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            const leftVal = btn.getAttribute('data-left');
            const rightVal = btn.getAttribute('data-right');
            
            sliderLeft.value = leftVal;
            sliderRight.value = rightVal;
            labelLeft.innerText = leftVal;
            labelRight.innerText = rightVal;
            
            publishCommand({ cmd: "drive", left: parseFloat(leftVal), right: parseFloat(rightVal) });
        });
    });

    // Emergency Stop
    document.getElementById('btn-estop').addEventListener('click', () => {
        sliderLeft.value = 0;
        sliderRight.value = 0;
        labelLeft.innerText = 0;
        labelRight.innerText = 0;
        publishCommand({ cmd: "stop" });
    });

    // Reset Encoders & Trails
    document.getElementById('btn-reset').addEventListener('click', () => {
        publishCommand({ cmd: "reset" });
        // Clear local simulator trails and reset positioning
        pathHistory = [];
        const width = canvas.width / (window.devicePixelRatio || 1);
        const height = canvas.height / (window.devicePixelRatio || 1);
        robotX = width / 2;
        robotY = height / 2;
        robotTheta = -Math.PI / 2;
        lastLeftPos = 0;
        lastRightPos = 0;
        leftWheelAngle = 0;
        rightWheelAngle = 0;
        drawRobot();
    });

    // Clear Trail Only button
    document.getElementById('clear-trail-btn').addEventListener('click', () => {
        pathHistory = [];
        drawRobot();
    });

    // Send Fuzzy Parameters
    document.getElementById('send-tuning-btn').addEventListener('click', () => {
        const ge = parseFloat(document.getElementById('gain-ge').value);
        const gde = parseFloat(document.getElementById('gain-gde').value);
        const gu = parseFloat(document.getElementById('gain-gu').value);

        if (isNaN(ge) || isNaN(gde) || isNaN(gu)) {
            alert("Por favor ingresa números válidos para las ganancias.");
            return;
        }

        publishCommand({ cmd: "tune", ge: ge, gde: gde, gu: gu });
        alert(`Nuevos parámetros enviados!\nGe: ${ge}, Gde: ${gde}, Gu: ${gu}`);
    });
});
