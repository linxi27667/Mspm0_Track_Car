"use strict";

const MAX_CAR_LOGS = 500;

const socket = io();

socket.on("connect", () => {
    console.log("[WS] connected");
    socket.emit("get_status");
});

socket.on("disconnect", () => {
    console.log("[WS] disconnected");
});

let frameCount = 0;
let lastTelemetryReceivedAt = null;
let currentTrackState = "??";
const motorParamNames = new Set(["MKP", "MKI", "MKD", "MSL", "MSR"]);

const stateMap = {
    ID: "空闲",
    LF: "循迹",
    RA: "直角弯",
    LL: "丢线",
    SL: "起点",
    CR: "十字",
    RT: "原地旋转",
    HH: "航向保持",
    MT: "电机测试",
    FN: "完成",
};

function setTelemetryText(id, value, digits = null) {
    const element = document.getElementById(id);
    if (!element) return;
    if (typeof value === "number" && Number.isFinite(value) && digits !== null) {
        element.textContent = value.toFixed(digits);
        return;
    }
    element.textContent = value ?? "--";
}

function updateMotorParamAvailability(state) {
    const allowed = ["ID", "FN", "MT"].includes(state);
    motorParamNames.forEach((name) => {
        document.querySelectorAll(`[data-name="${name}"]`).forEach((element) => {
            element.disabled = !allowed;
        });
    });
}

function updateMotionData(tele) {
    const section = document.getElementById("tele-motion-section");
    if (["tg", "ta", "te", "rr"].every((key) => key in tele)) {
        section.hidden = false;
        setTelemetryText("tele-motion-title", "原地旋转");
        setTelemetryText("tele-motion-target", tele.tg, 2);
        setTelemetryText("tele-motion-actual", tele.ta, 2);
        setTelemetryText("tele-motion-error", tele.te, 2);
        setTelemetryText("tele-motion-result", tele.rr);
        return;
    }
    if (["hh", "ht", "ha", "he", "hr"].every((key) => key in tele)) {
        section.hidden = false;
        setTelemetryText("tele-motion-title", "航向保持");
        setTelemetryText("tele-motion-target", tele.ht, 2);
        setTelemetryText("tele-motion-actual", tele.ha, 2);
        setTelemetryText("tele-motion-error", tele.he, 2);
        setTelemetryText("tele-motion-result", tele.hr);
        return;
    }
    section.hidden = true;
}

let motorTestRunning = false;
let motorTestPending = false;

function setMotorTestStatus(text, state = "idle") {
    const status = document.getElementById("motor-test-status");
    const startButton = document.getElementById("btn-motor-test");
    if (!status || !startButton) return;
    status.textContent = text;
    status.className = `rotate-status is-${state}`;
    startButton.disabled = state === "pending" || state === "running";
}

function updateMotorTestData(tele) {
    if (tele.st === "MT" && tele.mt === true) {
        motorTestRunning = true;
        motorTestPending = false;
        const mode = tele.mph < 3 ? `开环正转第${tele.mph + 1}档`
            : (tele.mph < 6 ? `闭环正转第${tele.mph - 2}档`
            : (tele.mph < 9 ? `闭环反转第${tele.mph - 5}档` : "停车残留检查"));
        setMotorTestStatus(
            `${mode} 目标${tele.mtg} PWM L${tele.mlp}/R${tele.mrp} 编码 L${tele.el}/R${tele.er}`,
            "running",
        );
        return;
    }
    if (motorTestRunning) {
        motorTestRunning = false;
        setMotorTestStatus("测试已结束，请查看 MOTORTEST 日志", "complete");
    }
}

socket.on("telemetry", (tele) => {
    frameCount += 1;
    document.getElementById("frame-counter").textContent = `帧: ${frameCount}`;

    lastTelemetryReceivedAt = Date.now();
    currentTrackState = tele.st;
    updateMotorParamAvailability(currentTrackState);
    setTelemetryText("tele-state-code", tele.st);
    setTelemetryText("tele-state-name", stateMap[tele.st] || "未知状态");
    setTelemetryText("tele-seq", tele.seq);
    setTelemetryText("tele-ms", tele.ms);
    setTelemetryText("tele-received", new Date(lastTelemetryReceivedAt).toLocaleTimeString());
    setTelemetryText("tele-bc", tele.bc);
    setTelemetryText("tele-lm", `0x${Number(tele.lm).toString(16).toUpperCase().padStart(3, "0")}`);
    setTelemetryText("tele-sa", tele.sa);
    setTelemetryText("tele-se", tele.se);
    setTelemetryText("tele-bp", tele.bp === 1 ? "1=黑" : "0=黑");
    setTelemetryText("tele-drop", tele.drop);

    (tele.line || []).forEach((value, index) => {
        setTelemetryText(`tele-line-${index}`, value);
    });

    setTelemetryText("tele-gz", tele.gz, 2);
    setTelemetryText("tele-le", tele.le, 2);
    setTelemetryText("tele-po", tele.po, 2);
    setTelemetryText("tele-el", tele.el);
    setTelemetryText("tele-er", tele.er);
    setTelemetryText("tele-kp", tele.kp, 3);
    setTelemetryText("tele-ki", tele.ki, 3);
    setTelemetryText("tele-kd", tele.kd, 3);
    setTelemetryText("tele-gd", tele.gd, 3);
    setTelemetryText("tele-vb", tele.vb);
    setTelemetryText("tele-vs", tele.vs);
    setTelemetryText("tele-vc", tele.vc);
    setTelemetryText("tele-rt", tele.rt, 3);
    updateMotionData(tele);
    updateMotorTestData(tele);
    document.getElementById("telemetry-raw").textContent = JSON.stringify(tele, null, 2);

    updateLinkHealth(tele);
});

setInterval(() => {
    const ageElement = document.getElementById("telemetry-age");
    if (!ageElement) {
        return;
    }
    if (lastTelemetryReceivedAt === null) {
        ageElement.textContent = "等待数据";
        return;
    }
    const ageMs = Date.now() - lastTelemetryReceivedAt;
    ageElement.textContent = `更新于 ${ageMs} ms 前`;
    ageElement.classList.toggle("is-stale", ageMs > 500);
}, 250);

socket.on("client_status", (data) => {
    const el = document.getElementById("car-status");
    if (data.connected) {
        el.textContent = `小车: 已连接 ${data.addr}`;
        el.className = "status-pill status-connected";
        document.getElementById("telemetry-source").textContent = `TCP ${data.addr}`;
    } else {
        el.textContent = "小车: 未连接";
        el.className = "status-pill status-disconnected";
        document.getElementById("telemetry-source").textContent = "TCP 未连接";
    }
});

socket.on("status", (data) => {
    document.getElementById("engine-info").textContent = `引擎: ${data.engine}`;
    if (typeof data.frame_count === "number" && data.frame_count > frameCount) {
        document.getElementById("frame-counter").textContent = `帧: ${data.frame_count}`;
    }

    // 请求待确认状态
    if (typeof data.request_pending === "boolean") {
        const pill = document.getElementById("link-health");
        if (data.request_pending) {
            pill.classList.add("link-warn");
        }
    }
});

document.querySelectorAll(".param-slider").forEach((slider) => {
    const name = slider.dataset.name;
    const input = document.querySelector(`.param-input[data-name="${name}"]`);
    slider.addEventListener("input", () => {
        input.value = slider.value;
    });
    input.addEventListener("input", () => {
        slider.value = input.value;
    });
});

function collectParams() {
    const params = {};
    document.querySelectorAll(".param-input").forEach((input) => {
        if (motorParamNames.has(input.dataset.name) &&
            !["ID", "FN", "MT"].includes(currentTrackState)) return;
        const value = Number.parseFloat(input.value);
        if (!Number.isNaN(value)) params[input.dataset.name] = value;
    });
    return params;
}

document.getElementById("btn-apply").addEventListener("click", () => {
    const params = collectParams();
    socket.emit("send_params", params);
    console.log("[SEND] params:", params);
});

function sendCmd(cmd) {
    if (["start", "stop"].includes(cmd) && !confirm(`确认发送指令: ${cmd}?`)) return;
    socket.emit("send_cmd", { cmd });
    console.log("[SEND] cmd:", cmd);
}

document.getElementById("btn-start").addEventListener("click", () => sendCmd("start"));
document.getElementById("btn-stop").addEventListener("click", () => sendCmd("stop"));
document.getElementById("btn-reconnect").addEventListener("click", () => sendCmd("reconnect"));

document.getElementById("btn-motor-test").addEventListener("click", () => {
    const warning = "确认以下条件全部满足：\n\n1. 两个驱动轮完全架空\n2. 车身已经固定\n3. 手和线缆远离车轮\n4. 激光模块已关闭\n\n开始约4.4秒的电机分档测试？";
    if (!confirm(warning)) return;
    motorTestPending = true;
    setMotorTestStatus("等待固件确认", "pending");
    socket.emit("send_motor_test", { lifted: true });
});

document.getElementById("btn-motor-test-stop").addEventListener("click", () => {
    sendCmd("stop");
});

const rotateControls = [
    document.getElementById("btn-rotate-left"),
    document.getElementById("btn-rotate-right"),
    document.getElementById("btn-rotate-custom"),
];
const rotateStatus = document.getElementById("rotate-status");
const rotateInput = document.getElementById("rotate-deg");
const headingHoldButton = document.getElementById("btn-heading-hold");
const headingHoldStatus = document.getElementById("heading-hold-status");
let headingHoldActive = false;

function setRotateControlsDisabled(disabled) {
    rotateControls.forEach((button) => { button.disabled = disabled; });
}

function setRotateStatus(text, state = "idle") {
    rotateStatus.textContent = text;
    rotateStatus.className = `rotate-status is-${state}`;
}

function requestRotate(degrees) {
    if (!Number.isFinite(degrees) || degrees === 0 || Math.abs(degrees) > 180) {
        setRotateStatus("角度必须在 -180~180 且不能为 0", "fault");
        return;
    }
    const direction = degrees > 0 ? "左转（逆时针）" : "右转（顺时针）";
    if (!confirm(`确认原地${direction} ${Math.abs(degrees)}°？`)) return;
    setRotateControlsDisabled(true);
    setRotateStatus(`等待确认 ${degrees.toFixed(1)}°`, "pending");
    socket.emit("send_rotate", { deg: degrees });
}

document.getElementById("btn-rotate-left").addEventListener("click", () => requestRotate(90));
document.getElementById("btn-rotate-right").addEventListener("click", () => requestRotate(-90));
document.getElementById("btn-rotate-custom").addEventListener("click", () => {
    requestRotate(Number.parseFloat(rotateInput.value));
});

function setHeadingHoldStatus(text, active = false, pending = false) {
    headingHoldActive = active;
    headingHoldButton.disabled = pending;
    headingHoldButton.textContent = active ? "停止航向保持" : "保持当前角度";
    headingHoldStatus.textContent = text;
    headingHoldStatus.className = `rotate-status is-${pending ? "pending" : active ? "running" : "idle"}`;
}

headingHoldButton.addEventListener("click", () => {
    if (headingHoldActive) {
        setHeadingHoldStatus("正在关闭", true, true);
        sendCmd("heading_hold_off");
        setTimeout(() => setHeadingHoldStatus("已关闭"), 250);
        return;
    }
    if (!confirm("确认保持当前车身角度？")) return;
    setHeadingHoldStatus("等待确认", false, true);
    sendCmd("heading_hold_on");
});

socket.on("heading_hold_status", (data) => {
    const target = Number(data.target_deg).toFixed(1);
    const actual = Number(data.actual_deg).toFixed(1);
    const error = Number(data.error_deg).toFixed(1);
    setHeadingHoldStatus(`保持中 目标${target}° 当前${actual}° 误差${error}°`, data.active === true);
});

socket.on("rotate_status", (data) => {
    const state = data.state || "terminal";
    const result = data.result || "unknown";
    if (state === "running") {
        const target = Number(data.target_deg).toFixed(1);
        const actual = Number(data.actual_deg).toFixed(1);
        const error = Number(data.error_deg).toFixed(1);
        setRotateStatus(`执行中 目标${target}° 实际${actual}° 误差${error}°`, "running");
        return;
    }
    setRotateControlsDisabled(false);
    if (state === "rejected") {
        setRotateStatus(`已拒绝: ${result}`, "fault");
    } else {
        setRotateStatus(`结束: ${result}`, result === "complete" ? "complete" : "fault");
    }
});

socket.on("request_status", (data) => {
    if (data.state === "pending" && data.cmd === "motor_test") {
        motorTestPending = true;
        setMotorTestStatus("等待固件确认", "pending");
        return;
    }
    if (data.state === "pending" && data.cmd === "rotate") return;
    if (data.state === "rejected" || data.state === "timeout") {
        if (motorTestPending) {
            motorTestPending = false;
            const motorDetail = data.message || data.error || "unknown";
            setMotorTestStatus(`测试请求失败: ${motorDetail}`, "fault");
        }
        setRotateControlsDisabled(false);
        const detail = data.message || data.error || "unknown";
        setRotateStatus(data.state === "timeout" ? `请求超时: ${detail}` : `已拒绝: ${detail}`, "fault");
        if (headingHoldButton.disabled) {
            setHeadingHoldStatus(data.state === "timeout" ? "航向保持请求超时" : `航向保持被拒绝: ${detail}`);
        }
    }
});

let lastSuggestion = null;

document.getElementById("btn-analyze").addEventListener("click", () => {
    const btn = document.getElementById("btn-analyze");
    btn.disabled = true;
    btn.textContent = "分析中";
    socket.emit("ai_analyze");
});

socket.on("ai_result", (data) => {
    const btn = document.getElementById("btn-analyze");
    const box = document.getElementById("ai-result");
    btn.disabled = false;
    btn.textContent = "分析";

    if (data.error) {
        box.innerHTML = `<p class="error">错误: ${data.error}</p>`;
        lastSuggestion = null;
        document.getElementById("btn-adopt").disabled = true;
        return;
    }

    lastSuggestion = data;
    const params = data.params || {};
    const paramsText = Object.keys(params).length ? JSON.stringify(params, null, 2) : "无需调整";
    box.innerHTML = `
        <div class="params">建议参数:<pre></pre></div>
        <div class="reason"></div>
        <div class="confidence"></div>
    `;
    box.querySelector("pre").textContent = paramsText;
    box.querySelector(".reason").textContent = data.reason || "";
    box.querySelector(".confidence").textContent =
        `置信度: ${(data.confidence || 0).toFixed(2)} | 引擎: ${data.engine || "?"}`;
    document.getElementById("btn-adopt").disabled = Object.keys(params).length === 0;
});

document.getElementById("btn-adopt").addEventListener("click", () => {
    if (!lastSuggestion || !lastSuggestion.params) return;
    for (const [name, value] of Object.entries(lastSuggestion.params)) {
        const input = document.querySelector(`.param-input[data-name="${name}"]`);
        const slider = document.querySelector(`.param-slider[data-name="${name}"]`);
        if (input) input.value = value;
        if (slider) slider.value = value;
    }
});

socket.on("params_update", (params) => {
    for (const [name, value] of Object.entries(params)) {
        const input = document.querySelector(`.param-input[data-name="${name}"]`);
        const slider = document.querySelector(`.param-slider[data-name="${name}"]`);
        if (input) input.value = value;
        if (slider) slider.value = value;
    }
});

socket.on("log", (entry) => {
    const box = document.getElementById("log-box");
    const div = document.createElement("div");
    div.className = `log-${entry.level}`;
    div.textContent = `[${entry.time}] ${entry.msg}`;
    box.appendChild(div);
    box.scrollTop = box.scrollHeight;
    while (box.children.length > 200) box.removeChild(box.firstChild);
});

const carLogBox = document.getElementById("car-log-box");
const carLogCount = document.getElementById("car-log-count");
const carLogFilter = document.getElementById("car-log-filter");
const carLogAutoscroll = document.getElementById("car-log-autoscroll");
const carLogPauseBtn = document.getElementById("btn-car-log-pause");
const carLogClearBtn = document.getElementById("btn-car-log-clear");
const carLogExportBtn = document.getElementById("btn-car-log-export");

let carLogs = [];
let carLogPaused = false;

function normalizeLogLevel(level, line) {
    const textLevel = String(level || "").toLowerCase();
    if (["error", "assert"].includes(textLevel)) return "error";
    if (["warning", "warn"].includes(textLevel)) return "warning";
    if (["debug", "trace", "verbose"].includes(textLevel)) return "debug";
    if (/\b(error|err|fail|fault)\b/i.test(line) || /^E\//.test(line)) return "error";
    if (/\b(warn|warning)\b/i.test(line) || /^W\//.test(line)) return "warning";
    if (/\b(debug|trace)\b/i.test(line) || /^D\//.test(line)) return "debug";
    return "info";
}

function appendCarLog(entry) {
    const line = entry.line || entry.msg || "";
    const message = entry.msg || line;
    const time = entry.time || new Date().toLocaleTimeString("zh-CN", { hour12: false });
    const level = normalizeLogLevel(entry.level, line);
    const tag = entry.tag || "";
    const mcuTime = entry.mcu_time || "";
    const row = document.createElement("div");
    row.className = `log-line level-${level}`;
    row.dataset.raw = `${time} ${entry.level || level} ${tag} ${mcuTime} ${line}`.toLowerCase();

    const timeEl = document.createElement("span");
    timeEl.className = "log-time";
    timeEl.textContent = time;

    const levelEl = document.createElement("span");
    levelEl.className = "log-level";
    levelEl.textContent = (entry.level || level).toUpperCase().slice(0, 4);

    const tagEl = document.createElement("span");
    tagEl.className = "log-tag";
    tagEl.textContent = tag || "-";

    const mcuEl = document.createElement("span");
    mcuEl.className = "log-mcu";
    mcuEl.textContent = mcuTime ? `${mcuTime}ms` : "-";

    const msgEl = document.createElement("span");
    msgEl.className = "log-message";
    msgEl.textContent = message;

    row.append(timeEl, levelEl, tagEl, mcuEl, msgEl);
    carLogBox.appendChild(row);
}

function applyCarLogFilter() {
    const needle = carLogFilter.value.trim().toLowerCase();
    [...carLogBox.children].forEach((row) => {
        row.classList.toggle("is-hidden", Boolean(needle) && !row.dataset.raw.includes(needle));
    });
}

function renderCarLogs() {
    carLogBox.replaceChildren();
    carLogs.forEach(appendCarLog);
    applyCarLogFilter();
    carLogCount.textContent = `${carLogs.length} 行`;
    if (carLogAutoscroll.checked) carLogBox.scrollTop = carLogBox.scrollHeight;
}

socket.on("car_log", (entry) => {
    carLogs.push(entry);
    if (carLogs.length > MAX_CAR_LOGS) carLogs.shift();
    carLogCount.textContent = `${carLogs.length} 行`;
    if (!carLogPaused) {
        appendCarLog(entry);
        while (carLogBox.children.length > MAX_CAR_LOGS) {
            carLogBox.removeChild(carLogBox.firstChild);
        }
        applyCarLogFilter();
        if (carLogAutoscroll.checked) carLogBox.scrollTop = carLogBox.scrollHeight;
    }
});

carLogFilter.addEventListener("input", applyCarLogFilter);

carLogPauseBtn.addEventListener("click", () => {
    carLogPaused = !carLogPaused;
    carLogPauseBtn.textContent = carLogPaused ? "继续" : "暂停";
    if (!carLogPaused) renderCarLogs();
});

carLogClearBtn.addEventListener("click", () => {
    carLogs = [];
    renderCarLogs();
});

carLogExportBtn.addEventListener("click", () => {
    const text = carLogs.map((entry) => `[${entry.time || ""}] ${entry.line || ""}`).join("\n");
    const blob = new Blob([text], { type: "text/plain;charset=utf-8" });
    const link = document.createElement("a");
    link.href = URL.createObjectURL(blob);
    link.download = `esp01s-log-${new Date().toISOString().replace(/[:.]/g, "-")}.txt`;
    link.click();
    URL.revokeObjectURL(link.href);
});

const PRESETS = {
    conservative: {
        KP: 3.5, KI: 0.0, KD: 1.5, GDAMP: 0.25,
        VBASE: 28, VSTR: 30, VCUR: 26, RATHR: 4.0,
    },
    balanced: {
        KP: 5.0, KI: 0.0, KD: 1.0, GDAMP: 0.15,
        VBASE: 40, VSTR: 50, VCUR: 30, RATHR: 4.0,
    },
    aggressive: {
        KP: 8.0, KI: 0.0, KD: 0.5, GDAMP: 0.10,
        VBASE: 60, VSTR: 70, VCUR: 45, RATHR: 4.0,
    },
};

document.querySelectorAll(".btn-preset").forEach((btn) => {
    btn.addEventListener("click", () => {
        const params = PRESETS[btn.dataset.preset];
        if (!params) return;
        if (!confirm(`加载 ${btn.textContent} 预设？参数只填入面板，不自动下发。`)) return;
        for (const [name, value] of Object.entries(params)) {
            const slider = document.querySelector(`.param-slider[data-name="${name}"]`);
            const input = document.querySelector(`.param-input[data-name="${name}"]`);
            if (slider) slider.value = value;
            if (input) input.value = value;
        }
    });
});

const btnAutoStart = document.getElementById("btn-auto-start");
const btnAutoStop = document.getElementById("btn-auto-stop");
const autoStatusEl = document.getElementById("auto-status");
const autoRoundsLog = document.getElementById("auto-rounds-log");

btnAutoStart.addEventListener("click", () => {
    const interval = Number.parseFloat(document.getElementById("auto-interval").value) || 5;
    const maxRounds = Number.parseInt(document.getElementById("auto-rounds").value, 10) || 10;
    if (!confirm(`启动自动调参？AI 将每 ${interval} 秒分析并下发参数。`)) return;
    socket.emit("auto_tuning_start", { interval, max_rounds: maxRounds });
    btnAutoStart.disabled = true;
    btnAutoStop.disabled = false;
    autoRoundsLog.replaceChildren();
});

btnAutoStop.addEventListener("click", () => {
    socket.emit("auto_tuning_stop");
});

socket.on("auto_tuning_status", (data) => {
    if (data.active) {
        autoStatusEl.innerHTML = `<span class="status-active">第${data.rounds}轮 ${data.status || "running"}</span>`;
        btnAutoStart.disabled = true;
        btnAutoStop.disabled = false;
        return;
    }

    let summary = `<span class="status-done">已结束（共${data.rounds}轮）</span>`;
    if (data.snapshots && data.snapshots.length > 0) {
        summary += `<div class="snapshot-summary">`;
        data.snapshots.forEach((snapshot) => {
            const params = JSON.stringify(snapshot.params_suggested || {});
            summary += `<div class="snapshot-row">R${snapshot.round}: ${params} (conf=${snapshot.confidence.toFixed(2)})</div>`;
        });
        summary += `</div>`;
    }
    autoStatusEl.innerHTML = summary;
    btnAutoStart.disabled = false;
    btnAutoStop.disabled = true;
});

socket.on("auto_tuning_round", (snapshot) => {
    const div = document.createElement("div");
    div.className = "round-entry";
    const params = JSON.stringify(snapshot.params_suggested || {});
    div.innerHTML =
        `<span class="round-num">R${snapshot.round}</span> ` +
        `<span class="round-params"></span> ` +
        `<span class="round-conf">conf=${snapshot.confidence.toFixed(2)}</span>`;
    div.querySelector(".round-params").textContent = params || "无需调整";
    autoRoundsLog.appendChild(div);
    autoRoundsLog.scrollTop = autoRoundsLog.scrollHeight;
    while (autoRoundsLog.children.length > 50) autoRoundsLog.removeChild(autoRoundsLog.firstChild);
});

// ============================================================
// 链路健康追踪
// ============================================================

let lastSeq = null;
let lastMs = null;
let lastFrameTime = null;
let dropCount = 0;

function updateLinkHealth(tele) {
    const linkEl = document.getElementById("link-health");
    const seq = tele.seq;
    const ms = tele.ms;

    // 帧间隔（浏览器侧）
    const now = Date.now();
    let intervalMs = 0;
    if (lastFrameTime) intervalMs = now - lastFrameTime;
    lastFrameTime = now;

    // 丢帧统计
    if (typeof seq === "number" && lastSeq !== null) {
        const expected = (lastSeq + 1) & 0xFFFF;
        if (seq !== expected) {
            const lost = ((seq - expected + 0x10000) & 0xFFFF);
            dropCount += lost;
        }
    }
    if (typeof seq === "number") lastSeq = seq;

    // MCU 帧年龄（浏览器接收时 ms 可能陈旧）
    let ageMs = 0;
    if (typeof ms === "number") {
        if (lastMs !== null) {
            // 检测 MCU 时间回退（重启）
            if (ms < lastMs) lastMs = ms;
        }
        lastMs = ms;
    }

    // 链路健康分级
    let cls = "link-good";
    let label = "良好";
    if (!intervalMs) {
        label = "-";
    } else if (intervalMs > 500 || dropCount > 50) {
        cls = "link-bad";
        label = `差(${intervalMs}ms,丢${dropCount})`;
    } else if (intervalMs > 200 || dropCount > 10) {
        cls = "link-warn";
        label = `延迟(${intervalMs}ms,丢${dropCount})`;
    } else {
        label = `${intervalMs}ms,丢${dropCount}`;
    }
    linkEl.textContent = `链路: ${label}`;
    linkEl.className = `status-pill ${cls}`;
}

// ============================================================
// 本地约束优化器
// ============================================================

const optStateMap = {
    idle: "空闲",
    proposing: "生成候选",
    awaiting_confirm: "等待确认",
    applying: "下发参数",
    running_trial_1: "试验1",
    running_trial_2: "试验2",
    rolling_back: "回滚",
    converged: "收敛",
    aborted: "中止",
};

const btnOptStart = document.getElementById("btn-opt-start");
const btnOptConfirm = document.getElementById("btn-opt-confirm");
const btnOptStop = document.getElementById("btn-opt-stop");
const btnOptExport = document.getElementById("btn-opt-export");
const btnOptExportRace = document.getElementById("btn-opt-export-race");
const optStatePill = document.getElementById("opt-state-pill");
const optCandidatesEl = document.getElementById("opt-candidates");
const optBestEl = document.getElementById("opt-best");
const optMessageEl = document.getElementById("opt-message");
const optCandidateList = document.getElementById("opt-candidate-list");

btnOptStart.addEventListener("click", () => {
    const targetLaps = Number.parseInt(document.getElementById("opt-target-laps").value, 10) || 5;
    if (!confirm(`启动优化器？目标 ${targetLaps} 圈，将自动搜索最佳参数。`)) return;
    socket.emit("optimizer_start", { target_laps: targetLaps });
});

btnOptConfirm.addEventListener("click", () => {
    socket.emit("optimizer_confirm");
});

btnOptStop.addEventListener("click", () => {
    socket.emit("optimizer_stop");
});

btnOptExport.addEventListener("click", () => {
    socket.emit("optimizer_export");
});

btnOptExportRace.addEventListener("click", () => {
    socket.emit("optimizer_export_race");
});

socket.on("optimizer_state", (data) => {
    const state = data.state || "idle";
    const label = optStateMap[state] || state;
    optStatePill.textContent = label;

    // 状态样式
    optStatePill.className = "opt-state-pill";
    if (["running_trial_1", "running_trial_2", "proposing", "applying"].includes(state)) {
        optStatePill.classList.add("is-active");
    } else if (state === "awaiting_confirm") {
        optStatePill.classList.add("is-awaiting");
    } else if (["aborted", "rolling_back"].includes(state)) {
        optStatePill.classList.add("is-fault");
    }

    // 候选计数
    const generated = data.candidates_generated ?? 0;
    const maxC = data.max_candidates ?? 0;
    optCandidatesEl.textContent = `${generated}/${maxC}`;

    // 最佳参数
    const best = data.best_params;
    if (best && Object.keys(best).length > 0) {
        const summary = `KP=${best.KP?.toFixed(1)} V=${best.VBASE?.toFixed(0)}`;
        optBestEl.textContent = summary;
    } else {
        optBestEl.textContent = "-";
    }

    // 消息
    const msg = data.msg || "";
    if (msg) {
        optMessageEl.textContent = msg;
        optMessageEl.className = "opt-message";
    } else {
        optMessageEl.textContent = label;
        optMessageEl.className = "opt-message";
    }

    // 按钮状态
    const isActive = state !== "idle" && state !== "converged" && state !== "aborted";
    btnOptStart.disabled = isActive;
    btnOptStop.disabled = !isActive;
    btnOptConfirm.disabled = state !== "awaiting_confirm";
    btnOptExport.disabled = generated === 0;
    btnOptExportRace.disabled = !best || Object.keys(best).length === 0;
});

socket.on("optimizer_candidate", (data) => {
    const row = document.createElement("div");
    const accepted = data.accepted;
    row.className = `opt-cand-row ${accepted ? "is-accepted" : "is-rejected"}`;

    const params = data.params || {};
    const paramSummary = Object.entries(params)
        .map(([k, v]) => `${k}=${typeof v === "number" ? v.toFixed(1) : v}`)
        .join(" ");

    const resultText = accepted ? "采纳" : (data.abort_reason || "拒绝");

    row.innerHTML =
        `<span class="opt-cand-idx">#${data.index}</span>` +
        `<span class="opt-cand-params"></span>` +
        `<span class="opt-cand-result">${resultText}</span>`;
    row.querySelector(".opt-cand-params").textContent = paramSummary;

    optCandidateList.appendChild(row);
    optCandidateList.scrollTop = optCandidateList.scrollHeight;
    while (optCandidateList.children.length > 50) {
        optCandidateList.removeChild(optCandidateList.firstChild);
    }
});

socket.on("optimizer_report", (report) => {
    const blob = new Blob([JSON.stringify(report, null, 2)], { type: "application/json" });
    const link = document.createElement("a");
    link.href = URL.createObjectURL(blob);
    link.download = `optimizer-report-${new Date().toISOString().replace(/[:.]/g, "-")}.json`;
    link.click();
    URL.revokeObjectURL(link.href);
});

socket.on("optimizer_race_header", (data) => {
    const box = document.getElementById("opt-race-header");
    if (data.error) {
        box.textContent = `错误: ${data.error}`;
        return;
    }
    box.textContent = data.content || "";
});

setInterval(() => {
    socket.emit("get_status");
}, 2000);

console.log("[INIT] host frontend ready");
