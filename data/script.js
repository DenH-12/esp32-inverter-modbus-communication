const form = document.getElementById("configForm");
const formStatus = document.getElementById("formStatus");
const saveBtn = document.getElementById("saveBtn");
const rebootBtn = document.getElementById("rebootBtn");
const discoveryBtn = document.getElementById("haDiscoveryBtn");
const haDiscoveryTopic = document.getElementById("haDiscoveryTopic");
const factoryReset = document.getElementById("factoryReset");

const wifiEl = document.getElementById('statusWifi');
const ipEl = document.getElementById('statusIp');
const mqttEl = document.getElementById('statusMqtt');
const rssiEl = document.getElementById('statusRssi');

const ledWifi = document.getElementById('ledWifi');
const ledMqtt = document.getElementById('ledMqtt');

// Modal elements
const customModal = document.getElementById('customModal');
const modalIcon = document.getElementById('modalIcon');
const modalTitle = document.getElementById('modalTitle');
const modalDesc = document.getElementById('modalDesc');
const modalConfirmBtn = document.getElementById('modalConfirmBtn');
const modalCancelBtn = document.getElementById('modalCancelBtn');

const mappingModal = document.getElementById('mappingModal');
const mappingForm = document.getElementById('mappingForm');
const jsonEditorStatus = document.getElementById('jsonEditorMsg');
let onModalConfirmCallback = null;

const editor = document.getElementById('code-editor');
const lineNumbers = document.getElementById('line-numbers');
const editorOverlay = document.getElementById('editor-overlay');

// Global variable to hold the entire mapping file
let fullMapping = {
    realtime_map: [],
    settings_map: []
};
let lastSyncedValue = '';
let editorDirty = false;

function escapeHtml(value) {
    return value
        .replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;');
}

function getDiffRange(baseValue, currentValue) {
    const maxLen = Math.max(baseValue.length, currentValue.length);
    let start = 0;

    while (start < maxLen && baseValue[start] === currentValue[start]) {
        start++;
    }

    let baseEnd = baseValue.length;
    let currentEnd = currentValue.length;

    while (baseEnd > start && currentEnd > start && baseValue[baseEnd - 1] === currentValue[currentEnd - 1]) {
        baseEnd--;
        currentEnd--;
    }

    return { start, end: currentEnd };
}

function renderEditorOverlay() {
    const currentValue = editor.value;
    const baseValue = lastSyncedValue;

    if (!editorDirty || currentValue === baseValue) {
        editorOverlay.innerHTML = escapeHtml(currentValue);
        return;
    }

    const { start, end } = getDiffRange(baseValue, currentValue);
    const before = escapeHtml(currentValue.slice(0, start));
    const changed = escapeHtml(currentValue.slice(start, end));
    const after = escapeHtml(currentValue.slice(end));

    editorOverlay.innerHTML = `${before}<span class="editor-insert">${changed}</span>${after}`;
}

function setEditorDirtyState(isDirty) {
    editorDirty = isDirty;
    editor.classList.toggle('dirty', isDirty);
    renderEditorOverlay();
}

function syncEditorToObj(keepDirty = false) {
    const nextValue = JSON.stringify(fullMapping, null, 2);
    editor.value = nextValue;
    updateLineNumbers();

    if (!keepDirty) {
        lastSyncedValue = nextValue;
        setEditorDirtyState(false);
    } else {
        setEditorDirtyState(true);
    }
}

function syncObjToEditor() {
    const editor = document.getElementById('code-editor');
    try {
        fullMapping = JSON.parse(editor.value);
        setStatus("JSON is valid.", jsonEditorStatus, "info");
    } catch (e) {
        console.error("Invalid JSON:", e);
        setStatus("Invalid JSON: " + e.message, jsonEditorStatus, "warn");
    }
}

async function uploadJsonFile() {
    try {
        fullMapping = JSON.parse(editor.value);

        // Upload to ESP32
        setStatus("Uploading mapping JSON...", jsonEditorMsg, "warn");

        const response = await fetch('/api/registers-map', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(fullMapping, null, 2) // Pretty print for easier debugging on ESP32
        });

        if (response.ok) {
            setStatus("Mapping JSON updated successfully! Reboot to apply changes.", jsonEditorMsg, "info");
            lastSyncedValue = editor.value;
            setEditorDirtyState(false);
        } else {
            throw new Error("Server failed to save JSON");
        }
    } catch (e) {
        setStatus("Upload failed: " + e.message, jsonEditorMsg, "error");
    }
}

// 1. Load the existing JSON from server on startup
async function loadFullMapping() {
    try {
        const response = await fetch('/api/registers-map'); // Adjust endpoint to your ESP32 path
        if (response.ok) {
            fullMapping = await response.json();
            syncEditorToObj(false);
            setStatus("Mapping JSON loaded successfully.", jsonEditorMsg, "info");
        }
    } catch (err) {
        setStatus("Failed to load mapping JSON: " + err.message, jsonEditorMsg, "error");
    }
}

function openMappingModal(addr) {
    document.getElementById('mapAddr').value = addr;
    document.getElementById('mapAddrLabel').textContent = `Address: ${addr}`;
    document.getElementById('mappingModal').classList.add('open');
    document.body.style.overflow = 'hidden';
}

function closeMappingModal() {
    document.getElementById('mappingModal').classList.remove('open');
    document.getElementById('mappingForm').reset();
    document.querySelectorAll('.setting-field').forEach(field => field.style.display = 'none');
    document.querySelectorAll('.sensor-field').forEach(field => field.style.display = '');
    document.getElementById('optionsList').innerHTML = ''; // Clear options list
    document.querySelector('.select-field').style.display = 'none'; // Hide select field
    document.body.style.overflow = '';
}

// Tab Switching
function switchTab(tabId, updateHistory = true) {
    document.querySelectorAll('.tab-content').forEach(t => t.style.display = 'none');
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));

    document.getElementById(tabId).style.display = '';
    // If calling from inline onclick, we need to find the button
    const btn = document.querySelector(`button[onclick="switchTab('${tabId}')"]`);
    if (btn) btn.classList.add('active');

    if (updateHistory) {
        const url = new URL(window.location);
        url.searchParams.set('tab', tabId);
        history.pushState({}, '', url);
    }
}

let scanResultsCache = [];

function renderScanResults(results) {
    const tbody = document.getElementById('scanResultsBody');
    const existingRegs = new Set(fullMapping.realtime_map.map(m => m.reg).concat(fullMapping.settings_map.map(m => m.reg)));

    if (!results || results.length === 0) {
        tbody.innerHTML = '<tr><td colspan="3" style="text-align:center; padding:30px; color:var(--text-muted);">No matching registers found.</td></tr>';
        return;
    }

    const rowsHtml = results.map(reg => {
        const isMapped = existingRegs.has(reg.addr);
        return `
            <tr>
                <td>${reg.addr}</td>
                <td style="color:var(--accent); font-weight:bold;">${reg.val}</td>
                <td>
                    ${isMapped
                        ? '<span style="color:var(--accent); font-weight:bold;">' + (fullMapping.settings_map.find(m => m.reg === reg.addr)?.name || fullMapping.realtime_map.find(n => n.reg === reg.addr)?.name) + '</span>'
                        : '<button class="btn-add" onclick="openMappingModal(' + reg.addr + ')">Map</button>'}
                </td>
            </tr>`;
    }).join('');

    tbody.innerHTML = rowsHtml;
}

function filterScanResults() {
    const valueFilterInput = document.getElementById('scanValueFilter');
    const filterTerms = (valueFilterInput.value || '')
        .split(',')
        .map(term => term.trim())
        .filter(Boolean);

    if (filterTerms.length === 0) {
        renderScanResults(scanResultsCache);
        return;
    }

    const filteredResults = scanResultsCache.filter(reg => {
        const regValue = String(reg.val ?? '');
        return filterTerms.some(term => regValue.includes(term));
    });

    renderScanResults(filteredResults);
}

// Scanning Logic
async function runRegisterScan() {
    const btn = document.getElementById('btnDoScan');
    const tbody = document.getElementById('scanResultsBody');
    const status = document.getElementById('scanStatusMsg');
    const scanForm = document.getElementById('scanForm');

    const startAddr = parseInt(document.getElementById('scanStart').value);
    const endAddr = parseInt(document.getElementById('scanEnd').value);
    const step = parseInt(document.getElementById('scanStep').value);

    if (!scanForm.checkValidity()) {
        status.style.display = 'block';
        setStatus("Please fill in all required fields.", status, "warn");
        return;
    } else if (startAddr > endAddr) {
        status.style.display = 'block';
        setStatus("Start address cannot be greater than End address", status, "error");
        return;
    }

    btn.disabled = true;
    status.style.display = 'block';
    tbody.innerHTML = '';

    let totalFound = 0;
    let isCancelled = false;
    scanResultsCache = [];

    setStatus(`Scanning: ${startAddr} .. ${endAddr} (Total range: ${startAddr}-${endAddr})...`, status, "warn");

    try {
        const res = await fetch('/api/scan-registers', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                startAddr: startAddr,
                endAddr: endAddr,
                scanStep: step
            })
        });

        if (!res.ok) throw new Error(`HTTP error ${res.status}`);

        const existingRegs = new Set(fullMapping.realtime_map.map(m => m.reg).concat(fullMapping.settings_map.map(m => m.reg)));

        while (true) {
            await new Promise(resolve => setTimeout(resolve, 1000));

            const statusRes = await fetch('/api/scan-registers');
            if (!statusRes.ok) continue;

            const data = await statusRes.json();
            const results = data.results || [];

            if (results.length > 0) {
                totalFound += results.length;
                scanResultsCache.push(...results);

                setStatus(`Scanning: ${data.current} .. ${data.end} (Total range: ${startAddr}-${endAddr})...`, status, "warn");
            }

            if (data.done && !data.running) {
                break;
            }
        };
    } catch (err) {
        console.error(`Error scanning range ${startAddr}-${endAddr}:`, err);
        setStatus(`Error during scan: ${err.message}`, status, "error");
        btn.disabled = false;

        return;
    }

    btn.disabled = false;
    setStatus(`Scan complete. Found ${totalFound} non-zero registers.`, status, "info");

    if (totalFound === 0) {
        tbody.innerHTML = '<tr><td colspan="3" style="text-align:center; padding:30px; color:var(--text-muted);">No data received.</td></tr>';
        return;
    }

    renderScanResults(scanResultsCache);
}

function showModal({ title, message, type = "danger", icon = "⚠️", confirmText = "Confirm", onConfirm }) {
    modalTitle.textContent = title;
    modalDesc.textContent = message;
    modalIcon.textContent = icon;
    modalConfirmBtn.textContent = confirmText;

    modalIcon.className = `modal-icon ${type}`;
    modalConfirmBtn.className = `btn ${type === 'danger' ? 'btn-danger' : type === 'primary' ? 'btn-primary' : 'btn-primary'}`;

    onModalConfirmCallback = onConfirm;
    customModal.classList.add('open');
    document.body.style.overflow = 'hidden';
}

function closeModal() {
    customModal.classList.remove('open');
    onModalConfirmCallback = null;
    document.body.style.overflow = '';
}

function setStatus(message, id, kind = "info") {
    id.textContent = message;
    id.className = "status-banner";
    if (kind === "error") id.classList.add("error");
    if (kind === "warn") id.classList.add("warn");
}

function updateStatusUI(config) {
    if (config.currentWifi === "Disconnected") {
        wifiEl.textContent = "Disconnected";
        ledWifi.className = "led error";
    } else {
        wifiEl.textContent = config.currentWifi;
        ledWifi.className = "led active";
    }

    ipEl.textContent = config.currentIp || "0.0.0.0";

    mqttEl.textContent = config.currentMqtt;
    if (config.currentMqtt === "Connected") {
        ledMqtt.className = "led active";
    } else {
        ledMqtt.className = "led error";
    }

    if (config.currentRssi !== undefined && config.currentRssi < 0) {
        rssiEl.textContent = config.currentRssi + " dBm";
    }
}

async function waitDeviceOnline() {
    const delay = (ms) => new Promise(resolve => setTimeout(resolve, ms));
    await delay(5000);
    while (true) {
        try {
            const res = await fetch("/api/status");
            if (res.ok) return;
        } catch (err) { }
        await delay(3000);
    }
}

async function executeHaDiscovery() {
    discoveryBtn.disabled = true;
    setStatus("Starting HA Discovery...", formStatus, "warn");

    try {
        const response = await fetch('/api/ha_discovery', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ haDiscoveryTopic: haDiscoveryTopic.value.trim() }),
        });

        if (!response.ok) throw new Error(`Server Error: ${response.status}`);

        const data = await response.json();
        if (data.status === "ok") {
            setStatus("HA Discovery published successfully.", formStatus, "info");
        } else {
            throw new Error("Failed to trigger Discovery");
        }
    } catch (error) {
        setStatus(`Error: ${error.message}`, formStatus, "error");
    } finally {
        discoveryBtn.disabled = false;
    }
}

async function readSavedSettings() {
    try {
        const response = await fetch("/api/settings");
        if (response.ok) {
            const config = await response.json();
            updateStatusUI(config);

            if (config.version) document.querySelector('.version-tag').textContent = config.version;
            if (config.wifiSsid) document.querySelector('input[name="wifiSsid"]').value = config.wifiSsid;
            if (config.wifiPassword) document.querySelector('input[name="wifiPassword"]').value = config.wifiPassword;
            if (config.mqttBroker) document.querySelector('input[name="mqttBroker"]').value = config.mqttBroker;
            if (config.mqttPort) document.querySelector('input[name="mqttPort"]').value = config.mqttPort;
            if (config.mqttUsername) document.querySelector('input[name="mqttUsername"]').value = config.mqttUsername;
            if (config.mqttPassword) document.querySelector('input[name="mqttPassword"]').value = config.mqttPassword;
            if (config.haDiscoveryTopic) document.querySelector('input[name="haDiscoveryTopic"]').value = config.haDiscoveryTopic;
            if (config.dataReadInterval) document.querySelector('input[name="dataReadInterval"]').value = config.dataReadInterval;
            if (config.sysDataSendInterval) document.querySelector('input[name="sysDataSendInterval"]').value = config.sysDataSendInterval;
            if (config.webServerPort) document.querySelector('input[name="webServerPort"]').value = config.webServerPort;
            if (config.modbusAddr) document.querySelector('input[name="modbusAddr"]').value = config.modbusAddr;
            if (config.modbusTx) document.querySelector('input[name="modbusTx"]').value = config.modbusTx;
            if (config.modbusRx) document.querySelector('input[name="modbusRx"]').value = config.modbusRx;
            if (config.modbusPortNum) document.querySelector('input[name="modbusPortNum"]').value = config.modbusPortNum;
            if (config.modbusBaudRate) document.querySelector('input[name="modbusBaudRate"]').value = config.modbusBaudRate;
        }
    } catch (error) {
        console.error("Failed to load setup:", error);
    }
}

async function executeSaveSettings() {
    saveBtn.disabled = true;
    setStatus("Saving config...", formStatus, "warn");

    try {
        const payload = Object.fromEntries(new FormData(form).entries());
        const response = await fetch("/api/settings", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(payload)
        });

        if (!response.ok) throw new Error("Save action failed");

        setStatus("Settings stored. Reboot to activate.", formStatus, "info");
    } catch (error) {
        setStatus("Failed to save settings.", formStatus, "error");
    } finally {
        saveBtn.disabled = false;
    }
}

async function executeReboot() {
    rebootBtn.disabled = true;
    setStatus("Sending reboot command...", formStatus, "warn");
    try {
        const response = await fetch("/api/reboot", { method: "POST" });
        if (!response.ok) throw new Error("Reboot failed");

        setStatus("Rebooting... Waiting for reconnect...", formStatus, "warn");
        await waitDeviceOnline();
        setStatus("Device back online!", formStatus, "info");
        readSavedSettings();
    } catch (error) {
        setStatus("Reboot command error.", formStatus, "error");
    } finally {
        rebootBtn.disabled = false;
    }
}

async function executeFactoryReset() {
    try {
        const response = await fetch("/api/factory-reset", {
            method: "POST",
            headers: { "Content-Type": "application/json" }
        });

        if (response.ok) {
            setStatus("Factory reset initiated. Reconnecting to default AP...", formStatus, "warn");
            setTimeout(() => { window.location.href = "http://192.168.4.1"; }, 5000);
        } else {
            setStatus("Failed to initiate factory reset.", formStatus, "error");
        }
    } catch (err) {
        console.error("Error sending reset request:", err);
        setStatus("Error occurred while sending reset request.", formStatus, "error");
    }
}

function updateLineNumbers() {
    const linesCount = editor.value.split('\n').length;

    const linesArray = Array.from({ length: linesCount }, (_, i) => i + 1);
    lineNumbers.innerText = linesArray.join('\n');
}

function toggleMappingFields() {
    const mapCategory = document.getElementById('mapCategory').value;

    const sensorFields = document.querySelectorAll('.sensor-field');
    const settingFields = document.querySelectorAll('.setting-field');
    const mapType = document.getElementById('mapType');
    const selectField = document.querySelector('.select-field');
    settingFields.forEach(field => field.style.display = 'none');
    sensorFields.forEach(field => field.style.display = 'none');

    if (mapCategory === 'realtime_map') {
        sensorFields.forEach(field => field.style.display = '');
    } else if (mapCategory === 'settings_map') {
        settingFields.forEach(field => field.style.display = '');
    }

    selectField.style.display = 'none';
    mapType.value = 'number'; // Reset mapType to default when switching categories
}

function toggleOptionsField() {
    const mapType = document.getElementById('mapType').value;
    const selectField = document.querySelector('.select-field');

    if (mapType === 'select') {
        selectField.style.display = '';
    } else if (mapType === 'number') {
        selectField.style.display = 'none';
    }
}

function addOption() {
    const optionsList = document.getElementById('optionsList');

    const keys = document.querySelectorAll('.select-input-val');
    const nextKey = keys.length > 0 ? (parseInt(keys[keys.length - 1].value) || 0) + 1 : 0;

    const div = document.createElement('div');
    div.className = 'select-item';
    div.innerHTML = `
      <div class="input-group key-group">
        <span class="input-label">Value</span>
        <input type="text" class="select-input-val" value="${nextKey}">
      </div>
      <div class="input-group val-group">
        <span class="input-label">Key</span>
        <input type="text" class="select-input-key" placeholder="Option">
      </div>
      <button type="button" class="btn-icon-danger" onclick="removeOption(this)" title="Видалити">
        <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
          <path d="M3 6h18"></path>
          <path d="M19 6v14c0 1-1 2-2 2H7c-1 0-2-1-2-2V6"></path>
          <path d="M8 6V4c0-1 1-2 2-2h4c1 0 2 1 2 2v2"></path>
        </svg>
      </button>
    `;

    optionsList.appendChild(div);
}

function removeOption(btn) {
    btn.closest('.select-item').remove();
}

function generateSlugId(name) {
    const slug = name
        .trim()
        .toLowerCase()
        .replace(/[^a-z0-9]+/g, '_')
        .replace(/^_+|_+$/g, '');

    return slug;
}

function toggleEditorMode(checkbox) {
    if (checkbox.checked) {
        editor.removeAttribute('readonly');
    } else {
        editor.setAttribute('readonly', 'true');
    }
}


discoveryBtn.addEventListener("click", () => {
    if (!haDiscoveryTopic.value.trim()) {
        setStatus("Set HA Topic first!", formStatus, "warn");
        haDiscoveryTopic.focus();
        return;
    }

    showModal({
        title: "Start HA Discovery?",
        message: "This will publish Home Assistant auto-discovery configs over MQTT.",
        type: "primary",
        icon: "📡",
        confirmText: "Start Discovery",
        onConfirm: executeHaDiscovery
    });
});

form.addEventListener("submit", (event) => {
    event.preventDefault();

    configForm.classList.add("was-validated");

    if (!form.checkValidity()) {
        setStatus("Please complete all required fields.", formStatus, "warn");
        return;
    }

    showModal({
        title: "Save Configuration?",
        message: "New settings will be stored in flash memory. A device reboot is required to apply Wi-Fi and MQTT changes.",
        type: "success",
        icon: "💾",
        confirmText: "Save Settings",
        onConfirm: executeSaveSettings
    });
});

rebootBtn.addEventListener("click", () => {
    showModal({
        title: "Reboot ESP32?",
        message: "The controller will restart and temporarily drop active connections.",
        type: "danger",
        icon: "⚡",
        confirmText: "Reboot Device",
        onConfirm: executeReboot
    });
});

factoryReset.addEventListener("click", () => {
    showModal({
        title: "Factory Reset?",
        message: "This will erase all saved settings and restore default configuration. The device will reboot.",
        type: "danger",
        icon: "⚠️",
        confirmText: "Reset Everything",
        onConfirm: executeFactoryReset
    });
});

// Initialize on load
document.addEventListener("DOMContentLoaded", () => {
    const urlParams = new URLSearchParams(window.location.search);
    const tabParam = urlParams.get('tab');

    const toggleEditCheckbox = document.getElementById('toggleEdit');
    toggleEditorMode(toggleEditCheckbox);

    if (tabParam && document.getElementById(tabParam)) {
        switchTab(tabParam);
    } else {
        switchTab('main');
    }

    loadFullMapping(); // Fetch the current JSON from ESP32
    readSavedSettings();

});

// Handle Form Submission
document.getElementById('mappingForm').addEventListener('submit', async (e) => {
    e.preventDefault();

    const mapCategory = document.getElementById('mapCategory').value;
    // Create the new entry object
    const newEntry = {
        name: document.getElementById('mapName').value,
        id: document.getElementById('mapId').value,
        reg: parseInt(document.getElementById('mapAddr').value),
        factor: parseFloat(document.getElementById('mapFactor').value) || 1.0,
        unit: document.getElementById('mapUnit').value || ""
    };

    if (mapCategory === 'realtime_map') {
        newEntry.signed = document.getElementById('mapSigned').value === "true";
        newEntry.class = document.getElementById('mapClass').value || null;

        // Check for duplicates
        const exists = fullMapping.realtime_map.find(m => m.reg === newEntry.reg);
        if (exists) {
            if (!confirm("Register already exists in map. Overwrite?")) return;
            fullMapping.realtime_map = fullMapping.realtime_map.filter(m => m.reg !== newEntry.reg);
        }

        // Add to local JSON array
        fullMapping.realtime_map.push(newEntry);
    }
    else if (mapCategory === 'settings_map') {
        newEntry.type = document.getElementById('mapType').value || "number";

        if (newEntry.type === 'select') {
            const options = {};
            document.querySelectorAll('.select-item').forEach(item => {
                const val = item.querySelector('.select-input-val').value;
                const key = item.querySelector('.select-input-key').value;
                if (key && val) {
                    options[val] = key;
                }
            });
            newEntry.options = options;
        }

        // Check for duplicates
        const exists = fullMapping.settings_map.find(m => m.reg === newEntry.reg);
        if (exists) {
            if (!confirm("Register already exists in map. Overwrite?")) return;
            fullMapping.settings_map = fullMapping.settings_map.filter(m => m.reg !== newEntry.reg);
        }

        // Add to local JSON array
        fullMapping.settings_map.push(newEntry);
    }

    closeMappingModal();
    syncEditorToObj(true);
});

document.querySelectorAll(".toggle-pwd").forEach(btn => {
    btn.addEventListener("click", () => {
        const input = document.getElementById(btn.dataset.target);
        if (input.type === "password") {
            input.type = "text";
            btn.textContent = "Hide";
        } else {
            input.type = "password";
            btn.textContent = "Show";
        }
    });
});

modalCancelBtn.addEventListener('click', closeModal);
modalConfirmBtn.addEventListener('click', () => {
    if (onModalConfirmCallback) onModalConfirmCallback();
    closeModal();
});

editor.addEventListener('input', () => {
    updateLineNumbers();
    setEditorDirtyState(true);
});

editor.addEventListener('scroll', () => {
    lineNumbers.scrollTop = editor.scrollTop;
    editorOverlay.scrollTop = editor.scrollTop;
    editorOverlay.scrollLeft = editor.scrollLeft;
});

window.addEventListener('popstate', () => {
    const urlParams = new URLSearchParams(window.location.search);
    const tabParam = urlParams.get('tab') || 'main';

    if (tabParam && document.getElementById(tabParam)) {
        switchTab(tabParam, false);
    }
});

document.addEventListener('input', (event) => {
    const mapNameInput = document.getElementById('mapName');

    if (event.target === mapNameInput) {
        const mapIdInput = document.getElementById('mapId');
        mapIdInput.value = generateSlugId(mapNameInput.value);
    }
});

editor.addEventListener('blur', () => {
    syncObjToEditor();
});

setInterval(async () => {
    try {
        const response = await fetch("/api/status");
        if (response.ok) {
            const config = await response.json();
            updateStatusUI(config);
        }
    } catch (error) { }
}, 5000);

// ----------------------- OTA Update ----------------------------

const otaForm = document.getElementById('otaForm');
const otaStatusMsg = document.getElementById('otaStatusMsg');

otaForm.addEventListener('submit', async (event) => {
    event.preventDefault();
    await otaUpdate();
});

async function otaUpdate() {
    const firmwareFile = document.getElementById('firmwareFile');
    const filesystemImage = document.getElementById('filesystemImage');
    const firmware = firmwareFile.files[0];
    const filesystem = filesystemImage.files[0];

    otaStatusMsg.style.display = "block";

    if (!firmware && !filesystem) {
        setStatus("Please select a firmware or filesystem image first.", otaStatusMsg, "warn");
        return;
    }

    const submitBtn = otaForm.querySelector('button[type="submit"]');
    submitBtn.disabled = true;

    const imagesToUpload = [];

    if (firmware) {
        if (!firmware.name.toLowerCase().endsWith('.bin')) {
            setStatus("Only .bin firmware files are supported.", otaStatusMsg, "warn");
            submitBtn.disabled = false;
            return;
        }
        imagesToUpload.push({ file: firmware, endpoint: '/api/update/firmware', label: 'Firmware' });
    }

    if (filesystem) {
        if (!filesystem.name.toLowerCase().endsWith('.bin')) {
            setStatus("Only .bin filesystem files are supported.", otaStatusMsg, "warn");
            submitBtn.disabled = false;
            return;
        }
        imagesToUpload.push({ file: filesystem, endpoint: '/api/update/spiffs', label: 'Filesystem' });
    }

    try {
        for (const item of imagesToUpload) {
            setStatus(`Uploading ${item.label.toLowerCase()}...`, otaStatusMsg, "warn");

            await new Promise((resolve, reject) => {
                const xhr = new XMLHttpRequest();

                xhr.open('POST', item.endpoint, true);
                xhr.setRequestHeader('Content-Type', 'application/octet-stream');

                xhr.upload.onprogress = (event) => {
                    if (event.lengthComputable) {
                        const percent = Math.round((event.loaded / event.total) * 100);
                        setStatus(`Uploading ${item.label.toLowerCase()}... ${percent}%`, otaStatusMsg, "warn");
                    }
                };

                xhr.onload = () => {
                    if (xhr.status >= 200 && xhr.status < 300) {
                        resolve(xhr.responseText);
                    } else {
                        reject(new Error(xhr.responseText || `HTTP ${xhr.status}`));
                    }
                };

                xhr.onerror = () => reject(new Error(`Network error during ${item.label.toLowerCase()} upload`));
                xhr.ontimeout = () => reject(new Error(`${item.label} upload timed out`));

                xhr.send(item.file);
            });

            if (item.label === 'Firmware') {
                setStatus("Firmware uploaded successfully. Device will reboot.", otaStatusMsg, "warn");
                firmwareFile.value = '';
            } else {
                setStatus("Filesystem image uploaded successfully.", otaStatusMsg, "info");
                filesystemImage.value = '';
            }
        }
    } catch (error) {
        console.error('OTA upload failed:', error);
        setStatus(`OTA upload failed: ${error.message}`, otaStatusMsg, "error");
    } finally {
        await waitDeviceOnline();
        setStatus("Device back online!", otaStatusMsg, "info");
        
        submitBtn.disabled = false;
    }
}