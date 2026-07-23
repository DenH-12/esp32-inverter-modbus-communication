#ifndef FRONTEND_H
#define FRONTEND_H

// This file contains the HTML content for the web interface of the ESP32 inverter configuration panel. The HTML is embedded as a raw string literal in C++.
static const char index_html[] = R"rawliteral(
<!DOCTYPE html>
<html lang="en">

<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Solar Inverter Control Panel</title>
  <link rel="icon"
    href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'><rect width='100%25' height='100%25' rx='20' fill='%231e293b'/><text x='50%25' y='65%25' font-family='monospace, sans-serif' font-weight='bold' font-size='48' fill='%23ffffff' text-anchor='middle'>ESP</text></svg>">
  <style>
    :root {
      --bg: #090d16;
      --card-bg: #111827;
      --card-border: #1f293d;
      --text-main: #f3f4f6;
      --text-muted: #9ca3af;
      --accent: #10b981;
      --accent-glow: rgba(16, 185, 129, 0.25);
      --primary: #3b82f6;
      --primary-glow: rgba(59, 130, 246, 0.25);
      --danger: #ef4444;
      --danger-glow: rgba(239, 68, 68, 0.25);
      --warn: #f59e0b;
    }

    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
    }

    body {
      font-family: system-ui, -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
      background-color: var(--bg);
      background-image:
        radial-gradient(at 0% 0%, rgba(59, 130, 246, 0.08) 0px, transparent 50%),
        radial-gradient(at 100% 100%, rgba(16, 185, 129, 0.05) 0px, transparent 50%);
      color: var(--text-main);
      min-height: 100vh;
      padding: 24px 16px;
    }

    .app {
      max-width: 1100px;
      margin: 0 auto;
    }

    /* Header */
    .header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 28px;
      padding-bottom: 16px;
      border-bottom: 1px solid var(--card-border);
    }

    .brand {
      display: flex;
      align-items: center;
      gap: 12px;
    }

    .brand-icon {
      width: 40px;
      height: 40px;
      background: linear-gradient(135deg, var(--primary), var(--accent));
      border-radius: 10px;
      display: flex;
      align-items: center;
      justify-content: center;
      font-weight: 800;
      font-size: 18px;
      color: #fff;
      box-shadow: 0 0 15px var(--primary-glow);
    }

    .title-block h1 {
      font-size: 20px;
      font-weight: 700;
      letter-spacing: -0.02em;
    }

    .title-block p {
      font-size: 13px;
      color: var(--text-muted);
    }

    .version-tag {
      font-size: 12px;
      font-weight: 600;
      padding: 4px 10px;
      border-radius: 20px;
      background: rgba(59, 130, 246, 0.1);
      color: var(--primary);
      border: 1px solid rgba(59, 130, 246, 0.2);
      text-decoration: none;   
      cursor: pointer;
      transition: opacity 0.2s;
    }

    .version-tag:hover {
      opacity: 0.8
    }

    /* Layout */
    .dashboard-grid {
      display: grid;
      grid-template-columns: 1fr 320px;
      gap: 24px;
    }

    /* Cards */
    .card {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 16px;
      padding: 20px;
      margin-bottom: 20px;
      box-shadow: 0 4px 20px rgba(0, 0, 0, 0.3);
    }

    .card-header {
      display: flex;
      align-items: center;
      gap: 10px;
      margin-bottom: 16px;
    }

    .card-header h2 {
      font-size: 15px;
      text-transform: uppercase;
      letter-spacing: 0.05em;
      color: var(--text-muted);
    }

    .form-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
      gap: 16px;
    }

    /* Field Styling */
    .field {
      display: flex;
      flex-direction: column;
      gap: 6px;
    }

    .field label {
      font-size: 12px;
      font-weight: 600;
      color: var(--text-muted);
    }

    .required-label::after {
      content: " *";
      color: var(--danger);
      font-weight: bold;
    }

    .input-wrapper {
      position: relative;
      display: flex;
      align-items: center;
    }

    .field input {
      width: 100%;
      background: #0d131f;
      border: 1px solid var(--card-border);
      border-radius: 8px;
      padding: 10px 12px;
      color: var(--text-main);
      font-size: 14px;
      transition: all 0.2s ease;
    }

    .field input:focus {
      outline: none;
      border-color: var(--primary);
      box-shadow: 0 0 10px var(--primary-glow);
    }

    .configForm.was-validated input:invalid {
      outline: none;
      border-color: var(--danger);
      box-shadow: 0 0 10px var(--primary-glow);
    }

    .toggle-pwd {
      position: absolute;
      right: 10px;
      background: none;
      border: none;
      color: var(--text-muted);
      cursor: pointer;
      font-size: 12px;
      padding: 2px 4px;
    }

    .toggle-pwd:hover {
      color: var(--text-main);
    }

    /* Buttons */
    .btn-group {
      display: flex;
      gap: 12px;
      margin-top: 8px;
    }

    .btn {
      flex: 1;
      padding: 12px 16px;
      border-radius: 10px;
      border: none;
      font-weight: 600;
      font-size: 14px;
      cursor: pointer;
      transition: all 0.2s ease;
      display: inline-flex;
      align-items: center;
      justify-content: center;
    }

    .btn-primary {
      background: var(--primary);
      color: white;
      box-shadow: 0 0 12px var(--primary-glow);
    }

    .btn-primary:hover:not(:disabled) {
      background: #2563eb;
    }

    .btn-secondary {
      background: rgba(255, 255, 255, 0.05);
      color: var(--text-main);
      border: 1px solid var(--card-border);
    }

    .btn-secondary:hover:not(:disabled) {
      background: rgba(255, 255, 255, 0.1);
    }

    .btn-danger {
      background: rgba(239, 68, 68, 0.15);
      color: var(--danger);
      border: 1px solid rgba(239, 68, 68, 0.3);
    }

    .btn-danger:hover:not(:disabled) {
      background: var(--danger);
      color: white;
      box-shadow: 0 0 12px var(--danger-glow);
    }

    .btn:disabled {
      opacity: 0.4;
      cursor: not-allowed;
    }

    /* Status Panel */
    .status-list {
      display: flex;
      flex-direction: column;
      gap: 14px;
    }

    .status-item {
      display: flex;
      justify-content: space-between;
      align-items: center;
      background: #0d131f;
      padding: 12px;
      border-radius: 8px;
      border: 1px solid var(--card-border);
    }

    .status-label {
      font-size: 13px;
      color: var(--text-muted);
    }

    .status-value-group {
      display: flex;
      align-items: center;
      gap: 8px;
      font-weight: 600;
      font-size: 13px;
    }

    .led {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: var(--text-muted);
    }

    .led.active {
      background: var(--accent);
      box-shadow: 0 0 8px var(--accent);
    }

    .led.error {
      background: var(--danger);
      box-shadow: 0 0 8px var(--danger);
    }

    .status-banner {
      padding: 12px 16px;
      border-radius: 10px;
      font-size: 13px;
      font-weight: 500;
      background: rgba(16, 185, 129, 0.1);
      color: var(--accent);
      border: 1px solid rgba(16, 185, 129, 0.2);
      margin-bottom: 16px;
    }

    .status-banner.error {
      background: rgba(239, 68, 68, 0.1);
      color: var(--danger);
      border-color: rgba(239, 68, 68, 0.2);
    }

    .status-banner.warn {
      background: rgba(245, 158, 11, 0.1);
      color: var(--warn);
      border-color: rgba(245, 158, 11, 0.2);
    }

    /* Custom Confirmation Modal */
    .modal-overlay {
      position: fixed;
      top: 0;
      left: 0;
      width: 100vw;
      height: 100vh;
      background: rgba(5, 8, 15, 0.75);
      backdrop-filter: blur(6px);
      display: flex;
      align-items: center;
      justify-content: center;
      z-index: 1000;
      opacity: 0;
      pointer-events: none;
      transition: opacity 0.2s ease;
    }

    .modal-overlay.open {
      opacity: 1;
      pointer-events: auto;
    }

    .modal {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 16px;
      width: 90%;
      max-width: 400px;
      padding: 24px;
      box-shadow: 0 20px 40px rgba(0, 0, 0, 0.6);
      transform: scale(0.95);
      transition: transform 0.2s ease;
    }

    .modal-overlay.open .modal {
      transform: scale(1);
    }

    .modal-icon {
      width: 48px;
      height: 48px;
      border-radius: 12px;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 22px;
      margin-bottom: 16px;
    }

    .modal-icon.danger {
      background: rgba(239, 68, 68, 0.15);
      color: var(--danger);
      border: 1px solid rgba(239, 68, 68, 0.3);
    }

    .modal-icon.primary {
      background: rgba(59, 130, 246, 0.15);
      color: var(--primary);
      border: 1px solid rgba(59, 130, 246, 0.3);
    }

    .modal-icon.success {
      background: rgba(16, 185, 129, 0.15);
      color: var(--accent);
      border: 1px solid rgba(16, 185, 129, 0.3);
    }

    .modal-title {
      font-size: 18px;
      font-weight: 700;
      margin-bottom: 8px;
    }

    .modal-desc {
      font-size: 14px;
      color: var(--text-muted);
      line-height: 1.5;
      margin-bottom: 20px;
    }

    .modal-actions {
      display: flex;
      gap: 10px;
    }

    @media (max-width: 850px) {
      .dashboard-grid {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>

<body>
  <div class="app">
    <header class="header">
      <div class="brand">
        <div class="brand-icon">⚡</div>
        <div class="title-block">
          <h1>Solar Inverter Node</h1>
          <p>ESP32 Modbus-MQTT Gateway</p>
        </div>
      </div>
  <a id="version-tag" class="version-tag" href="https://github.com/DenH-12/esp32-inverter-modbus-communication/releases" target="_blank" rel="noopener noreferrer">v0.0.0</a>
  </header>
    <div class="dashboard-grid">
      <!-- Form -->
      <main>
        <form id="configForm" class="configForm" novalidate>

          <div class="card">
            <div class="card-header">
              <h2>01. Wi-Fi Connection</h2>
            </div>
            <div class="form-grid">
              <div class="field">
                <label for="wifiSsid" class="required-label">Network SSID</label>
                <div class="input-wrapper">
                  <input id="wifiSsid" name="wifiSsid" type="text" placeholder="Access Point Name" required />
                </div>
              </div>

              <div class="field">
                <label for="wifiPassword" class="required-label">Password</label>
                <div class="input-wrapper">
                  <input id="wifiPassword" name="wifiPassword" type="password" placeholder="••••••••" required />
                  <button type="button" class="toggle-pwd" data-target="wifiPassword">Show</button>
                </div>
              </div>
            </div>
          </div>

          <div class="card">
            <div class="card-header">
              <h2>02. MQTT Broker</h2>
            </div>
            <div class="form-grid">
              <div class="field">
                <label for="mqttBroker" class="required-label">Server IP / Host</label>
                <div class="input-wrapper">
                  <input id="mqttBroker" name="mqttBroker" type="text" placeholder="192.168.1.50" required />
                </div>
              </div>

              <div class="field">
                <label for="mqttPort" class="required-label">Port</label>
                <div class="input-wrapper">
                  <input id="mqttPort" name="mqttPort" type="number" placeholder="1883" required />
                </div>
              </div>

              <div class="field">
                <label for="mqttUsername">Username</label>
                <div class="input-wrapper">
                  <input id="mqttUsername" name="mqttUsername" type="text" placeholder="Optional" />
                </div>
              </div>

              <div class="field">
                <label for="mqttPassword">Password</label>
                <div class="input-wrapper">
                  <input id="mqttPassword" name="mqttPassword" type="password" placeholder="Optional" />
                  <button type="button" class="toggle-pwd" data-target="mqttPassword">Show</button>
                </div>
              </div>
            </div>
          </div>

          <div class="card">
            <div class="card-header">
              <h2>03. Telemetry & Integration</h2>
            </div>
            <div class="form-grid">
              <div class="field">
                <label for="haDiscoveryTopic" class="required-label">HA Topic</label>
                <div class="input-wrapper">
                  <input id="haDiscoveryTopic" name="haDiscoveryTopic" placeholder="homeassistant" required />
                </div>
              </div>

              <div class="field" style="justify-content: flex-end;">
                <button type="button" id="haDiscoveryBtn" class="btn btn-secondary">Run Discovery</button>
              </div>

              <div class="field">
                <label for="dataReadInterval" class="required-label">Read Poll (sec)</label>
                <div class="input-wrapper">
                  <input id="dataReadInterval" name="dataReadInterval" type="number" min="1" placeholder="2" required />
                </div>
              </div>

              <div class="field">
                <label for="sysDataSendInterval" class="required-label">Push Telemetry (sec)</label>
                <div class="input-wrapper">
                  <input id="sysDataSendInterval" name="sysDataSendInterval" type="number" min="1" placeholder="2"
                    required />
                </div>
              </div>

              <div class="field">
                <label for="webServerPort" class="required-label">Webserver Port</label>
                <div class="input-wrapper">
                  <input id="webServerPort" name="webServerPort" type="number" min="80" max="65535" placeholder="80...65535" required />
                </div>
              </div>
            </div>
          </div>

          <div id="formStatus" class="status-banner">System ready.</div>

          <div class="btn-group">
            <button type="submit" id="saveBtn" class="btn btn-primary">Save Configuration</button>
            <button type="button" id="rebootBtn" class="btn btn-danger">Reboot ESP32</button>
          </div>

        </form>
      </main>

      <!-- Sidebar -->
      <aside>
        <div class="card">
          <div class="card-header">
            <h2>Live Diagnostics</h2>
          </div>
          <div class="status-list">
            <div class="status-item">
              <span class="status-label">Wi-Fi Network</span>
              <div class="status-value-group">
                <div id="ledWifi" class="led"></div>
                <span id="statusWifi">Connecting...</span>
              </div>
            </div>

            <div class="status-item">
              <span class="status-label">ESP32 IP</span>
              <div class="status-value-group">
                <span id="statusIp">0.0.0.0</span>
              </div>
            </div>

            <div class="status-item">
              <span class="status-label">MQTT Broker</span>
              <div class="status-value-group">
                <div id="ledMqtt" class="led"></div>
                <span id="statusMqtt">Disconnected</span>
              </div>
            </div>

            <div class="status-item">
              <span class="status-label">Signal (RSSI)</span>
              <div class="status-value-group">
                <span id="statusRssi">0 dBm</span>
              </div>
            </div>
          </div>
        </div>
        <button type="button" id="factoryReset" class="btn btn-danger">Factory Reset</button>
      </aside>

    </div>
  </div>

  <!-- Custom Confirmation Modal Overlay -->
  <div id="customModal" class="modal-overlay">
    <div class="modal">
      <div id="modalIcon" class="modal-icon danger">⚠️</div>
      <div id="modalTitle" class="modal-title">Confirm Action</div>
      <div id="modalDesc" class="modal-desc">Are you sure you want to proceed?</div>
      <div class="modal-actions">
        <button type="button" id="modalCancelBtn" class="btn btn-secondary">Cancel</button>
        <button type="button" id="modalConfirmBtn" class="btn btn-danger">Confirm</button>
      </div>
    </div>
  </div>

  <script>
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

    let onModalConfirmCallback = null;

    function showModal({ title, message, type = "danger", icon = "⚠️", confirmText = "Confirm", onConfirm }) {
      modalTitle.textContent = title;
      modalDesc.textContent = message;
      modalIcon.textContent = icon;
      modalConfirmBtn.textContent = confirmText;

      modalIcon.className = `modal-icon ${type}`;
      modalConfirmBtn.className = `btn ${type === 'danger' ? 'btn-danger' : type === 'primary' ? 'btn-primary' : 'btn-primary'}`;

      onModalConfirmCallback = onConfirm;
      customModal.classList.add('open');
    }

    function closeModal() {
      customModal.classList.remove('open');
      onModalConfirmCallback = null;
    }

    modalCancelBtn.addEventListener('click', closeModal);
    modalConfirmBtn.addEventListener('click', () => {
      if (onModalConfirmCallback) onModalConfirmCallback();
      closeModal();
    });

    function setStatus(message, kind = "info") {
      formStatus.textContent = message;
      formStatus.className = "status-banner";
      if (kind === "error") formStatus.classList.add("error");
      if (kind === "warn") formStatus.classList.add("warn");
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
      setStatus("Starting HA Discovery...", "warn");

      try {
        const response = await fetch('/api/ha_discovery', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ haDiscoveryTopic: haDiscoveryTopic.value.trim() }),
        });

        if (!response.ok) throw new Error(`Server Error: ${response.status}`);

        const data = await response.json();
        if (data.status === "ok") {
          setStatus("HA Discovery published successfully.", "info");
        } else {
          throw new Error("Failed to trigger Discovery");
        }
      } catch (error) {
        setStatus(`Error: ${error.message}`, "error");
      } finally {
        discoveryBtn.disabled = false;
      }
    }

    async function readSavedSettings()  
    {
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
          }
        } catch (error) {
          console.error("Failed to load setup:", error);
        }
}

    discoveryBtn.addEventListener("click", () => {
      if (!haDiscoveryTopic.value.trim()) {
        setStatus("Set HA Topic first!", "warn");
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

    async function executeSaveSettings() {
      saveBtn.disabled = true;
      setStatus("Saving config...", "warn");

      try {
        const payload = Object.fromEntries(new FormData(form).entries());
        const response = await fetch("/api/settings", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload)
        });

        if (!response.ok) throw new Error("Save action failed");

        setStatus("Settings stored. Reboot to activate.", "info");
      } catch (error) {
        setStatus("Failed to save settings.", "error");
      } finally {
        saveBtn.disabled = false;
      }
    }

    form.addEventListener("submit", (event) => {
      event.preventDefault();

      configForm.classList.add("was-validated");

      if (!form.checkValidity()) {
        setStatus("Please complete all required fields.", "warn");
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

    async function executeReboot() {
      rebootBtn.disabled = true;
      setStatus("Sending reboot command...", "warn");
      try {
        const response = await fetch("/api/reboot", { method: "POST" });
        if (!response.ok) throw new Error("Reboot failed");

        setStatus("Rebooting... Waiting for reconnect...", "warn");
        await waitDeviceOnline();
        setStatus("Device back online!", "info");
        readSavedSettings();
      } catch (error) {
        setStatus("Reboot command error.", "error");
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
            alert("Factory reset initiated. Reconnecting to default AP...");
            setTimeout(() => { window.location.href = "http://192.168.4.1"; }, 5000);
          } else {
            alert("Failed to initiate factory reset.");
          }
        } catch (err) {
          console.error("Error sending reset request:", err);
        }
      }

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

    document.addEventListener("DOMContentLoaded", readSavedSettings);

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

    setInterval(async () => {
      try {
        const response = await fetch("/api/status");
        if (response.ok) {
          const config = await response.json();
          updateStatusUI(config);
        }
      } catch (error) { }
    }, 5000);
  </script>
</body>

</html>
)rawliteral";

#endif // FRONTEND_H