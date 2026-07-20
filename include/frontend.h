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
      --bg: #07111f;
      --panel: rgba(11, 24, 43, 0.92);
      --panel-2: #12253f;
      --text: #eef7ff;
      --muted: #8da8c2;
      --accent: #26c281;
      --accent-2: #34b7ff;
      --warn: #ffb347;
      --danger: #ff5f7e;
    }

    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      font-family: Inter, "Segoe UI", Roboto, Arial, sans-serif;
      background: radial-gradient(circle at top, #12304c, var(--bg));
      color: var(--text);
      min-height: 100vh;
      padding: 24px;
    }

    .app {
      max-width: 1200px;
      margin: 0 auto;
    }

    .header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      flex-wrap: wrap;
      gap: 16px;
      margin-bottom: 20px;
    }

    .title-block h1 {
      margin: 0 0 6px;
      font-size: 28px;
    }

    .title-block p {
      margin: 0;
      color: var(--muted);
    }

    .version {
      padding: 10px 14px;
      border-radius: 999px;
      background: rgba(38, 194, 129, 0.16);
      color: var(--accent);
      font-weight: 700;
      letter-spacing: 0.04em;
    }

    .grid {
      display: grid;
      grid-template-columns: 1.3fr 0.9fr;
      gap: 20px;
    }

    .panel {
      background: var(--panel);
      border: 1px solid rgba(255, 255, 255, 0.08);
      border-radius: 20px;
      padding: 20px;
      box-shadow: 0 16px 40px rgba(0, 0, 0, 0.24);
    }

    .hero {
      display: flex;
      flex-direction: column;
      gap: 18px;
    }

    .hero-top {
      display: flex;
      justify-content: space-between;
      align-items: flex-start;
      gap: 12px;
    }

    .hero-top h2,
    .panel h3 {
      margin: 0 0 8px;
      font-size: 18px;
    }

    .hero-top p,
    .panel p {
      margin: 0;
      color: var(--muted);
      line-height: 1.5;
    }

    .metric-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 14px;
    }

    .card {
      background: linear-gradient(135deg, rgba(255, 255, 255, 0.05), rgba(255, 255, 255, 0.02));
      border: 1px solid rgba(255, 255, 255, 0.07);
      border-radius: 16px;
      padding: 14px;
    }

    .card .label {
      color: var(--muted);
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }

    .card .value {
      margin-top: 8px;
      font-size: 24px;
      font-weight: 700;
    }

    .card .unit {
      color: var(--muted);
      font-size: 13px;
    }

    .sparkline {
      background: var(--panel-2);
      border-radius: 16px;
      padding: 14px;
    }

    .sparkline svg {
      width: 100%;
      height: 150px;
    }

    .controls {
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
    }

    button {
      border: 0;
      border-radius: 999px;
      padding: 10px 14px;
      cursor: pointer;
      font-weight: 700;
      color: var(--text);
      background: linear-gradient(135deg, var(--accent-2), #4c79ff);
      transition: transform 0.2s ease, opacity 0.2s ease;
    }

    button:hover {
      transform: translateY(-1px);
    }

    .details {
      display: grid;
      gap: 12px;
      margin-top: 16px;
    }

    .detail-row {
      display: flex;
      justify-content: space-between;
      padding: 10px 0;
      border-bottom: 1px solid rgba(255, 255, 255, 0.08);
    }

    .detail-row:last-child {
      border-bottom: 0;
    }

    .muted {
      color: var(--muted);
    }

    .chip {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      padding: 5px 10px;
      border-radius: 999px;
      background: rgba(255, 255, 255, 0.07);
      font-size: 12px;
    }

    .chip.good {
      color: var(--accent);
    }

    .chip.warn {
      color: var(--warn);
    }

    .chip.bad {
      color: var(--danger);
    }

    .small {
      font-size: 14px;
    }

    .config-form {
      display: grid;
      gap: 14px;
      margin-top: 8px;
    }

    .config-grid {
      display: grid;
      gap: 12px;
    }

    .field {
      display: grid;
      gap: 6px;
    }

    .field label {
      color: var(--muted);
      font-size: 13px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }

    .field input,
    .field select {
      width: 100%;
      border: 1px solid rgba(255, 255, 255, 0.12);
      border-radius: 12px;
      padding: 12px 13px;
      background: rgba(255, 255, 255, 0.04);
      color: var(--text);
      font: inherit;
    }

    .field input:focus,
    .field select:focus {
      outline: 2px solid rgba(52, 183, 255, 0.25);
      border-color: var(--accent-2);
    }

    .actions {
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      margin-top: 4px;
      justify-content: space-between;
    }

    .status-banner {
      margin-top: 10px;
      padding: 12px 14px;
      border-radius: 12px;
      background: rgba(38, 194, 129, 0.12);
      color: var(--accent);
      border: 1px solid rgba(38, 194, 129, 0.2);
    }

    .status-banner.error {
      color: var(--danger);
      background: rgba(255, 95, 126, 0.12);
      border-color: rgba(255, 95, 126, 0.2);
    }

    .status-banner.warn {
      color: var(--warn);
      background: rgba(255, 179, 71, 0.12);
      border-color: rgba(255, 179, 71, 0.2);
    }

    button:disabled {
      opacity: 0.7;
      cursor: not-allowed;
      transform: none;
    }

    #rebootBtn {
      background: linear-gradient(135deg, var(--danger), #ffb347);
    }

    .show-password-container {
      display: flex;
      align-items: center;
      gap: 6px;
      font-size: 13px;
      color: var(--muted);
    }

    .show-password-container input {
      width: auto;
      cursor: pointer;
    }

    .show-password-container label {
      cursor: pointer;
      user-select: none;
    }

    .status-box {
      background-color: rgba(0, 0, 0, 0.03);
      border-left: 4px solid #3498db;
      padding: 10px !important;
      margin-bottom: 15px;
      border-radius: 0 6px 6px 0;
    }

    .text-good {
      color: #2ecc71;
    }

    .text-warn {
      color: #f39c12;
    }

    .text-danger {
      color: #e74c3c;
    }

    @media (max-width: 900px) {
      .grid {
        grid-template-columns: 1fr;
      }
    }

    @media (max-width: 560px) {
      body {
        padding: 14px;
      }
    }
  </style>
</head>

<body>
  <div class="app">
    <header class="header">
      <div class="title-block">
        <h1>ESP32 Solar Inverter</h1>
        <p>Set up Wi‑Fi and MQTT connection details for your inverter.</p>
      </div>
      <div class="version">v1.0.0</div>
    </header>

    <div class="grid">
      <section class="panel hero">
        <div class="hero-top">
          <div>
            <h2>Connection Settings</h2>
            <p>Enter your network and broker details below.</p>
          </div>
        </div>

        <form id="configForm" class="config-form">
          <div class="config-grid">
            <div class="field">
              <label for="wifiSsid">Wi‑Fi SSID</label>
              <input id="wifiSsid" name="wifiSsid" type="text" placeholder="MyHomeWiFi" />
            </div>

            <div class="field">
              <label for="wifiPassword">Wi‑Fi Password</label>
              <input id="wifiPassword" name="wifiPassword" type="password" placeholder="••••••••" />
              <div class="show-password-container">
                <input type="checkbox" id="showWifiPass" class="show-password-check">
                <label for="showWifiPass">Show Password</label>
              </div>
            </div>

            <div class="field">
              <label for="mqttBroker">MQTT Broker</label>
              <input id="mqttBroker" name="mqttBroker" type="text" placeholder="mqtt.example.com" />
            </div>

            <div class="field">
              <label for="mqttPort">MQTT Port</label>
              <input id="mqttPort" name="mqttPort" type="number" min="1" max="65535" value="1883" />
            </div>

            <div class="field">
              <label for="mqttUsername">MQTT Username</label>
              <input id="mqttUsername" name="mqttUsername" type="text" placeholder="optional" />
            </div>

            <div class="field">
              <label for="mqttPassword">MQTT Password</label>
              <input id="mqttPassword" name="mqttPassword" type="password" placeholder="optional" />
              <div class="show-password-container">
                <input type="checkbox" id="showMqttPass" class="show-password-check">
                <label for="showMqttPass">Show Password</label>
              </div>
            </div>
          <div class="actions">
            <button type="submit" id="saveBtn">Save Settings</button>
            <button type="button" id="rebootBtn">Reboot</button>
          </div>

          <div class="status-banner" id="formStatus">Ready to save your settings.</div>
        </form>
      </section>

      <aside class="panel">
        <h3>Required for the device</h3>
        <p>Use a stable Wi‑Fi network and an MQTT broker that accepts connections from your ESP32.</p>
        <!-- Status -->
        <div class="details status-box">
          <div class="detail-row">
            <span class="muted">Wi-Fi Status</span>
            <strong id="statusWifi">Connecting...</strong>
          </div>
          <div class="detail-row">
            <span class="muted">ESP32 IP</span>
            <strong id="statusIp">0.0.0.0</strong>
          </div>
          <div class="detail-row">
            <span class="muted">MQTT Broker</span>
            <strong id="statusMqtt">Disconnected</strong>
          </div>
          <div class="detail-row">
            <span class="muted">Signal (RSSI)</span>
            <strong id="statusRssi">0 dBm</strong>
          </div>
        </div>

        <div class="details">
          <div class="detail-row">
            <span class="muted">Wi‑Fi</span>
            <strong>Client mode</strong>
          </div>
          <div class="detail-row">
            <span class="muted">MQTT</span>
            <strong>Publish & subscribe</strong>
          </div>
          <div class="detail-row">
            <span class="muted">Storage</span>
            <strong>Persistent config</strong>
          </div>
        </div>

        <div class="details">
          <div class="chip good small">Secure credentials</div>
          <div class="chip small">MQTT topic ready</div>
          <div class="chip warn small">Reboot after save</div>
        </div>
      </aside>

    </div>
  </div>

  <script>
    const form = document.getElementById("configForm");
    const formStatus = document.getElementById("formStatus");
    const saveBtn = document.getElementById("saveBtn");
    const rebootBtn = document.getElementById("rebootBtn");

    const wifiEl = document.getElementById('statusWifi');
    const ipEl = document.getElementById('statusIp');
    const mqttEl = document.getElementById('statusMqtt');
    const rssiEl = document.getElementById('statusRssi');

    function setStatus(message, kind = "info") {
      formStatus.textContent = message;
      formStatus.className = "status-banner";

      if (kind === "error") {
        formStatus.classList.add("error");
      } else if (kind === "warn") {
        formStatus.classList.add("warn");
      }
    }

    function updateStatusUI(config) {
      // Wi-Fi status
      if (config.currentWifi === "Disconnected") {
        wifiEl.textContent = "Disconnected";
        wifiEl.className = "text-danger";
      } else {
        wifiEl.textContent = "Connected to: " + config.currentWifi;
        wifiEl.className = "text-good";
      }

      // IP address
      ipEl.textContent = config.currentIp || "0.0.0.0";
      if (config.currentIp !== "0.0.0.0") ipEl.className = "text-good";

      // MQTT status
      mqttEl.textContent = config.currentMqtt;
      if (config.currentMqtt === "Connected") {
        mqttEl.className = "text-good";
      } else {
        mqttEl.className = "text-danger";
      }

      // RSSI
      if (config.currentRssi < 0) {
        rssiEl.textContent = config.currentRssi + " dBm";
        if (config.currentRssi > -67) rssiEl.className = "text-good";
        else if (config.currentRssi > -80) rssiEl.className = "text-warn";
        else rssiEl.className = "text-danger";
      }
    }

    async function waitDeviceOnline() {
      const delay = (ms) => new Promise(resolve => setTimeout(resolve, ms));

      await delay(5000);

      while (true) {
        try {
          const res = await fetch("/api/config", { method: "GET" });
          if (res.ok) {
            return;
          }
        } catch (err) {
        }
        await delay(3000);
      }
    }


    document.addEventListener("DOMContentLoaded", async () => {
      try {
        const response = await fetch("/api/config");

        if (response.ok) {
          const config = await response.json();

          updateStatusUI(config);

          if (config.wifiSsid) {
            document.querySelector('input[name="wifiSsid"]').value = config.wifiSsid;
          }
          if (config.wifiPassword) {
            document.querySelector('input[name="wifiPassword"]').value = config.wifiPassword;
          }
          if (config.mqttBroker) {
            document.querySelector('input[name="mqttBroker"]').value = config.mqttBroker;
          }
          if (config.mqttPort) {
            document.querySelector('input[name="mqttPort"]').value = config.mqttPort;
          }
          if (config.mqttUsername) {
            document.querySelector('input[name="mqttUsername"]').value = config.mqttUsername;
          }
          if (config.mqttPassword) {
            document.querySelector('input[name="mqttPassword"]').value = config.mqttPassword;
          }
        } else {
          console.error("Failed to fetch config:", response.status);
        }
      } catch (error) {
        console.error("Failed to fetch config from ESP32:", error);
      }
    });

    form.addEventListener("submit", async (event) => {
      event.preventDefault();
      saveBtn.disabled = true;
      setStatus("Saving settings...", "warn");

      try {
        const payload = Object.fromEntries(new FormData(form).entries());
        const response = await fetch("/api/config", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload)
        });

        if (!response.ok) {
          console.error("Failed to save settings:", response.statusText);
          throw new Error("Could not save settings");
        }

        setStatus("Settings saved. Reboot to apply them.", "info");
      } catch (error) {
        setStatus("Unable to save settings right now.", "error");
      } finally {
        saveBtn.disabled = false;
      }
    });

    rebootBtn.addEventListener("click", async () => {
      if (!confirm("Are you sure you want to reboot the device?")) return;

      rebootBtn.disabled = true;
      setStatus("Rebooting device...", "warn");
      try {
        const response = await fetch("/api/reboot", { method: "POST" });
        if (!response.ok) {
          throw new Error("Failed to reboot");
        }
        setStatus("Device is rebooting...", "warn");

        await waitDeviceOnline();

        setStatus("Device rebooted", "info");
      } catch (error) {
        setStatus("Failed to send reboot command.", "error");
      } finally {
        rebootBtn.disabled = false;
      }
    });

    document.querySelectorAll(".show-password-check").forEach(checkbox => {
      checkbox.addEventListener("change", (event) => {
        const currentCheckbox = event.target;
        const fieldContainer = currentCheckbox.closest(".field");
        const passwordInput = fieldContainer.querySelector("input[type='password'], input[type='text']");
        
        if (passwordInput) {
          passwordInput.type = currentCheckbox.checked ? "text" : "password";
        }
      });
    });

    setInterval(async () => {
      try {
        const response = await fetch("/api/config");
        if (response.ok) {
          const config = await response.json();
          updateStatusUI(config);
        } else {
          console.error("Failed to fetch config:", response.status);
        }
      } catch (error) {
        console.error("Failed to fetch config from ESP32:", error);
      }
    }, 15000); // Update every 15 seconds
  </script>
</body>

</html>
)rawliteral";

#endif // FRONTEND_H