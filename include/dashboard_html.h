#pragma once

// ============================================================================
// OpenLogic Multi-Camera BLE Bench Dashboard
// Minimal Monochrome (Black & White) - DJI / GoPro / Insta360 Dedicated UIs
// ============================================================================
static const char DASHBOARD_HTML[] = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <meta name="apple-mobile-web-app-capable" content="yes" />
  <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent" />
  <meta name="apple-mobile-web-app-title" content="CamSwitch" />
  <meta name="mobile-web-app-capable" content="yes" />
  <meta name="theme-color" content="#000000" />
  <title>OpenLogic — Multi-Cam BLE Control</title>
  <style>
    :root {
      --bg: #000000;
      --surface: #0a0a0a;
      --surface-2: #141414;
      --border: #222222;
      --border-strong: #444444;
      --fg: #ffffff;
      --muted: #888888;
      --dim: #555555;
      --mono: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background: var(--bg);
      color: var(--fg);
      padding: 16px;
      line-height: 1.4;
      -webkit-font-smoothing: antialiased;
    }
    .app { max-width: 1080px; margin: 0 auto; display: grid; gap: 14px; }
    
    /* Topbar */
    .topbar {
      display: flex; flex-wrap: wrap; justify-content: space-between; align-items: center;
      gap: 12px; padding-bottom: 12px; border-bottom: 1px solid var(--border);
    }
    .brand-title { font-family: var(--mono); font-size: 13px; font-weight: 700; letter-spacing: 1px; text-transform: uppercase; }
    .brand-sub { color: var(--muted); font-size: 11px; margin-left: 8px; font-weight: 400; }

    /* Camera Switcher Bar */
    .cam-selector {
      display: flex; gap: 6px; background: var(--surface); padding: 4px; border: 1px solid var(--border); border-radius: 4px;
    }
    .cam-tab {
      background: transparent; border: 1px solid transparent; color: var(--muted);
      font-family: var(--mono); font-size: 11px; font-weight: 600; padding: 6px 14px;
      cursor: pointer; text-transform: uppercase; border-radius: 2px; transition: all 0.15s;
    }
    .cam-tab:hover { color: var(--fg); }
    .cam-tab.active { background: #ffffff; color: #000000; border-color: #ffffff; }

    /* Grid Layout */
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(340px, 1fr)); gap: 14px; }
    .card {
      background: var(--surface); border: 1px solid var(--border); border-radius: 4px; padding: 14px;
    }
    .card-title {
      font-family: var(--mono); font-size: 11px; font-weight: 700; color: var(--muted);
      text-transform: uppercase; letter-spacing: 0.8px; margin-bottom: 12px;
      display: flex; justify-content: space-between; align-items: center;
    }

    /* Badges & Metrics */
    .badge {
      font-family: var(--mono); font-size: 10px; font-weight: 700; padding: 2px 6px;
      border-radius: 2px; border: 1px solid var(--border-strong); text-transform: uppercase;
    }
    .badge-live { background: #ffffff; color: #000000; border-color: #ffffff; }
    .badge-rec { background: #ffffff; color: #000000; animation: blink 1s infinite alternate; }
    @keyframes blink { from { opacity: 1; } to { opacity: 0.3; } }

    .stat-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; }
    .stat-box {
      background: var(--surface-2); border: 1px solid var(--border); padding: 8px 10px; border-radius: 2px;
    }
    .stat-label { font-family: var(--mono); font-size: 10px; color: var(--muted); text-transform: uppercase; letter-spacing: 0.5px; }
    .stat-val { font-family: var(--mono); font-size: 14px; font-weight: 700; margin-top: 2px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }

    /* Button Controls */
    .btn-row { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 8px; }
    .btn {
      background: var(--surface-2); border: 1px solid var(--border-strong); color: var(--fg);
      font-family: var(--mono); font-size: 11px; font-weight: 600; padding: 8px 12px;
      cursor: pointer; text-transform: uppercase; border-radius: 2px; transition: all 0.15s; flex: 1;
      min-width: 100px; text-align: center;
    }
    .btn:hover { background: #ffffff; color: #000000; border-color: #ffffff; }
    .btn-primary { background: #ffffff; color: #000000; border-color: #ffffff; }
    .btn-primary:hover { background: #cccccc; border-color: #cccccc; }
    .btn-danger { border-color: #ffffff; }

    /* Select Dropdowns */
    select, input[type="text"] {
      background: var(--surface-2); border: 1px solid var(--border-strong); color: var(--fg);
      font-family: var(--mono); font-size: 11px; padding: 8px 10px; border-radius: 2px; width: 100%;
    }

    /* Terminal & Packet Inspector */
    .terminal {
      background: #000000; border: 1px solid var(--border); border-radius: 2px;
      font-family: var(--mono); font-size: 11px; color: #cccccc; height: 160px;
      overflow-y: auto; padding: 10px; line-height: 1.5; white-space: pre-wrap; word-break: break-all;
    }
    .log-line { border-bottom: 1px solid #141414; padding: 2px 0; }
    .log-ts { color: var(--dim); margin-right: 6px; }
    .log-tag { font-weight: 700; color: #ffffff; margin-right: 6px; }
    
    .hidden { display: none !important; }
    .btn:disabled, select:disabled { opacity: 0.35 !important; cursor: not-allowed !important; pointer-events: none !important; }
    .toast {
      position: fixed; bottom: 20px; right: 20px; background: #ffffff; color: #000000;
      padding: 8px 14px; font-family: var(--mono); font-size: 11px; font-weight: 700;
      border-radius: 2px; box-shadow: 0 4px 12px rgba(0,0,0,0.6); z-index: 9999;
      opacity: 0; pointer-events: none; transition: opacity 0.25s ease;
    }
    .toast.show { opacity: 1; }
  </style>
</head>
<body>
<div class="app">
  <!-- Topbar -->
  <div class="topbar">
    <div>
      <span class="brand-title">OPENLOGIC // CAMSWITCH</span>
      <span class="brand-sub">MULTI-BRAND BENCH DASHBOARD</span>
    </div>
    <div style="display:flex; flex-wrap:wrap; gap:8px; align-items:center;">
      <button class="btn" style="flex:0; padding:6px 12px; font-size:10px; background:#1c1c1c;" onclick="openInBrowser()">OPEN IN BROWSER</button>
      <!-- Camera Selection Tabs -->
      <div class="cam-selector">
        <button class="cam-tab active" id="tab-0" onclick="selectCamera(0)">DJI ACTION</button>
        <button class="cam-tab" id="tab-1" onclick="selectCamera(1)">GOPRO HERO</button>
        <button class="cam-tab" id="tab-2" onclick="selectCamera(2)">INSTA360</button>
      </div>
    </div>
  </div>

  <!-- Main Grid -->
  <div class="grid">
    <!-- ===================================================================== -->
    <!-- CAMERA TELEMETRY PANELS (Dedicated per Brand)                         -->
    <!-- ===================================================================== -->

    <!-- DJI OSMO ACTION TELEMETRY -->
    <div class="card" id="telem-dji">
      <div class="card-title">
        <span>TELEMETRY // DJI OSMO ACTION</span>
        <span class="badge" id="badge-link-0">DISCONNECTED</span>
      </div>

      <div class="stat-grid">
        <div class="stat-box">
          <div class="stat-label">RECORDING STATUS</div>
          <div class="stat-val" id="dji-rec-status">STANDBY</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">BATTERY LEVEL</div>
          <div class="stat-val" id="dji-battery">-- %</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">RECORD DURATION</div>
          <div class="stat-val" id="dji-rec-time">00:00</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">REMAIN VIDEO TIME</div>
          <div class="stat-val" id="dji-rem-time">--:--</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">SD REMAIN STORAGE</div>
          <div class="stat-val" id="dji-rem-cap">-- MB</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">PHOTOS REMAINING</div>
          <div class="stat-val" id="dji-rem-photos">--</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">ACTIVE MODE</div>
          <div class="stat-val" id="dji-mode">NORMAL VIDEO</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">RESOLUTION / FPS</div>
          <div class="stat-val" id="dji-res-fps">--</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">EIS STABILIZATION</div>
          <div class="stat-val" id="dji-eis">--</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">THERMAL STATUS</div>
          <div class="stat-val" id="dji-temp">NORMAL</div>
        </div>
      </div>

      <div style="margin-top: 12px; font-family: var(--mono); font-size: 10px; color: var(--muted); display: flex; justify-content: space-between;">
        <span>MAC: <b id="val-mac-0" style="color:var(--fg)">--:--:--:--:--:--</b></span>
        <span>PACKETS: <b id="val-pkt-0" style="color:var(--fg)">0</b></span>
      </div>
    </div>

    <!-- GOPRO HERO TELEMETRY -->
    <div class="card hidden" id="telem-gopro">
      <div class="card-title">
        <span>TELEMETRY // GOPRO HERO</span>
        <span class="badge" id="badge-link-1">DISCONNECTED</span>
      </div>

      <div class="stat-grid">
        <div class="stat-box">
          <div class="stat-label">ENCODING STATUS</div>
          <div class="stat-val" id="gopro-rec-status">STANDBY</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">BATTERY LEVEL</div>
          <div class="stat-val" id="gopro-battery">-- %</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">RECORD DURATION</div>
          <div class="stat-val" id="gopro-rec-time">00:00</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">REMAIN VIDEO TIME</div>
          <div class="stat-val" id="gopro-rem-time">--:--</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">REMAIN PHOTOS</div>
          <div class="stat-val" id="gopro-rem-photos">--</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">STORAGE REMAIN</div>
          <div class="stat-val" id="gopro-rem-cap">-- MB</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">ACTIVE PRESET</div>
          <div class="stat-val" id="gopro-mode">STANDARD VIDEO</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">HYPERSMOOTH EIS</div>
          <div class="stat-val" id="gopro-eis">HYPERSMOOTH</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">SYSTEM THERMAL</div>
          <div class="stat-val" id="gopro-temp">NORMAL</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">LINK STATE</div>
          <div class="stat-val" id="gopro-link">STANDBY</div>
        </div>
      </div>

      <div style="margin-top: 12px; font-family: var(--mono); font-size: 10px; color: var(--muted); display: flex; justify-content: space-between;">
        <span>MAC: <b id="val-mac-1" style="color:var(--fg)">--:--:--:--:--:--</b></span>
        <span>PACKETS: <b id="val-pkt-1" style="color:var(--fg)">0</b></span>
      </div>
    </div>

    <!-- INSTA360 GPS REMOTE TELEMETRY -->
    <div class="card hidden" id="telem-insta">
      <div class="card-title">
        <span>TELEMETRY // INSTA360 GPS REMOTE</span>
        <span class="badge" id="badge-link-2">DISCONNECTED</span>
      </div>

      <div class="stat-grid">
        <div class="stat-box">
          <div class="stat-label">REMOTE LINK STATE</div>
          <div class="stat-val" id="insta-rec-status">ADVERTISING</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">BATTERY LEVEL</div>
          <div class="stat-val" id="insta-battery">100 %</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">ACTIVE STATUS</div>
          <div class="stat-val" id="insta-mode">REMOTE STANDBY</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">OPERATING MODE</div>
          <div class="stat-val" id="insta-res-fps">COMMAND ONLY</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">EIS FLOWSTATE</div>
          <div class="stat-val" id="insta-eis">FLOWSTATE</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">SYSTEM THERMAL</div>
          <div class="stat-val" id="insta-temp">NORMAL</div>
        </div>
      </div>

      <div style="margin-top: 12px; font-family: var(--mono); font-size: 10px; color: var(--muted); display: flex; justify-content: space-between;">
        <span>MAC: <b id="val-mac-2" style="color:var(--fg)">--:--:--:--:--:--</b></span>
        <span>PACKETS: <b id="val-pkt-2" style="color:var(--fg)">0</b></span>
      </div>
    </div>

    <!-- ===================================================================== -->
    <!-- CAMERA OPERATIONS & CONTROLS (Dedicated per Brand)                    -->
    <!-- ===================================================================== -->

    <!-- DJI OSMO ACTION CONTROLS -->
    <div class="card" id="controls-dji">
      <div class="card-title">
        <span>OPERATIONS // DJI OSMO ACTION</span>
        <div style="display:flex; gap:6px;">
          <button class="btn" style="flex:0; padding: 2px 8px; font-size:10px" onclick="clearSavedMac()">CLEAR MAC / RESCAN</button>
          <button class="btn" style="flex:0; padding: 2px 8px; font-size:10px" onclick="saveAsDefault()">SAVE AS BOOT DEFAULT</button>
        </div>
      </div>

      <!-- Shutter Row -->
      <div class="btn-row">
        <button class="btn btn-primary" onclick="postAction('record_start')">START RECORD</button>
        <button class="btn btn-danger" onclick="postAction('record_stop')">STOP RECORD</button>
        <button class="btn" onclick="postAction('record_toggle')">TOGGLE SHUTTER</button>
      </div>

      <div style="margin-top: 10px; display: grid; gap: 8px;">
        <!-- Capture Mode & Photo Trigger -->
        <div style="display:flex; gap: 8px;">
          <select id="dji-mode-sel" onchange="postAction('set_mode', this.value)">
            <option value="0">MODE: NORMAL VIDEO</option>
            <option value="1">MODE: LOW LIGHT / NIGHT</option>
            <option value="2">MODE: SLOW MOTION</option>
            <option value="3">MODE: TIMELAPSE</option>
            <option value="4">MODE: PHOTO</option>
            <option value="5">MODE: HYPERLAPSE</option>
          </select>
          <button class="btn" style="flex:0; min-width:130px;" onclick="postAction('key_event')">PHOTO TRIGGER</button>
        </div>

        <!-- Key Reporting (QS & Snapshot) -->
        <div class="btn-row">
          <button class="btn" onclick="postAction('dji_qs_short')">QS SHORT (HILIGHT)</button>
          <button class="btn" onclick="postAction('dji_qs_long')">QS LONG PRESS</button>
          <button class="btn" onclick="postAction('dji_snapshot')">SNAPSHOT</button>
        </div>

        <!-- Telemetry Push, Pairing & Mock GPS -->
        <div class="btn-row">
          <button class="btn" onclick="postAction('dji_pair')">FORCE PAIR DIALOG (MODE=1)</button>
          <button class="btn" onclick="postAction('push_mode', '3')">2HZ TELEM PUSH</button>
          <button class="btn" onclick="postAction('mock_gps')">MOCK GPS (00/17)</button>
        </div>

        <!-- Power Management -->
        <div class="btn-row">
          <button class="btn" onclick="postAction('power_sleep')">POWER SLEEP (1A)</button>
          <button class="btn" onclick="postAction('power_normal')">POWER NORMAL</button>
          <button class="btn btn-primary" onclick="postAction('wake')">WKP WAKEUP (2S)</button>
        </div>
      </div>
    </div>

    <!-- GOPRO HERO CONTROLS -->
    <div class="card hidden" id="controls-gopro">
      <div class="card-title">
        <span>OPERATIONS // GOPRO HERO</span>
        <div style="display:flex; gap:6px;">
          <button class="btn" style="flex:0; padding: 2px 8px; font-size:10px" onclick="clearSavedMac()">CLEAR MAC / RESCAN</button>
          <button class="btn" style="flex:0; padding: 2px 8px; font-size:10px" onclick="saveAsDefault()">SAVE AS BOOT DEFAULT</button>
        </div>
      </div>

      <!-- Shutter Row -->
      <div class="btn-row">
        <button class="btn btn-primary" onclick="postAction('record_start')">START RECORD</button>
        <button class="btn btn-danger" onclick="postAction('record_stop')">STOP RECORD</button>
        <button class="btn" onclick="postAction('record_toggle')">TOGGLE SHUTTER</button>
      </div>

      <div style="margin-top: 10px; display: grid; gap: 8px;">
        <!-- Preset Mode & Hilight Tag -->
        <div style="display:flex; gap: 8px;">
          <select id="gopro-mode-sel" onchange="postAction('set_mode', this.value)">
            <option value="0">PRESET: VIDEO</option>
            <option value="1">PRESET: PHOTO</option>
            <option value="2">PRESET: TIMELAPSE</option>
          </select>
          <button class="btn" style="flex:0; min-width:130px;" onclick="postAction('gopro_hilight')">TAG HILIGHT</button>
        </div>

        <!-- Resolution & FPS Select Dropdowns -->
        <div style="display:flex; gap: 8px;">
          <select id="gopro-res-sel" onchange="postAction('set_res', this.value)">
            <option value="1">RES: 4K</option>
            <option value="4">RES: 2.7K</option>
            <option value="9">RES: 1080P</option>
            <option value="18">RES: 4K 4:3</option>
            <option value="100">RES: 5.3K</option>
          </select>
          <select id="gopro-fps-sel" onchange="postAction('set_fps', this.value)">
            <option value="0">FPS: 240</option>
            <option value="1">FPS: 120</option>
            <option value="2">FPS: 100</option>
            <option value="5">FPS: 60</option>
            <option value="6">FPS: 50</option>
            <option value="8">FPS: 30</option>
            <option value="9">FPS: 25</option>
            <option value="10">FPS: 24</option>
          </select>
        </div>

        <!-- HyperSmooth Stabilization Select Dropdown -->
        <div>
          <select id="gopro-eis-sel" onchange="postAction('set_eis', this.value)">
            <option value="0">HYPERSMOOTH: OFF</option>
            <option value="1">HYPERSMOOTH: ON / LOW</option>
            <option value="2">HYPERSMOOTH: HIGH</option>
            <option value="3">HYPERSMOOTH: BOOST</option>
            <option value="4">HYPERSMOOTH: AUTOBOOST</option>
            <option value="100">HYPERSMOOTH: STANDARD</option>
          </select>
        </div>

        <!-- Beeper / Locate & Wi-Fi AP Controls -->
        <div class="btn-row">
          <button class="btn" onclick="postAction('gopro_locate_on')">LOCATE / BEEP ON</button>
          <button class="btn" onclick="postAction('gopro_locate_off')">LOCATE / BEEP OFF</button>
          <button class="btn" onclick="postAction('gopro_wifi_on')">WI-FI AP ON</button>
          <button class="btn" onclick="postAction('gopro_wifi_off')">WI-FI AP OFF</button>
        </div>

        <!-- Telemetry & Power -->
        <div class="btn-row">
          <button class="btn" onclick="postAction('gopro_query')">QUERY STATUS (0x13)</button>
          <button class="btn" onclick="postAction('power_sleep')">SLEEP (01 05)</button>
          <button class="btn btn-primary" onclick="postAction('wake')">WAKE CAMERA</button>
        </div>
      </div>
    </div>

    <!-- INSTA360 GPS REMOTE CONTROLS -->
    <div class="card hidden" id="controls-insta">
      <div class="card-title">
        <span>OPERATIONS // INSTA360 GPS REMOTE</span>
        <div style="display:flex; gap:6px;">
          <button class="btn" style="flex:0; padding: 2px 8px; font-size:10px" onclick="clearSavedMac()">CLEAR MAC / RESCAN</button>
          <button class="btn" style="flex:0; padding: 2px 8px; font-size:10px" onclick="saveAsDefault()">SAVE AS BOOT DEFAULT</button>
        </div>
      </div>

      <!-- Shutter & Mode Row -->
      <div class="btn-row">
        <button class="btn btn-primary" onclick="postAction('record_toggle')">SHUTTER TOGGLE</button>
        <button class="btn" onclick="postAction('set_mode', '0')">MODE CYCLE</button>
      </div>

      <!-- Screen & Power Row -->
      <div class="btn-row" style="margin-top: 8px;">
        <button class="btn" onclick="postAction('toggle_screen')">SCREEN TOGGLE</button>
        <button class="btn btn-danger" onclick="postAction('power_sleep')">POWER OFF</button>
      </div>
    </div>

    <!-- BETAFLIGHT UART BRIDGE & TELEMETRY -->
    <div class="card" id="card-bf-bridge">
      <div class="card-title">
        <span>BETAFLIGHT UART BRIDGE & TELEMETRY</span>
        <span class="badge" id="bf-bridge-badge">OFFLINE</span>
      </div>

      <!-- Setup Mode & Betaflight BLE Banner -->
      <div id="bridge-active-banner" style="background:#141414; border:1px solid #333333; padding:12px; margin-bottom:12px; font-family:var(--mono); font-size:11px; color:#ffffff; line-height:1.5;">
        <b>WI-FI SETUP MODE</b> — BLE radio is paused while Wi-Fi web configuration is active.<br>
        To connect Betaflight Configurator over BLE via <b>app.betaflight.com</b> (device: <b>OpenLogic_BF</b>):<br>
        • Directly in <b>Flight Mode</b> while disarmed (DJI / GoPro)<br>
        • Or click <b>ENTER BETAFLIGHT BLE MODE</b> below (works for all cameras including Insta360).
      </div>

      <div class="stat-grid">
        <div class="stat-box">
          <div class="stat-label">FC MSP LINK</div>
          <div class="stat-val" id="bf-fc-status">OFFLINE</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">FC ARM STATE</div>
          <div class="stat-val" id="bf-arm-status">DISARMED</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">AUX CH PWM</div>
          <div class="stat-val" id="bf-aux-val">--</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">BLE BRIDGE</div>
          <div class="stat-val" id="bf-ble-status">READY</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">UART RX BYTES</div>
          <div class="stat-val" id="bf-rx-bytes">0 B</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">UART TX BYTES</div>
          <div class="stat-val" id="bf-tx-bytes">0 B</div>
        </div>
      </div>

      <div style="margin-top: 12px; display: grid; gap: 8px;">
        <button class="btn btn-primary" id="btn-start-flight" onclick="startFlightMode()">SAVE &amp; START FLIGHT MODE (BLE)</button>
        <button class="btn" id="btn-enter-bf-ble" onclick="enterBfBleMode()">ENTER BETAFLIGHT BLE MODE (app.betaflight.com)</button>

        <div style="display:flex; gap:8px;">
          <button class="btn" id="btn-auto-arm" onclick="toggleAutoArm()">AUTO-REC ON ARM: ON</button>
        </div>

        <div style="display:grid; grid-template-columns: 1fr 1fr; gap:8px;">
          <div style="display:flex; flex-direction:column; gap:4px;">
            <span style="font-family:var(--mono); font-size:10px; color:var(--muted);">DISARM STOP DELAY:</span>
            <select id="sel-disarm-dly" onchange="postAction('set_disarm_delay', this.value)">
              <option value="0">0s (INSTANT)</option>
              <option value="2">2s</option>
              <option value="3" selected>3s (DEFAULT)</option>
              <option value="5">5s</option>
              <option value="10">10s</option>
            </select>
          </div>
          <div style="display:flex; flex-direction:column; gap:4px;">
            <span style="font-family:var(--mono); font-size:10px; color:var(--muted);">AUX REC SWITCH:</span>
            <select id="sel-aux-ch" onchange="postAction('set_aux_ch', this.value)">
              <option value="0" selected>OFF (ARM ONLY)</option>
              <option value="1">AUX 1</option>
              <option value="2">AUX 2</option>
              <option value="3">AUX 3</option>
              <option value="4">AUX 4</option>
              <option value="5">AUX 5</option>
              <option value="6">AUX 6</option>
              <option value="7">AUX 7</option>
              <option value="8">AUX 8</option>
            </select>
          </div>
        </div>
      </div>
    </div>
  </div>

  <!-- Raw Packet Inspector & Live Terminal -->
  <div class="card">
    <div class="card-title">
      <span>RAW BLE PACKET INSPECTOR & SYSTEM LOG</span>
      <button class="btn" style="flex:0; padding: 2px 8px; font-size:10px" onclick="clearLogs()">CLEAR LOGS</button>
    </div>
    <div style="margin-bottom: 8px; font-family: var(--mono); font-size: 11px; color: var(--muted);">
      LAST PACKET: <span id="val-last-hex" style="color:#ffffff; font-weight:700">--</span>
    </div>
    <div class="terminal" id="term"></div>
  </div>
  <div id="toast" class="toast"></div>
</div>

<script>
let currentBrand = 0;
let lastLogId = 0;
let pendingSwitch = false;
let ws = null;
let wsReconnectTimer = null;
let currentAutoArm = true;

function startFlightMode() {
  postAction('start_flight');
  showToast('Rebooting into BLE Flight Mode in 250ms...');
}

function enterBfBleMode() {
  if (confirm('Enter dedicated Betaflight BLE Bridge mode?\n\nThis will reboot into BLE NUS Peripheral mode (OpenLogic_BF) for app.betaflight.com.\nCamera BLE and Wi-Fi will be paused.')) {
    postAction('enter_bf_ble');
    showToast('Rebooting into Betaflight BLE Mode in 250ms...');
  }
}

function toggleAutoArm() {
  currentAutoArm = !currentAutoArm;
  postAction('set_auto_arm', currentAutoArm ? '1' : '0');
  showToast('Auto-Rec on Arm: ' + (currentAutoArm ? 'ENABLED' : 'DISABLED'));
}

function showToast(msg) {
  const t = document.getElementById('toast');
  if (!t) return;
  t.innerText = msg;
  t.classList.add('show');
  setTimeout(() => t.classList.remove('show'), 2500);
}

function openInBrowser() {
  const isAndroid = /Android/i.test(navigator.userAgent);
  if (isAndroid) {
    window.location.href = "intent://192.168.4.1/#Intent;scheme=http;action=android.intent.action.VIEW;end";
  } else {
    alert("TO OPEN IN SAFARI / CHROME:\n\n1. At top-right of this popup, tap 'Done' or 'Cancel'\n2. Choose 'Use Without Internet'\n3. Open Safari and go to:\n   http://192.168.4.1");
  }
}

function renderCameraUI(brand) {
  for (let i = 0; i < 3; i++) {
    document.getElementById('tab-' + i).classList.toggle('active', i === brand);
  }
  
  // Toggle Telemetry Panels
  document.getElementById('telem-dji').classList.toggle('hidden', brand !== 0);
  document.getElementById('telem-gopro').classList.toggle('hidden', brand !== 1);
  document.getElementById('telem-insta').classList.toggle('hidden', brand !== 2);

  // Toggle Control Panels
  document.getElementById('controls-dji').classList.toggle('hidden', brand !== 0);
  document.getElementById('controls-gopro').classList.toggle('hidden', brand !== 1);
  document.getElementById('controls-insta').classList.toggle('hidden', brand !== 2);
}

function selectCamera(brand) {
  currentBrand = brand;
  pendingSwitch = false;
  renderCameraUI(brand);
  showToast('Camera brand selected & saved to NVS');

  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ cmd: 'select_cam', val: String(brand) }));
  } else {
    fetch('/api/select_cam?brand=' + brand, { method: 'POST' }).catch(console.error);
  }
}

function postAction(cmd, val) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ cmd: cmd, val: val }));
  } else {
    let url = '/api/action?cmd=' + encodeURIComponent(cmd);
    if (val !== undefined && val !== null) url += '&val=' + encodeURIComponent(val);
    fetch(url, { method: 'POST' }).catch(console.error);
  }
}

function saveAsDefault() {
  postAction('save_default');
  showToast('Saved active camera brand as boot default');
}

function clearSavedMac() {
  postAction('clear_mac');
  showToast('Cleared saved MAC & started fresh BLE scan');
}

function clearLogs() {
  document.getElementById('term').innerHTML = '';
  lastLogId = 0;
  fetch('/api/logs/clear', { method: 'POST' }).catch(console.error);
}

function formatCapacity(mb) {
  if (!mb || mb === 0) return "-- MB";
  if (mb >= 1024) {
    return (mb / 1024.0).toFixed(1) + " GB";
  }
  return mb + " MB";
}

function appendLog(l) {
  if (!l || l.id === undefined) return;
  if (l.id <= lastLogId) return;
  lastLogId = l.id;
  const term = document.getElementById('term');
  const div = document.createElement('div');
  div.className = 'log-line';

  const spanTs = document.createElement('span');
  spanTs.className = 'log-ts';
  spanTs.textContent = '[' + l.ts + '] ';

  const spanTag = document.createElement('span');
  spanTag.className = 'log-tag';
  spanTag.textContent = l.tag + ': ';

  const spanMsg = document.createElement('span');
  spanMsg.textContent = l.msg;

  div.appendChild(spanTs);
  div.appendChild(spanTag);
  div.appendChild(spanMsg);

  term.appendChild(div);
  term.scrollTop = term.scrollHeight;
}

function setControlsDisabled(disabled, stateStr) {
  const panelIds = ['controls-dji', 'controls-gopro', 'controls-insta'];
  panelIds.forEach(id => {
    const panel = document.getElementById(id);
    if (!panel) return;
    const elements = panel.querySelectorAll('button, select');
    elements.forEach(el => {
      const txt = (el.innerText || '').toUpperCase();
      const click = el.getAttribute('onclick') || '';
      // Keep SAVE DEFAULT, CLEAR MAC, and WAKE buttons always enabled
      if (txt.includes('DEFAULT') || txt.includes('CLEAR MAC') || click.includes('wake') || click.includes('power_normal')) {
        el.disabled = false;
      } else if (click.includes('dji_pair') && (stateStr === 'CONNECTING' || stateStr === 'PAIRED' || stateStr === 'READY')) {
        // Keep FORCE PAIR DIALOG enabled during CONNECTING as well
        el.disabled = false;
      } else {
        el.disabled = disabled;
      }
    });
  });
}

function applyState(d) {
  if (!d) return;

  // Sync UI tab with active brand from server only if not actively user-switching
  if (!pendingSwitch && d.activeBrand !== undefined && d.activeBrand !== currentBrand) {
    currentBrand = d.activeBrand;
    renderCameraUI(d.activeBrand);
  }

  // Link badges for all 3 panels
  const stateStr = d.stateStr || 'DISCONNECTED';
  for (let i = 0; i < 3; i++) {
    const badge = document.getElementById('badge-link-' + i);
    if (badge) {
      badge.innerText = stateStr;
      badge.className = 'badge';
      if (stateStr === 'RECORDING') badge.classList.add('badge-rec');
      else if (stateStr === 'PAIRED' || stateStr === 'READY') badge.classList.add('badge-live');
    }
  }

  // Disable camera command buttons while DISCONNECTED (except wake, clear mac, save default, force pair, power normal)
  const isConnected = (stateStr === 'PAIRED' || stateStr === 'RECORDING' || stateStr === 'READY');
  setControlsDisabled(!isConnected, stateStr);

  // Sync <select> dropdowns to incoming telemetry state
  if (document.activeElement !== document.getElementById('dji-mode-sel')) {
    const m = (d.currentMode || '').toUpperCase();
    const sel = document.getElementById('dji-mode-sel');
    if (sel) {
      if (m.includes('LOW LIGHT') || m.includes('NIGHT')) sel.value = '1';
      else if (m.includes('HYPERLAPSE')) sel.value = '5';
      else if (m.includes('SLOW')) sel.value = '2';
      else if (m.includes('TIMELAPSE')) sel.value = '3';
      else if (m.includes('PHOTO')) sel.value = '4';
      else if (m.includes('VIDEO')) sel.value = '0';
    }
  }

  if (document.activeElement !== document.getElementById('gopro-mode-sel')) {
    const gm = (d.currentMode || '').toUpperCase();
    const sel = document.getElementById('gopro-mode-sel');
    if (sel) {
      if (gm.includes('PHOTO')) sel.value = '1';
      else if (gm.includes('TIMELAPSE')) sel.value = '2';
      else if (gm.includes('VIDEO')) sel.value = '0';
    }
  }

  if (document.activeElement !== document.getElementById('gopro-res-sel')) {
    const rf = (d.resolutionFps || '').toUpperCase();
    const sel = document.getElementById('gopro-res-sel');
    if (sel) {
      if (rf.includes('5.3K')) sel.value = '100';
      else if (rf.includes('4K 4:3')) sel.value = '18';
      else if (rf.includes('4K')) sel.value = '1';
      else if (rf.includes('2.7K')) sel.value = '4';
      else if (rf.includes('1080P')) sel.value = '9';
    }
  }

  if (document.activeElement !== document.getElementById('gopro-fps-sel')) {
    const rf = (d.resolutionFps || '').toUpperCase();
    const sel = document.getElementById('gopro-fps-sel');
    if (sel) {
      if (rf.includes('240')) sel.value = '0';
      else if (rf.includes('120')) sel.value = '1';
      else if (rf.includes('100')) sel.value = '2';
      else if (rf.includes('60')) sel.value = '5';
      else if (rf.includes('50')) sel.value = '6';
      else if (rf.includes('30')) sel.value = '8';
      else if (rf.includes('25')) sel.value = '9';
      else if (rf.includes('24')) sel.value = '10';
    }
  }

  if (document.activeElement !== document.getElementById('gopro-eis-sel')) {
    const s = (d.stabilization || '').toUpperCase();
    const sel = document.getElementById('gopro-eis-sel');
    if (sel) {
      if (s.includes('AUTOBOOST')) sel.value = '4';
      else if (s.includes('BOOST')) sel.value = '3';
      else if (s.includes('HIGH')) sel.value = '2';
      else if (s.includes('ON') || s.includes('LOW')) sel.value = '1';
      else if (s.includes('OFF')) sel.value = '0';
      else if (s.includes('STANDARD')) sel.value = '100';
    }
  }

  // Format capacity
  const formattedCap = formatCapacity(d.remMB);

  // DJI Action Telemetry
  if (currentBrand === 0) {
    document.getElementById('dji-rec-status').innerText = d.isRecording ? 'RECORDING' : 'IDLE';
    document.getElementById('dji-battery').innerText = (d.battery !== undefined ? d.battery : '--') + ' %';
    document.getElementById('dji-rec-time').innerText = d.recTime || '00:00';
    document.getElementById('dji-rem-time').innerText = d.remTime || '--:--';
    document.getElementById('dji-rem-cap').innerText = formattedCap;
    document.getElementById('dji-rem-photos').innerText = d.remPhotos !== undefined ? d.remPhotos : '--';
    document.getElementById('dji-mode').innerText = d.currentMode || 'NORMAL VIDEO';
    document.getElementById('dji-res-fps').innerText = d.resolutionFps || '--';
    document.getElementById('dji-eis').innerText = d.stabilization || '--';
    document.getElementById('dji-temp').innerText = d.tempStatus || 'NORMAL';
    document.getElementById('val-mac-0').innerText = d.mac || '--:--:--:--:--:--';
    document.getElementById('val-pkt-0').innerText = d.packetCount || 0;
  }
  // GoPro HERO Telemetry
  else if (currentBrand === 1) {
    document.getElementById('gopro-rec-status').innerText = d.isRecording ? 'ENCODING' : 'STANDBY';
    document.getElementById('gopro-battery').innerText = (d.battery !== undefined ? d.battery : '--') + ' %';
    document.getElementById('gopro-rec-time').innerText = d.recTime || '00:00';
    document.getElementById('gopro-rem-time').innerText = d.remTime || '--:--';
    document.getElementById('gopro-rem-photos').innerText = d.remPhotos !== undefined ? d.remPhotos : '--';
    document.getElementById('gopro-rem-cap').innerText = formattedCap;
    document.getElementById('gopro-mode').innerText = d.currentMode || 'STANDARD VIDEO';
    document.getElementById('gopro-eis').innerText = d.stabilization || 'HYPERSMOOTH';
    document.getElementById('gopro-temp').innerText = d.tempStatus || 'NORMAL';
    document.getElementById('gopro-link').innerText = stateStr;
    document.getElementById('val-mac-1').innerText = d.mac || '--:--:--:--:--:--';
    document.getElementById('val-pkt-1').innerText = d.packetCount || 0;
  }
  // Insta360 Action Telemetry
  else if (currentBrand === 2) {
    document.getElementById('insta-rec-status').innerText = (stateStr === 'PAIRED' || stateStr === 'RECORDING') ? 'CONNECTED' : 'ADVERTISING';
    document.getElementById('insta-battery').innerText = '100 %';
    document.getElementById('insta-mode').innerText = d.currentMode || 'REMOTE STANDBY';
    document.getElementById('insta-res-fps').innerText = 'COMMAND ONLY';
    document.getElementById('insta-eis').innerText = 'FLOWSTATE';
    document.getElementById('insta-temp').innerText = 'NORMAL';
    document.getElementById('val-mac-2').innerText = d.mac || '--:--:--:--:--:--';
    document.getElementById('val-pkt-2').innerText = d.packetCount || 0;
  }

  // Last Raw Packet Hex
  if (d.lastHex) {
    document.getElementById('val-last-hex').innerText = d.lastHex;
  }

  // Betaflight Bridge & MSP Telemetry State Sync
  const bridgeBadge = document.getElementById('bf-bridge-badge');
  const fcStatus = document.getElementById('bf-fc-status');
  const armStatus = document.getElementById('bf-arm-status');
  const auxVal = document.getElementById('bf-aux-val');
  const bleStatus = document.getElementById('bf-ble-status');
  const rxBytes = document.getElementById('bf-rx-bytes');
  const txBytes = document.getElementById('bf-tx-bytes');
  const btnAutoArm = document.getElementById('btn-auto-arm');
  const selDisarmDly = document.getElementById('sel-disarm-dly');
  const selAuxCh = document.getElementById('sel-aux-ch');

  if (bridgeBadge) {
    if (d.bootMode === 2) {
      bridgeBadge.innerText = d.bfConnected ? 'BLE CONNECTED' : 'BLE ADVERTISING';
      bridgeBadge.className = 'badge badge-live';
    } else {
      bridgeBadge.innerText = d.bfConnected ? 'BLE CONNECTED' : 'SETUP MODE';
      bridgeBadge.className = d.bfConnected ? 'badge badge-live' : 'badge';
    }
  }

  if (fcStatus && d.fcConnected !== undefined) {
    fcStatus.innerText = d.fcConnected ? 'ONLINE (115200)' : 'OFFLINE';
    fcStatus.style.color = d.fcConnected ? '#ffffff' : 'var(--muted)';
  }

  if (armStatus && d.fcArmed !== undefined) {
    armStatus.innerText = d.fcArmed ? 'ARMED' : 'DISARMED';
    armStatus.style.color = d.fcArmed ? '#ffffff' : 'var(--muted)';
  }

  if (auxVal) {
    if (d.auxRecCh && d.auxRecCh > 0) {
      auxVal.innerText = (d.auxValue ? d.auxValue + ' µs' : '--');
      auxVal.style.color = '#ffffff';
    } else {
      auxVal.innerText = 'DISABLED';
      auxVal.style.color = 'var(--muted)';
    }
  }

  if (bleStatus) {
    bleStatus.innerText = d.bfConnected ? 'CONNECTED' : (d.bootMode === 2 ? 'ADVERTISING' : 'READY');
    bleStatus.style.color = d.bfConnected ? '#22c55e' : '#ffffff';
  }

  if (rxBytes && d.bfRxBytes !== undefined) {
    rxBytes.innerText = d.bfRxBytes + ' B';
  }
  if (txBytes && d.bfTxBytes !== undefined) {
    txBytes.innerText = d.bfTxBytes + ' B';
  }

  if (btnAutoArm && d.autoRecArm !== undefined) {
    currentAutoArm = !!d.autoRecArm;
    btnAutoArm.innerText = 'AUTO-REC ON ARM: ' + (currentAutoArm ? 'ON' : 'OFF');
    btnAutoArm.style.borderColor = currentAutoArm ? '#ffffff' : '#333333';
  }

  if (selDisarmDly && d.disarmDelaySec !== undefined && document.activeElement !== selDisarmDly) {
    selDisarmDly.value = String(d.disarmDelaySec);
  }

  if (selAuxCh && d.auxRecCh !== undefined && document.activeElement !== selAuxCh) {
    selAuxCh.value = String(d.auxRecCh);
  }
}

function connectWs() {
  if (wsReconnectTimer) {
    clearTimeout(wsReconnectTimer);
    wsReconnectTimer = null;
  }
  const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const host = window.location.host || '192.168.4.1';
  try {
    ws = new WebSocket(proto + '//' + host + '/ws');
  } catch (e) {
    wsReconnectTimer = setTimeout(connectWs, 1500);
    return;
  }

  ws.onopen = () => {
    console.log('WebSocket connected');
    pendingSwitch = false;
  };

  ws.onmessage = (evt) => {
    try {
      const data = JSON.parse(evt.data);
      if (data.type === 'logs_cleared') {
        document.getElementById('term').innerHTML = '';
        lastLogId = 0;
      } else if (data.type === 'log') {
        appendLog(data);
      } else if (data.type === 'state' || data.activeBrand !== undefined) {
        applyState(data);
      } else if (Array.isArray(data)) {
        data.forEach(appendLog);
      }
    } catch (err) {
      console.error('WS Parse Error', err);
    }
  };

  ws.onclose = () => {
    wsReconnectTimer = setTimeout(connectWs, 1500);
  };

  ws.onerror = () => {
    if (ws) ws.close();
  };
}

// Immediate initial render & WebSocket connect (no polling loops)
window.addEventListener('DOMContentLoaded', () => {
  renderCameraUI(0);
  setControlsDisabled(true, 'DISCONNECTED');

  // Initial one-shot fetch for instantaneous hydration before WS establishes
  fetch('/api/state').then(r => r.json()).then(applyState).catch(() => {});
  fetch('/api/logs?since=0').then(r => r.json()).then(logs => {
    if (Array.isArray(logs)) logs.forEach(appendLog);
  }).catch(() => {});

  connectWs();
});
</script>
</body>
</html>
)rawliteral";
