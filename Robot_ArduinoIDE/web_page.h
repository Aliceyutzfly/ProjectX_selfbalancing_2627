#pragma once
#include <Arduino.h>
#include "robot_config.h"

// Embedded assets: no Internet, CDN, filesystem upload, or external images.
// 1. PROGMEM stores the page in flash. A C++ raw string literal
// allows HTML quotes/newlines without C++ escape sequences.
const char HTML_PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="theme-color" content="#101827">
<title>XIAO Robot</title>
<!-- 2. CSS controls colors, layout, button sizes, and disabled/pressed states. -->
<style>
/* Base theme and centered phone-sized content area. */
:root{color-scheme:dark;font-family:system-ui,sans-serif;background:#101827;color:#eef3fb}
*{box-sizing:border-box}body{margin:0;padding:24px 16px}main{max-width:560px;margin:auto}
header{display:flex;align-items:center;justify-content:space-between;gap:12px}h1{font-size:28px;margin:0}
.eyebrow{color:#84d8c0;font-size:12px;letter-spacing:2px;margin:0 0 8px}#connection{font-size:13px;color:#f5c474}
.panel{background:#1b2639;border:1px solid #34425b;border-radius:18px;padding:20px;margin-top:20px}
/* Mode selector buttons; aria-pressed also supplies their visual highlight. */
.modes{display:grid;grid-template-columns:1fr 1fr;gap:10px}button{font:inherit;border:1px solid #4e607c;border-radius:12px;background:#293951;color:#fff;font-size:18px;min-height:68px;padding:16px;cursor:pointer}
[hidden]{display:none!important}#uprightValue{width:100%;box-sizing:border-box;padding:12px;margin:10px 0;font:inherit;background:#101827;color:#fff;border:1px solid #4e607c;border-radius:8px}
#motorOff{width:100%;margin-top:10px;background:#8c3346;border-color:#d66d82}
button:disabled{opacity:.4;cursor:default}button[aria-pressed="true"]{background:#126e5b;border-color:#83e5c7}
button:focus-visible{outline:3px solid #83e5c7;outline-offset:3px}.hint{color:#b5c4d9;font-size:14px;line-height:1.5;margin:14px 0 0}
/* Live sensor readout and a three-column directional control pad. */
.readings{display:flex;justify-content:space-between;align-items:center;margin-bottom:16px}#distance{font-size:24px;font-variant-numeric:tabular-nums}#activity{color:#84d8c0;font-size:13px}
.pad{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));grid-template-areas:". forward ." "left stop right" ". backward .";gap:10px}.drive{font-size:38px;min-height:104px;touch-action:none;user-select:none;-webkit-user-select:none;-webkit-touch-callout:none}
.drive.active{background:#126e5b;border-color:#83e5c7}#forward{grid-area:forward}#left{grid-area:left}#right{grid-area:right}#backward{grid-area:backward}
/* STOP sits between left/right; emergency motor-off stays below the pad. */
#stop{grid-area:stop;width:100%;background:#8c3346;border-color:#d66d82;font-weight:700;margin:0;min-height:104px;padding:8px;touch-action:manipulation}
.robot-status{margin-top:16px}.robot-status .readings{margin:10px 0 0}.robot-status .hint{margin:6px 0;font-variant-numeric:tabular-nums}#captureUpright,#saveUpright{width:100%;margin-top:10px}
.pid-fields{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px}.pid-fields input{width:100%;box-sizing:border-box;padding:12px;margin-top:6px;font:inherit;background:#101827;color:#fff;border:1px solid #4e607c;border-radius:8px}#applyPID,#savePID,#applyDeadZone,#saveDeadZone{width:100%;margin-top:10px}
footer{font-size:12px;color:#92a5bf;line-height:1.6;margin-top:18px}kbd{font-family:inherit}#modeLabel{font-size:14px;color:#b5c4d9}
</style>
</head>
<!-- 3. HTML defines what is displayed. Buttons start disabled until connected. -->
<body><main>
<p class="eyebrow">XIAO / ESP32-C3</p>
<header><h1>Robot control</h1><span id="connection" role="status">Connecting...</span></header>
<!-- Filled by the robot from AP_SSID when this page is requested. -->
<section class="robot-status" aria-label="Wi-Fi and IMU readings">
<p class="hint" style="overflow-wrap:anywhere">Wi-Fi: <strong>%ROBOT_SSID%</strong></p>
<p class="hint" id="angles" aria-live="polite">Angle: -- deg | Target: -- deg | Tilt: -- deg</p>
<p class="hint" id="balanceInfo" aria-live="polite">Waiting for IMU status...</p>
<div class="readings"><span id="distance">-- cm</span><span id="activity">Stopped</span></div>
</section>
<section class="panel" aria-label="Driving mode">
<div class="modes"><button id="manual" aria-pressed="true" disabled>Remote drive</button><button id="distanceMode" aria-pressed="false" disabled>Avoid obstacles</button><button id="calibrationMode" aria-pressed="false" disabled>IMU calibration</button><button id="pidMode" aria-pressed="false" disabled>PID settings</button></div>
<p id="modeLabel" aria-live="polite">Remote: arrows drive; attitude correction stays active.</p>
</section>
<section class="panel" id="distanceCalibrationPanel" aria-label="Distance reference calibration" hidden>
<h2>Distance reference</h2>
<p class="hint" id="distanceReferenceHint"></p>
<button id="pauseDistance" disabled>Pause to position robot</button>
<button id="calibrateDistance" disabled>Set current reading as reference</button>
<button id="resumeDistance" disabled>Resume avoidance</button>
<p class="hint" id="distanceReferenceReading">Raw: -- cm | Correction: -- cm</p>
<p class="hint" id="distanceCalibrationMessage" role="status">Waiting for distance status...</p>
</section>
<section class="panel" id="calibrationPanel" aria-label="IMU calibration" hidden>
<h2>Set upright angle</h2>
<p class="hint">Hold the robot upright. Capture or enter the angle, then save.</p>
<label for="uprightValue">Upright angle (degrees)</label>
<input id="uprightValue" type="number" min="%MIN_UPRIGHT_DEG%" max="%MAX_UPRIGHT_DEG%" step="0.01" inputmode="decimal" disabled>
<button id="captureUpright" disabled>Capture current angle</button>
<button id="saveUpright" disabled>Save upright angle</button>
<p class="hint" id="calibrationMessage" role="status">Waiting for robot status...</p>
</section>
<!-- Distance/activity are filled by status messages; data-command names match firmware. -->
<section class="panel" id="drivePanel" aria-label="Robot controls">

<div class="pad">
<button class="drive" id="forward" data-command="forward" aria-label="Forward" disabled>&#8593;</button>
<button class="drive" id="left" data-command="turnLeft" aria-label="Turn left" disabled>&#8592;</button>
<button id="stop" disabled>STOP</button>
<button class="drive" id="right" data-command="turnRight" aria-label="Turn right" disabled>&#8594;</button>
<button class="drive" id="backward" data-command="backward" aria-label="Backward" disabled>&#8595;</button>
</div>

<button id="motorOff" disabled>EMERGENCY STOP</button>
</section>
<section class="panel" id="pidPanel" aria-label="Attitude PID tuning" hidden>
<h2>Attitude PID</h2>
<p class="hint">Enter all three gains, then press Apply or Save. Motors stay off here; select Remote drive to test.</p>
<div class="pid-fields">
<label for="pidP">P<input id="pidP" type="number" min="0" max="%PID_MAX_KP%" step="0.001" inputmode="decimal" disabled></label>
<label for="pidI">I<input id="pidI" type="number" min="0" max="%PID_MAX_KI%" step="0.001" inputmode="decimal" disabled></label>
<label for="pidD">D<input id="pidD" type="number" min="0" max="%PID_MAX_KD%" step="0.001" inputmode="decimal" disabled></label>
</div>
<button id="applyPID" disabled>Apply PID</button>
<button id="savePID" disabled>Save PID to robot</button>
<h3>Angle dead zone</h3>
<p class="hint">No attitude correction within this many degrees on either side of upright. Driving still works. Set 0 to disable the band.</p>
<div class="pid-fields"><label for="deadZone">+/- degrees<input id="deadZone" type="number" min="0" max="%MAX_DEAD_ZONE_DEG%" step="0.1" value="%DEFAULT_DEAD_ZONE_DEG%" inputmode="decimal" disabled></label></div>
<button id="applyDeadZone" disabled>Apply dead zone</button>
<button id="saveDeadZone" disabled>Save dead zone to robot</button>
<p class="hint" id="deadZoneActive">Active dead zone: waiting...</p>
<p class="hint" id="deadZoneMessage" role="status">Default: +/-%DEFAULT_DEAD_ZONE_DEG% degrees.</p>
<p class="hint" id="pidActive">Waiting for gains...</p>
<p class="hint" id="pidMessage" role="status">Apply is temporary; Save retains gains after restart.</p>
</section>
<footer>Keep this page visible while driving. Keyboard: arrows or WASD.</footer>
</main>
<!-- 4. JavaScript connects controls to the robot over a persistent WebSocket. -->
<script>
'use strict';
// Numeric placeholders are filled from robot_config.h when serving the page.
const config = {
  uprightMin:%MIN_UPRIGHT_DEG%, uprightMax:%MAX_UPRIGHT_DEG%,
  pidMax:[%PID_MAX_KP%,%PID_MAX_KI%,%PID_MAX_KD%], deadZoneMax:%MAX_DEAD_ZONE_DEG%,
  forwardCM:%FORWARD_DISTANCE_CM%, reverseCM:%REVERSE_DISTANCE_CM%,
  distanceReferenceCM:%DISTANCE_REFERENCE_CM%,
  wsPort:%WEBSOCKET_PORT%, heartbeatMS:%HEARTBEAT_INTERVAL_MS%,
  reconnectMS:%RECONNECT_INTERVAL_MS%, confirmationMS:%SETTINGS_CONFIRM_TIMEOUT_MS%
};
// heldInput distinguishes a particular touch pointer from a keyboard key.
const $ = id => document.getElementById(id);
const driveButtons = [...document.querySelectorAll('.drive')];
let socket, connected = false, hasStatus = false, mode = 'MANUAL', held = null, heldInput = null;
let latestAngle = null, imuReady = false;
let distanceCalibrating = false, distancePaused = false;
// Both settings forms use the same validation, send and confirmation flow.
const tuning = {
  pid: {name:'PID', fields:['pidP','pidI','pidD'], keys:['pid_p','pid_i','pid_d'], limits:config.pidMax,
    active:'pidActive', message:'pidMessage', invalid:'Enter P 0..' + config.pidMax[0] + ', I 0..' + config.pidMax[1] + ', D 0..' + config.pidMax[2] + '.',
    format:v => 'P ' + v[0] + ' | I ' + v[1] + ' | D ' + v[2]},
  deadzone: {name:'dead zone', fields:['deadZone'], keys:['deadzone_deg'], limits:[config.deadZoneMax],
    active:'deadZoneActive', message:'deadZoneMessage', invalid:'Enter a dead zone from 0 to ' + config.deadZoneMax + ' degrees.',
    format:v => '+/-' + v[0] + ' degrees'}
};
function resetTuning() {
  Object.values(tuning).forEach(form => {
    form.loaded = false; form.revision = 0; form.pending = null; form.feedback = '';
  });
}
resetTuning();
const keys = {ArrowUp:'forward',w:'forward',ArrowDown:'backward',s:'backward',ArrowLeft:'turnLeft',a:'turnLeft',ArrowRight:'turnRight',d:'turnRight'};
function send(command) {
  if (!connected || socket.readyState !== WebSocket.OPEN) return false;
  socket.send(command);
  return true;
}
// Forget a held input locally and remove button highlights; this alone
// sends no command, so reconnecting cannot resurrect an old movement.
function clearHeld() {
  held = null; heldInput = null;
  driveButtons.forEach(button => button.classList.remove('active'));
}
// Releasing a movement sends STOP only if there was actually a held input.
function releaseDrive() {
  const wasHeld = held !== null;
  clearHeld();
  if (wasHeld) send('stop');
}
// STOP clears base driving; emergency stop disables the controller too.
function stopAll() { clearHeld(); send('stop'); }
function canDrive() {
  return connected && hasStatus && mode === 'MANUAL';
}
// Manual mode accepts one touch/key at a time.
function startDrive(command, input) {
  if (!canDrive() || heldInput !== null) return;
  held = command; heldInput = input;
  driveButtons.forEach(button => button.classList.toggle('active', button.dataset.command === command));
  send(command);
}
function updateControls() {
  const manual = mode === 'MANUAL', distance = mode === 'DISTANCE';
  const calibration = mode === 'CALIBRATION', tuning = mode === 'PID_TUNING';
  const enabled = connected && hasStatus;
  driveButtons.forEach(button => button.disabled = !canDrive());
  $('manual').disabled = $('distanceMode').disabled = !enabled;
  $('stop').disabled = $('motorOff').disabled = !enabled;
  $('calibrationMode').disabled = $('pidMode').disabled = !enabled;
  $('distanceCalibrationPanel').hidden = !distance;
  $('pauseDistance').disabled = !enabled || !distance || distanceCalibrating || distancePaused;
  $('calibrateDistance').disabled = $('resumeDistance').disabled = !enabled || !distance || distanceCalibrating || !distancePaused;
  $('calibrateDistance').textContent = 'Set current reading as ' + config.distanceReferenceCM + ' cm';
  $('distanceReferenceHint').textContent = 'Pause, then position the robot ' + config.distanceReferenceCM + ' cm from a wall or flat object and set the reference. Keep it still until saved, then press Resume avoidance.';
  $('pidMode').setAttribute('aria-pressed', String(tuning));
  $('pidPanel').hidden = !tuning;
  $('drivePanel').hidden = tuning;
  ['pidP','pidI','pidD','applyPID','savePID','deadZone','applyDeadZone','saveDeadZone'].forEach(id => $(id).disabled = !enabled || !tuning);
  $('calibrationMode').setAttribute('aria-pressed', String(calibration));
  $('calibrationPanel').hidden = !calibration;
  $('uprightValue').disabled = !enabled || !calibration;
  $('captureUpright').disabled = !enabled || !calibration || !imuReady || latestAngle === null;
  $('saveUpright').disabled = !enabled || !calibration || !imuReady;
  $('manual').setAttribute('aria-pressed', String(manual));
  $('distanceMode').setAttribute('aria-pressed', String(distance));
  $('modeLabel').textContent = tuning ? 'PID settings: adjust and save gains; motors off.' : calibration ? 'Calibration: set and save upright angle; motors off.' : distance && distancePaused ? 'Distance paused: retry calibration or press Resume avoidance.' : distance ? 'Distance: forward above ' + config.forwardCM + ' cm; reverse below ' + config.reverseCM + ' cm; otherwise left. No echo keeps moving.' : 'Remote: arrows drive; attitude correction stays active.';
}
// Firmware changes the mode; the browser waits for its status acknowledgement.
function selectMode(command) { clearHeld(); send(command); }
// Pointer events support mouse, touch, and pen. Pointer capture ensures
// release is detected even if the finger moves outside the original button.
driveButtons.forEach(button => {
  button.addEventListener('pointerdown', event => {
    if (event.button !== 0) return;
    event.preventDefault();
    if (heldInput !== null) return;
    button.setPointerCapture(event.pointerId);
    startDrive(button.dataset.command, event.pointerId);
  });
  const end = event => { if (heldInput === event.pointerId) releaseDrive(); };
  button.addEventListener('pointerup', end);
  button.addEventListener('pointercancel', end);
  button.addEventListener('lostpointercapture', end);
  button.addEventListener('contextmenu', event => event.preventDefault());
});
$('manual').addEventListener('click', () => selectMode('mode:manual'));
$('distanceMode').addEventListener('click', () => selectMode('mode:distance'));
$('resumeDistance').addEventListener('click', () => selectMode('mode:distance'));
$('pauseDistance').addEventListener('click', () => selectMode('distance:pause'));
$('calibrateDistance').addEventListener('click', () => {
  if (!connected || !hasStatus || mode !== 'DISTANCE' || distanceCalibrating || !distancePaused) return;
  clearHeld();
  send('distance:calibrate');
});
$('motorOff').addEventListener('click', () => selectMode('motors:off'));
$('stop').addEventListener('click', stopAll);
$('calibrationMode').addEventListener('click', () => selectMode('mode:calibration'));
$('pidMode').addEventListener('click', () => selectMode('mode:pid'));
$('captureUpright').addEventListener('click', () => {
  if (mode === 'CALIBRATION' && imuReady && latestAngle !== null) $('uprightValue').value = latestAngle.toFixed(2);
});
$('saveUpright').addEventListener('click', () => {
  if (mode !== 'CALIBRATION' || !imuReady) return;
  const text = String($('uprightValue').value || '').trim();
  const value = Number(text);
  if (!text || !Number.isFinite(value) || value < config.uprightMin || value > config.uprightMax) {
    $('calibrationMessage').textContent = 'Enter an angle from ' + config.uprightMin + ' to ' + config.uprightMax + ' degrees.';
    return;
  }
  send('imu:save:' + value.toFixed(2));
});
// Typing never applies a setting. Only these buttons send it to the robot.
function submitTuning(kind, save) {
  if (!connected || !hasStatus || mode !== 'PID_TUNING') return;
  const form = tuning[kind];
  const raw = form.fields.map(id => String($(id).value || '').trim());
  const values = raw.map(Number);
  if (raw.some(value => !value) || values.some((v,i) => !Number.isFinite(v) || v < 0 || v > form.limits[i])) {
    $(form.message).textContent = form.invalid;
    return;
  }
  clearHeld();
  const request = {values:values.map(v => Number(v.toFixed(3))), revision:form.revision};
  form.pending = request;
  form.feedback = 'Waiting for robot confirmation...';
  $(form.message).textContent = form.feedback;
  if (!send(kind + (save ? ':save:' : ':apply:') + request.values.map(v => v.toFixed(3)).join(','))) {
    form.pending = null;
    $(form.message).textContent = form.feedback = 'Settings not sent - reconnect and try again.';
    return;
  }
  setTimeout(() => {
    if (form.pending !== request) return;
    form.pending = null;
    $(form.message).textContent = form.feedback = 'No ' + form.name + ' confirmation received - check connection and uploaded firmware.';
  }, config.confirmationMS);
}

function showTuning(kind, state) {
  const form = tuning[kind], values = form.keys.map(key => state[key]);
  const valid = values.every(Number.isFinite), message = state[kind + '_message'] || '';
  if (valid) {
    if (!form.loaded) form.fields.forEach((id,i) => $(id).value = String(values[i]));
    form.loaded = true;
    $(form.active).textContent = 'Active ' + (kind === 'deadzone' ? 'dead zone: ' : '') + form.format(values);
  }
  const revision = state[kind + '_update'];
  if (Number.isFinite(revision)) {
    form.revision = revision;
    if (form.pending && revision > form.pending.revision) {
      const matches = valid && values.every((v,i) => Math.abs(v - form.pending.values[i]) < 0.0005);
      form.feedback = matches && !/failed|cannot|invalid/i.test(message) ?
        'Robot confirmed ' + form.format(values) + '. ' + message : form.name + ' not applied as requested. ' + message;
      form.pending = null;
    }
  }
  $(form.message).textContent = form.feedback || message || 'Apply is temporary; Save remembers settings after restart.';
}
$('applyPID').addEventListener('click', () => submitTuning('pid', false));
$('savePID').addEventListener('click', () => submitTuning('pid', true));
$('applyDeadZone').addEventListener('click', () => submitTuning('deadzone', false));
$('saveDeadZone').addEventListener('click', () => submitTuning('deadzone', true));
// Keyboard driving uses keydown/keyup. Ignore operating-system key repeats;
// the periodic refresh below handles continuous held motion.
document.addEventListener('keydown', event => {
  if (event.target && ['INPUT','TEXTAREA','SELECT'].includes(event.target.tagName)) return;
  if (event.code === 'Space') { event.preventDefault(); stopAll(); return; }
  const command = keys[event.key];
  if (!command) return;
  event.preventDefault();
  if (!event.repeat) startDrive(command, event.code);
});
document.addEventListener('keyup', event => {
  if (heldInput === event.code) { event.preventDefault(); releaseDrive(); }
});
// Losing focus, hiding the tab, or leaving the page sends STOP to
// request upright balancing. If that message cannot arrive, firmware timeouts provide the fallback.
window.addEventListener('blur', stopAll);
window.addEventListener('pagehide', stopAll);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopAll(); });
function connect() {
  socket = new WebSocket('ws://' + location.hostname + ':' + config.wsPort + '/');
  // Connected starts with no held movement. The firmware sends the current state.
  socket.onopen = () => {
    connected = true;
    distanceCalibrating = distancePaused = false;
    hasStatus = false;
    resetTuning();
    clearHeld();
    $('connection').textContent = 'Getting robot status...';
    updateControls();
  };
  // Parse robot telemetry; the robot is the authority for the actual mode.
  // null distance is displayed as No echo rather than as a numeric distance.
  socket.onmessage = event => {
    let state;
    try { state = JSON.parse(event.data); } catch (_) { return; }
    if (!['MANUAL', 'DISTANCE', 'CALIBRATION', 'PID_TUNING'].includes(state.mode)) return;
    const enteringCalibration = state.mode === 'CALIBRATION' && mode !== 'CALIBRATION';
    if (state.mode !== mode) clearHeld();
    mode = state.mode;
    hasStatus = true;
    latestAngle = Number.isFinite(state.angle_deg) ? state.angle_deg : null;
    imuReady = state.balance_ready === true;
    if (enteringCalibration && Number.isFinite(state.target_deg)) $('uprightValue').value = state.target_deg.toFixed(2);
    $('calibrationMessage').textContent = state.calibration_message || 'Hold upright, capture or enter an angle, then save.';
    $('connection').textContent = 'Connected';
    $('distance').textContent = state.distance_cm === null ? 'No echo' : Number(state.distance_cm).toFixed(1) + ' cm';
    $('activity').textContent = state.activity;
    distanceCalibrating = state.distance_calibrating === true;
    distancePaused = state.distance_paused === true;
    const rawDistance = Number.isFinite(state.distance_raw_cm) ? state.distance_raw_cm.toFixed(1) : '--';
    const correction = Number.isFinite(state.distance_offset_cm) ? state.distance_offset_cm.toFixed(1) : '--';
    $('distanceReferenceReading').textContent = 'Raw: ' + rawDistance + ' cm | Correction: ' + correction + ' cm';
    $('distanceCalibrationMessage').textContent = state.distance_calibration_message || 'Position the robot at the reference distance, then calibrate.';
    showTuning('pid', state);
    showTuning('deadzone', state);
    const degrees = value => value == null ? '--' : Number(value).toFixed(1);
    $('angles').textContent = 'Angle: ' + degrees(state.angle_deg) + ' deg | Target: ' +
      degrees(state.target_deg) + ' deg | Tilt: ' + degrees(state.tilt_deg) + ' deg';
    const pwm = Number.isFinite(state.balance_pwm) ? String(state.balance_pwm) : '--';
    const left = Number.isFinite(state.output_left) ? state.output_left : '--';
    const right = Number.isFinite(state.output_right) ? state.output_right : '--';
    $('balanceInfo').textContent = (state.balance_active ? 'Attitude correction active' :
      state.balance_pending ? 'Waiting for controller' : 'Controller off') +
      ' | Correction PWM: ' + pwm + ' | Wheels L ' + left + ' R ' + right +
      (state.output_limited ? ' | Output limited' : '') +
      (!state.balance_active ? ' | ' + (state.balance_message || 'IMU status unavailable') : '');
    updateControls();
  };
  // Disable controls and clear held movement before scheduling a reconnect.
  socket.onclose = () => {
    connected = hasStatus = imuReady = false;
    distanceCalibrating = distancePaused = false;
    latestAngle = null;
    resetTuning();
    clearHeld();
    $('connection').textContent = 'Disconnected';
    $('angles').textContent = 'Angle: -- deg | Target: -- deg | Tilt: -- deg (disconnected)';
    $('balanceInfo').textContent = 'Disconnected - no live IMU data';
    $('activity').textContent = 'Disconnected - check robot';
    $('distance').textContent = '-- cm';
    updateControls();
    setTimeout(connect, config.reconnectMS);
  };
  socket.onerror = () => socket.close();
}
// Held commands refresh the motor dead-man timer. Heartbeats alone never move it.
// Each heartbeat keeps the session alive in every mode. A held
// movement additionally refreshes the separate manual-drive watchdog.
// Hidden pages send neither. Firmware stops on a connection timeout.
setInterval(() => {
  if (!connected || document.hidden) return;
  send('heartbeat');
  if (held && canDrive()) send(held);
}, config.heartbeatMS);
connect();
</script></body></html>
)HTML";

// Page values and validation limits come from the same header as the firmware.
inline String renderControlPage() {
  String robotName = AP_SSID;
  robotName.replace("&", "&amp;");
  robotName.replace("<", "&lt;");
  robotName.replace(">", "&gt;");
  robotName.replace("\"", "&quot;");
  robotName.replace("'", "&#39;");
  String page = FPSTR(HTML_PAGE);
  page.replace("%ROBOT_SSID%", robotName);
  page.replace("%MIN_UPRIGHT_DEG%", String(MIN_UPRIGHT_DEG, 3));
  page.replace("%MAX_UPRIGHT_DEG%", String(MAX_UPRIGHT_DEG, 3));
  page.replace("%PID_MAX_KP%", String(PID_MAX_KP, 3));
  page.replace("%PID_MAX_KI%", String(PID_MAX_KI, 3));
  page.replace("%PID_MAX_KD%", String(PID_MAX_KD, 3));
  page.replace("%MAX_DEAD_ZONE_DEG%", String(MAX_DEAD_ZONE_DEG, 3));
  page.replace("%DEFAULT_DEAD_ZONE_DEG%", String(DEFAULT_DEAD_ZONE_DEG, 3));
  page.replace("%FORWARD_DISTANCE_CM%", String(FORWARD_DISTANCE_CM, 3));
  page.replace("%REVERSE_DISTANCE_CM%", String(REVERSE_DISTANCE_CM, 3));
  page.replace("%DISTANCE_REFERENCE_CM%", String(DISTANCE_REFERENCE_CM, 3));
  page.replace("%WEBSOCKET_PORT%", String(WEBSOCKET_PORT));
  page.replace("%HEARTBEAT_INTERVAL_MS%", String(HEARTBEAT_INTERVAL_MS));
  page.replace("%RECONNECT_INTERVAL_MS%", String(RECONNECT_INTERVAL_MS));
  page.replace("%SETTINGS_CONFIRM_TIMEOUT_MS%", String(SETTINGS_CONFIRM_TIMEOUT_MS));
  return page;
}
