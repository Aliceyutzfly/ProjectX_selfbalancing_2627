#pragma once
#include <Arduino.h>

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
*{box-sizing:border-box}body{margin:0;padding:24px 16px}main{max-width:460px;margin:auto}
header{display:flex;align-items:center;justify-content:space-between;gap:12px}h1{font-size:28px;margin:0}
.eyebrow{color:#84d8c0;font-size:12px;letter-spacing:2px;margin:0 0 8px}#connection{font-size:13px;color:#f5c474}
.panel{background:#1b2639;border:1px solid #34425b;border-radius:18px;padding:20px;margin-top:20px}
/* Mode selector buttons; aria-pressed also supplies their visual highlight. */
.modes{display:grid;grid-template-columns:1fr 1fr;gap:10px}button{font:inherit;border:1px solid #4e607c;border-radius:12px;background:#293951;color:#fff;min-height:52px;padding:12px;cursor:pointer}
#motorOff{width:100%;margin-top:10px;background:#8c3346;border-color:#d66d82}
button:disabled{opacity:.4;cursor:default}button[aria-pressed="true"]{background:#126e5b;border-color:#83e5c7}
button:focus-visible{outline:3px solid #83e5c7;outline-offset:3px}.hint{color:#b5c4d9;font-size:14px;line-height:1.5;margin:14px 0 0}
/* Live sensor readout and a three-column directional control pad. */
.readings{display:flex;justify-content:space-between;align-items:center;margin-bottom:16px}#distance{font-size:24px;font-variant-numeric:tabular-nums}#activity{color:#84d8c0;font-size:13px}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}.drive{font-size:28px;min-height:76px;touch-action:none;user-select:none;-webkit-user-select:none;-webkit-touch-callout:none}
.drive.active{background:#126e5b;border-color:#83e5c7}#forward{grid-column:2}#left{grid-column:1}#right{grid-column:3}#backward{grid-column:2}
/* STOP remains prominent below the movement controls. */
#stop{width:100%;background:#8c3346;border-color:#d66d82;font-weight:700;margin-top:18px;min-height:58px;touch-action:manipulation}
footer{font-size:12px;color:#92a5bf;line-height:1.6;margin-top:18px}kbd{font-family:inherit}#modeLabel{font-size:14px;color:#b5c4d9}
</style>
</head>
<!-- 3. HTML defines what is displayed. Buttons start disabled until connected. -->
<body><main>
<p class="eyebrow">XIAO / ESP32-C3</p>
<header><h1>Robot control</h1><span id="connection" role="status">Connecting...</span></header>
<!-- Filled by the robot from AP_SSID when this page is requested. -->
<p class="hint" style="overflow-wrap:anywhere">Wi-Fi: <strong>%ROBOT_SSID%</strong></p>
<section class="panel" aria-label="Driving mode">
<div class="modes"><button id="manual" aria-pressed="true" disabled>Remote drive</button><button id="distanceMode" aria-pressed="false" disabled>Avoid obstacles</button></div>
<p id="modeLabel" aria-live="polite">Remote drive</p>
<p class="hint" id="balanceInfo" aria-live="polite">STOP enables upright balancing.</p>
<p class="hint">Both modes use the same S-shaped ramp for starting and changing direction. Release/STOP slows down, then balances upright. Driving has no balance correction. Obstacle mode turns at 20 cm.</p>
</section>
<!-- Distance/activity are filled by status messages; data-command names match firmware. -->
<section class="panel" aria-label="Robot controls">
<div class="readings"><span id="distance">-- cm</span><span id="activity">Stopped</span></div>
<div class="pad">
<button class="drive" id="forward" data-command="forward" aria-label="Forward" disabled>&#8593;</button>
<button class="drive" id="left" data-command="turnLeft" aria-label="Turn left" disabled>&#8592;</button>
<button class="drive" id="right" data-command="turnRight" aria-label="Turn right" disabled>&#8594;</button>
<button class="drive" id="backward" data-command="backward" aria-label="Backward" disabled>&#8595;</button>
</div>
<button id="stop" disabled>STOP</button>
<button id="motorOff" disabled>MOTORS OFF</button>
<p class="hint">Keyboard: arrow keys or WASD. Remote release/STOP slows down, then enables balance; the wheels can move to stay upright. MOTORS OFF disables drive and balance immediately. Disconnection also switches everything off.</p>
</section>
<footer>Connect to %ROBOT_SSID% and open http://192.168.4.1.<br>No mobile data needed. Keep this page visible while driving.</footer>
</main>
<!-- 4. JavaScript connects controls to the robot over a persistent WebSocket. -->
<script>
'use strict';
// Cache page elements and track the connection, mode, and one active input.
// heldInput distinguishes a particular touch pointer from a keyboard key.
const $ = id => document.getElementById(id);
const driveButtons = [...document.querySelectorAll('.drive')];
let socket, connected = false, hasStatus = false, mode = 'MANUAL', held = null, heldInput = null;
const keys = {ArrowUp:'forward',w:'forward',ArrowDown:'backward',s:'backward',ArrowLeft:'turnLeft',a:'turnLeft',ArrowRight:'turnRight',d:'turnRight'};
// Send only while the socket is open. Returning false means nothing was sent.
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
// Normal STOP uses the manual slowdown; MOTORS OFF cuts drive immediately.
function stopAll() { clearHeld(); send('stop'); }
function canDrive(command) {
  return connected && hasStatus && mode === 'MANUAL';
}
// Accept movement commands only in manual mode.
// Ignore additional simultaneous
// fingers/keys so they cannot silently replace the active input.
function startDrive(command, input) {
  if (!canDrive(command) || heldInput !== null) return;
  held = command; heldInput = input;
  driveButtons.forEach(button => button.classList.toggle('active', button.dataset.command === command));
  send(command);
}
// Keep enabled controls and mode highlights aligned with firmware telemetry.
function updateControls() {
  const enabled = connected && hasStatus;
  driveButtons.forEach(button => button.disabled = !canDrive(button.dataset.command));
  $('manual').disabled = $('distanceMode').disabled = !enabled;
  $('stop').disabled = $('motorOff').disabled = !enabled;
  $('manual').setAttribute('aria-pressed', String(mode === 'MANUAL'));
  $('distanceMode').setAttribute('aria-pressed', String(mode === 'DISTANCE'));
  $('modeLabel').textContent = mode === 'DISTANCE' ? 'Obstacle avoidance running' : 'Remote drive';
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
// Mode/STOP buttons send the same short text commands understood by firmware.
$('manual').addEventListener('click', () => selectMode('mode:manual'));
$('distanceMode').addEventListener('click', () => selectMode('mode:distance'));
$('motorOff').addEventListener('click', () => selectMode('motors:off'));
$('stop').addEventListener('click', stopAll);
// Keyboard driving uses keydown/keyup. Ignore operating-system key repeats;
// the periodic refresh below handles continuous held motion.
document.addEventListener('keydown', event => {
  if (event.code === 'Space') { event.preventDefault(); stopAll(); return; }
  const command = keys[event.key];
  if (!command) return;
  event.preventDefault();
  if (!event.repeat) startDrive(command, event.code);
});
document.addEventListener('keyup', event => {
  if (heldInput === event.code) { event.preventDefault(); releaseDrive(); }
});
// Losing focus, hiding the tab, or leaving the page attempts an immediate
// stop. If that message cannot arrive, firmware timeouts provide the fallback.
window.addEventListener('blur', stopAll);
window.addEventListener('pagehide', stopAll);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopAll(); });
// Open a connection to this board on port 81; HTTP loaded the page on port 80.
function connect() {
  socket = new WebSocket('ws://' + location.hostname + ':81/');
  // Connected starts with no held movement. The firmware sends the current state.
  socket.onopen = () => {
    connected = true; hasStatus = false; clearHeld();
    $('connection').textContent = 'Getting robot status...';
    updateControls();
  };
  // Parse robot telemetry; the robot is the authority for the actual mode.
  // null distance is displayed as No echo rather than as a numeric distance.
  socket.onmessage = event => {
    let state;
    try { state = JSON.parse(event.data); } catch (_) { return; }
    if (!['MANUAL', 'DISTANCE'].includes(state.mode)) return;
    if (state.mode !== mode) clearHeld();
    mode = state.mode;
    hasStatus = true;
    $('connection').textContent = 'Connected';
    $('distance').textContent = state.balance_active ? 'Ultrasonic paused' : state.distance_cm === null ? 'No echo' : Number(state.distance_cm).toFixed(1) + ' cm';
    $('activity').textContent = state.activity;
    const tilt = state.tilt_deg == null ? '--' : Number(state.tilt_deg).toFixed(1);
    const pwm = Number.isFinite(state.balance_pwm) ? String(state.balance_pwm) : '--';
    $('balanceInfo').textContent = (state.balance_active ? 'Balancing on STOP' :
      state.balance_requested ? 'STOP balance unavailable' : 'Balance off') +
      ' | ' + (state.balance_message || 'IMU status unavailable') +
      ' | Tilt: ' + tilt + ' deg | PWM: ' + pwm;
    updateControls();
  };
  // Disable controls and clear held movement before scheduling a reconnect.
  socket.onclose = () => {
    connected = false; hasStatus = false; clearHeld();
    $('connection').textContent = 'Disconnected';
    $('activity').textContent = 'Disconnected - check robot';
    $('distance').textContent = '-- cm';
    updateControls();
    setTimeout(connect, 1500);
  };
  socket.onerror = () => socket.close();
}
// Held commands refresh the motor dead-man timer. Heartbeats alone never move it.
// Every 150 ms: heartbeat keeps the session alive in either mode. A held
// movement additionally refreshes the separate manual-drive watchdog.
// Hidden pages send neither. Firmware stops on a connection timeout.
setInterval(() => {
  if (!connected || document.hidden) return;
  send('heartbeat');
  if (held && canDrive(held)) send(held);
}, 150);
// Start the connection after the page elements and handlers have been created.
connect();
</script></body></html>
)HTML";
