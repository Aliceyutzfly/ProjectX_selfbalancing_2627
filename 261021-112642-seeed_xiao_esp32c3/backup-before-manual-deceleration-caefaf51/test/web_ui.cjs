// Run with Node.js: node test/web_ui.cjs
// Executes the real embedded JavaScript against a small browser-event harness.
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const header = fs.readFileSync(path.join(__dirname, '../include/web_page.h'), 'utf8');
const html = header.split('R"HTML(')[1].split(')HTML"')[0];
assert(!/<(?:script|link|img)\b[^>]*(?:src|href)=['"]https?:\/\//i.test(html), 'UI must not fetch external assets');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
class Target {
  constructor() { this.events = {}; this.disabled = true; this.dataset = {}; this.classes = new Set(); this.classList = {remove: c => this.classes.delete(c), toggle: (c, yes) => yes ? this.classes.add(c) : this.classes.delete(c)}; }
  addEventListener(name, fn) { (this.events[name] ||= []).push(fn); }
  setAttribute(name, value) { this[name] = value; }
  setPointerCapture() {}
  fire(name, values = {}) { for (const fn of this.events[name] || []) fn({preventDefault() {}, ...values}); }
}
const elements = Object.fromEntries(['forward','backward','left','right','manual','distanceMode','balanceMode','disarm','balanceInfo','stop','connection','activity','distance','modeLabel'].map(id => [id, new Target()]));
const commands = {forward:'forward',backward:'backward',left:'turnLeft',right:'turnRight'};
const buttons = Object.keys(commands).map(id => { elements[id].dataset.command = commands[id]; return elements[id]; });
const document = new Target(), window = new Target();
document.hidden = false;
document.getElementById = id => elements[id];
document.querySelectorAll = () => buttons;
const intervals = [], timers = [], sockets = [];
class Socket {
  static OPEN = 1;
  constructor(url) { this.url = url; this.readyState = 1; this.sent = []; sockets.push(this); }
  send(command) { this.sent.push(command); }
  close() { this.readyState = 3; this.onclose(); }
}
vm.runInNewContext(script, {document,window,WebSocket:Socket,location:{hostname:'192.168.4.1'},setInterval:fn=>intervals.push(fn),setTimeout:fn=>timers.push(fn)});
const socket = sockets[0];
assert.equal(socket.url, 'ws://192.168.4.1:81/');
socket.onopen();
assert.equal(elements.forward.disabled, true, 'wait for actual robot state before enabling drive');
socket.onmessage({data:JSON.stringify({mode:'MANUAL',distance_cm:42,activity:'Stopped',balance_armed:false})});
assert.equal(elements.forward.disabled, false);
function pointer(button, name, pointerId = 1) { button.fire(name, {button:0,pointerId}); }
function state(mode, distance = 42) { socket.onmessage({data:JSON.stringify({mode,distance_cm:distance,activity:'Test'})}); }
function last() { return socket.sent.at(-1); }
pointer(elements.forward, 'pointerdown');
assert.equal(last(), 'forward');
intervals[0]();
assert.deepEqual(socket.sent.slice(-2), ['heartbeat','forward']);
pointer(elements.forward, 'pointerup');
assert.equal(last(), 'stop');
intervals[0](); assert.equal(last(), 'heartbeat');
for (const event of ['pointercancel','lostpointercapture']) {
  pointer(elements.left,'pointerdown'); pointer(elements.left,event);
  assert.equal(last(),'stop',event);
}
pointer(elements.forward,'pointerdown');
pointer(elements.right,'pointerdown',2);
assert.equal(last(),'forward','second touch must not replace the first');
pointer(elements.right,'pointerup',2);
assert.equal(last(),'forward','second touch must not cancel the first');
pointer(elements.forward,'pointerup');
assert.equal(last(),'stop');
document.fire('keydown',{key:'w',code:'KeyW',repeat:false}); assert.equal(last(),'forward');
document.fire('keyup',{key:'w',code:'KeyW'}); assert.equal(last(),'stop');
pointer(elements.backward,'pointerdown'); window.fire('blur'); assert.equal(last(),'stop');
elements.distanceMode.fire('click'); assert.equal(last(),'mode:distance');
state('DISTANCE'); assert.equal(elements.forward.disabled,true);
const count = socket.sent.length;
document.fire('keydown',{key:'ArrowUp',code:'ArrowUp',repeat:false});
assert.equal(socket.sent.length,count,'keyboard must not drive in distance mode');
elements.stop.fire('click'); assert.equal(last(),'stop','STOP applies in autonomous mode');
state('MANUAL',null); assert.equal(elements.distance.textContent,'No echo');
pointer(elements.forward,'pointerdown'); document.hidden = true; document.fire('visibilitychange');
assert.equal(last(),'stop');
const hiddenCount = socket.sent.length; intervals[0](); assert.equal(socket.sent.length,hiddenCount);
document.hidden = false;
pointer(elements.forward,'pointerdown'); socket.close();
assert.equal(elements.forward.disabled,true);
timers[0](); const reconnect = sockets[1]; reconnect.onopen();
const reconnectCount = reconnect.sent.length; intervals[0]();
assert.equal(reconnect.sent.length,reconnectCount+1);
assert.equal(reconnect.sent[0],'heartbeat','reconnect must not resume an old movement');
// Balance is a separate mode: only forward/backward; STOP means upright;
// disarm has its own command and connection loss must not imply power-off.
reconnect.onmessage({data:JSON.stringify({mode:'BALANCE',distance_cm:null,activity:'Balancing upright',balance_armed:true,balance_ready:true,tilt_deg:0.5,target_tilt_deg:0,balance_message:'Balancing upright'})});
assert.equal(elements.forward.disabled,false);
assert.equal(elements.backward.disabled,false);
assert.equal(elements.left.disabled,true);
assert.equal(elements.right.disabled,true);
assert.equal(elements.manual.disabled,true,'must explicitly disarm before leaving balance');
assert.equal(elements.disarm.disabled,false);
assert.equal(elements.distance.textContent,'Ultrasonic off');
assert.match(elements.balanceInfo.textContent,/Tilt: 0.5 deg/);
const balanceCount = reconnect.sent.length;
document.fire('keydown',{key:'a',code:'KeyA',repeat:false});
assert.equal(reconnect.sent.length,balanceCount,'no steering command in balance');
pointer(elements.forward,'pointerdown'); assert.equal(reconnect.sent.at(-1),'forward');
intervals[0](); assert.equal(reconnect.sent.at(-1),'forward');
pointer(elements.forward,'pointerup'); assert.equal(reconnect.sent.at(-1),'stop');
assert.match(elements.stop.textContent,/HOLD UPRIGHT/);
elements.disarm.fire('click'); assert.equal(reconnect.sent.at(-1),'balance:off');
reconnect.onclose();
assert.match(elements.activity.textContent,/Balance continues/);
assert.equal(elements.disarm.disabled,true);
// A fault must clear a held input and disable drive without automatic re-arm.
const lastTimer = timers.at(-1); lastTimer();
const third = sockets.at(-1); third.onopen();
third.onmessage({data:JSON.stringify({mode:'BALANCE',distance_cm:null,activity:'Balancing upright',balance_armed:true})});
pointer(elements.forward,'pointerdown');
third.onmessage({data:JSON.stringify({mode:'BALANCE',distance_cm:null,activity:'Tilt limit',balance_armed:false})});
assert.equal(elements.forward.disabled,true);
const faultCount = third.sent.length; intervals[0]();
assert.equal(third.sent.length,faultCount+1);
assert.equal(third.sent.at(-1),'heartbeat','fault does not keep refreshing old drive');
elements.balanceMode.fire('click'); assert.equal(third.sent.at(-1),'mode:balance');
module.exports = 'PASS: manual/obstacle regressions; balance arming controls; no steering; hold upright; explicit disarm; fault clears motion; reconnect does not resume motion; offline assets';
if (require.main === module) console.log(module.exports);
