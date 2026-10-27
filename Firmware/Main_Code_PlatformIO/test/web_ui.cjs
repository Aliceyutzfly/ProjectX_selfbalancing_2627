// Run with Node.js: node test/web_ui.cjs
// Executes the real embedded JavaScript against a small browser-event harness.
const vm = require('node:vm');
const assert = require('node:assert/strict');
const {renderPage} = require('./render_page.cjs');
const html = renderPage();
const statusArea=html.match(/<section class="robot-status"[\s\S]*?<\/section>/)[0];
assert(statusArea.includes('XIAO-Robot') && statusArea.includes('id="angles"'));
assert(!html.includes('Both modes start'),'remove the long movement explanation');
assert(html.includes('min-height:104px'),'larger directional buttons');
assert(!/<(?:script|link|img)\b[^>]*(?:src|href)=['"]https?:\/\//i.test(html), 'UI must not fetch external assets');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
class Target {
  constructor() { this.events = {}; this.disabled = true; this.dataset = {}; this.classes = new Set(); this.classList = {remove: c => this.classes.delete(c), toggle: (c, yes) => yes ? this.classes.add(c) : this.classes.delete(c)}; }
  addEventListener(name, fn) { (this.events[name] ||= []).push(fn); }
  setAttribute(name, value) { this[name] = value; }
  setPointerCapture() {}
  fire(name, values = {}) { for (const fn of this.events[name] || []) fn({preventDefault() {}, ...values}); }
}
const elements = Object.fromEntries(['distanceCalibrationPanel','pauseDistance','calibrateDistance','resumeDistance','distanceReferenceHint','distanceReferenceReading','distanceCalibrationMessage','forward','backward','left','right','manual','distanceMode','motorOff','stop','connection','activity','distance','modeLabel','angles','balanceInfo','calibrationMode','calibrationPanel','uprightValue','captureUpright','saveUpright','calibrationMessage','deadZone','applyDeadZone','saveDeadZone','deadZoneActive','deadZoneMessage','pidMode','pidPanel','drivePanel','pidP','pidI','pidD','applyPID','savePID','pidActive','pidMessage'].map(id => [id, new Target()]));
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
socket.onmessage({data:JSON.stringify({mode:'MANUAL',distance_cm:42,activity:'Stopped'})});
assert.equal(elements.forward.disabled, false);
assert.equal(elements.motorOff.disabled, false, 'immediate motor-off must be available during manual ramps');
elements.motorOff.fire('click');
assert.equal(socket.sent.at(-1), 'motors:off', 'hard off is separate from the smooth stop command');
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
state('DISTANCE'); assert.equal(elements.forward.disabled,true);assert.match(elements.modeLabel.textContent,/No echo keeps moving/);
assert.equal(elements.distanceCalibrationPanel.hidden,false);
assert.match(elements.calibrateDistance.textContent,/20 cm/);
assert.equal(elements.calibrateDistance.disabled,true);
elements.pauseDistance.fire('click');assert.equal(last(),'distance:pause');
socket.onmessage({data:JSON.stringify({mode:'DISTANCE',distance_cm:32,distance_paused:true})});
assert.equal(elements.calibrateDistance.disabled,false);assert.equal(elements.pauseDistance.disabled,true);
elements.calibrateDistance.fire('click');assert.equal(last(),'distance:calibrate');
socket.onmessage({data:JSON.stringify({mode:'DISTANCE',distance_cm:20,distance_raw_cm:32,distance_offset_cm:-12,distance_calibrating:true,distance_paused:true,distance_calibration_message:'Measuring reference'})});
assert.equal(elements.calibrateDistance.disabled,true);assert.equal(elements.resumeDistance.disabled,true);assert.match(elements.modeLabel.textContent,/paused/);
const beforeDuplicate=socket.sent.length;elements.calibrateDistance.fire('click');assert.equal(socket.sent.length,beforeDuplicate);
socket.onmessage({data:JSON.stringify({mode:'DISTANCE',distance_cm:20,distance_raw_cm:32,distance_offset_cm:-12,distance_calibrating:false,distance_paused:true,distance_calibration_message:'Reference saved'})});
assert.equal(elements.calibrateDistance.disabled,false);assert.match(elements.distanceReferenceReading.textContent,/32.0 cm.*-12.0 cm/);assert.match(elements.distanceCalibrationMessage.textContent,/saved/);
elements.resumeDistance.fire('click');assert.equal(last(),'mode:distance');state('DISTANCE');
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
reconnect.onmessage({data:JSON.stringify({mode:'MANUAL',distance_cm:42,activity:'Stopped'})});
assert.equal(elements.forward.disabled,false);
assert.equal(elements.left.disabled,false);
assert.equal(elements.right.disabled,false);
elements.distanceMode.fire('click');
reconnect.onmessage({data:JSON.stringify({mode:'DISTANCE',distance_cm:15,activity:'Turn right'})});
assert.equal(elements.forward.disabled,true);
assert.equal(elements.motorOff.disabled,false);
elements.motorOff.fire('click');
assert.equal(reconnect.sent.at(-1),'motors:off');
assert(!html.includes('id="balanceMode"'),'still only two driving modes');
reconnect.onmessage({data:JSON.stringify({mode:'MANUAL',distance_cm:42,activity:'Forward',angle_deg:179,target_deg:178,tilt_deg:1,balance_active:false,balance_pwm:0,balance_message:'Balance off'})});
assert.match(elements.angles.textContent,/Angle: 179.0 deg.*Target: 178.0 deg/);
assert.equal(elements.forward.disabled,false);
reconnect.onmessage({data:JSON.stringify({mode:'DISTANCE',distance_cm:10,activity:'Turn left',angle_deg:177,target_deg:178,tilt_deg:-1,balance_active:false})});
assert.match(elements.angles.textContent,/Angle: 177.0 deg.*Target: 178.0 deg/);
reconnect.onmessage({data:JSON.stringify({mode:'MANUAL',distance_cm:null,activity:'Balancing on STOP',angle_deg:178,target_deg:178,tilt_deg:0,balance_active:true,balance_pwm:20,balance_message:'Balancing on STOP'})});
assert.match(elements.angles.textContent,/Angle: 178.0 deg.*Target: 178.0 deg/);
assert.match(elements.balanceInfo.textContent,/Attitude correction active/);
assert.equal(elements.forward.disabled,false,'balancing while stopped must not lock arrows');
assert.equal(elements.left.disabled,false);
assert.equal(elements.distance.textContent,'No echo','sensor display remains live while the controller is enabled');
reconnect.onmessage({data:JSON.stringify({mode:'MANUAL',distance_cm:null,activity:'Stopped',angle_deg:null,target_deg:178,tilt_deg:null,balance_active:false,balance_pending:true,balance_message:'IMU timeout'})});
assert.match(elements.angles.textContent,/Angle: -- deg.*Target: 178.0 deg/);
elements.calibrationMode.fire('click');assert.equal(reconnect.sent.at(-1),'mode:calibration');
reconnect.onmessage({data:JSON.stringify({mode:'CALIBRATION',distance_cm:10,activity:'Stopped',angle_deg:181.25,target_deg:178,tilt_deg:3.25,balance_active:false,balance_ready:true,calibration_message:'Hold upright'})});
assert.equal(elements.forward.disabled,true);assert.equal(elements.left.disabled,true);
assert.equal(elements.calibrationPanel.hidden,false);assert.equal(elements.captureUpright.disabled,false);
assert.equal(elements.uprightValue.value,'178.00');
elements.captureUpright.fire('click');assert.equal(elements.uprightValue.value,'181.25');
elements.saveUpright.fire('click');assert.equal(reconnect.sent.at(-1),'imu:save:181.25');
const savedCount=reconnect.sent.length;
elements.uprightValue.value='NaN';elements.saveUpright.fire('click');assert.equal(reconnect.sent.length,savedCount);
elements.uprightValue.value='';elements.saveUpright.fire('click');assert.equal(reconnect.sent.length,savedCount);
elements.uprightValue.value='178.75';
reconnect.onmessage({data:JSON.stringify({mode:'CALIBRATION',distance_cm:10,activity:'Stopped',angle_deg:181.3,target_deg:178,tilt_deg:3.3,balance_ready:true})});
assert.equal(elements.uprightValue.value,'178.75','telemetry must not overwrite an edited value');
const keyboardCount=reconnect.sent.length;document.fire('keydown',{key:'w',code:'KeyW',repeat:false});assert.equal(reconnect.sent.length,keyboardCount);
reconnect.onmessage({data:JSON.stringify({mode:'CALIBRATION',distance_cm:10,activity:'Stopped',angle_deg:181.25,target_deg:181.25,tilt_deg:0,balance_ready:true,calibration_message:'Upright angle saved'})});
assert.match(elements.angles.textContent,/Target: 181.3 deg/);assert.match(elements.calibrationMessage.textContent,/saved/);
// New PID controls initialise once, preserve edits, clear held movement, and
// distinguish runtime Apply from persistent Save.
reconnect.onmessage({data:JSON.stringify({mode:'MANUAL',distance_cm:40,activity:'Forward',angle_deg:178,target_deg:178,tilt_deg:0,balance_active:true,pid_p:21,pid_i:0,pid_d:0.8,output_left:200,output_right:180,output_limited:true})});
assert.equal(elements.pidP.value,'21');assert.equal(elements.pidI.value,'0');assert.equal(elements.pidD.value,'0.8');
assert.equal(elements.applyPID.disabled,true);assert.equal(elements.pidPanel.hidden,true);assert.match(elements.balanceInfo.textContent,/Output limited/);
elements.pidP.value='12';elements.pidI.value='0';elements.pidD.value='0.3';
reconnect.onmessage({data:JSON.stringify({mode:'MANUAL',distance_cm:40,activity:'Forward',pid_p:21,pid_i:0,pid_d:0.8})});
assert.equal(elements.pidP.value,'12','telemetry cannot overwrite edited gains');
pointer(elements.forward,'pointerdown');
elements.pidMode.fire('click');assert.equal(reconnect.sent.at(-1),'mode:pid');
reconnect.onmessage({data:JSON.stringify({mode:'PID_TUNING',distance_cm:40,activity:'PID settings - motors off'})});
assert.equal(elements.pidPanel.hidden,false);assert.equal(elements.drivePanel.hidden,true);assert.equal(elements.forward.disabled,true);assert.equal(elements.applyPID.disabled,false);
elements.applyPID.fire('click');assert.equal(reconnect.sent.at(-1),'pid:apply:12.000,0.000,0.300');
const afterApply=reconnect.sent.length;intervals[0]();
assert.equal(reconnect.sent.length,afterApply+1);assert.equal(reconnect.sent.at(-1),'heartbeat','applying gains must cancel held driving');
pointer(elements.forward,'pointerup');assert.equal(reconnect.sent.at(-1),'heartbeat','late release cannot re-enable control after apply');
reconnect.onmessage({data:JSON.stringify({mode:'PID_TUNING',distance_cm:40,activity:'Stopped',pid_p:12,pid_i:0,pid_d:0.3,pid_update:1,pid_message:'PID applied for this session - select Remote drive to resume'})});
assert.match(elements.pidActive.textContent,/Active P 12/);assert.match(elements.pidMessage.textContent,/Robot confirmed P 12/);
elements.savePID.fire('click');assert.equal(reconnect.sent.at(-1),'pid:save:12.000,0.000,0.300');
reconnect.onmessage({data:JSON.stringify({mode:'PID_TUNING',distance_cm:40,activity:'Stopped',pid_p:12,pid_i:0,pid_d:0.3,pid_update:2,pid_message:'PID save failed - previous gains restored'})});
assert.match(elements.pidMessage.textContent,/not applied as requested/);
for(const invalid of ['', '-1','201','NaN','Infinity']){elements.pidP.value=invalid;const n=reconnect.sent.length;elements.applyPID.fire('click');assert.equal(reconnect.sent.length,n);}
reconnect.onmessage({data:JSON.stringify({mode:'PID_TUNING',distance_cm:40,deadzone_deg:2,deadzone_update:0})});
assert.equal(elements.deadZone.value,'2');elements.deadZone.value='3.5';elements.applyDeadZone.fire('click');assert.equal(reconnect.sent.at(-1),'deadzone:apply:3.500');
reconnect.onmessage({data:JSON.stringify({mode:'PID_TUNING',distance_cm:40,deadzone_deg:3.5,deadzone_update:1,deadzone_message:'Dead zone applied'})});assert.match(elements.deadZoneMessage.textContent,/Robot confirmed/);assert.match(elements.deadZoneActive.textContent,/3.5/);
elements.saveDeadZone.fire('click');assert.equal(reconnect.sent.at(-1),'deadzone:save:3.500');
elements.deadZone.value='-1';const beforeBadZone=reconnect.sent.length;elements.applyDeadZone.fire('click');assert.equal(reconnect.sent.length,beforeBadZone);
elements.pidP.value='15';elements.applyPID.fire('click');timers.at(-1)();assert.match(elements.pidMessage.textContent,/No PID confirmation received/);
reconnect.onmessage({data:JSON.stringify({mode:'PID_TUNING',distance_cm:40,deadzone_deg:3.5,deadzone_update:2,deadzone_message:'Dead zone save failed - previous value restored'})});
assert.match(elements.deadZoneMessage.textContent,/not applied as requested/);
elements.deadZone.value='91';const invalidZoneCount=reconnect.sent.length;elements.applyDeadZone.fire('click');assert.equal(reconnect.sent.length,invalidZoneCount);
const editCount=reconnect.sent.length;
document.fire('keydown',{key:'d',code:'KeyD',repeat:false,target:{tagName:'INPUT'}});
document.fire('keydown',{key:'ArrowUp',code:'ArrowUp',repeat:false,target:{tagName:'INPUT'}});
assert.equal(reconnect.sent.length,editCount,'editing a PID/calibration input cannot drive the robot');
reconnect.onmessage({data:JSON.stringify({mode:'MANUAL',distance_cm:40,activity:'Stopped'})});
assert.equal(elements.pidPanel.hidden,true);assert.equal(elements.drivePanel.hidden,false);assert.equal(elements.applyPID.disabled,true);
reconnect.close();assert.match(elements.angles.textContent,/disconnected/);
assert.equal(elements.applyPID.disabled,true);assert.equal(elements.savePID.disabled,true);
assert.equal(elements.saveUpright.disabled,true);assert.equal(elements.captureUpright.disabled,true);
module.exports = 'PASS: wireless PID apply/save, edit isolation and held-input cancellation; two-mode controls, motor-off, held commands, release, keyboard, focus, reconnect, and offline assets';
if (require.main === module) console.log(module.exports);

const customized = renderPage({PID_MAX_KP:31,MIN_UPRIGHT_DEG:100,MAX_DEAD_ZONE_DEG:12,WEBSOCKET_PORT:82,HEARTBEAT_INTERVAL_MS:175,DISTANCE_REFERENCE_CM:25});
assert(customized.includes('max="31.000"') && customized.includes('min="100.000"') && customized.includes('max="12.000"'));
assert(customized.includes('wsPort:82') && customized.includes('heartbeatMS:175') && customized.includes('distanceReferenceCM:25.000'));
