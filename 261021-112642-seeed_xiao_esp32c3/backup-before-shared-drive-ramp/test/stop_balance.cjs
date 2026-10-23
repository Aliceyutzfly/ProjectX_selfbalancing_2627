// Execute the actual main.cpp transition functions with mocked hardware/time.
// Mechanical syntax translation only; this does not simulate PID or the chassis.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const source = fs.readFileSync(require('node:path').join(__dirname,'../src/main.cpp'),'utf8');
const names = ['stopMotors','stopAndManual','finishManualStop','drive','updateManualMotion','requestUserStop','checkControlTimeouts'];
function extract(name) {
  const match = new RegExp('(?:void|bool) '+name+'\\([^)]*\\) \\{').exec(source);
  assert(match, name);
  let end = match.index + match[0].length, depth = 1;
  for (; depth; end++) { if(source[end]==='{')depth++; if(source[end]==='}')depth--; }
  return source.slice(match.index,end)
    .replace(/^(?:void|bool) /,'function ')
    .replace(/const String& command/g,'command')
    .replace(/uint32_t now/g,'now')
    .replace(/\bconst (?:uint32_t|int32_t|float|int) /g,'const ')
    .replace(/\bint /g,'let ')
    .replace(/\b(?:int32_t|float|int)\(/g,'Number(')
    .replace(/\b(\d+\.\d+)f\b/g,'$1');
}
const context = vm.createContext({assert,Math});
vm.runInContext(`
var MANUAL=0,DISTANCE=1,currentMode=MANUAL,controller=0;
var manualMoving=false,manualStopping=false,manualStarting=false,idleBalanceRequested=false;
var manualTargetLeft=0,manualTargetRight=0,manualStartLeft=0,manualStartRight=0;
var manualAppliedLeft=0,manualAppliedRight=0,manualStopStarted=0,manualRampStarted=0;
var lastDrive=0,lastHeartbeat=0,activity='Stopped',leftPWM=254,rightPWM=254;
var MANUAL_RAMP_MS=1000,MANUAL_STOP_RAMP_MS=1000,CONTROL_TIMEOUT_MS=800;
var clock=0,armed=false,canArm=true,armCalls=0,writes=[],disconnected=[];
function millis(){return clock;}
function lroundf(x){return Math.round(x);}
function balanceStatus(){return {armed};}
function balanceArm(){armCalls++;return armed=canArm;}
function balanceDisarm(){armed=false;writes.push([0,0]);}
function balanceWriteManual(l,r){if(!armed)writes.push([l,r]);}
var wsServer={disconnect(id){disconnected.push(id);}};
${names.map(extract).join('\n')}
function tick(t){clock=t;lastHeartbeat=t;checkControlTimeouts(t);updateManualMotion(t);}
function press(command){drive(command);lastDrive=clock;manualMoving=true;}

checkControlTimeouts(0);
assert.equal(armCalls,0,'idle at boot must not arm');
requestUserStop();
assert.equal(armed,true,'explicit STOP while idle balances');
const firstArms=armCalls;
requestUserStop();
assert.equal(armCalls,firstArms,'repeated STOP must not reset active PID');
press('forward');
assert.equal(armed,false,'driving disables balance before ramp starts');
assert.equal(idleBalanceRequested,false);
tick(500);
assert.equal(manualAppliedLeft,127,'S-shaped ramp midpoint');
const rampStarted=manualRampStarted;
press('forward');
assert.equal(manualRampStarted,rampStarted,'held command must not restart ramp');
requestUserStop();
const stopStarted=manualStopStarted;
tick(750);
requestUserStop();
assert.equal(manualStopStarted,stopStarted,'repeated STOP must not extend slowdown');
assert.equal(armed,false,'no balance during manual slowdown');
tick(1500);
assert.equal(armed,true,'balance starts at slowdown completion despite 800ms drive timeout');
assert.equal(manualMoving,false);

armed=false; // Simulated sensor/tilt fault.
const afterFaultArms=armCalls;
tick(1600);tick(1700);
assert.equal(armCalls,afterFaultArms,'fault must not auto-rearm');
requestUserStop();
assert.equal(armed,true,'explicit STOP retries after fault');
stopAndManual(); // MOTORS OFF handler.
tick(1800);
assert.equal(armed,false,'motor-off remains off');
assert.equal(idleBalanceRequested,false);

requestUserStop();
clock=2701;checkControlTimeouts(clock);
assert.equal(controller,-1,'heartbeat loss releases controller');
assert.equal(armed,false,'disconnect removes balance and drive');
tick(2800);
assert.equal(armed,false,'no controller cannot auto-arm');
controller=0;lastHeartbeat=clock;stopAndManual();
assert.equal(armed,false,'reconnect remains off');
press('forward');
clock=3701;lastHeartbeat=clock;checkControlTimeouts(clock);
assert.equal(manualMoving,false,'stale movement stops even with heartbeat');
assert.equal(armed,false,'stale movement must not start balancing');

currentMode=DISTANCE;requestUserStop();
assert.equal(currentMode,MANUAL);
assert.equal(armed,true,'STOP exits autonomy into upright balancing');
stopAndManual();currentMode=DISTANCE;stopMotors();
assert.equal(armed,false,'invalid obstacle reading only stops motor output');
canArm=false;currentMode=MANUAL;requestUserStop();
const rejectedArms=armCalls;
tick(3800);
assert.equal(armCalls,rejectedArms,'rejected STOP balance must not auto-retry');
`, context);
module.exports = 'PASS: actual main.cpp STOP/drive transitions, ramps, repeated commands, fault latch, motor-off, timeouts, reconnect and autonomy exit; hardware/PID mocked';
if(require.main===module)console.log(module.exports);
