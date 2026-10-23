// Runs the actual main.cpp movement/timeout functions with mocked time and motors.
// Mechanical C++ syntax translation; not a native firmware or physics simulation.
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const source = fs.readFileSync(path.join(__dirname,'../src/main.cpp'),'utf8');
const names = ['stopMotors','serviceIdleBalance','stopAndManual','requestMotion','drive','updateDriveMotion','requestUserStop','handleDistanceMode','checkControlTimeouts'];
function extract(name) {
  const match = new RegExp('(?:void|bool) '+name+'\\([^)]*\\) \\{').exec(source);
  assert(match,name);
  let end = match.index + match[0].length, depth = 1;
  for (; depth; end++) { if(source[end]==='{')depth++; if(source[end]==='}')depth--; }
  return source.slice(match.index,end)
    .replace(/^(?:void|bool) /,'function ')
    .replace(/const String& command/g,'command')
    .replace(/uint32_t now/g,'now')
    .replace(/Motion motion/g,'motion')
    .replace(/\bconst (?:BalanceStatus|uint32_t|float|int) /g,'const ')
    .replace(/\b(?:float|int)\(/g,'Number(')
    .replace(/\b(\d+\.\d+)f\b/g,'$1');
}
const context = vm.createContext({assert,Math});
vm.runInContext(`
var MANUAL=0,DISTANCE=1,currentMode=MANUAL,controller=0;
var STOPPED=0,FORWARD=1,BACKWARD=2,TURN_LEFT=3,TURN_RIGHT=4;
var idleBalanceRequested=true,idleBalanceAttempted=false;
var armed=false,ready=true,canArm=true,angle=178,armCalls=0,BALANCE_UPRIGHT_DEG=178,BALANCE_TILT_LIMIT_DEG=20;
var requestedMotion=STOPPED,rampActive=false,rampStarted=0,rampDuration=0;
var rampStartLeft=0,rampStartRight=0,targetLeft=0,targetRight=0;
var appliedLeft=0,appliedRight=0,activity='Stopped';
var leftPWM=200,rightPWM=180,DRIVE_RAMP_MS=1000,STOP_RAMP_MS=1000;
var CONTROL_TIMEOUT_MS=800,FORWARD_DISTANCE_CM=20,REVERSE_DISTANCE_CM=5;
var lastDrive=0,lastHeartbeat=0,distanceValid=false,distanceCM=-1;
var clock=0,writes=[],disconnected=[];
function millis(){return clock;}
function lroundf(x){return Math.round(x);}
function fabs(x){return Math.abs(x);}
function balanceStatus(){return {armed,ready,angle};}
function balanceArm(){armCalls++;return armed=canArm;}
function balanceDisarm(){armed=false;moveMotor(0,0);}
function balanceWriteManual(l,r){if(!armed)moveMotor(l,r);}
function moveMotor(l,r){writes.push([l,r]);}
var wsServer={disconnect(id){disconnected.push(id);}};
${names.map(extract).join('\n')}
function tick(t,held=false){clock=t;lastHeartbeat=t;if(held)lastDrive=t;checkControlTimeouts(t);updateDriveMotion(t);serviceIdleBalance();}
function sense(d){distanceCM=d;distanceValid=true;handleDistanceMode();}

assert.equal(writes.length,0,'initialization does not drive');
serviceIdleBalance();assert.equal(armed,true,'initially stopped robot starts balancing when ready/upright');
const firstArmCalls=armCalls;serviceIdleBalance();assert.equal(armCalls,firstArmCalls);

assert.equal(drive('invalid'),false);
drive('forward');lastDrive=clock;assert.equal(armed,false,'movement disables balance');
assert.equal(appliedLeft,0,'forward does not jump to full PWM');
tick(250,true);assert.equal(appliedLeft,31);assert.equal(appliedRight,28);
const forwardStart=rampStarted;
drive('forward');assert.equal(rampStarted,forwardStart,'held command does not restart ramp');
tick(500,true);assert.equal(appliedLeft,100);assert.equal(appliedRight,90);
tick(1000,true);assert.equal(appliedLeft,200);assert.equal(appliedRight,180);
requestUserStop();
tick(1250);assert.equal(appliedLeft,169,'STOP must be S shaped, not linear 150');
const stopStart=rampStarted;requestUserStop();assert.equal(rampStarted,stopStart);
tick(1500);assert.equal(appliedLeft,100);assert.equal(appliedRight,90);
tick(2000);assert.equal(appliedLeft,0);assert.equal(appliedRight,0);
assert.equal(activity,'Stopped');assert.equal(rampActive,false);assert.equal(armed,true,'S stop hands off to balance');
tick(2500);assert.equal(appliedLeft,0,'no restart after normal STOP');

drive('backward');tick(3000,true);
assert.equal(appliedLeft,-100);assert.equal(appliedRight,-90);
drive('turnLeft');
assert.equal(appliedLeft,-200,'left turn applies immediately during a ramp');
assert.equal(appliedRight,180);assert.equal(rampActive,false);
tick(3100,true);assert.equal(appliedRight,180,'cancelled backward ramp cannot overwrite turn');
drive('turnRight');assert.equal(appliedLeft,200);assert.equal(appliedRight,-180);
requestUserStop();tick(3600);assert.equal(appliedLeft,100);assert.equal(appliedRight,-90);
tick(4100);assert.equal(appliedLeft,0);assert.equal(appliedRight,0);

// Reverse from full forward: one shared S ramp passes through zero.
drive('forward');tick(5100,true);drive('backward');
tick(5600,true);assert.equal(appliedLeft,0);assert.equal(appliedRight,0);
tick(6100,true);assert.equal(appliedLeft,-200);assert.equal(appliedRight,-180);

stopAndManual();currentMode=DISTANCE;
sense(20.1);assert.equal(requestedMotion,FORWARD);
const autonomousStart=rampStarted;tick(6600);
assert.equal(appliedLeft,100);assert.equal(appliedRight,90);
sense(30);assert.equal(rampStarted,autonomousStart,'repeated sensor decision does not restart ramp');
tick(7100);assert.equal(appliedLeft,200,'distance mode ignores manual lastDrive age');
sense(20);assert.equal(requestedMotion,TURN_LEFT);assert.equal(appliedLeft,-200);assert.equal(appliedRight,180);
sense(5);assert.equal(requestedMotion,TURN_LEFT);assert.equal(rampActive,false);
sense(4.9);assert.equal(requestedMotion,BACKWARD);assert.equal(rampActive,true);
assert.equal(appliedRight,180,'backward transition begins from current turn PWM');
tick(7600);assert.equal(appliedLeft,-200);assert.equal(appliedRight,0);
tick(8100);assert.equal(appliedRight,-180);
sense(21);assert.equal(requestedMotion,FORWARD);assert.equal(rampActive,true);
distanceValid=false;handleDistanceMode();
assert.equal(appliedLeft,0,'invalid echo stops immediately');assert.equal(appliedRight,0);
assert.equal(rampActive,false);assert.equal(activity,'No valid echo');
tick(8600);assert.equal(appliedLeft,0,'invalid echo cancels future ramp writes');

// Normal STOP from distance cancels autonomy but preserves a smooth slowdown.
sense(10);requestUserStop();
assert.equal(currentMode,MANUAL);assert.equal(requestedMotion,STOPPED);assert.equal(rampActive,true);
tick(9100);assert.equal(appliedLeft,-100);assert.equal(appliedRight,90);
tick(9600);assert.equal(appliedLeft,0);
drive('forward');tick(10100,true);stopAndManual();
tick(10600);assert.equal(appliedLeft,0,'MOTORS OFF cancels ramp');
drive('forward');clock=11401;lastHeartbeat=clock;checkControlTimeouts(clock);
assert.equal(appliedLeft,0);assert.equal(requestedMotion,STOPPED,'manual watchdog stops drive');
currentMode=DISTANCE;sense(10);clock=12202;checkControlTimeouts(clock);
assert.equal(controller,-1);assert.equal(currentMode,MANUAL);assert.equal(appliedLeft,0);
assert.equal(appliedRight,0);assert.equal(rampActive,false);
controller=0;lastHeartbeat=clock;stopAndManual();tick(12300);
assert.equal(appliedLeft,0,'reconnect never resumes motion');
assert.equal(armed,false,'reconnect after safety stop remains off');
ready=false;requestUserStop();serviceIdleBalance();
assert.equal(idleBalanceAttempted,false,'normal STOP waits for fresh IMU');
ready=true;angle=199;serviceIdleBalance();assert.equal(armed,false,'outside 20 degrees must wait');
angle=198;serviceIdleBalance();assert.equal(armed,true,'20 degree boundary may start balancing');
const activeArms=armCalls;requestUserStop();serviceIdleBalance();
assert.equal(armCalls,activeArms,'repeated STOP does not reset active PID');
armed=false; // Simulate a latched sensor/fall fault in the background task.
serviceIdleBalance();assert.equal(armCalls,activeArms,'fault must not auto-rearm');
angle=178;requestUserStop();serviceIdleBalance();assert.equal(armed,true,'explicit STOP retries after fault');
stopAndManual();serviceIdleBalance();assert.equal(armed,false,'MOTORS OFF stays off');
controller=-1;idleBalanceRequested=true;idleBalanceAttempted=false;
checkControlTimeouts(clock);serviceIdleBalance();assert.equal(armed,true,'initial stationary balance may operate without browser');
stopAndManual();checkControlTimeouts(clock);serviceIdleBalance();assert.equal(armed,false,'disabled balance does not restart without browser');

`,context);
module.exports='PASS: actual movement functions with mocked hardware/time: stopped-balance activation and fault latch, shared S forward/backward/STOP, immediate turns, thresholds 5/20, repeat handling, invalid echo, watchdogs and reconnect';
if(require.main===module)console.log(module.exports);
