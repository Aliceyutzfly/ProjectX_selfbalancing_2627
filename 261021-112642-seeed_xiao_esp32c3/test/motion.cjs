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
    .replace(/\bconst (?:BalanceStatus|uint32_t|float|int|bool) /g,'const ')
    .replace(/\b(?:float|int)\(/g,'Number(')
    .replace(/\b(\d+\.\d+)f\b/g,'$1');
}
const context = vm.createContext({assert,Math});
vm.runInContext(`
var MANUAL=0,DISTANCE=1,CALIBRATION=2,currentMode=MANUAL,controller=0;
var STOPPED=0,FORWARD=1,BACKWARD=2,TURN_LEFT=3,TURN_RIGHT=4;
var idleBalanceRequested=true,idleBalanceAttempted=false;
var armed=false,ready=true,canArm=true,angle=178,armCalls=0,BALANCE_UPRIGHT_DEG=178,BALANCE_TILT_LIMIT_DEG=20;
var requestedMotion=STOPPED,rampActive=false,rampStarted=0,rampDuration=0;
var rampStartLeft=0,rampStartRight=0,targetLeft=0,targetRight=0;
var appliedLeft=0,appliedRight=0,activity='Stopped';
var leftPWM=200,rightPWM=180,DRIVE_RAMP_MS=1000,DRIVE_START_PWM=50;
var CONTROL_TIMEOUT_MS=800,FORWARD_DISTANCE_CM=20,REVERSE_DISTANCE_CM=5;
var lastDrive=0,lastHeartbeat=0,distanceValid=false,distanceCM=-1;
var clock=0,writes=[],disconnected=[];
function millis(){return clock;}
function lroundf(x){return Math.round(x);}
function min(a,b){return Math.min(a,b);}
function fabs(x){return Math.abs(x);}
function balanceStatus(){return {armed,ready,angle,target:BALANCE_UPRIGHT_DEG};}
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
assert.equal(appliedLeft,50,'forward begins at starting PWM');assert.equal(appliedRight,50);
tick(250,true);assert.equal(appliedLeft,73);assert.equal(appliedRight,70);
const forwardStart=rampStarted;
drive('forward');assert.equal(rampStarted,forwardStart,'held command does not restart ramp');
tick(500,true);assert.equal(appliedLeft,125);assert.equal(appliedRight,115);
tick(1000,true);assert.equal(appliedLeft,200);assert.equal(appliedRight,180);
requestUserStop();
assert.equal(rampActive,false,'release must cancel the forward ramp');
assert.equal(appliedLeft,0);assert.equal(appliedRight,0);
assert.equal(armed,true,'release starts balancing in the same callback');
const releaseArms=armCalls;requestUserStop();assert.equal(armCalls,releaseArms);
tick(2500);assert.equal(appliedLeft,0,'old drive ramp must not restart');

drive('backward');assert.equal(appliedLeft,-50);assert.equal(appliedRight,-50);tick(3000,true);
assert.equal(appliedLeft,-125);assert.equal(appliedRight,-115);
drive('turnLeft');
assert.equal(appliedLeft,-200,'left turn applies immediately during a ramp');
assert.equal(appliedRight,180);assert.equal(rampActive,false);
tick(3100,true);assert.equal(appliedRight,180,'cancelled backward ramp cannot overwrite turn');
drive('turnRight');assert.equal(appliedLeft,200);assert.equal(appliedRight,-180);
requestUserStop();assert.equal(armed,true,'turn release starts balancing immediately');assert.equal(rampActive,false);
tick(3600);assert.equal(appliedLeft,0);assert.equal(appliedRight,0);
tick(4100);assert.equal(appliedLeft,0);assert.equal(appliedRight,0);

// Reverse from full forward: one shared S ramp passes through zero.
drive('forward');tick(5100,true);drive('backward');
tick(5600,true);assert.equal(appliedLeft,0);assert.equal(appliedRight,0);
tick(6100,true);assert.equal(appliedLeft,-200);assert.equal(appliedRight,-180);

stopAndManual();currentMode=DISTANCE;
sense(20.1);assert.equal(requestedMotion,FORWARD);
const autonomousStart=rampStarted;tick(6600);
assert.equal(appliedLeft,125);assert.equal(appliedRight,115);
sense(30);assert.equal(rampStarted,autonomousStart,'repeated sensor decision does not restart ramp');
tick(7100);assert.equal(appliedLeft,200,'distance mode ignores manual lastDrive age');
sense(20);assert.equal(requestedMotion,TURN_LEFT);assert.equal(appliedLeft,-200);assert.equal(appliedRight,180);
sense(5);assert.equal(requestedMotion,TURN_LEFT);assert.equal(rampActive,false);
sense(4.9);assert.equal(requestedMotion,BACKWARD);assert.equal(rampActive,true);
assert.equal(appliedRight,180,'backward transition begins from current turn PWM');
tick(7600);assert.equal(appliedLeft,-200);assert.equal(appliedRight,0);
tick(8100);assert.equal(appliedRight,-180);
sense(21);assert.equal(requestedMotion,FORWARD);assert.equal(rampActive,true);
const noEchoRampStart=rampStarted;
distanceValid=false;handleDistanceMode();
assert.equal(requestedMotion,FORWARD,'no echo keeps last forward decision');
assert.equal(rampActive,true);assert.equal(rampStarted,noEchoRampStart,'no echo must not restart the ramp');
tick(8600);assert.equal(appliedLeft,0,'existing backward-to-forward ramp continues through zero');
// Missing the first reading defaults to forward, using the normal starting PWM.
stopAndManual();currentMode=DISTANCE;distanceValid=false;handleDistanceMode();
assert.equal(requestedMotion,FORWARD);assert.equal(appliedLeft,DRIVE_START_PWM);
sense(10);distanceValid=false;handleDistanceMode();assert.equal(requestedMotion,TURN_LEFT);
assert.equal(appliedLeft,-200,'no echo preserves a previous turn');
sense(4);distanceValid=false;handleDistanceMode();assert.equal(requestedMotion,BACKWARD);


// STOP from distance exits autonomy and immediately starts balancing.
sense(10);requestUserStop();
assert.equal(currentMode,MANUAL);assert.equal(requestedMotion,STOPPED);assert.equal(rampActive,false);assert.equal(armed,true);
tick(9100);assert.equal(appliedLeft,0);assert.equal(appliedRight,0);
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

// Exact requested 50->254 curve over the current two-second duration.
stopAndManual();controller=0;leftPWM=rightPWM=254;DRIVE_RAMP_MS=2000;
lastHeartbeat=clock;drive('forward');assert.equal(appliedLeft,50);
const sampleStart=clock;
tick(sampleStart+500,true);assert.equal(appliedLeft,82);
tick(sampleStart+1000,true);assert.equal(appliedLeft,152);
tick(sampleStart+1500,true);assert.equal(appliedLeft,222);
tick(sampleStart+2000,true);assert.equal(appliedLeft,254);
stopAndManual();leftPWM=30;rightPWM=40;drive('forward');
assert.equal(appliedLeft,30,'starting PWM cannot exceed lower wheel trim');
assert.equal(appliedRight,40);

// Calibration blocks movement and balancing even if commands reach the firmware.
stopAndManual();currentMode=CALIBRATION;idleBalanceRequested=true;idleBalanceAttempted=false;
const calibrationArms=armCalls;
requestMotion(FORWARD);assert.equal(requestedMotion,STOPPED);assert.equal(rampActive,false);
serviceIdleBalance();assert.equal(armCalls,calibrationArms);
requestUserStop();assert.equal(currentMode,CALIBRATION);assert.equal(armed,false);
// Service uses the loaded runtime target, rather than the 178-degree default.
currentMode=MANUAL;BALANCE_UPRIGHT_DEG=230;angle=230;requestUserStop();serviceIdleBalance();
assert.equal(armed,true,'loaded upright target is the reference for the starting window');
`,context);
module.exports='PASS: actual movement functions with mocked hardware/time: stopped-balance activation and fault latch, S forward/backward ramps and immediate release-to-balance, immediate turns, thresholds 5/20, repeat handling, invalid echo, watchdogs and reconnect';
if(require.main===module)console.log(module.exports);
