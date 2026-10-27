const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict');
const source=fs.readFileSync(path.join(__dirname,'../src/main.cpp'),'utf8');
function extract(name){
 const m=new RegExp('(?:void|bool) '+name+'\\([^)]*\\) \\{').exec(source);assert(m,name);
 let end=m.index+m[0].length,depth=1;for(;depth;end++){if(source[end]==='{')depth++;if(source[end]==='}')depth--;}
 return source.slice(m.index,end).replace(/^(?:void|bool) /,'function ').replace(/const String& command/g,'command').replace(/Motion motion/g,'motion').replace(/uint32_t now/g,'now').replace(/\bconst (?:bool|BalanceStatus|uint32_t|float|int) /g,'const ').replace(/\b(?:int|float)\(/g,'Number(').replace(/\b(\d+\.\d+)f\b/g,'$1');
}
const context=vm.createContext({assert,Math});
vm.runInContext(`
var MANUAL=0,DISTANCE=1,CALIBRATION=2,PID_TUNING=3,currentMode=MANUAL,controller=0;
var STOPPED=0,FORWARD=1,BACKWARD=2,TURN_LEFT=3,TURN_RIGHT=4,requestedMotion=STOPPED;
var rampActive=false,rampStarted=0,rampStartLeft=0,rampStartRight=0;
var targetLeft=0,targetRight=0,appliedLeft=0,appliedRight=0,activity='Stopped';
var armPending=true,clock=0,lastDrive=0,lastHeartbeat=0;
var leftPWM=200,rightPWM=180,DRIVE_START_PWM=50,DRIVE_RAMP_MS=2000,CONTROL_TIMEOUT_MS=800;
var FORWARD_DISTANCE_CM=20,REVERSE_DISTANCE_CM=5,distanceCM=-1,distanceValid=false,calibrationMessage='';
var ready=true,armed=false,armCalls=0,baseLeft=0,baseRight=0,writes=[],disconnected=[];
function millis(){return clock;}function min(a,b){return Math.min(a,b);}function lroundf(x){return Math.round(x);}
function balanceSetDrive(l,r){baseLeft=l;baseRight=r;}
function balanceStatus(){return {ready,armed};}
function balanceArm(){armCalls++;return armed=ready;}
function balanceDisarm(){armed=false;baseLeft=baseRight=0;writes.push([0,0]);}
var wsServer={disconnect(id){disconnected.push(id);}};
${['isSettingsMode','clearDriveRequest','stopMotors','stopAndManual','serviceController','requestMotion','drive','updateDriveMotion','requestUserStop','handleDistanceMode','checkControlTimeouts','enterCalibration'].map(extract).join('\n')}
function tick(t,held=false){clock=t;lastHeartbeat=t;if(held)lastDrive=t;checkControlTimeouts(t);updateDriveMotion(t);serviceController();}
function sense(d){distanceValid=true;distanceCM=d;handleDistanceMode();}

serviceController();assert.equal(armed,true);const initialArms=armCalls;
drive('forward');assert.equal(armed,true,'driving must retain the same active controller');
assert.equal(baseLeft,50);assert.equal(baseRight,50);assert.equal(writes.length,0,'drive never writes hardware');
tick(500,true);assert.equal(appliedLeft,73);assert.equal(appliedRight,70);
const started=rampStarted;drive('forward');assert.equal(rampStarted,started);
tick(1000,true);assert.equal(baseLeft,125);assert.equal(baseRight,115);
tick(2000,true);assert.equal(baseLeft,200);assert.equal(baseRight,180);
assert.equal(armCalls,initialArms,'forward ramp does not re-arm/reset PID');
requestUserStop();assert.equal(baseLeft,0);assert.equal(baseRight,0);assert.equal(armed,true);
assert.equal(rampActive,false);assert.equal(armCalls,initialArms);
drive('turnLeft');assert.equal(baseLeft,-200);assert.equal(baseRight,180);assert.equal(rampActive,false);
drive('turnRight');assert.equal(baseLeft,200);assert.equal(baseRight,-180);assert.equal(writes.length,0);
drive('forward');tick(3000,true);assert.equal(baseLeft,200);assert.equal(baseRight,0);
tick(4000,true);assert.equal(baseRight,180);
drive('backward');tick(5000,true);assert.equal(baseLeft,0);assert.equal(baseRight,0);
tick(6000,true);assert.equal(baseLeft,-200);assert.equal(baseRight,-180);

stopAndManual();currentMode=DISTANCE;sense(21);assert.equal(baseLeft,50);assert.equal(armed,true);
tick(7000);assert.equal(baseLeft,125,'autonomy ignores manual command age');
const autoStart=rampStarted;distanceValid=false;handleDistanceMode();assert.equal(rampStarted,autoStart);
tick(8000);assert.equal(baseLeft,200);
sense(20);assert.equal(baseLeft,-200);assert.equal(baseRight,180);
sense(5);assert.equal(requestedMotion,TURN_LEFT);
sense(4.9);tick(9000);assert.equal(baseLeft,-200);assert.equal(baseRight,0);
distanceValid=false;handleDistanceMode();assert.equal(requestedMotion,BACKWARD);
tick(10000);assert.equal(baseRight,-180);
requestUserStop();assert.equal(currentMode,MANUAL);assert.equal(baseLeft,0);assert.equal(armed,true);

// A fault does not recover merely because the same held request keeps arriving.
drive('forward');armed=false;const afterFault=armCalls;tick(10100,true);drive('forward');serviceController();
assert.equal(armCalls,afterFault,'fault latch survives held refreshes');
requestUserStop();assert.equal(armed,true,'explicit STOP can resume after a repaired fault');
stopAndManual();tick(10200);assert.equal(armed,false,'emergency stop stays off');
drive('forward');assert.equal(armed,true,'new explicit direction resumes');
clock=11001;lastHeartbeat=clock;checkControlTimeouts(clock);assert.equal(armed,false,'stale manual request shuts down');
currentMode=DISTANCE;sense(10);clock=11802;checkControlTimeouts(clock);
assert.equal(controller,-1);assert.equal(armed,false);assert.equal(baseLeft,0);assert.equal(currentMode,MANUAL);
controller=0;lastHeartbeat=clock;serviceController();assert.equal(armed,false,'reconnect does not auto-rearm');

ready=false;requestUserStop();assert.equal(armed,false);assert.equal(armPending,true);
ready=true;serviceController();assert.equal(armed,true);
enterCalibration();assert.equal(currentMode,CALIBRATION);assert.equal(armed,false);
requestMotion(FORWARD);serviceController();assert.equal(armed,false);assert.equal(baseLeft,0);
requestUserStop();assert.equal(currentMode,CALIBRATION);assert.equal(armed,false);
currentMode=MANUAL;requestUserStop();assert.equal(armed,true);
stopAndManual();currentMode=PID_TUNING;armPending=true;
requestMotion(FORWARD);serviceController();assert.equal(armed,false);assert.equal(baseLeft,0);
requestUserStop();assert.equal(currentMode,PID_TUNING);assert.equal(armed,false,'STOP in PID settings cannot enable motors');
assert.equal(drive('invalid'),false);
`,context);
module.exports='PASS: real request/ramp functions with hardware mocked: continuous ownership during forward/backward/turn/STOP, distance/no-echo rules, S ramps, fault latch, timeouts, calibration and emergency stop';
if(require.main===module)console.log(module.exports);
