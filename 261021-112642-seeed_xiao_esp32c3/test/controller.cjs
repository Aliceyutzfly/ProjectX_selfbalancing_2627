const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict');
const config=fs.readFileSync(path.join(__dirname,'../include/robot_config.h'),'utf8');
const pwmDeadZone=Number(config.match(/constexpr int MOTOR_PWM_DEAD_ZONE = (\d+)/)[1]);
const correctionLimit=Number(config.match(/constexpr int ATTITUDE_MAX_PWM = (\d+)/)[1]);
const source=fs.readFileSync(path.join(__dirname,'../src/balance.cpp'),'utf8');
function extract(name){
 const m=new RegExp('(?:void|bool) '+name+'\\([^)]*\\) \\{').exec(source);assert(m,name);
 let end=m.index+m[0].length,depth=1;for(;depth;end++){if(source[end]==='{')depth++;if(source[end]==='}')depth--;}
 return source.slice(m.index,end).replace(/^(?:void|bool) /,'function ').replace(/const char\* reason/g,'reason').replace(/uint32_t now/g,'now').replace(/int left, int right/g,'left, right').replace(/double kp, double ki, double kd/g,'kp, ki, kd').replace(/\bconst (?:int|bool) /g,'const ').replace(/\bbool accepted/g,'let accepted').replace(/\bint\(/g,'Number(');
}
const context=vm.createContext({assert,Math,Number});
vm.runInContext(`
var MOTOR_PWM_DEAD_ZONE=${pwmDeadZone};
var deadZone=2,MAX_DEAD_ZONE_DEG=90;
var input=181,uprightAngle=178,target=178,output=0,motorPWM=0;
var gainP=21,gainI=0,gainD=0.8,armed=false,initialized=true,angleValid=true,goodSamples=10;
var stateMutex=1,portMAX_DELAY=0,MANUAL=0,AUTOMATIC=1,lastSample=0,clock=0,lastDriveUpdate=0;
var baseLeft=0,baseRight=0,actualLeft=0,actualRight=0,outputLimited=false,message='',writes=[];
var BALANCE_OUTPUT_SIGN=-1,BALANCE_MAX_PWM=255,ATTITUDE_MAX_PWM=${correctionLimit},CONTROL_TIMEOUT_MS=800;
var PID_MAX_KP=200,PID_MAX_KI=500,PID_MAX_KD=50,pidValue=0,computeReady=true,tuneCalls=[];
function millis(){return clock;}function xSemaphoreTake(){}function xSemaphoreGive(){}
function filterPWM(x){return Math.abs(x)<MOTOR_PWM_DEAD_ZONE?0:x;}function abs(x){return Math.abs(x);}function fabs(x){return Math.abs(x);}function lround(x){return Math.round(x);}function constrain(x,a,b){return Math.max(a,Math.min(b,x));}
function isfinite(x){return Number.isFinite(x);}function motorWriter(l,r){writes.push([l,r]);}
var pid={SetMode(){},SetTunings(p,i,d){tuneCalls.push([p,i,d]);gainP=p;gainI=i;gainD=d;},Compute(){if(computeReady)output=pidValue;return computeReady;}};
${['resetPIDLocked','updateAttitudeCorrection','writeMixedMotorOutputs','disarmLocked','sensorFault','updateBalance','balanceArm','balanceSetDrive','balanceValidTunings','balanceSetTunings'].map(extract).join('\n')}

assert.equal(balanceArm(),true);
balanceSetDrive(100,80);assert.equal(writes.length,0,'publishing requests never drives hardware');
pidValue=-20;updateBalance(clock);assert.equal(actualLeft,filterPWM(120));assert.equal(actualRight,filterPWM(100));
assert.equal(outputLimited,false);assert.equal(armed,true);
balanceSetDrive(0,0);updateBalance(clock);assert.equal(motorPWM,20,'PID still computes its correction');assert.equal(actualLeft,0,'small stopped correction is suppressed');assert.equal(actualRight,0);
balanceSetDrive(-180,180);updateBalance(clock);assert.equal(actualLeft,-160);assert.equal(actualRight,200);
balanceSetDrive(254,254);pidValue=-10000;updateBalance(clock);
assert.equal(motorPWM,ATTITUDE_MAX_PWM,'bound correction separately');assert.equal(actualLeft,255);assert.equal(actualRight,255);assert.equal(outputLimited,true);
balanceSetDrive(-254,-254);pidValue=10000;updateBalance(clock);assert.equal(motorPWM,-ATTITUDE_MAX_PWM);assert.equal(actualLeft,-255);assert.equal(actualRight,-255);
computeReady=false;balanceSetDrive(0,0);updateBalance(clock);assert.equal(actualLeft,-ATTITUDE_MAX_PWM,'new base still applies when PID holds its last correction');computeReady=true;
input=180;pidValue=-50;balanceSetDrive(100,80);updateBalance(clock);assert.equal(motorPWM,0);assert.equal(output,0);assert.equal(actualLeft,filterPWM(100));assert.equal(actualRight,filterPWM(80),'angle dead zone preserves base drive before PWM filtering');
input=176;updateBalance(clock);assert.equal(motorPWM,0,'negative boundary included');
input=180.01;updateBalance(clock);assert.equal(motorPWM,50,'outside band resumes correction');
input=177;balanceSetDrive(0,0);updateBalance(clock);assert.equal(actualLeft,0);assert.equal(actualRight,0,'stopped within band has no motor output');
for(const angle of [90,130,178,225,270]){input=angle;pidValue=0;balanceSetDrive(40,30);updateBalance(clock);assert.equal(armed,true,'no angle cutoff');assert.equal(target,178,'drive never changes angle target');}
// Filter each wheel after mixing; exactly the configured threshold passes.
input=178;pidValue=0;
for(const pwm of [-151,-150,-149,-80,-50,-1,0,1,50,80,149,150,151]){
 balanceSetDrive(pwm,-pwm);updateBalance(clock);
 const expected=Math.abs(pwm)<MOTOR_PWM_DEAD_ZONE?0:pwm;
 assert.equal(actualLeft,expected);assert.equal(actualRight,expected===0?0:-expected);
 assert.equal(writes.at(-1)[0],expected,'hardware receives filtered PWM');
}
input=181;pidValue=30;balanceSetDrive(179,-180);updateBalance(clock);
assert.equal(actualLeft,0,'mixed output 149 is suppressed');assert.equal(actualRight,-210);
pidValue=-30;balanceSetDrive(120,0);updateBalance(clock);
assert.equal(actualLeft,150,'mixed output exactly 150 passes');assert.equal(actualRight,0);
clock=801;updateBalance(clock);assert.equal(armed,false);assert.equal(actualLeft,0);assert.equal(actualRight,0);assert.match(message,/timed out/);
lastSample=clock;assert.equal(balanceArm(),true);sensorFault('IMU failed');assert.equal(armed,false);assert.equal(baseLeft,0);assert.equal(actualLeft,0);assert.equal(angleValid,false);
angleValid=true;goodSamples=10;lastSample=0;assert.equal(balanceArm(),false,'freshness still required');
assert.equal(balanceValidTunings(NaN,0,1),false);assert.equal(balanceValidTunings(-1,0,1),false);assert.equal(balanceValidTunings(201,0,1),false);
assert.equal(balanceSetTunings(12,0,0.3),true);assert.equal(gainP,12);assert.equal(gainI,0);assert.equal(gainD,0.3);
lastSample=clock;balanceArm();assert.equal(balanceSetTunings(10,0,1),false,'cannot retune an active controller');assert.equal(gainP,12);
`,context);
module.exports='PASS: real mixer/lifecycle with PID/hardware mocked: feedforward plus bounded correction, final saturation, differential steering, fixed angle target, stale-input/sensor shutdown and gain bounds';
if(require.main===module)console.log(module.exports);
