const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const root=require('node:path').join(__dirname,'..');
const source=fs.readFileSync(root+'/src/main.cpp','utf8');
const config=fs.readFileSync(root+'/include/robot_config.h','utf8');
function constant(name){return parseFloat(config.match(new RegExp('constexpr \\w+ '+name+' = ([^;]+);'))[1]);}
function extract(name){
 const match=new RegExp('void '+name+'\\([^)]*\\) \\{').exec(source);assert(match,name);
 let end=match.index+match[0].length,depth=1;
 for(;depth;end++){if(source[end]==='{')depth++;if(source[end]==='}')depth--;}
 return source.slice(match.index,end).replace(/^void /,'function ')
  .replace('uint32_t now, bool freshReading','now, freshReading').replace('uint32_t now','now')
  .replace('const String& command','command').replace(/\bconst (?:float|uint32_t|int) /g,'const ')
  .replace(/\bsize_t /g,'let ').replace(/\b(\d+(?:\.\d+)?)f\b/g,'$1')
  .replace('sizeof(float)','4').replace('DISTANCE_CALIBRATION_SAMPLES / 2','Math.floor(DISTANCE_CALIBRATION_SAMPLES / 2)')
  .replace(/sizeof\("([^"]+)"\) - 1/g,(_,prefix)=>String(prefix.length));
}
const names=['SENSOR_INTERVAL_MS','ECHO_TIMEOUT_US','TRIGGER_LOW_US','TRIGGER_HIGH_US','SOUND_SPEED_CM_PER_US',
 'SENSOR_MIN_CM','SENSOR_MAX_CM','FORWARD_DISTANCE_CM','REVERSE_DISTANCE_CM','CONTROL_TIMEOUT_MS',
 'DISTANCE_REFERENCE_CM','DISTANCE_CALIBRATION_SAMPLES','DISTANCE_CALIBRATION_TIMEOUT_MS'];
const context=vm.createContext({assert,Math,Number});
vm.runInContext(`
${names.map(name=>'var '+name+'='+constant(name)+';').join('\n')}
var REMOTE_MODE=0,DISTANCE_MODE=1,CALIBRATION_MODE=2,PID_TUNING_MODE=3,currentMode=REMOTE_MODE;
var STOPPED=0,FORWARD=1,BACKWARD=2,TURN_LEFT=3,requestedMotion=STOPPED;
var rawDistanceCM=-1,rawDistanceValid=false,distanceCM=-1,distanceValid=false,distanceOffsetCM=0;
var distanceStorageReady=false,distancePaused=false,distanceCalibrating=false,distanceReferenceSamples=new Array(DISTANCE_CALIBRATION_SAMPLES);
var distanceReferenceCount=0,distanceCalibrationStarted=0,distanceCalibrationMessage='',activity='',clock=1000,lastSensor=0;
var armed=true,armPending=false,rampActive=false,rampStartLeft=0,rampStartRight=0,targetLeft=0,targetRight=0,appliedLeft=0,appliedRight=0;
var beginOK=true,writeFail=false,stored=undefined,puts=0,stops=0,motions=[],controller=0,lastHeartbeat=1000,lastDrive=1000,disconnected=[];
var pulses=[],PIN_TRIG=10,PIN_ECHO=20,LOW=0,HIGH=1;
var wsServer={disconnect(id){disconnected.push(id);}};
var distancePrefs={begin(){return beginOK;},getFloat(key,fallback){return stored===undefined?fallback:stored;},isKey(){return stored!==undefined;},putFloat(key,value){puts++;if(writeFail)return 0;stored=value;return 4;}};
function millis(){return clock;}function isfinite(x){return Number.isFinite(x);}function max(a,b){return Math.max(a,b);}
function digitalWrite(){}function delayMicroseconds(){}function pulseInLong(){return pulses.shift()||0;}
function balanceDisarm(){armed=false;stops++;}function isSettingsMode(){return currentMode===CALIBRATION_MODE||currentMode===PID_TUNING_MODE;}
function requestMotion(motion){requestedMotion=motion;motions.push(motion);}function drive(){return false;}
function enterCalibration(){stopAndManual();currentMode=CALIBRATION_MODE;}
${['cancelDistanceCalibration','clearDriveRequest','stopMotors','stopAndManual','requestUserStop','loadDistanceCalibration',
   'startDistanceCalibration','updateDistanceCalibration','sampleDistance','handleDistanceMode','checkControlTimeouts','handleCommand'].map(extract).join('\n')}
function sample(raw){clock+=SENSOR_INTERVAL_MS;pulses.push(raw==null?0:raw*2/SOUND_SPEED_CM_PER_US);sampleDistance(clock);}
function complete(readings){readings.forEach(sample);}

loadDistanceCalibration();assert.equal(distanceOffsetCM,0);assert.equal(puts,0);
currentMode=DISTANCE_MODE;handleCommand('distance:pause');assert.equal(distancePaused,true);assert.equal(distanceCalibrating,false);assert.equal(armed,false);handleCommand('mode:distance');stops=0;
currentMode=REMOTE_MODE;startDistanceCalibration();assert.equal(stops,0,'capture accepted only in distance mode');
currentMode=DISTANCE_MODE;requestedMotion=FORWARD;startDistanceCalibration();
assert.equal(armed,false);assert.equal(requestedMotion,STOPPED);assert.equal(distancePaused,true);assert.equal(distanceCalibrating,true);
const began=distanceCalibrationStarted;updateDistanceCalibration(clock,false);assert.equal(distanceReferenceCount,0,'cached readings never count');
sample(null);assert.equal(distanceReferenceCount,0,'missing Echo never counts');
sample(32);handleCommand('distance:calibrate');assert.equal(distanceReferenceCount,1);assert.equal(distanceCalibrationStarted,began,'duplicates never restart capture');
complete([32.5,90,31.5,32]);
assert.equal(distanceCalibrating,false);assert.equal(distancePaused,true);assert.equal(puts,1);assert(Math.abs(distanceOffsetCM+12)<1e-6,'median rejects the outlier');
assert(Math.abs(distanceCM-20)<1e-6);const motionCount=motions.length;handleDistanceMode();assert.equal(motions.length,motionCount,'saved capture stays stopped');
distanceOffsetCM=0;loadDistanceCalibration();assert(Math.abs(distanceOffsetCM+12)<1e-6,'correction survives simulated restart');
handleCommand('mode:distance');assert.equal(distancePaused,false);assert.equal(distanceValid,false);
sample(40);handleDistanceMode();assert.equal(requestedMotion,FORWARD);
sample(22);handleDistanceMode();assert.equal(requestedMotion,TURN_LEFT);
sample(16);handleDistanceMode();assert.equal(requestedMotion,BACKWARD);
sample(3);handleDistanceMode();assert.equal(distanceCM,0);assert.equal(requestedMotion,BACKWARD,'near corrected reading still reverses');
requestedMotion=FORWARD;sample(null);const beforeNoEcho=motions.length;handleDistanceMode();assert.equal(distanceValid,false);assert.equal(motions.length,beforeNoEcho);assert.equal(requestedMotion,FORWARD);

writeFail=true;startDistanceCalibration();complete([36,36,36,36,36]);assert(Math.abs(distanceOffsetCM+12)<1e-6);assert.match(distanceCalibrationMessage,/failed/);assert.equal(distancePaused,true);
writeFail=false;startDistanceCalibration();const beforeTimeout=puts;sample(null);clock=distanceCalibrationStarted+DISTANCE_CALIBRATION_TIMEOUT_MS;updateDistanceCalibration(clock,false);
assert.equal(distanceCalibrating,false);assert.equal(distancePaused,true);assert.equal(puts,beforeTimeout);assert(Math.abs(distanceOffsetCM+12)<1e-6);

for(const command of ['stop','motors:off','mode:manual','mode:calibration','mode:pid','mode:distance']){
 currentMode=DISTANCE_MODE;startDistanceCalibration();sample(32);const beforeCancel=puts;
 handleCommand(command);assert.equal(distanceCalibrating,false,command+' cancels capture');updateDistanceCalibration(clock,true);assert.equal(puts,beforeCancel);
}
currentMode=DISTANCE_MODE;startDistanceCalibration();lastHeartbeat=clock-CONTROL_TIMEOUT_MS-1;checkControlTimeouts(clock);
assert.equal(distanceCalibrating,false);assert.equal(currentMode,REMOTE_MODE);assert.equal(armed,false);

controller=0;lastHeartbeat=clock;currentMode=DISTANCE_MODE;distanceStorageReady=false;startDistanceCalibration();
assert.equal(distancePaused,true);assert.equal(distanceCalibrating,false);assert.match(distanceCalibrationMessage,/unavailable/);
for(const invalid of [NaN,Infinity,1000,-1000]){stored=invalid;loadDistanceCalibration();assert.equal(distanceOffsetCM,0);}
stored=0;loadDistanceCalibration();assert.equal(distanceOffsetCM,0);assert.match(distanceCalibrationMessage,/loaded/);
beginOK=false;loadDistanceCalibration();assert.equal(distanceStorageReady,false);assert.equal(distanceOffsetCM,0);
`,context);
module.exports='PASS: actual distance calibration functions with mocked Echo/storage/time: fresh median reference, persistence, corrected movement rules, no-echo handling, save failure, timeout and cancellation';
