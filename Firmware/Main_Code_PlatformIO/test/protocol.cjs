// Verify the telemetry schema and exercise the production command router.
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict');
const root=path.join(__dirname,'..'),main=fs.readFileSync(path.join(root,'src/main.cpp'),'utf8');
function extract(source,name){
 const match=new RegExp('(?:String|void) '+name+'\\([^)]*\\) \\{').exec(source);assert(match,name);
 let end=match.index+match[0].length,depth=1;for(;depth;end++){if(source[end]==='{')depth++;if(source[end]==='}')depth--;}
 return source.slice(match.index,end).replace(/^(?:String|void) /,'function ')
  .replace(/const String& command/g,'command').replace(/const BalanceStatus /g,'const ')
  .replace(/\bString json/g,'let json').replace(/json\.reserve\(1024\);/g,'')
  .replace('auto text = [](const char* value)', 'const text = (value) =>')
  .replace('auto add = [&json](const char* key, const String& value)', 'const add = (key, value) =>')
  .replace(/json\.length\(\)/g,'json.length').replace(/\bString\(/g,'formatString(')
  .replace(/sizeof\("([^"]+)"\) - 1/g,(_,prefix)=>String(prefix.length));
}
const fixture={ready:true,armed:true,angleValid:true,angle:179.23,target:178,pwm:51,message:'Drive plus attitude correction',driveLeft:100,driveRight:110,outputLeft:151,outputRight:161,limited:false,kp:21,ki:0,kd:0.8,deadZone:2};
const context=vm.createContext({assert,JSON,fixture,formatString(value,digits){return digits===undefined?String(value):Number(value).toFixed(digits);}});
vm.runInContext(`
var MANUAL=0,DISTANCE=1,CALIBRATION=2,PID_TUNING=3,currentMode=MANUAL,activity='Stopped',calls=[];
var distanceCM=22.5,distanceValid=true,armPending=true;
var pidUpdate=3,deadZoneUpdate=2,pidMessage='PID saved',deadZoneMessage='Dead zone saved',calibrationMessage='Hold upright';
var clock=1000,lastDrive=0,lastSensor=0,lastHeartbeat=0,SENSOR_INTERVAL_MS=80;
function millis(){return clock;}function balanceStatus(){return fixture;}
function modeName(){return ['MANUAL','DISTANCE','CALIBRATION','PID_TUNING'][currentMode];}
function stopAndManual(){calls.push(['off']);currentMode=MANUAL;}
function stopMotors(){calls.push(['stopMotors']);}function requestUserStop(){calls.push(['stop']);}
function enterCalibration(){calls.push(['calibration']);currentMode=CALIBRATION;}
function saveUprightCalibration(value){calls.push(['upright',value]);}
function updatePIDSettings(value,save){calls.push(['pid',value,save]);}
function updateDeadZone(value,save){calls.push(['deadzone',value,save]);}
function drive(command){if(!['forward','backward','turnLeft','turnRight'].includes(command))return false;calls.push(['drive',command]);return true;}
${extract(main,'statusJSON')}
${extract(main,'handleCommand')}
for(const [command,expected] of [
 ['imu:save:178.50',['upright','178.50']],['pid:apply:21,0,0.8',['pid','21,0,0.8',false]],
 ['pid:save:12,1,0.4',['pid','12,1,0.4',true]],['deadzone:apply:2',['deadzone','2',false]],
 ['deadzone:save:3.5',['deadzone','3.5',true]],['forward',['drive','forward']],['stop',['stop']],['motors:off',['off']]
]){currentMode=MANUAL;calls=[];handleCommand(command);assert.deepEqual(calls.at(-1),expected);}
calls=[];handleCommand('heartbeat');assert.equal(lastHeartbeat,1000);assert.equal(calls.length,0);
handleCommand('mode:pid');assert.equal(currentMode,PID_TUNING);assert.equal(calls.at(-1)[0],'off');
calls=[];handleCommand('forward');assert.equal(calls.length,0,'settings block drive');
handleCommand('mode:distance');assert.equal(currentMode,DISTANCE);assert.equal(distanceValid,false);assert.equal(lastSensor,920);
handleCommand('mode:manual');assert.equal(currentMode,MANUAL);assert.equal(calls.at(-1)[0],'stop');
`,context);
// Expected wire schema is independent of object field order.
for(const valid of [true,false])for(const armed of [true,false]){
 Object.assign(fixture,{angleValid:valid,armed});
 vm.runInContext(`distanceValid=${valid};`,context);
 const state=JSON.parse(vm.runInContext('statusJSON()',context));
 assert.equal(Object.keys(state).length,25);assert.equal(state.mode,'MANUAL');assert.equal(state.balance_active,armed);
 assert.equal(state.distance_cm,valid?22.5:null);assert.equal(state.angle_deg,valid?179.23:null);assert.equal(state.tilt_deg,valid?1.23:null);
 assert.equal(state.pid_p,21);assert.equal(state.pid_i,0);assert.equal(state.pid_d,0.8);assert.equal(state.deadzone_deg,2);
 assert.equal(state.pid_update,3);assert.equal(state.deadzone_update,2);assert.equal(state.output_left,151);assert.equal(state.output_limited,false);
}
module.exports='PASS: actual telemetry serialization and command routing: field schema/nulls/booleans, PID/dead-zone payloads, mode transitions and drive blocking';
if(require.main===module)console.log(module.exports);
