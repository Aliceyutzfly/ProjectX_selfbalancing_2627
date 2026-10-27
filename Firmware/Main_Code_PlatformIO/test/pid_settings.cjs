// Actual firmware parsing/settings functions; storage, PID and hardware mocked.
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict');
const root=path.join(__dirname,'..');
const main=fs.readFileSync(path.join(root,'src/main.cpp'),'utf8');
const balance=fs.readFileSync(path.join(root,'src/balance.cpp'),'utf8');
function extract(source,name){
 const m=new RegExp('(?:void|bool) '+name+'\\([^)]*\\) \\{').exec(source);assert(m,name);
 let end=m.index+m[0].length,depth=1;for(;depth;end++){if(source[end]==='{')depth++;if(source[end]==='}')depth--;}
 return source.slice(m.index,end).replace(/^(?:void|bool) /,'function ')
  .replace(/const String& value, double gains\[3\]/g,'value, gains')
  .replace(/const String& value, bool save/g,'value, save')
  .replace(/double kp, double ki, double kd/g,'kp, ki, kd').replace(/double degrees/g,'degrees')
  .replace(/double (gains|saved)\[3\] = \{([^}]+)\};/g,'let $1 = [$2];')
  .replace(/double (gains|saved)\[3\];/g,'let $1 = new Array(3);')
  .replace('const char* start = value.c_str();','let start = String(value);')
  .replace('char* end = nullptr;','let end = null;')
  .replace('strtod(start, &end)','parseDegrees(start, tail => end = tail)')
  .replace("*end != ','","end[0] != ','").replace("*end != '\\0'","end !== ''")
  .replace('start = end + 1;','start = end.slice(1);')
  .replace(/sizeof\((?:gains|saved)\)/g,'24').replace(/sizeof\(double\)/g,'8')
  .replace(/\bconst (?:BalanceStatus|bool|double) /g,'const ')
  .replace(/for \(size_t i/g,'for (let i');
}
const context=vm.createContext({assert,Number,parseDegrees(text,setTail){
 const m=/^\s*[+-]?(?:(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?|inf(?:inity)?|nan)/i.exec(text);
 if(!m){setTail(text);return 0;}setTail(text.slice(m[0].length));return Number(m[0]);
}});
vm.runInContext(`
var REMOTE_MODE=0,DISTANCE_MODE=1,CALIBRATION_MODE=2,PID_TUNING_MODE=3;
var MANUAL=0,CALIBRATION=2,PID_TUNING=3,currentMode=MANUAL,armed=false,stateMutex=0,portMAX_DELAY=0;
var deadZone=2,DEFAULT_DEAD_ZONE_DEG=2,MAX_DEAD_ZONE_DEG=90,deadZoneUpdate=0,deadZoneMessage="",savedZone=undefined;
var gainP=21,gainI=0,gainD=0.8,output=0,motorPWM=0;
var BALANCE_KP=21,BALANCE_KI=0,BALANCE_KD=0.8,PID_MAX_KP=200,PID_MAX_KI=500,PID_MAX_KD=50;
var pidUpdate=0,pidStorageReady=false,pidMessage='',beginOk=true,writeFail=false,stored=undefined,blobLength=24,puts=0,stops=0,controlRequested=true;
var pid={SetMode(){},SetTunings(p,i,d){gainP=p;gainI=i;gainD=d;}};
var pidPrefs={begin(){return beginOk;},getDouble(key,fallback){return savedZone===undefined?fallback:savedZone;},putDouble(key,value){if(writeFail)return 0;savedZone=value;return 8;},getBytesLength(){return stored===undefined?0:blobLength;},getBytes(key,out,size){if(stored===undefined)return 0;for(let i=0;i<3;i++)out[i]=stored[i];return blobLength;},putBytes(key,values,size){puts++;if(writeFail)return 0;stored=values.slice();blobLength=size;return size;}};
function xSemaphoreTake(){}function xSemaphoreGive(){}function isfinite(x){return Number.isFinite(x);}
function balanceStatus(){return {deadZone,kp:gainP,ki:gainI,kd:gainD,armed};}
function isSettingsMode(){return currentMode===CALIBRATION || currentMode===PID_TUNING;}
function stopMotors(){armed=false;controlRequested=false;stops++;}
${extract(balance,'resetPIDLocked')}
${extract(balance,'balanceValidTunings')}
${extract(balance,'balanceSetTunings')}
${extract(balance,'balanceSetDeadZone')}
${extract(main,'updateDeadZone')}
${extract(main,'loadPIDSettings')}
${extract(main,'parsePIDValues')}
${extract(main,'updatePIDSettings')}

loadPIDSettings();assert.equal(deadZone,2);assert.equal(gainP,21);assert.equal(gainI,0);assert.equal(gainD,0.8);assert.equal(puts,0);
stored=[12,0.1,0.3];loadPIDSettings();assert.equal(gainP,12);assert.equal(gainI,0.1);assert.equal(gainD,0.3);
stored=[NaN,0,1];loadPIDSettings();assert.equal(gainP,21,'corrupt saved triple falls back to defaults');
stored=[15,1,1];blobLength=16;loadPIDSettings();assert.equal(gainP,21,'wrong-sized blob is ignored');
beginOk=false;loadPIDSettings();assert.equal(pidStorageReady,false);assert.equal(gainI,0);
beginOk=true;stored=undefined;blobLength=24;loadPIDSettings();
for(const bad of ['','21,0','21,,1','21,0,1,2','21,0,1x','NaN,0,1','21,Infinity,1','-1,0,1','201,0,1','21,501,1','21,0,51']){
 const n=stops;updatePIDSettings(bad,false);assert.equal(stops,n,'invalid input cannot apply '+bad);assert.equal(gainP,21);
}
armed=true;updatePIDSettings('10.000,0.000,0.200',false);
assert.equal(armed,false);assert.equal(controlRequested,false);assert.equal(gainP,10);assert.equal(gainD,0.2);assert.equal(puts,0,'Apply never writes flash');assert.match(pidMessage,/session/);
updatePIDSettings('12,0.2,0.3',true);assert.equal(puts,1,'one atomic blob write per Save');assert.equal(stored[0],12);assert.equal(stored[1],0.2);assert.equal(stored[2],0.3);
gainP=21;gainI=0;gainD=0.8;loadPIDSettings();assert.equal(gainP,12);assert.equal(gainI,0.2);assert.equal(gainD,0.3,'saved values load at simulated restart');
writeFail=true;updatePIDSettings('15,1,0.9',true);assert.equal(gainP,12);assert.equal(gainI,0.2);assert.equal(gainD,0.3);assert.equal(stored[0],12);assert.match(pidMessage,/failed/);
writeFail=false;pidStorageReady=false;const n=stops;updatePIDSettings('20,0,1',true);assert.equal(stops,n);assert.equal(gainP,12);
updatePIDSettings('20,0,1',false);assert.equal(gainP,20,'Apply works without storage');
currentMode=PID_TUNING;updatePIDSettings('9,0,0.1',false);assert.equal(currentMode,PID_TUNING);assert.equal(armed,false);
currentMode=PID_TUNING;pidStorageReady=true;writeFail=false;
updateDeadZone('3.5',false);assert.equal(deadZone,3.5);assert.equal(savedZone,undefined);assert.equal(armed,false);
updateDeadZone('1.5',true);assert.equal(savedZone,1.5);deadZone=2;loadPIDSettings();assert.equal(deadZone,1.5);
for(const bad of ['', '-1','91','NaN','2x']){updateDeadZone(bad,false);assert.equal(deadZone,1.5);}
writeFail=true;updateDeadZone('4',true);assert.equal(deadZone,1.5);assert.match(deadZoneMessage,/failed/);
writeFail=false;savedZone=NaN;loadPIDSettings();assert.equal(deadZone,2);
currentMode=MANUAL;updateDeadZone('5',false);assert.equal(deadZone,2);
currentMode=PID_TUNING;updateDeadZone('0',false);assert.equal(deadZone,0);
currentMode=CALIBRATION;updatePIDSettings('8,0,0.1',false);assert.equal(currentMode,CALIBRATION);assert.equal(armed,false);
`,context);
module.exports='PASS: actual PID setting functions with mocks: bounds/strict parsing, defaults I=0, atomic saved triple, startup load, Apply without flash, stopped retune and storage-failure rollback';
if(require.main===module)console.log(module.exports);
