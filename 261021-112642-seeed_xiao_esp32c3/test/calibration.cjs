// Execute actual firmware calibration functions with mocked Preferences/IMU.
// Syntax translation does not test NVS hardware, I2C, or RTOS scheduling.
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict');
const root=path.join(__dirname,'..');
const main=fs.readFileSync(path.join(root,'src/main.cpp'),'utf8');
const balance=fs.readFileSync(path.join(root,'src/balance.cpp'),'utf8');
function extract(source,name){
  const match=new RegExp('(?:void|bool) '+name+'\\([^)]*\\) \\{').exec(source);assert(match,name);
  let end=match.index+match[0].length,depth=1;
  for(;depth;end++){if(source[end]==='{')depth++;if(source[end]==='}')depth--;}
  return source.slice(match.index,end)
    .replace(/^(?:void|bool) /,'function ')
    .replace(/const String& value/g,'value').replace(/double degrees/g,'degrees')
    .replace('const char* start = value.c_str();','const start = String(value);')
    .replace('char* end = nullptr;','let end = null;')
    .replace('strtod(start, &end)','parseDegrees(start, tail => end = tail)')
    .replace("*end != '\\0'","end !== ''")
    .replace(/\bconst (?:double|bool|BalanceStatus) /g,'const ')
    .replace(/sizeof\(double\)/g,'8');
}
const context=vm.createContext({assert,Number,parseDegrees(text,setTail){
  const match=/^\s*[+-]?(?:(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?|inf(?:inity)?|nan)/i.exec(text);
  if(!match){setTail(text);return 0;}
  setTail(text.slice(match[0].length));return Number(match[0]);
}});
vm.runInContext(`
var MANUAL=0,CALIBRATION=2,currentMode=MANUAL;
var armed=false,ready=true,target=178,uprightAngle=178,output=0,motorPWM=0;
var MIN_UPRIGHT_DEG=90,MAX_UPRIGHT_DEG=270,DEFAULT_BALANCE_UPRIGHT_DEG=178;
var stateMutex=1,portMAX_DELAY=0,imuStorageReady=false,calibrationMessage='';
var beginOk=true,writeFail=false,stored=undefined,puts=0,motorsOff=false;
var imuPrefs={begin(){return beginOk;},getDouble(key,fallback){return stored===undefined?fallback:stored;},putDouble(key,value){puts++;if(writeFail)return 0;stored=value;return 8;}};
var pid={SetMode(){}};
function xSemaphoreTake(){}function xSemaphoreGive(){}
function isfinite(value){return Number.isFinite(value);}
function balanceStatus(){return {ready,armed,target};}
function stopAndManual(){currentMode=MANUAL;armed=false;motorsOff=true;}
${extract(balance,'balanceSetUpright')}
${extract(main,'loadUprightCalibration')}
${extract(main,'enterCalibration')}
${extract(main,'saveUprightCalibration')}

loadUprightCalibration();assert.equal(target,178);assert.equal(puts,0,'first-use default is not written repeatedly');
stored=181.25;loadUprightCalibration();assert.equal(target,181.25);assert.equal(uprightAngle,181.25);
stored=500;loadUprightCalibration();assert.equal(target,178,'bad saved range falls back');
stored=NaN;loadUprightCalibration();assert.equal(target,178,'bad saved NaN falls back');
beginOk=false;loadUprightCalibration();assert.equal(target,178);assert.equal(imuStorageReady,false);
beginOk=true;stored=181.25;loadUprightCalibration();

const beforeOutside=puts;saveUprightCalibration('180');assert.equal(puts,beforeOutside,'ignore save outside settings');
armed=true;enterCalibration();assert.equal(currentMode,CALIBRATION);assert.equal(armed,false);assert.equal(motorsOff,true);
for(const bad of ['', 'abc','NaN','Infinity','-1','89.9','270.1','178junk','178 ']){
  const before=puts;saveUprightCalibration(bad);assert.equal(puts,before,'reject '+bad);assert.equal(target,181.25);
}
ready=false;saveUprightCalibration('180');assert.equal(target,181.25);assert.match(calibrationMessage,/fresh IMU/);
ready=true;armed=true;saveUprightCalibration('180');assert.equal(target,181.25);assert.equal(balanceSetUpright(180),false,'module rejects changing an armed target');
armed=false;imuStorageReady=false;saveUprightCalibration('180');assert.equal(target,181.25);
imuStorageReady=true;writeFail=true;saveUprightCalibration('180');assert.equal(target,181.25,'restore target after flash failure');assert.equal(stored,181.25);assert.match(calibrationMessage,/Save failed/);
writeFail=false;saveUprightCalibration('179.75');assert.equal(target,179.75);assert.equal(uprightAngle,179.75);assert.equal(stored,179.75);assert.equal(armed,false);assert.equal(currentMode,CALIBRATION);assert.match(calibrationMessage,/saved/);
// Simulate restarting the application: defaults are replaced by stored NVS.
target=uprightAngle=178;loadUprightCalibration();assert.equal(target,179.75);
saveUprightCalibration('90');assert.equal(target,90);saveUprightCalibration('270');assert.equal(target,270);
assert.equal(balanceSetUpright(Infinity),false);assert.equal(balanceSetUpright(0),false);
`,context);
module.exports='PASS: actual calibration functions with mocked storage/IMU: default 178, saved startup load, strict validation, freshness, motors-off requirement, failure rollback and persistence round trip';
if(require.main===module)console.log(module.exports);
