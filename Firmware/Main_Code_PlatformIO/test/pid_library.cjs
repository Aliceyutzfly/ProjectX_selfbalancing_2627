// Executes the installed PID_v1 calculation/tuning bodies with mocked clock/input.
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict');
const source=fs.readFileSync(path.join(__dirname,'../.pio/libdeps/seeed_xiao_esp32c3/PID/PID_v1.cpp'),'utf8');
function body(signature){
 const at=source.indexOf(signature);assert(at>=0,signature);
 const begin=source.indexOf('{',at);let end=begin+1,depth=1;
 for(;depth;end++){if(source[end]==='{')depth++;if(source[end]==='}')depth--;}
 return source.slice(begin+1,end-1).replace(/\*myInput/g,'inputValue').replace(/\*mySetpoint/g,'targetValue').replace(/\*myOutput/g,'outputValue').replace(/\(double\)/g,'').replace(/\b(?:unsigned long|double) /g,'let ');
}
vm.runInNewContext(`
let inAuto=true,SampleTime=10,clock=0,lastTime=0,lastInput=0,outputSum=0,outMin=-60,outMax=60;
let inputValue=0,targetValue=0,outputValue=0,pOn=1,pOnE=true,P_ON_E=1,REVERSE=1,controllerDirection=0;
let dispKp=0,dispKi=0,dispKd=0,kp=0,ki=0,kd=0;
function millis(){return clock;}
function tune(Kp,Ki,Kd,POn=1){${body('void PID::SetTunings(double Kp, double Ki, double Kd, int POn)')}}
function compute(){${body('bool PID::Compute()')}}
function sample(p,i,d,error){
 tune(p,i,d);inputValue=178-error;targetValue=178;lastInput=inputValue;outputSum=0;clock+=10;
 assert.equal(compute(),true);return outputValue;
}
assert.equal(sample(1,0,0,1),1);assert.equal(sample(2,0,0,1),2,'changing P changes the actual library output');
assert.equal(sample(0,0,0,5),0,'all zero gains produce zero correction');
assert.equal(sample(21,0,0,5),60);assert.equal(sample(40,0,0,5),60,'different gains can have identical saturated output');
assert.equal(sample(0,10,0,1),0.1,'I uses the sample period');
tune(0,0,0.8);outputSum=0;lastInput=178;inputValue=178.1;clock+=10;compute();assert(Math.abs(outputValue+8)<1e-9,'D responds to angle change');
`,{assert});
module.exports='PASS: installed PID_v1 tuning/Compute bodies: gains change output, zero gains disable correction, P/I/D responses and saturation';
if(require.main===module)console.log(module.exports);
