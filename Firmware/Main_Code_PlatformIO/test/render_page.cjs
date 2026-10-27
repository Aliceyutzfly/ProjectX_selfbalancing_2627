// Render the actual embedded HTML template with constants from the firmware header.
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const root=path.join(__dirname,'..');
const config=fs.readFileSync(path.join(root,'include/robot_config.h'),'utf8');
const header=fs.readFileSync(path.join(root,'include/web_page.h'),'utf8');
const defaults=Object.fromEntries([...config.matchAll(/constexpr\s+\w+\s+(\w+)(?:\[\])?\s*=\s*([^;]+);/g)].map(m=>[m[1],m[2]]));
function renderPage(overrides={}){
 const values={...defaults,...overrides};
 let html=header.split('R"HTML(')[1].split(')HTML"')[0];
 const ssid=JSON.parse(values.AP_SSID).replaceAll('&','&amp;').replaceAll('<','&lt;').replaceAll('>','&gt;').replaceAll('"','&quot;').replaceAll("'",'&#39;');
 html=html.replaceAll('%ROBOT_SSID%',ssid);
 for(const [,placeholder,name,digits] of header.matchAll(/page\.replace\("(%\w+%)",\s*String\((\w+)(?:,\s*(\d+))?\)\);/g)){
  const number=parseFloat(values[name]);assert(Number.isFinite(number),name);
  html=html.replaceAll(placeholder,digits===undefined?String(number):number.toFixed(Number(digits)));
 }
 assert(!/%[A-Z_]+%/.test(html),'unresolved page configuration');
 return html;
}
module.exports={renderPage};
