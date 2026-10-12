#include "driver/twai.h"
#include <WiFi.h>
#include <WebServer.h>

#define CAN_TX 23
#define CAN_RX 22
const char* AP_SSID = "A-OBD";
const char* AP_PASS = "12345678";

// ---- Data logging configuration ------------------------------------------
#define LOG_CAP          3600    // max UNACKNOWLEDGED samples held on the ESP32 (3600 @ 1 Hz = 1 hour, ~43 KB)
#define LOG_INTERVAL_MS  1000    // sampling period
#define LOG_MAX_PER_REQ  300     // max samples per /log response

struct Telemetry {
  uint16_t soc;
  uint16_t soh;
  uint16_t deltaSoc;
  uint16_t voltage;
  uint16_t imbalance;
  int      balancing;
  int      rpm;
  int      driveMode;
  bool     sideStand;
  bool     frontBrake, rearBrake, highBeam, startSwitch;
  bool     killSwitch, storageSwitch, horn;
  bool     indicatorLeftSide, indicatorRightSide, indicatorCenterSide, keyOn;
  int      rangeKm;
  uint16_t cellV[14];
  uint8_t  cellSoh[14];
  uint32_t chargerOV;
  uint32_t chargerOI;
  uint32_t chargerIV;
  int16_t  motorI;
  int16_t  motorTemp;
  bool     displayFan;
  uint16_t chargerFan1, chargerFan2;
  bool     forcedEco, smartEco;
  bool     seatLock, chargerLock1, chargerLock2;
  bool     chargerConnected;
};

Telemetry snap;
SemaphoreHandle_t mtx;
WebServer server(80);
unsigned long lastDiag = 0;
uint32_t maxQueueBacklog = 0;

// ---- Data logging state (only touched from the Arduino loop task) ---------
struct LogSample {
  uint32_t t;          // device uptime, seconds
  uint16_t voltage;    // pack voltage, 0.01 V units (same as snap.voltage)
  int16_t  motorTemp;  // deg C
  uint8_t  range;      // km
  uint8_t  mode;       // raw drive mode code
  int16_t  motorI;     // motor battery current, 0.1 A units (signed)
};
static LogSample logBuf[LOG_CAP];
static uint32_t logHead  = 0;   // sequence number of newest sample (0 = none yet)
static uint32_t logSent  = 0;   // highest sequence number transmitted to a browser
static uint32_t logAcked = 0;   // highest sequence number the browser confirmed (everything <= this is deleted)
static uint32_t logBootId = 0;  // random per boot so the browser can detect restarts
unsigned long lastLogSample = 0;

inline const char* driveModeName(int v){
  switch(v){
    case 1: return "SPORTS";
    case 2: return "DRIVE";
    case 3: return "ECO";
    case 6: return "WARP";
    case 8: return "S. ECO";
    default: return "UNKNOWN";
  }
}

// ---------------------------------------------------------------------------
// Web page. Split into several smaller literals and sent back-to-back by
// handleRoot() so no single literal is huge (avoids toolchain raw-string limits).
// ---------------------------------------------------------------------------
const char htmlPart1[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head>
<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>A OBD Scanner v5</title>
<style>
:root{--bg:#eceeef;--card:#fff;--text:#2b2e34;--muted:#6b7280;--border:#e2e6ea;--accent:#5a7a6a;}
*{box-sizing:border-box}
body{font-family:Inter,Arial,sans-serif;background:var(--bg);color:var(--text);margin:0;}
header{background:var(--card);border-bottom:1px solid var(--border);padding:14px 10px;text-align:center;}
header h1{margin:0;font-size:22px;}
.tabs{display:flex;justify-content:center;background:var(--card);border-bottom:1px solid var(--border);position:sticky;top:0;z-index:10;}
.tab{flex:1;max-width:160px;padding:12px 6px;border:none;background:var(--card);color:var(--muted);font-size:13px;font-weight:600;cursor:pointer;border-bottom:3px solid transparent;transition:all .2s;}
.tab.active{color:var(--accent);border-bottom-color:var(--accent);background:#f8f9fa;}
.tab:hover{background:#f3f4f5;}
.card{max-width:520px;margin:16px auto;background:var(--card);border-radius:14px;overflow:hidden;border:1px solid var(--border);box-shadow:0 2px 10px rgba(0,0,0,0.06);}
.card h2{margin:0;padding:10px;background:#f3f4f5;color:var(--muted);font-size:13px;letter-spacing:0.6px;text-transform:uppercase;border-bottom:1px solid var(--border);}
table{width:100%;border-collapse:collapse;font-size:15px;}
td{padding:10px 16px;border-bottom:1px solid #f0f1f2;text-align:left;}
td:first-child{color:var(--muted);}
td.v{color:var(--accent);font-weight:700;text-align:right;}
td.on{color:var(--accent);font-weight:700;text-align:right;}
td.off{color:#9aa0a8;text-align:right;}
.ctable{max-width:700px;margin:16px auto;}
.ctable table{font-size:14px;}
.ctable td{padding:8px 12px;text-align:center;}
.ctable th{padding:8px 12px;text-align:center;font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:0.5px;border-bottom:1px solid var(--border);}
.ctable tr:nth-child(even){background:#f8f9fa;}
.ctable tr:hover{background:#eef1f3;}
.vbar{display:inline-block;height:8px;border-radius:4px;vertical-align:middle;margin-left:4px;transition:width .3s;}
.panel{display:none;}
.panel.active{display:block;}
.rng{display:flex;gap:8px;padding:10px 16px;}
.rng button{flex:1;padding:8px 4px;border:1px solid var(--border);background:var(--card);color:var(--muted);border-radius:8px;font-size:13px;font-weight:600;cursor:pointer;}
.rng button.active{color:var(--accent);border-color:var(--accent);background:#f8f9fa;}
.rng button:disabled{opacity:.5;cursor:default;}
.cs{padding:8px 16px 0;font-size:12px;color:var(--muted);}
.chart{display:block;width:100%;height:190px;}
</style>
<script>
let curTab='dash';
let bzD=false,bzC=false,bzG=false,bzO=false;

// fetch JSON with a timeout so a stalled request can never freeze a poller
async function fj(url,ms){
  const c=new AbortController();
  const t=setTimeout(()=>c.abort(),ms||4000);
  try{
    const r=await fetch(url,{cache:'no-store',signal:c.signal});
    if(!r.ok)throw new Error('HTTP '+r.status);
    return await r.json();
  }finally{clearTimeout(t);}
}

function showTab(t){
  curTab=t;
  document.querySelectorAll('.panel').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(b=>b.classList.remove('active'));
  document.getElementById(t).classList.add('active');
  document.getElementById('tb_'+t).classList.add('active');
  if(t==='dash') update();
  if(t==='cells') updateCells();
  if(t==='charger') updateCharger();
  if(t==='others') updateOthers();
  if(t==='charts'&&window.chartsOpen) chartsOpen();
}
function sw(id,val,onTxt,offTxt){let e=document.getElementById(id);if(!e)return;e.innerText=val?onTxt:offTxt;e.className=val?'on':'off';}
async function update(){
 if(curTab!=='dash'||bzD)return;
 bzD=true;
 try{
  let j=await fj('/data');
  document.getElementById('soc').innerText=j.soc.toFixed(2)+' %';
  document.getElementById('soh').innerText=j.soh.toFixed(2)+' %';
  document.getElementById('bdelta').innerText=j.bdelta.toFixed(2)+' %';
  document.getElementById('volt').innerText=j.volt.toFixed(2)+' V';
  document.getElementById('imb').innerText=j.imb.toFixed(4)+' V';
  document.getElementById('mode').innerText=j.mode;
  document.getElementById('range').innerText=j.range.toFixed(0)+' km';
  sw('fbrake',j.fbrake,'Pressed','Not Pressed');
  sw('rbrake',j.rbrake,'Pressed','Not Pressed');
  sw('hbeam',j.hbeam,'Pressed','Not Pressed');
  sw('start',j.start,'Pressed','Not Pressed');
  sw('kill',j.kill,'Pressed','Not Pressed');
  sw('storage',j.storage,'Pressed','Not Pressed');
  sw('horn',j.horn,'Pressed','Not Pressed');
  sw('indl',j.indl,'Pressed','Not Pressed');
  sw('indr',j.indr,'Pressed','Not Pressed');
  sw('indc',j.indc,'Pressed','Not Pressed');
  sw('key',j.key,'Enabled','Disabled');
  sw('stand',j.stand,'DOWN','UP');
  sw('dfan',j.dfan,'ON','OFF');
 }catch(e){}
 bzD=false;
}
async function updateCells(){
 if(curTab!=='cells'||bzC)return;
 bzC=true;
 try{
  let j=await fj('/cells');
  let pos=j.v.filter(x=>x>0);
  let mn=pos.length?Math.min(...pos):0;
  let mx=pos.length?Math.max(...j.v):0;
  let rng=mx-mn||1;
  for(let i=0;i<14;i++){
    document.getElementById('cv'+i).innerText=j.v[i].toFixed(4)+' V';
    document.getElementById('sh'+i).innerText=j.h[i].toFixed(0)+' %';
    let bw=mx>0?((j.v[i]-mn)/rng*60+4):0;
    let bar=document.getElementById('vb'+i);
    bar.style.width=bw+'px';
    let dev=Math.abs(j.v[i]-((mn+mx)/2));
    bar.style.background=dev<0.01?'#5a7a6a':dev<0.03?'#e6a817':'#d94f4f';
    let shb=document.getElementById('hb'+i);
    if(shb){shb.style.background=j.h[i]>90?'#5a7a6a':j.h[i]>70?'#e6a817':'#d94f4f';shb.style.width=(j.h[i])+'px';}
  }
 }catch(e){}
 bzC=false;
}
async function updateCharger(){
 if(curTab!=='charger'||bzG)return;
 bzG=true;
 try{
  let j=await fj('/charger');
  document.getElementById('coutv').innerText=j.coutv.toFixed(2)+' V';
  document.getElementById('couti').innerText=j.couti.toFixed(2)+' A';
  document.getElementById('cinv').innerText=j.cinv.toFixed(2)+' V';
  document.getElementById('bal').innerText=j.bal;
  document.getElementById('cfan1').innerText=j.fan1;
  document.getElementById('cfan2').innerText=j.fan2;
  sw('cconnected',j.cconnected,'Connected','Disconnected');
  sw('clock1',j.clock1,'Locked','Unlocked');
  sw('clock2',j.clock2,'Locked','Unlocked');
  let w=j.watts;
  document.getElementById('cwatts').innerText=w>=1000?(w/1000).toFixed(2)+' kW':w.toFixed(1)+' W';
 }catch(e){}
 bzG=false;
}
async function updateOthers(){
 if(curTab!=='others'||bzO)return;
 bzO=true;
 try{
  let j=await fj('/data');
  sw('feco',j.feco,'ON','OFF');
  sw('seco',j.seco,'ON','OFF');
  document.getElementById('rpm').innerText=j.rpm;
 }catch(e){}
 try{
  let j2=await fj('/charger');
  sw('seatlock',j2.seat,'Locked','Unlocked');
  document.getElementById('mbi').innerText=j2.mbi.toFixed(2)+' A';
  document.getElementById('mt').innerText=j2.mt.toFixed(1)+' \u00B0C';
 }catch(e){}
 bzO=false;
}
setInterval(update,1000);
setInterval(updateCells,1500);
setInterval(updateCharger,1500);
setInterval(updateOthers,1500);
window.onload=function(){showTab('dash');};
</script></head><body>
)HTML";

const char htmlPart2[] PROGMEM = R"HTML(
<header><h1>&#9889; A OBD Scanner v5</h1><p style="margin:6px 0 0;color:var(--muted);font-size:13px;">Developed by <b style="color:var(--text);">sam0_0</b> &bull; A OBD Scanner</p><p style="margin:10px 0 0;"><a href="https://www.buymeacoffee.com/sam0_0" target="_blank" style="display:inline-block;background:#ffdd00;color:#1a1a1a;padding:7px 16px;border-radius:999px;font-size:13px;font-weight:700;text-decoration:none;box-shadow:0 2px 6px rgba(0,0,0,0.12);">&#9749; Support this project &mdash; Buy Me a Coffee</a></p></header>
<div class="tabs">
<button class="tab active" id="tb_dash" onclick="showTab('dash')">Dashboard</button>
<button class="tab" id="tb_cells" onclick="showTab('cells')">Cell Info</button>
<button class="tab" id="tb_charger" onclick="showTab('charger')">Charger</button>
<button class="tab" id="tb_others" onclick="showTab('others')">Others</button>
<button class="tab" id="tb_charts" onclick="showTab('charts')">Charts</button>
</div>

<div id="dash" class="panel active">
<div class="card"><h2>Battery &amp; Drive</h2>
<table>
<tr><td>SoC</td><td class='v' id='soc'>--</td></tr>
<tr><td>SoH</td><td class='v' id='soh'>--</td></tr>
<tr><td>Delta SoC</td><td class='v' id='bdelta'>--</td></tr>
<tr><td>Voltage</td><td class='v' id='volt'>--</td></tr>
<tr><td>Voltage Imbalance</td><td class='v' id='imb'>--</td></tr>
<tr><td>Drive Mode</td><td class='v' id='mode'>--</td></tr>
<tr><td>Range</td><td class='v' id='range'>--</td></tr>
</table></div>
<div class="card"><h2>Switches &amp; Inputs</h2>
<table>
<tr><td>Key</td><td class='off' id='key'>--</td></tr>
<tr><td>Start Switch</td><td class='off' id='start'>--</td></tr>
<tr><td>Front Brake</td><td class='off' id='fbrake'>--</td></tr>
<tr><td>Rear Brake</td><td class='off' id='rbrake'>--</td></tr>
<tr><td>High Beam</td><td class='off' id='hbeam'>--</td></tr>
<tr><td>Horn</td><td class='off' id='horn'>--</td></tr>
<tr><td>Indicator Left</td><td class='off' id='indl'>--</td></tr>
<tr><td>Indicator Right</td><td class='off' id='indr'>--</td></tr>
<tr><td>Indicator Center</td><td class='off' id='indc'>--</td></tr>
<tr><td>Kill Switch</td><td class='off' id='kill'>--</td></tr>
<tr><td>Storage Switch</td><td class='off' id='storage'>--</td></tr>
<tr><td>Sidestand</td><td class='off' id='stand'>--</td></tr>
<tr><td>Display Fan</td><td class='off' id='dfan'>--</td></tr>
</table></div>
</div>

<div id="cells" class="panel">
<div class="card ctable"><h2>Cell Voltages (14S)</h2>
<table><tr><th>Cell</th><th>Voltage</th><th style="width:80px">Bar</th></tr>
<tbody id="cvtable"></tbody></table></div>
<div class="card ctable"><h2>Cell State of Health</h2>
<table><tr><th>Cell</th><th>SoH</th><th style="width:80px">Bar</th></tr>
<tbody id="cshtable"></tbody></table></div>
</div>

<div id="charger" class="panel">
<div class="card"><h2>Charger</h2>
<table>
<tr><td>Output Voltage</td><td class='v' id='coutv'>--</td></tr>
<tr><td>Output Current</td><td class='v' id='couti'>--</td></tr>
<tr><td>Input Voltage</td><td class='v' id='cinv'>--</td></tr>
<tr><td>Power [calculated]</td><td class='v' id='cwatts'>--</td></tr>
<tr><td>Balancing State</td><td class='v' id='bal'>--</td></tr>
<tr><td>Charger Connected</td><td class='off' id='cconnected'>--</td></tr>
<tr><td>Fan Speed 1</td><td class='v' id='cfan1'>--</td></tr>
<tr><td>Fan Speed 2</td><td class='v' id='cfan2'>--</td></tr>
<tr><td>Charger Lock 1</td><td class='off' id='clock1'>--</td></tr>
<tr><td>Charger Lock 2</td><td class='off' id='clock2'>--</td></tr>
</table></div>
</div>

<div id="others" class="panel">
<div class="card"><h2>Drive Modes</h2>
<table>
<tr><td>Forced Eco</td><td class='off' id='feco'>--</td></tr>
<tr><td>Smart Eco</td><td class='off' id='seco'>--</td></tr>
</table></div>
<div class="card"><h2>Locks &amp; Motor</h2>
<table>
<tr><td>Seat Lock</td><td class='off' id='seatlock'>--</td></tr>
<tr><td>Motor RPM</td><td class='v' id='rpm'>--</td></tr>
<tr><td>Motor Battery Current</td><td class='v' id='mbi'>--</td></tr>
<tr><td>Motor Temperature</td><td class='v' id='mt'>--</td></tr>
</table></div>
</div>
)HTML";

const char htmlPart3[] PROGMEM = R"HTML(
<div id="charts" class="panel">
<div id="warn" class="card" style="display:none;background:#fdecea;border-color:#d94f4f;color:#8a1c1c;padding:12px 16px;font-size:13px;font-weight:600;line-height:1.4;"></div>
<div class="card"><h2>Data Link</h2>
<table>
<tr><td>Delivery</td><td class='off' id='lkstat'>--</td></tr>
<tr><td>Phone storage</td><td class='off' id='lkstore'>--</td></tr>
<tr><td>Samples stored on phone</td><td class='v' id='lkcount'>0</td></tr>
<tr><td>Last verified sample</td><td class='v' id='lkack'>--</td></tr>
<tr><td>Not recoverable</td><td class='v' id='lkdrop'>0</td></tr>
<tr><td>Checksum retries</td><td class='v' id='lkretry'>0</td></tr>
</table></div>
<div class="card"><div class="rng">
<button id="btn_csv" onclick="exportCSV()">Download CSV</button>
<button id="btn_clr" onclick="clearData()">Clear phone data</button>
</div></div>
<div class="card"><div class="rng">
<button id="rb_300" onclick="setChartRange(300)">5 min</button>
<button id="rb_900" class="active" onclick="setChartRange(900)">15 min</button>
<button id="rb_3600" onclick="setChartRange(3600)">1 hour</button>
<button id="rb_7200" onclick="setChartRange(7200)">2 hours</button>
</div></div>
<div class="card"><h2>Motor Temperature (&deg;C)</h2><div class="cs" id="st_temp">--</div><canvas class="chart" id="ch_temp"></canvas></div>
<div class="card"><h2>Battery Voltage (V)</h2><div class="cs" id="st_volt">--</div><canvas class="chart" id="ch_volt"></canvas></div>
<div class="card"><h2>Motor Current Drawn (A)</h2><div class="cs" id="st_cur">--</div><canvas class="chart" id="ch_cur"></canvas></div>
<div class="card"><h2>Ride Range (km)</h2><div class="cs" id="st_range">--</div><canvas class="chart" id="ch_range"></canvas></div>
<div class="card"><h2>Ride Mode</h2><div class="cs" id="st_mode">--</div><canvas class="chart" id="ch_mode"></canvas></div>
</div>

<script>
(function(){
  let cv='',sh='';
  for(let i=0;i<14;i++){
    cv+='<tr><td>C'+(i+1)+'</td><td id="cv'+i+'">--</td><td><div class="vbar" id="vb'+i+'" style="width:4px"></div></td></tr>';
    sh+='<tr><td>C'+(i+1)+'</td><td id="sh'+i+'">--</td><td><div class="vbar" id="hb'+i+'" style="width:0px;height:8px;border-radius:4px;"></div></td></tr>';
  }
  document.getElementById('cvtable').innerHTML=cv;
  document.getElementById('cshtable').innerHTML=sh;
})();
</script>
)HTML";

// Part 4: phone-side storage (IndexedDB), CSV export
const char htmlPart4[] PROGMEM = R"HTML(
<script>
(function(){
  const MODES=['UNKNOWN','S. ECO','ECO','DRIVE','SPORTS','WARP'];
  const modeLevel=m=>m===8?1:m===3?2:m===2?3:m===1?4:m===6?5:0;
  const modeName=m=>({1:'SPORTS',2:'DRIVE',3:'ECO',6:'WARP',8:'S. ECO'})[m]||'UNKNOWN';
  const DBN='aobd_log_v2',MAXMEM=7200;
  const $=id=>document.getElementById(id);
  // S = in-memory window for charts: [absSec, volt(0.01V), tempC, rangeKm, modeCode]
  let db=null,dbOk=false,ready=false,boot=null,lastSeq=0,S=[],rangeSec=900,polling=false;
  let dropped=0,lastAck=0,retries=0,total=0,writeFail=false;

  // ---- IndexedDB helpers ----
  function openDB(){
    return new Promise((res,rej)=>{
      if(!window.indexedDB){rej(new Error('IndexedDB not available'));return;}
      const rq=indexedDB.open(DBN,1);
      rq.onupgradeneeded=()=>{
        const d=rq.result;
        d.createObjectStore('samples',{keyPath:'id',autoIncrement:true});
        d.createObjectStore('meta');
      };
      rq.onsuccess=()=>res(rq.result);
      rq.onerror=()=>rej(rq.error);
      rq.onblocked=()=>rej(new Error('IndexedDB blocked'));
    });
  }
  function txDone(tx){
    return new Promise((res,rej)=>{
      tx.oncomplete=()=>res();
      tx.onerror=()=>rej(tx.error||new Error('tx error'));
      tx.onabort=()=>rej(tx.error||new Error('tx aborted'));
    });
  }
  function mkTx(stores,mode){
    try{return db.transaction(stores,mode,{durability:'strict'});}
    catch(e){return db.transaction(stores,mode);}
  }
  function metaObj(){return {boot:boot,lastSeq:lastSeq,lastAck:lastAck,dropped:dropped};}
  async function saveMeta(){
    if(!dbOk)return;
    try{
      const tx=mkTx(['meta'],'readwrite');
      tx.objectStore('meta').put(metaObj(),'state');
      await txDone(tx);
    }catch(e){}
  }
  // Atomically store a batch AND the new lastSeq. Resolves only after the data is committed.
  async function commitBatch(rows,meta){
    const tx=mkTx(['samples','meta'],'readwrite');
    const st=tx.objectStore('samples');
    for(const r of rows)st.add(r);
    tx.objectStore('meta').put(meta,'state');
    await txDone(tx);
  }
  async function loadState(){
    db=await openDB();
    const tx=db.transaction(['samples','meta'],'readonly');
    const mreq=tx.objectStore('meta').get('state');
    const creq=tx.objectStore('samples').count();
    const rows=[];
    const cur=tx.objectStore('samples').openCursor(null,'prev');
    cur.onsuccess=()=>{
      const c=cur.result;
      if(c&&rows.length<MAXMEM){rows.push(c.value);c.continue();}
    };
    await txDone(tx);
    const m=mreq.result;
    if(m){boot=m.boot;lastSeq=m.lastSeq>>>0;lastAck=m.lastAck>>>0;dropped=m.dropped||0;}
    total=creq.result||0;
    rows.reverse();
    S=rows.map(r=>[r.t,r.v,r.temp,r.range,r.mode,r.cur||0]);
  }

  function stat(txt,cls){const e=$('lkstat');if(!e)return;e.innerText=txt;e.className=cls;}
  function linkInfo(){
    $('lkcount').innerText=dbOk?total:S.length;
    $('lkack').innerText=lastAck?('#'+lastAck):'--';
    $('lkdrop').innerText=dropped;
    $('lkretry').innerText=retries;
    const e=$('lkstore');
    e.innerText=!dbOk?'NOT SAVING (memory only)':writeFail?'WRITE FAILING':'IndexedDB \u2713';
    e.className=(dbOk&&!writeFail)?'on':'off';
    const w=$('warn');
    if(!dbOk){
      w.style.display='block';
      w.innerText='WARNING: phone storage is unavailable (private/blocked browser mode?). Data is only in memory and is lost when this page reloads. The ESP32 is NOT told to delete anything, but it only keeps the last ~1 hour, so older samples will be overwritten. Open this page in a normal (non-private) browser tab, or download the CSV regularly.';
    }else if(writeFail){
      w.style.display='block';
      w.innerText='WARNING: saving to phone storage is failing (storage full or blocked). Nothing is being acknowledged, so the ESP32 keeps the data (last ~1 hour only). Download the CSV and clear phone data to free space.';
    }else w.style.display='none';
  }

  // ---- CSV export / clear ----
  const p2=n=>(n<10?'0':'')+n;
  function fmtFull(t){
    const d=new Date(t*1000);
    return d.getFullYear()+'-'+p2(d.getMonth()+1)+'-'+p2(d.getDate())+' '+p2(d.getHours())+':'+p2(d.getMinutes())+':'+p2(d.getSeconds());
  }
  function csvRow(t,dev,b,seq,v,temp,rg,mode,cur){
    return fmtFull(t)+','+t+','+dev+','+b+','+seq+','+(v/100).toFixed(2)+','+temp+','+rg+','+mode+','+modeName(mode)+','+((cur||0)/10).toFixed(1);
  }
  async function exportCSV(){
    const btn=$('btn_csv');btn.disabled=true;
    try{
      const lines=['timestamp_local,unix_time_s,device_uptime_s,boot_id,seq,voltage_V,motor_temp_C,range_km,mode_code,mode,motor_current_A'];
      if(dbOk){
        const tx=db.transaction('samples','readonly');
        const rq=tx.objectStore('samples').openCursor();
        rq.onsuccess=()=>{
          const c=rq.result;
          if(c){const r=c.value;lines.push(csvRow(r.t,r.dev,r.boot,r.seq,r.v,r.temp,r.range,r.mode,r.cur));c.continue();}
        };
        await txDone(tx);
      }else{
        for(const e of S)lines.push(csvRow(e[0],'','','',e[1],e[2],e[3],e[4],e[5]));
      }
      if(lines.length<2){alert('No data to export yet.');return;}
      const blob=new Blob(['\uFEFF'+lines.join('\r\n')+'\r\n'],{type:'text/csv;charset=utf-8'});
      const url=URL.createObjectURL(blob);
      const d=new Date();
      const name='aobd_log_'+d.getFullYear()+p2(d.getMonth()+1)+p2(d.getDate())+'_'+p2(d.getHours())+p2(d.getMinutes())+p2(d.getSeconds())+'.csv';
      const a=document.createElement('a');
      a.href=url;a.download=name;a.style.display='none';
      document.body.appendChild(a);a.click();
      setTimeout(()=>{URL.revokeObjectURL(url);a.remove();},10000);
    }catch(e){
      alert('Export failed: '+(e&&e.message?e.message:e));
    }finally{btn.disabled=false;}
  }
  async function clearData(){
    if(!confirm('Delete ALL saved chart data from this phone? Download the CSV first if you want to keep it.'))return;
    try{
      if(dbOk){
        const tx=mkTx(['samples'],'readwrite');
        tx.objectStore('samples').clear();
        await txDone(tx);
      }
      S=[];total=0;dropped=0;retries=0;
      await saveMeta();   // keeps boot/lastSeq so already-acknowledged samples are not reported as lost
      linkInfo();render();
    }catch(e){alert('Clear failed: '+(e&&e.message?e.message:e));}
  }
  window.exportCSV=exportCSV;
  window.clearData=clearData;
)HTML";

// Part 5: polling, verification, safe acknowledge
const char htmlPart5[] PROGMEM = R"HTML(
  async function doAck(){
    const aj=await fj('/ack?seq='+lastSeq+'&boot='+boot);
    if(aj.ok&&aj.acked>=lastSeq){lastAck=aj.acked;await saveMeta();return true;}
    return false;
  }

  async function poll(){
    if(polling||!ready)return;
    polling=true;let again=false;
    try{
      // retry an earlier ack that failed (data is already safely on the phone)
      if(dbOk&&boot!==null&&lastSeq>lastAck){
        try{await doAck();}catch(e){}
      }
      const j=await fj('/log?since='+lastSeq,8000);
      if(boot!==null&&j.boot!==boot){
        boot=j.boot;lastSeq=0;lastAck=0;
        await saveMeta();
        stat('Device restarted - resyncing','off');again=true;
      }else{
        boot=j.boot;
        const s=j.s;let cs=0,ok=(s.length===j.n);
        for(let i=0;i<s.length;i++){
          const e=s[i];
          cs=(cs+e[0]+e[1]+e[2]+(e[3]&0xFFFF)+e[4]+e[5]+(e[6]&0xFFFF))>>>0;
          if(i>0&&e[0]!==s[i-1][0]+1)ok=false;
        }
        if(ok&&cs!==(j.cs>>>0))ok=false;
        if(ok&&j.n>0&&(s[0][0]!==j.first||s[s.length-1][0]!==j.last))ok=false;
        if(!ok){
          retries++;stat('Checksum mismatch - retrying','off');
        }else if(j.n>0){
          const nowAbs=Math.floor(Date.now()/1000);
          let prev=S.length?S[S.length-1][0]:0;
          const rows=[],mem=[];
          for(let i=0;i<s.length;i++){
            const e=s[i];
            let a=nowAbs-(j.now-e[1]);
            if(a<prev)a=prev;
            prev=a;
            rows.push({t:a,dev:e[1],boot:j.boot,seq:e[0],v:e[2],temp:e[3],range:e[4],mode:e[5],cur:e[6]});
            mem.push([a,e[2],e[3],e[4],e[5],e[6]]);
          }
          // Lost samples are counted ONCE, from the real sequence gap between what the phone
          // already holds (lastSeq) and the first sample received. lastSeq and the new total are
          // committed together, so the same gap can never be counted again by a later batch.
          const gap=Math.max(0,s[0][0]-lastSeq-1);
          const newDropped=dropped+gap;
          let saved=true;
          if(dbOk){
            try{
              await commitBatch(rows,{boot:j.boot,lastSeq:j.last,lastAck:lastAck,dropped:newDropped});
            }catch(e){saved=false;}
          }
          writeFail=!saved;
          if(!saved){
            // NOTHING advanced: lastSeq unchanged and no /ack sent, so the ESP32 keeps the data
            stat('Phone storage failed - NOT acknowledged, retrying','off');
          }else{
            for(const m of mem)S.push(m);
            if(S.length>MAXMEM)S.splice(0,S.length-MAXMEM);
            total+=rows.length;
            lastSeq=j.last;dropped=newDropped;
            if(dbOk){
              let acked=false;
              try{acked=await doAck();}catch(e){}
              stat(acked?'Delivered & verified \u2713':'Saved on phone - ack pending','off');
              if(acked)$('lkstat').className='on';
            }else{
              stat('WARNING: memory only - NOT saved, NOT acknowledged','off');
            }
            if(j.n>=j.max)again=true;
          }
        }else{
          // nothing new, but samples between lastSeq and the device's oldest are gone: count that gap once
          if(j.oldest-1>lastSeq){dropped+=j.oldest-1-lastSeq;lastSeq=j.oldest-1;await saveMeta();}
          stat((dbOk?lastAck:lastSeq)?'Up to date \u2713':'Waiting for vehicle data...',(dbOk?lastAck:lastSeq)?'on':'off');
        }
      }
      linkInfo();render();
    }catch(e){
      stat('Link error - retrying','off');
    }finally{
      polling=false;
      if(again)setTimeout(poll,50);
    }
  }
)HTML";

// Part 6: charts
const char htmlPart6[] PROGMEM = R"HTML(
  function fmtClock(t,sec){
    const d=new Date(t*1000);
    return p2(d.getHours())+':'+p2(d.getMinutes())+(sec?':'+p2(d.getSeconds()):'');
  }

  function draw(id,pts,o){
    const cv=$(id);if(!cv)return;
    const dpr=window.devicePixelRatio||1,w=cv.clientWidth,h=cv.clientHeight;
    if(!w||!h)return;
    cv.width=Math.round(w*dpr);cv.height=Math.round(h*dpr);
    const c=cv.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,w,h);
    c.font='11px Arial';
    const L=o.levels?62:46,R=22,T=10,B=22,pw=w-L-R,ph=h-T-B;
    if(!pts.length){
      c.fillStyle='#9aa0a8';c.textAlign='center';c.textBaseline='middle';
      c.fillText('Waiting for data...',w/2,h/2);return;
    }
    const t1=pts[pts.length-1][0];let t0=pts[0][0];if(t1-t0<10)t0=t1-10;
    let ymin,ymax;
    if(o.levels){ymin=-0.5;ymax=o.levels.length-0.5;}
    else{
      let lo=Infinity,hi=-Infinity;
      for(const p of pts){if(p[1]<lo)lo=p[1];if(p[1]>hi)hi=p[1];}
      if(hi-lo<o.minSpan){const m=(hi+lo)/2;lo=m-o.minSpan/2;hi=m+o.minSpan/2;}
      const pad=(hi-lo)*0.1;ymin=lo-pad;ymax=hi+pad;
      if(o.floor0&&ymin<0)ymin=0;
    }
    const X=t=>L+(t-t0)/(t1-t0)*pw,Y=v=>T+(1-(v-ymin)/(ymax-ymin))*ph;
    c.strokeStyle='#e2e6ea';c.fillStyle='#6b7280';c.lineWidth=1;
    c.textAlign='right';c.textBaseline='middle';
    if(o.levels){
      for(let i=0;i<o.levels.length;i++){
        const y=Math.round(Y(i))+0.5;
        c.beginPath();c.moveTo(L,y);c.lineTo(L+pw,y);c.stroke();
        c.fillText(o.levels[i],L-6,y);
      }
    }else{
      for(let i=0;i<=4;i++){
        const v=ymin+(ymax-ymin)*i/4,y=Math.round(Y(v))+0.5;
        c.beginPath();c.moveTo(L,y);c.lineTo(L+pw,y);c.stroke();
        c.fillText(v.toFixed(o.dec),L-6,y);
      }
    }
    c.textBaseline='top';
    for(let i=0;i<=4;i++){
      c.textAlign=i===0?'left':i===4?'right':'center';
      c.fillText(fmtClock(t0+(t1-t0)*i/4,(t1-t0)<=900),L+pw*i/4,T+ph+6);
    }
    c.strokeStyle='#5a7a6a';c.lineWidth=2;c.lineJoin='round';c.beginPath();
    let prev=null;
    for(const p of pts){
      const x=X(p[0]),y=Y(p[1]);
      if(prev===null||p[0]-prev[0]>10)c.moveTo(x,y);
      else if(o.step){c.lineTo(x,Y(prev[1]));c.lineTo(x,y);}
      else c.lineTo(x,y);
      prev=p;
    }
    c.stroke();
    const lp=pts[pts.length-1];
    c.fillStyle='#5a7a6a';c.beginPath();c.arc(X(lp[0]),Y(lp[1]),3.5,0,Math.PI*2);c.fill();
  }

  function numStats(arr,dec,unit){
    if(!arr.length)return '--';
    let lo=Infinity,hi=-Infinity;
    for(const p of arr){if(p[1]<lo)lo=p[1];if(p[1]>hi)hi=p[1];}
    return 'Now '+arr[arr.length-1][1].toFixed(dec)+unit+'  \u00B7  Min '+lo.toFixed(dec)+unit+'  \u00B7  Max '+hi.toFixed(dec)+unit;
  }

  function render(){
    if(curTab!=='charts')return;
    const pts={temp:[],volt:[],cur:[],range:[],mode:[]};
    const n=S.length;
    if(n){
      const from=S[n-1][0]-rangeSec;
      let i=n-1;while(i>0&&S[i-1][0]>=from)i--;
      for(;i<n;i++){
        const e=S[i];
        pts.volt.push([e[0],e[1]/100]);
        pts.temp.push([e[0],e[2]]);
        pts.cur.push([e[0],(e[5]||0)/10]);
        pts.range.push([e[0],e[3]]);
        pts.mode.push([e[0],modeLevel(e[4])]);
      }
    }
    draw('ch_temp',pts.temp,{dec:0,minSpan:10});
    draw('ch_volt',pts.volt,{dec:2,minSpan:1});
    draw('ch_cur',pts.cur,{dec:1,minSpan:10});
    draw('ch_range',pts.range,{dec:0,minSpan:10,floor0:true});
    draw('ch_mode',pts.mode,{levels:MODES,step:true});
    $('st_temp').innerText=numStats(pts.temp,0,' \u00B0C');
    $('st_volt').innerText=numStats(pts.volt,2,' V');
    $('st_cur').innerText=numStats(pts.cur,1,' A');
    $('st_range').innerText=numStats(pts.range,0,' km');
    if(pts.mode.length){
      let ch=0;
      for(let i=1;i<pts.mode.length;i++)if(pts.mode[i][1]!==pts.mode[i-1][1])ch++;
      $('st_mode').innerText='Now '+MODES[pts.mode[pts.mode.length-1][1]]+'  \u00B7  Mode changes '+ch;
    }else $('st_mode').innerText='--';
  }

  window.setChartRange=function(sec){
    rangeSec=sec;
    [300,900,3600,7200].forEach(k=>$('rb_'+k).classList.toggle('active',k===sec));
    render();
  };
  window.chartsOpen=function(){linkInfo();poll();render();};
  window.addEventListener('resize',render);
  document.addEventListener('visibilitychange',function(){if(!document.hidden)poll();});

  // Open storage first, then collect in the background on EVERY tab (not only Charts)
  (async function init(){
    try{await loadState();dbOk=true;}
    catch(e){dbOk=false;db=null;}
    ready=true;
    linkInfo();
    stat(dbOk?'Starting...':'WARNING: phone storage unavailable - memory only','off');
    poll();
    setInterval(poll,2000);
  })();
})();
</script>
</body></html>
)HTML";

void handleRoot(){
  server.sendHeader("Cache-Control", "no-cache");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent_P(htmlPart1);
  server.sendContent_P(htmlPart2);
  server.sendContent_P(htmlPart3);
  server.sendContent_P(htmlPart4);
  server.sendContent_P(htmlPart5);
  server.sendContent_P(htmlPart6);
  server.sendContent("");                 // terminate chunked response
}

char jsonBuf[768];
void handleData(){
  xSemaphoreTake(mtx, portMAX_DELAY);
  Telemetry t = snap;
  xSemaphoreGive(mtx);
  int n=snprintf(jsonBuf,sizeof(jsonBuf),
    "{\"soc\":%d.%02d,\"soh\":%d.%02d,\"bdelta\":%d.%02d,\"volt\":%d.%02d,\"imb\":%d.%04d,\"rpm\":%d,\"mode\":\"%s\",\"range\":%d,\"fbrake\":%s,\"rbrake\":%s,\"hbeam\":%s,\"start\":%s,\"kill\":%s,\"storage\":%s,\"horn\":%s,\"indl\":%s,\"indr\":%s,\"indc\":%s,\"key\":%s,\"stand\":%s,\"dfan\":%s,\"feco\":%s,\"seco\":%s}",
    t.soc/100,abs(t.soc%100), t.soh/100,abs(t.soh%100), t.deltaSoc/100,abs(t.deltaSoc%100),
    t.voltage/100,abs(t.voltage%100), t.imbalance/10000,abs(t.imbalance%10000),
    t.rpm,driveModeName(t.driveMode),t.rangeKm,
    t.frontBrake?"true":"false",t.rearBrake?"true":"false",t.highBeam?"true":"false",t.startSwitch?"true":"false",
    t.killSwitch?"true":"false",t.storageSwitch?"true":"false",t.horn?"true":"false",
    t.indicatorLeftSide?"true":"false",t.indicatorRightSide?"true":"false",t.indicatorCenterSide?"true":"false",t.keyOn?"true":"false",t.sideStand?"true":"false",
    t.displayFan?"true":"false",t.forcedEco?"true":"false",t.smartEco?"true":"false");
  if(n<0||n>=(int)sizeof(jsonBuf)){server.send(500,"application/json","{}");return;}
  server.send(200,"application/json",jsonBuf);
}

char cellBuf[512];
void handleCells(){
  xSemaphoreTake(mtx, portMAX_DELAY);
  Telemetry t = snap;
  xSemaphoreGive(mtx);
  char *p=cellBuf; int rem=sizeof(cellBuf);
  int n=snprintf(p,rem,"{\"v\":["); if(n<0||n>=rem){server.send(500,"application/json","{}");return;} p+=n; rem-=n;
  for(int i=0;i<14;i++){ n=snprintf(p,rem,"%d.%04d%s",t.cellV[i]/10000,abs(t.cellV[i]%10000),i<13?",":""); if(n<0||n>=rem){server.send(500,"application/json","{}");return;} p+=n; rem-=n; }
  n=snprintf(p,rem,"],\"h\":["); if(n<0||n>=rem){server.send(500,"application/json","{}");return;} p+=n; rem-=n;
  for(int i=0;i<14;i++){ n=snprintf(p,rem,"%d%s",t.cellSoh[i],i<13?",":""); if(n<0||n>=rem){server.send(500,"application/json","{}");return;} p+=n; rem-=n; }
  n=snprintf(p,rem,"]}");
  if(n<0||n>=rem){server.send(500,"application/json","{}");return;}
  server.send(200,"application/json",cellBuf);
}

char chargerBuf[512];
void handleCharger(){
  xSemaphoreTake(mtx, portMAX_DELAY);
  Telemetry t = snap;
  xSemaphoreGive(mtx);
  const int32_t motorI=t.motorI;
  const int32_t motorMag=motorI<0?-motorI:motorI;
  const char *motorSign=motorI<0?"-":"";
  const uint64_t powerProduct=(uint64_t)t.chargerOV*(uint64_t)t.chargerOI;
  const uint32_t wattsWhole=(uint32_t)(powerProduct/1000000ULL);
  const unsigned wattsFrac=(unsigned)((powerProduct%1000000ULL)/10000ULL);
  int n=snprintf(chargerBuf,sizeof(chargerBuf),
    "{\"coutv\":%d.%03d,\"couti\":%d.%03d,\"cinv\":%d.%03d,\"mbi\":%s%d.%01d,\"mt\":%d,\"bal\":%d,\"watts\":%u.%02u,\"fan1\":%d,\"fan2\":%d,\"cconnected\":%s,\"seat\":%s,\"clock1\":%s,\"clock2\":%s}",
    (int)(t.chargerOV/1000),abs((int)(t.chargerOV%1000)),
    (int)(t.chargerOI/1000),abs((int)(t.chargerOI%1000)),
    (int)(t.chargerIV/1000),abs((int)(t.chargerIV%1000)),
    motorSign,(int)(motorMag/10),abs((int)(motorMag%10)),
    (int)t.motorTemp, t.balancing, wattsWhole, wattsFrac,
    t.chargerFan1, t.chargerFan2,
    t.chargerConnected?"true":"false",t.seatLock?"true":"false",t.chargerLock1?"true":"false",t.chargerLock2?"true":"false");
  if(n<0||n>=(int)sizeof(chargerBuf)){server.send(500,"application/json","{}");return;}
  server.send(200,"application/json",chargerBuf);
}

// ---------------------------------------------------------------------------
// Data collection: 1 Hz ring buffer in RAM, deleted once the phone confirms it
// ---------------------------------------------------------------------------

// Oldest sequence number still held on the device. Samples <= logAcked have been
// confirmed by the phone and are deleted; samples older than LOG_CAP are overwritten.
static inline uint32_t logOldestSeq(){
  uint32_t lo = (logHead > LOG_CAP) ? logHead - LOG_CAP + 1 : 1;
  if (logAcked + 1 > lo) lo = logAcked + 1;
  return lo;
}

// Must match the checksum computed in the browser.
static inline uint32_t logSampleSum(uint32_t seq, const LogSample &e){
  return seq + e.t + (uint32_t)e.voltage + (uint32_t)(uint16_t)e.motorTemp
       + (uint32_t)e.range + (uint32_t)e.mode + (uint32_t)(uint16_t)e.motorI;
}

void logSampleTick(){
  unsigned long now = millis();
  if (now - lastLogSample < LOG_INTERVAL_MS) return;
  lastLogSample = now;

  xSemaphoreTake(mtx, portMAX_DELAY);
  const uint16_t v   = snap.voltage;
  const int16_t  mt  = snap.motorTemp;
  const int      rg  = snap.rangeKm;
  const int      dm  = snap.driveMode;
  const int16_t  mi  = snap.motorI;
  xSemaphoreGive(mtx);

  // NOTE: samples are logged even when voltage/other values are 0, so the
  // charts also show the periods when the vehicle / CAN bus was not alive.

  uint32_t seq = logHead + 1;
  LogSample &e = logBuf[seq % LOG_CAP];
  e.t         = (uint32_t)(now / 1000UL);
  e.voltage   = v;
  e.motorTemp = mt;
  e.range     = (uint8_t)(rg < 0 ? 0 : (rg > 255 ? 255 : rg));
  e.mode      = (uint8_t)(dm < 0 ? 0 : (dm > 255 ? 255 : dm));
  e.motorI    = mi;
  logHead     = seq;
}

// GET /log?since=N  -> samples with seq > N (max LOG_MAX_PER_REQ), with count + checksum
void handleLog(){
  uint32_t since = server.hasArg("since") ? (uint32_t)strtoul(server.arg("since").c_str(), NULL, 10) : 0;
  const uint32_t head   = logHead;
  const uint32_t oldest = logOldestSeq();

  uint32_t first = since + 1, dropped = 0, n = 0, last = 0, cs = 0;
  if (first < oldest) { dropped = oldest - first; first = oldest; }   // already delivered/deleted, or overwritten

  if (first <= head) {
    n = head - first + 1;
    if (n > LOG_MAX_PER_REQ) n = LOG_MAX_PER_REQ;
    last = first + n - 1;
    for (uint32_t s = first; s <= last; s++) cs += logSampleSum(s, logBuf[s % LOG_CAP]);
  } else {
    first = 0;                            // nothing new
  }

  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  char out[1024];
  size_t p = 0;
  p += snprintf(out + p, sizeof(out) - p,
    "{\"boot\":%lu,\"now\":%lu,\"oldest\":%lu,\"head\":%lu,\"first\":%lu,\"last\":%lu,"
    "\"n\":%lu,\"max\":%d,\"cs\":%lu,\"dropped\":%lu,\"s\":[",
    (unsigned long)logBootId, (unsigned long)(millis() / 1000UL),
    (unsigned long)oldest, (unsigned long)head,
    (unsigned long)first, (unsigned long)last, (unsigned long)n,
    (int)LOG_MAX_PER_REQ, (unsigned long)cs, (unsigned long)dropped);

  for (uint32_t i = 0; i < n; i++) {
    if (p > sizeof(out) - 96) { out[p] = 0; server.sendContent(out); p = 0; }
    const uint32_t s = first + i;
    const LogSample &e = logBuf[s % LOG_CAP];
    p += snprintf(out + p, sizeof(out) - p, "%s[%lu,%lu,%u,%d,%u,%u,%d]",
                  i ? "," : "", (unsigned long)s, (unsigned long)e.t,
                  (unsigned)e.voltage, (int)e.motorTemp, (unsigned)e.range, (unsigned)e.mode, (int)e.motorI);
  }
  p += snprintf(out + p, sizeof(out) - p, "]}");
  out[p] = 0;
  server.sendContent(out);
  server.sendContent("");                 // terminate chunked response

  if (n > 0 && last > logSent) logSent = last;
}

// GET /ack?seq=N&boot=B -> browser confirms it received, verified and PERSISTED everything up to N.
// The device then deletes those samples (they no longer count against LOG_CAP).
char ackBuf[96];
void handleAck(){
  uint32_t seq  = server.hasArg("seq")  ? (uint32_t)strtoul(server.arg("seq").c_str(),  NULL, 10) : 0;
  uint32_t boot = server.hasArg("boot") ? (uint32_t)strtoul(server.arg("boot").c_str(), NULL, 10) : 0;
  bool ok = (boot == logBootId) && (seq <= logSent);   // only accept acks for data actually sent
  if (ok && seq > logAcked) logAcked = seq;            // advancing this deletes everything <= seq
  snprintf(ackBuf, sizeof(ackBuf), "{\"ok\":%d,\"acked\":%lu,\"head\":%lu}",
           ok ? 1 : 0, (unsigned long)logAcked, (unsigned long)logHead);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", ackBuf);
}

void setupCAN(){
  pinMode(CAN_RX, INPUT_PULLUP);
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX,(gpio_num_t)CAN_RX,TWAI_MODE_NORMAL);
  g.rx_queue_len=128; g.tx_queue_len=0;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  ESP_ERROR_CHECK(twai_driver_install(&g,&t,&f));
  ESP_ERROR_CHECK(twai_start());
}

void applyFrame(Telemetry &t, const twai_message_t &m){
  if(m.extd||m.rtr) return;
  const uint8_t *d=m.data;
  switch(m.identifier){
    case 0x131: if(m.data_length_code<6)return; t.soc=d[4]+(d[5]<<8); t.balancing=((d[2]>>4)|((d[3]&0x0F)<<4))&1; t.chargerConnected=((d[2]>>6)&1); break;
    case 0x14D: if(m.data_length_code<6)return; t.soh=d[4]+(d[5]<<8); break;
    case 0x13D: if(m.data_length_code<6)return; t.deltaSoc=d[4]+(d[5]<<8); break;
    case 0x141: if(m.data_length_code<8)return; t.voltage=d[0]+(d[1]<<8); t.imbalance=d[2]+(d[3]<<8); break;
    case 0x200: if(m.data_length_code<8)return; t.rangeKm=d[6]; t.displayFan=((d[7]>>4)&1); break;
    case 0x510: if(m.data_length_code<4)return; { int16_t raw=(int16_t)(d[2]+(d[3]<<8)); t.rpm=raw*5/6; } break;
    case 0x101: if(m.data_length_code<5)return; t.driveMode=d[1]; t.smartEco=((d[3]>>6)&1); t.forcedEco=((d[4]>>2)&1); break;
    case 0x102:
      if(m.data_length_code<5)return;
      t.sideStand=(d[4]&1);
      t.frontBrake=((d[0]>>0)&1); t.rearBrake=((d[0]>>1)&1); t.highBeam=((d[0]>>2)&1);
      t.startSwitch=((d[0]>>4)&1); t.killSwitch=((d[0]>>5)&1); t.storageSwitch=((d[0]>>6)&1);
      t.horn=((d[1]>>1)&1); t.indicatorLeftSide=((d[2]>>0)&1); t.indicatorRightSide=((d[2]>>1)&1); t.indicatorCenterSide=((d[2]>>2)&1); t.keyOn=((d[4]>>3)&1);
      break;
    case 0x103:
      if(m.data_length_code<2)return;
      t.seatLock=((d[1]>>1)&1); t.chargerLock2=((d[1]>>2)&1); t.chargerLock1=((d[1]>>3)&1);
      break;
    case 0x208:
      if(m.data_length_code<4)return;
      t.chargerFan1=d[0]+(d[1]<<8); t.chargerFan2=d[2]+(d[3]<<8);
      break;
    case 0x132: if(m.data_length_code<8)return; t.cellV[0]=d[0]+(d[1]<<8); t.cellV[1]=d[2]+(d[3]<<8); t.cellV[2]=d[4]+(d[5]<<8); t.cellV[3]=d[6]+(d[7]<<8); break;
    case 0x133: if(m.data_length_code<8)return; t.cellV[4]=d[0]+(d[1]<<8); t.cellV[5]=d[2]+(d[3]<<8); t.cellV[6]=d[4]+(d[5]<<8); t.cellV[7]=d[6]+(d[7]<<8); break;
    case 0x134: if(m.data_length_code<8)return; t.cellV[8]=d[0]+(d[1]<<8); t.cellV[9]=d[2]+(d[3]<<8); t.cellV[10]=d[4]+(d[5]<<8); t.cellV[11]=d[6]+(d[7]<<8); break;
    case 0x135: if(m.data_length_code<4)return; t.cellV[12]=d[0]+(d[1]<<8); t.cellV[13]=d[2]+(d[3]<<8); break;
    case 0x14e: if(m.data_length_code<8)return; t.cellSoh[0]=d[0]; t.cellSoh[1]=d[1]; t.cellSoh[2]=d[2]; t.cellSoh[3]=d[3]; t.cellSoh[4]=d[4]; t.cellSoh[5]=d[5]; t.cellSoh[6]=d[6]; t.cellSoh[7]=d[7]; break;
    case 0x14f: if(m.data_length_code<6)return; t.cellSoh[8]=d[0]; t.cellSoh[9]=d[1]; t.cellSoh[10]=d[2]; t.cellSoh[11]=d[3]; t.cellSoh[12]=d[4]; t.cellSoh[13]=d[5]; break;
    case 0x201:
      if(m.data_length_code<8)return;
      t.chargerOV=d[0]|(d[1]<<8)|(d[2]<<16)|((uint32_t)d[3]<<24);
      t.chargerOI=d[4]|(d[5]<<8)|(d[6]<<16)|((uint32_t)d[7]<<24);
      break;
    case 0x203:
      if(m.data_length_code<4)return;
      t.chargerIV=d[0]|(d[1]<<8)|(d[2]<<16)|((uint32_t)d[3]<<24);
      break;
    case 0x18c:
      if(m.data_length_code<5)return;
      t.motorI=((d[2]>>5)|((d[3]&31)<<3))+((((d[3]>>5)|((d[4]&31)<<3))&1)*256)-(((d[3]>>5)&1)*512);
      t.motorTemp=(((d[4]&63)<<2)|(d[3]>>6))-40;
      break;
  }
}

void canTask(void *){
  Telemetry local;
  xSemaphoreTake(mtx, portMAX_DELAY);
  local = snap;
  xSemaphoreGive(mtx);
  uint32_t backlog = 0;
  for(;;){
    twai_message_t msg;
    if(twai_receive(&msg,0)==ESP_OK){
      applyFrame(local, msg);
      backlog++;
    } else {
      if(backlog>0){
        xSemaphoreTake(mtx, portMAX_DELAY);
        snap = local;
        xSemaphoreGive(mtx);
        if(backlog>maxQueueBacklog) maxQueueBacklog=backlog;
        backlog=0;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

void setup(){
  Serial.begin(115200);
  mtx = xSemaphoreCreateMutex();          // created first so nothing can take a null handle
  logBootId = esp_random();
  WiFi.persistent(false); WiFi.disconnect(true,true); delay(200);
  WiFi.mode(WIFI_AP); delay(200);
  WiFi.softAP(AP_SSID,AP_PASS,6,0,4); WiFi.setTxPower(WIFI_POWER_8_5dBm);
  server.on("/",handleRoot); server.on("/data",handleData); server.on("/cells",handleCells); server.on("/charger",handleCharger);
  server.on("/log",handleLog); server.on("/ack",handleAck);
  server.begin();
  setupCAN();
  xTaskCreatePinnedToCore(canTask,"can",4096,NULL,2,NULL,0);
}

void loop(){
  server.handleClient();
  logSampleTick();
  if(millis()-lastDiag>=5000){
    lastDiag=millis();
    twai_status_info_t st;
    if(twai_get_status_info(&st)==ESP_OK){
      Serial.printf("[DIAG] rx_missed:%lu rx_overrun:%lu arb_lost:%lu max_backlog:%lu heap:%lu\n",
        (unsigned long)st.rx_missed_count,(unsigned long)st.rx_overrun_count,
        (unsigned long)st.arb_lost_count,(unsigned long)maxQueueBacklog,
        (unsigned long)ESP.getFreeHeap());
    }
    uint32_t oldest = logOldestSeq();
    Serial.printf("[LOG] head:%lu sent:%lu acked:%lu stored_on_device:%lu\n",
      (unsigned long)logHead,(unsigned long)logSent,(unsigned long)logAcked,
      (unsigned long)(logHead >= oldest ? logHead - oldest + 1 : 0));
    maxQueueBacklog=0;
  }
  vTaskDelay(pdMS_TO_TICKS(1));
}
