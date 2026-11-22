#pragma once
// Página de controle servida pelo robô (docs/plano-web.md, fase 2 + DS4).
// Fala com o firmware pelo WebSocket /ws usando o mesmo protocolo de texto do terminal.
static const char WEB_PAGE[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="pt-BR"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>S.B.R</title>
<style>
*{box-sizing:border-box}body{font:15px system-ui;margin:0;padding:8px;background:#14161a;color:#e6e6e6;max-width:900px;margin:auto}
header{display:flex;flex-wrap:wrap;gap:6px 12px;align-items:center;margin-bottom:8px}
#st{font-weight:700;padding:4px 10px;border-radius:6px;background:#333}
.s2{background:#2d4a6b!important}.s3{background:#7a6a1e!important}.s4{background:#1f6b3a!important}.s6{background:#8a2626!important}.s1{background:#5a3d7a!important}
.dim{color:#9aa}.row{display:flex;gap:8px;flex-wrap:wrap;margin:6px 0}
button{font:inherit;padding:12px 14px;border-radius:8px;border:1px solid #444;background:#262a31;color:#eee;flex:1;min-width:70px}
button:active{background:#3a404a}button.stop{border-color:#a33}select{font:inherit;padding:8px;background:#262a31;color:#eee;border:1px solid #444;border-radius:8px}
#pad{display:flex;gap:12px;align-items:center;justify-content:center;margin:8px 0}
#joy{width:200px;height:200px;border-radius:50%;background:#1d2127;border:2px solid #3a404a;position:relative;touch-action:none}
#knob{width:70px;height:70px;border-radius:50%;background:#4a90d9;position:absolute;left:65px;top:65px;pointer-events:none}
canvas{width:100%;height:120px;background:#0d0f12;border-radius:6px;display:block;margin:4px 0}
.leg{font-size:12px}.leg b{display:inline-block;width:10px;height:3px;vertical-align:middle;margin:0 3px 0 8px}
#log{height:160px;overflow:auto;background:#000;padding:6px;font:12px monospace;white-space:pre-wrap;border-radius:6px}
input{font:inherit;padding:10px;flex:3;background:#0d0f12;color:#eee;border:1px solid #444;border-radius:8px}
</style></head><body>
<header><span id="st">conectando...</span><span id="info" class="dim"></span></header>
<div class="row">
 <button onclick="send('start')">△ start</button>
 <button onclick="send('drive 0 0')">○ parar</button>
 <button onclick="send('cal')">□ calibrar</button>
 <button class="stop" onclick="send('drive 0 0');send('stop')">✕ stop</button>
</div>
<div id="pad"><div id="joy"><div id="knob"></div></div>
 <div><div class="dim">vel. máx</div><select id="lvl"></select><div id="cmd" class="dim" style="margin-top:8px"></div><div id="gp" class="dim"></div></div></div>
<div class="leg"><b style="background:#4a90d9"></b>pitch (°)<b style="background:#d9a54a"></b>acel. (°)<b style="background:#888"></b>erro (°)</div><canvas id="c0"></canvas>
<div class="leg"><b style="background:#4ad97a"></b>velocidade (m/s)<b style="background:#d94a4a"></b>pedida</div><canvas id="c1"></canvas>
<div class="leg"><b style="background:#b04ad9"></b>giro (°/s)<b style="background:#d94a4a"></b>pedido</div><canvas id="c2"></canvas>
<div class="row"><button onclick="csv()">baixar CSV (últimos 30 s)</button><button onclick="paused=!paused">pausar gráficos</button></div>
<div id="log"></div>
<form class="row" onsubmit="send(c.value);c.value='';return false"><input id="c" placeholder="comando (help, tune kp 0.35, angle...)" autocomplete="off" autocapitalize="off"><button style="flex:1">enviar</button></form>
<script>
const $=id=>document.getElementById(id);
const STATES={0:'INIT',1:'CALIBRANDO',2:'IDLE',3:'STARTING',4:'ACTIVE',5:'MOVING',6:'FAULT'};
const KEYS=['pitch','acc','gyroint','rate','yaw','speed','accel','err','voff','vtgt','yrate','wtgt','state','tick','temp'];
const LEVELS=[0.1,0.2,0.3,0.4,0.5,0.65], W_MAX=30, DEAD=0.12, N=1500; // 30 s a 50 Hz
LEVELS.forEach((v,i)=>$('lvl').add(new Option(v+' m/s',i))); $('lvl').value=2;
let ws, buf='', cur={}, hist=[], frames=0, fps=0, paused=false, lastState=null;

function add(t){const l=$('log'); l.textContent+=t+'\n'; if(l.textContent.length>20000) l.textContent=l.textContent.slice(-15000); l.scrollTop=1e9}
function send(t){ if(ws&&ws.readyState==1) ws.send(t) }
function connect(){
  ws=new WebSocket('ws://'+location.host+'/ws');
  ws.onopen=()=>{ send('stream on'); };
  ws.onclose=()=>{ $('st').textContent='desconectado'; $('st').className=''; setTimeout(connect,1000) };
  ws.onmessage=e=>{ buf+=e.data; const lines=buf.split('\n'); buf=lines.pop();
    for(let l of lines){ while(l.startsWith('sbr> ')) l=l.slice(5);
      const m=l.match(/^>(\w+):(\S+)/);
      if(m){ cur[m[1]]=parseFloat(m[2]);
        if(m[1]=='temp'){ frames++; cur.t=performance.now()/1000; hist.push(cur); if(hist.length>N) hist.shift(); cur={}; }
      } else if(l.trim()) add(l); } };
}
setInterval(()=>{ fps=frames; frames=0; const f=hist[hist.length-1]||{};
  const s=f.state; if(s!==undefined){ $('st').textContent=STATES[s]||s; $('st').className='s'+s;
    if(s!==lastState && lastState!==null) add('— estado: '+(STATES[s]||s)); lastState=s; }
  $('info').textContent=`${fps} quadros/s · loop ${f.tick??'-'} Hz · MPU ${f.temp??'-'}°C`;
  if(ws&&ws.readyState==1&&fps==0) send('stream on');  // ponte/outro cliente desligou o stream
},1000);

// --- comando: joystick na tela, DS4 (Gamepad API) e teclado; manda a 20 Hz só com comando (+ um zero ao soltar)
let joy={x:0,y:0}, keys={}, sentNonzero=false, gpPrev=[];
const jEl=$('joy'), knob=$('knob');
function joyMove(e){ const r=jEl.getBoundingClientRect(), R=r.width/2;
  let x=(e.clientX-r.left-R)/R, y=(e.clientY-r.top-R)/R; const d=Math.hypot(x,y); if(d>1){x/=d;y/=d}
  joy={x,y}; knob.style.left=(65+x*65)+'px'; knob.style.top=(65+y*65)+'px'; }
function joyEnd(){ joy={x:0,y:0}; knob.style.left='65px'; knob.style.top='65px'; }
jEl.addEventListener('pointerdown',e=>{jEl.setPointerCapture(e.pointerId); joyMove(e)});
jEl.addEventListener('pointermove',e=>{ if(jEl.hasPointerCapture(e.pointerId)) joyMove(e) });
jEl.addEventListener('pointerup',joyEnd); jEl.addEventListener('pointercancel',joyEnd);
addEventListener('keydown',e=>{ if(e.target.tagName!='INPUT') keys[e.key.toLowerCase()]=1 });
addEventListener('keyup',e=>{ keys[e.key.toLowerCase()]=0 });
addEventListener('blur',()=>{keys={}; joyEnd()});
const shape=x=>Math.abs(x)<DEAD?0:(Math.abs(x)-DEAD)/(1-DEAD)*Math.sign(x);
setInterval(()=>{
  let fwd=-shape(joy.y), turn=-shape(joy.x);           // tela: pra cima = frente, direita = virar à direita
  if(keys.w) fwd=1; if(keys.s) fwd=-1; if(keys.a) turn=1; if(keys.d) turn=-1;
  const g=navigator.getGamepads?[...navigator.getGamepads()].find(p=>p):null;
  if(g){ $('gp').textContent='controle: '+g.id.slice(0,24);
    const b=i=>g.buttons[i]&&g.buttons[i].pressed, edge=i=>b(i)&&!gpPrev[i];
    if(edge(3)) send('start'); if(edge(1)) send('drive 0 0'); if(edge(2)) send('cal');
    if(edge(0)){ send('drive 0 0'); send('stop'); }
    if(edge(4)) $('lvl').value=Math.max(0,+$('lvl').value-1); if(edge(5)) $('lvl').value=Math.min(LEVELS.length-1,+$('lvl').value+1);
    gpPrev=g.buttons.map(x=>x.pressed);
    const gf=-shape(g.axes[1]||0), gt=-shape(g.axes[2]||0);  // analógico esquerdo vertical, direito horizontal
    if(gf) fwd=gf; if(gt) turn=gt;
  }
  const v=fwd*LEVELS[$('lvl').value], w=turn*W_MAX, active=!!(v||w);
  $('cmd').textContent=active?`v ${v.toFixed(2)} m/s · giro ${w.toFixed(0)}°/s`:'';
  if(active||sentNonzero) send(`drive ${v.toFixed(2)} ${w.toFixed(0)}`);
  sentNonzero=active;
},50);

// --- gráficos
function plot(id, series, minSpan){
  const c=$(id), W=c.width=c.clientWidth*devicePixelRatio, H=c.height=c.clientHeight*devicePixelRatio, x=c.getContext('2d');
  if(hist.length<2) return;
  let lo=Infinity, hi=-Infinity;
  for(const f of hist) for(const [k] of series){ const v=f[k]; if(v<lo) lo=v; if(v>hi) hi=v; }
  const mid=(lo+hi)/2, span=Math.max(hi-lo, minSpan)*1.1; lo=mid-span/2; hi=mid+span/2;
  const t1=hist[hist.length-1].t, X=t=>W-(t1-t)/30*W, Y=v=>H-(v-lo)/(hi-lo)*H;
  x.strokeStyle='#333'; x.beginPath(); x.moveTo(0,Y(0)); x.lineTo(W,Y(0)); x.stroke();
  x.lineWidth=1.5*devicePixelRatio;
  for(const [k,col] of series){ x.strokeStyle=col; x.beginPath(); hist.forEach((f,i)=>i?x.lineTo(X(f.t),Y(f[k])):x.moveTo(X(f.t),Y(f[k]))); x.stroke(); }
  x.fillStyle='#9aa'; x.font=11*devicePixelRatio+'px system-ui'; x.fillText(hi.toFixed(2),4,12*devicePixelRatio); x.fillText(lo.toFixed(2),4,H-4);
}
(function draw(){ if(!paused){
  plot('c0',[['err','#888'],['acc','#d9a54a'],['pitch','#4a90d9']],4);
  plot('c1',[['vtgt','#d94a4a'],['speed','#4ad97a']],0.2);
  plot('c2',[['wtgt','#d94a4a'],['yrate','#b04ad9']],20); }
  requestAnimationFrame(draw); })();

function csv(){ const t0=hist.length?hist[0].t:0;
  const rows=['t,'+KEYS.join(',')].concat(hist.map(f=>[(f.t-t0).toFixed(3)].concat(KEYS.map(k=>f[k])).join(',')));
  const a=document.createElement('a'); a.href=URL.createObjectURL(new Blob([rows.join('\n')],{type:'text/csv'}));
  a.download='sbr-'+new Date().toISOString().slice(0,19).replace(/[:T]/g,'-')+'.csv'; a.click(); }
connect();
</script></body></html>)HTML";
