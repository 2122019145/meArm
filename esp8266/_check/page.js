
const $=id=>document.getElementById(id);
let joints=[],cur=[],one=1;
function esc(s){return String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));}
function line(s){const l=$('log');const at=l.scrollTop+l.clientHeight>=l.scrollHeight-24;
  l.textContent+=s+'\n';if(l.textContent.length>20000)l.textContent=l.textContent.slice(-16000);if(at)l.scrollTop=l.scrollHeight;}
async function send(c,danger,quiet){
  if(danger&&!confirm('确认发送「'+c+'」？机械臂会动作！'))return false;
  line('> '+c);await fetch('/cmd?c='+encodeURIComponent(c));return true;}
function sendBox(){const v=$('cmd').value.trim();if(v){send(v,false);$('cmd').value='';}}
async function loadButtons(){const a=await(await fetch('/buttons')).json();const box=$('btns');box.innerHTML='';
  const g={};for(const b of a)(g[b.g]=g[b.g]||[]).push(b);
  for(const k in g){const h=document.createElement('h2');h.textContent=k;box.appendChild(h);
    const d=document.createElement('div');d.className='grid';
    for(const b of g[k]){const el=document.createElement('button');
      el.textContent=b.l;el.className=(b.f&2)?'stop':(b.d?'danger':'safe');
      el.onclick=()=>send(b.c,!!b.d);d.appendChild(el);}
    box.appendChild(d);}}
async function loadJoints(){const d=await(await fetch('/joints')).json();one=d.one;joints=d.list;
  const box=$('joints');box.innerHTML='';cur=joints.map(j=>j.v);
  if(!joints.length){$('jsec').style.display='none';$('bAll').style.display='none';return;}
  $('jtitle').textContent=one?'四关节（拖动后点“发送”，或“依次发送”排队）':'关节（拖动后点“发送”，一次发全部）';
  joints.forEach((j,i)=>{const e=document.createElement('div');e.className='j';
    e.innerHTML='<b>'+j.n+'</b><input type="range" min="'+j.lo+'" max="'+j.hi+'" value="'+j.v+'" id="r'+i+'">'+
      '<span id="v'+i+'">'+j.v+'°</span> <button class="safe" onclick="sendOne('+i+')">发送</button>';
    box.appendChild(e);
    e.querySelector('input').addEventListener('input',ev=>{$('v'+i).textContent=ev.target.value+'°';cur[i]=+ev.target.value;});});}
function vals(){return joints.map((j,i)=>+$('r'+i).value);}
async function sendOne(i){const v=+$('r'+i).value;line('> '+joints[i].n+v);
  if(one)await fetch('/joint?i='+i+'&v='+v);
  else await fetch('/joint?i='+i+'&v='+v+'&all='+vals().join(','));}
async function sendAll(){
  const v=vals();
  if(one){
    if(!confirm('依次发送全部关节角？机械臂会动作！'))return;
    for(let i=0;i<joints.length;i++)await fetch('/joint?i='+i+'&v='+v[i]+'&q=1');
    line('> 已排入队列：'+joints.map((j,i)=>j.n+v[i]).join(' '));
  }else{
    if(!confirm('发送全部关节角？机械臂会动作！'))return;
    await fetch('/joint?i=0&v='+v[0]+'&all='+v.join(','));
    line('> '+joints.map((j,i)=>j.n+v[i]).join(','));}}
function setAuto(){fetch('/auto?on='+($('auto').checked?1:0));}
async function poll(){
  try{const s=await(await fetch('/state')).json();
    const short=t=>t&&t.length>30?t.slice(0,30)+'…':t;
    $('pState').textContent=s.state;$('pPose').textContent=short(s.pose);
    $('pNet').textContent=s.net;$('pQ').textContent='队列 '+s.q;
    const c=s.cards||[['状态',s.state],['关节角',s.pose],['摇杆/爪',s.joy],['记录',s.ram],
      ['WiFi',s.net],['堆内存',s.heap+' B'],['运行',s.up+' s'],['队列',s.q+' 条']];
    $('status').innerHTML=c.map(x=>'<div>'+esc(x[0])+'<br><b>'+esc(x[1])+'</b></div>').join('');
    if(s.new&&s.new.length)s.new.forEach(l=>line(l));
  }catch(e){$('pNet').textContent='连接断开…';}}
loadButtons();loadJoints();poll();
setInterval(poll,1200);setAuto();
