const $ = (sel)=>document.querySelector(sel);
const $all = (sel)=>Array.from(document.querySelectorAll(sel));

let state = { mission:null, robots:[] };

function setTab(tab){
  $all('.tab-btn').forEach(b=>b.classList.toggle('active', b.dataset.tab===tab));
  $all('.tab').forEach(s=>s.classList.toggle('active', s.id===tab));
}

$all('.tab-btn').forEach(b=>b.addEventListener('click', ()=>setTab(b.dataset.tab)));

async function getJSON(url){
  const res = await fetch(url);
  return res.json();
}
async function postJSON(url, body){
  const res = await fetch(url, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
  return res.json();
}

function fmt(n){ return typeof n==='number'? n.toFixed(1): n; }

function render(){
  // dashboard summary
  if(state.mission){
    $('#mission-active').textContent = state.mission.active? 'Active' : 'Inactive';
    $('#mission-size').textContent = `${fmt(state.mission.size.w)} x ${fmt(state.mission.size.h)} m, spacing ${fmt(state.mission.spacing)} m`;
  }
  // robots table
  const tbody = $('#robots-table tbody');
  tbody.innerHTML = '';
  for(const r of state.robots){
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td>${r.serial}</td>
      <td>${r.status}</td>
      <td>${r.battery}%</td>
      <td>${fmt(r.x)}, ${fmt(r.y)}</td>
      <td>${r.next_goal? `${fmt(r.next_goal[0])}, ${fmt(r.next_goal[1])}` : '—'}</td>
      <td>
        <button data-serial="${r.serial}" class="rth-one">🏠 Return</button>
      </td>
    `;
    tbody.appendChild(tr);
  }
  // map
  drawMap();
}

function drawMap(){
  const c = document.getElementById('map');
  const ctx = c.getContext('2d');
  ctx.clearRect(0,0,c.width,c.height);
  // padding and scale
  const pad = 20;
  const W = c.width - pad*2;
  const H = c.height - pad*2;
  const fieldW = state?.mission?.size?.w || 60;
  const fieldH = state?.mission?.size?.h || 40;
  const scale = Math.min(W/Math.max(fieldW,1), H/Math.max(fieldH,1));
  const ox = pad + (W - fieldW*scale)/2;
  const oy = pad + (H - fieldH*scale)/2;
  // field rect
  ctx.strokeStyle = '#4e6aff';
  ctx.lineWidth = 2;
  ctx.strokeRect(ox, oy, fieldW*scale, fieldH*scale);
  // robots
  for(const r of state.robots){
    const rx = ox + (r.x - (state.mission?.origin?.x || 0)) * scale;
    const ry = oy + (r.y - (state.mission?.origin?.y || 0)) * scale;
    ctx.beginPath();
    ctx.arc(rx, ry, 5, 0, Math.PI*2);
    ctx.fillStyle = '#e3e8ef';
    ctx.fill();
    // heading line
    const angle = (r.heading || 0) * Math.PI/180;
    ctx.beginPath();
    ctx.moveTo(rx, ry);
    ctx.lineTo(rx + Math.cos(angle)*12, ry + Math.sin(angle)*12);
    ctx.strokeStyle = '#94a3b8';
    ctx.stroke();
  }
}

async function init(){
  state = await getJSON('/api/state');
  // seed mission form
  $('#origin-x').value = state.mission.origin.x;
  $('#origin-y').value = state.mission.origin.y;
  $('#size-w').value = state.mission.size.w;
  $('#size-h').value = state.mission.size.h;
  $('#spacing').value = state.mission.spacing;
  render();
  // SSE
  const ev = new EventSource('/events');
  ev.onmessage = (m)=>{
    const data = JSON.parse(m.data);
    if(data.type === 'state'){
      state = data.payload;
    } else if(data.type === 'mission'){
      state.mission = data.payload;
    } else if(data.type === 'robots'){
      state.robots = data.payload;
    }
    render();
  }
  // buttons
  $('#start-btn').onclick = ()=> postJSON('/api/command', {type:'start', target:'all'});
  $('#pause-btn').onclick = ()=> postJSON('/api/command', {type:'pause', target:'all'});
  $('#resume-btn').onclick = ()=> postJSON('/api/command', {type:'resume', target:'all'});
  $('#rth-btn').onclick = ()=> postJSON('/api/command', {type:'return_home', target:'all'});
  $('#save-mission').onclick = async ()=>{
    const payload = {
      origin: {x: Number($('#origin-x').value), y: Number($('#origin-y').value)},
      size: {w: Number($('#size-w').value), h: Number($('#size-h').value)},
      spacing: Number($('#spacing').value)
    };
    await postJSON('/api/mission', payload);
  };
  $('#register-btn').onclick = async ()=>{
    const serial = $('#serial-input').value.trim();
    if(!serial) return;
    await postJSON('/api/robots/register', {serial});
    $('#serial-input').value='';
  };
  // table action delegation
  $('#robots-table').addEventListener('click', (e)=>{
    const btn = e.target.closest('.rth-one');
    if(btn){
      const serial = btn.getAttribute('data-serial');
      postJSON('/api/command', {type:'return_home', target:serial});
    }
  });
}
window.addEventListener('load', init);
