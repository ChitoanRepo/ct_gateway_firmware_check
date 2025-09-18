const char *status_html = R"rawliteral(
<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>ESP MASTER - Status</title>
<style>
body{font-family: Arial, Helvetica, sans-serif; background:#f2f2f2; margin:0; padding:10px;}
.header{display:flex; align-items:center; justify-content:space-between; margin-bottom:10px;}
.controls{display:flex; flex-direction:column; gap:8px; align-items:flex-end;}
.btn{padding:8px 12px; border-radius:6px; cursor:pointer; border:none; font-weight:600;}
.btn-main{background:#007bff; color:#fff;}
.btn-toggle{background:#28a745; color:#fff;}
.btn-edit{background:#fd7e14; color:#fff;}
.btn-delete{background:#dc3545; color:#fff;}
.btn-relay-on{background:#28a745; color:#fff;}
.btn-relay-off{background:#6c757d; color:#fff;}
.container{display:grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap:12px;}
.card{background:#fff; border-radius:8px; padding:12px; box-shadow:0 1px 4px rgba(0,0,0,0.1);}
.card h3{margin:0 0 8px 0; font-size:16px;}
.row{display:flex; justify-content:space-between; margin:6px 0; align-items:center;}
.small{font-size:12px; color:#666;}
.online{color:green; font-weight:600;}
.offline{color:#999; font-weight:600;}
.slider{width:100%;}
.label-inline{display:flex; gap:8px; align-items:center;}
.footer{margin-top:12px; font-size:12px; color:#666;}
.badge{padding:4px 6px; border-radius:6px; background:#eee; font-size:12px;}
.relay-grid{display:grid; grid-template-columns: repeat(2,1fr); gap:8px;}
.relay-grid button{width:100%; padding:10px; font-weight:600;}
</style>
</head>
<body>
<div class="header">
  <div class="hleft"><h2>ESP MASTER - Status</h2></div>
  <div class="hright">
    <div class="controls">
      <button class="btn btn-main" id="btn-setting">Setting</button>
      <button class="btn btn-main" id="btn-addnode">Add Node</button>
      <button class="btn btn-main" id="btn-refresh">Refresh</button>
    </div>
  </div>
</div>

<div class="container">
  <!-- Master Info -->
  <div class="card" id="masterCard">
    <h3>Master Station</h3>
    <div class="row"><span class="small">Time:</span><span id="masterTime">--:--:--</span></div>
    <div class="row"><span class="small">Temp:</span><span id="masterTemp">-- °C</span></div>
    <div class="row"><span class="small">Fan:</span><span id="masterFan">--</span></div>
  </div>

  <!-- Relay Master -->
  <div class="card" id="relayCard">
    <h3>Master Relays</h3>
    <div id="relayBtns" class="relay-grid"></div>
    <div style="margin-top:8px;">
      <button class="btn btn-main" onclick="toggleAllRelays()">Toggle All</button>
    </div>
  </div>
</div>

<!-- Nodes -->
<div class="container" id="nodesContainer"></div>

<!-- WiFi + Fan Settings -->
<div class="container" id="settingsCard" style="display:none;">
  <div class="card">
    <h3>WiFi & Fan Settings</h3>
    <form id="settingsForm">
      <div class="row"><label class="small">SSID:</label><input type="text" id="wifi_ssid" name="ssid" style="flex:1;"></div>
      <div class="row"><label class="small">Password:</label><input type="password" id="wifi_pass" name="pass" style="flex:1;"></div>
      <div class="row"><label class="small">Fan Threshold:</label><input type="number" id="fan" name="fan" style="flex:1;" value="50"></div>
      <div style="margin-top:8px; text-align:right;">
        <button type="submit" class="btn btn-main">Save</button>
      </div>
    </form>
  </div>
</div>

<div class="footer">Auto-refresh every 3s. Move slider to change dimming (sent to node).</div>

<script>
let status = null;

function createNodeCard(n, s) {
  let div = document.createElement('div');
  div.className = 'card';
  div.id = 'node-'+n.id;
  let h = document.createElement('h3');
  h.innerHTML = n.label + ' <span class="small badge">ID:'+n.id+'</span>';
  div.appendChild(h);

  // Connection + Relay
  let row = document.createElement('div');
  row.className = 'row';
  row.innerHTML = '<span class="'+(s && s.connected ? 'online' : 'offline')+'">'+(s && s.connected ? 'Connected' : 'Disconnected')+'</span>'
    + '<span class="small">Relay: ' + (n.relay ? 'ON' : 'OFF') + '</span>';
  div.appendChild(row);

  // V / I
  let row2 = document.createElement('div');
  row2.className = 'row';
  row2.innerHTML = '<div class="small">V: '+n.voltage.toFixed(2)+' V</div><div class="small">I: '+n.current.toFixed(2)+' A</div>';
  div.appendChild(row2);

  // Temp / Uptime
  let row3 = document.createElement('div');
  row3.className = 'row';
  row3.innerHTML = '<div class="small">Temp: ' + (s ? s.temperature.toFixed(1)+' °C' : 'N/A') + '</div><div class="small">Uptime: '+(s ? s.time+'s' : 'N/A')+'</div>';
  div.appendChild(row3);

  // Slider
  let sliderRow = document.createElement('div');
  sliderRow.style.marginTop = '8px';
  sliderRow.innerHTML = '<div class="label-inline"><div class="small">Dimming</div><div id="val-'+n.id+'" class="small">'+((s)?s.slider:0)+'</div></div>';
  div.appendChild(sliderRow);
  let slider = document.createElement('input');
  slider.type = 'range'; slider.min = 0; slider.max = 255;
  slider.value = (s ? s.slider : 0);
  slider.className = 'slider';
  slider.oninput = ev => document.getElementById('val-'+n.id).innerText = ev.target.value;
  slider.onchange = ev => {
    fetch('/api/node/dim', {
      method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded'},
      body:'node='+encodeURIComponent(n.id)+'&value='+encodeURIComponent(ev.target.value)
    });
  };
  div.appendChild(slider);

  // Action buttons
  let rowBtn = document.createElement('div');
  rowBtn.className = 'row';
  rowBtn.innerHTML = `
    <button class="btn btn-toggle" onclick="toggleRelayNode(${n.id})">Toggle</button>
    <button class="btn btn-edit" onclick="editNode(${n.id}, '${n.label}')">Edit</button>
    <button class="btn btn-delete" onclick="deleteNode(${n.id})">Delete</button>`;
  div.appendChild(rowBtn);

  return div;
}

function renderStatus() {
  if (!status) return;

  // Master info
  document.getElementById('masterTime').innerText = status.time;
  document.getElementById('masterTemp').innerText = status.temp.toFixed(1)+' °C';
  document.getElementById('masterFan').innerText = status.fan ? 'ON' : 'OFF';

  // Relay buttons
  let relayHtml = '';
  status.relays.forEach((r,i)=>{
    relayHtml += `<button class="btn ${r?'btn-relay-on':'btn-relay-off'}" onclick="toggleRelayMaster(${i})">Relay ${i+1}: ${r?'ON':'OFF'}</button>`;
  });
  document.getElementById('relayBtns').innerHTML = relayHtml;

  // Nodes
  let cont = document.getElementById('nodesContainer');
  cont.innerHTML = '';
  let slavesMap = {};
  if (status.slaves) status.slaves.forEach(s=>{slavesMap[s.id]=s;});
  if (status.nodes && status.nodes.length>0) {
    status.nodes.forEach(n=>{
      let s = slavesMap[n.id] || null;
      if (s) s.slider = s.slider || s.sliderValue || 0;
      let card = createNodeCard(n, s);
      cont.appendChild(card);
    });
  } else {
    cont.innerHTML = '<div class="card">No nodes found. Use Add Node.</div>';
  }

  if (status.ssid) document.getElementById('wifi_ssid').value = status.ssid;
  if (status.fanThreshold) document.getElementById('fan').value = status.fanThreshold;
}

function fetchStatus() {
  fetch('/api/status').then(r=>r.json()).then(j=>{
    status=j; renderStatus();
  });
}

function toggleRelayMaster(idx){
  fetch('/api/relay?ch='+idx,{method:'POST'}).then(fetchStatus);
}

function toggleAllRelays(){
  fetch('/api/relay?ch=all',{method:'POST'}).then(fetchStatus);
}

function toggleRelayNode(id){
  fetch('/api/node/relay',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'node='+id}).then(fetchStatus);
}

function deleteNode(id){
  if(!confirm("Delete node "+id+"?")) return;
  fetch('/api/node/remove',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'node='+id}).then(fetchStatus);
}

function editNode(id,label){
  let newId = prompt("Enter new ID:", id);
  if(newId===null) return;
  let newLabel = prompt("Enter new Label:", label);
  if(newLabel===null) return;
  fetch('/api/node/edit',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:'node='+id+'&id='+encodeURIComponent(newId)+'&name='+encodeURIComponent(newLabel)}).then(fetchStatus);
}

document.getElementById('btn-refresh').addEventListener('click', fetchStatus);
document.getElementById('btn-addnode').addEventListener('click', ()=>{
  alert("Scanning nodes... please wait");
  fetch('/api/node/add',{method:'POST'})
    .then(r=>r.json())
    .then(j=>{
      console.log(j);
      setTimeout(fetchStatus,3000);
    });
});
document.getElementById('btn-setting').addEventListener('click', ()=>{
  let card = document.getElementById('settingsCard');
  card.style.display = 'block';
  fetch('/api/status').then(r=>r.json()).then(j=>{
    if(j.ssid) document.getElementById('wifi_ssid').value = j.ssid;
    if(j.fanThreshold) document.getElementById('fan').value = j.fanThreshold;
  });
});

document.getElementById('settingsForm').addEventListener('submit', function(ev){
  ev.preventDefault();
  let data = new URLSearchParams();
  data.append('ssid', document.getElementById('wifi_ssid').value);
  data.append('pass', document.getElementById('wifi_pass').value);
  data.append('fan', document.getElementById('fan').value);
  fetch('/api/config/save',{method:'POST',body:data})
    .then(()=>alert('Settings saved!'));
});

setInterval(fetchStatus,3000);
fetchStatus();
</script>
</body>
</html>
)rawliteral";
