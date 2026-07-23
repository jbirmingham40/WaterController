#pragma once

// PROGMEM HTML/CSS/JS for the settings web portal. Kept out of WebPortal.cpp
// so the request-handling logic isn't buried under markup. No external
// fonts/scripts - this has to work while the device is AP-only/offline.

static const char PORTAL_CSS[] PROGMEM = R"CSS(
body{background:#10151a;color:#e7edf2;font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;margin:0;padding:16px;}
.wrap{max-width:480px;margin:0 auto;}
h1{font-size:1.3em;margin:0 0 16px;}
h2{font-size:1em;color:#8fb3cc;margin:24px 0 8px;text-transform:uppercase;letter-spacing:.05em;}
.card{background:#1a2229;border-radius:10px;padding:16px;margin-bottom:12px;}
label{display:block;font-size:.85em;color:#9fb0bd;margin:10px 0 4px;}
input{width:100%;box-sizing:border-box;background:#0d1216;border:1px solid #2c3a44;border-radius:6px;color:#e7edf2;padding:9px;font-size:1em;}
button{background:#2a7fb8;color:#fff;border:none;border-radius:6px;padding:10px 16px;font-size:1em;cursor:pointer;margin-top:10px;}
button.secondary{background:#354552;}
button.danger{background:#b8422a;}
button:disabled{opacity:.5;cursor:default;}
.row{display:flex;justify-content:space-between;align-items:center;padding:6px 0;border-bottom:1px solid #23303a;}
.row:last-child{border-bottom:none;}
.row span.v{font-weight:600;}
.badge{display:inline-block;padding:2px 8px;border-radius:10px;font-size:.8em;}
.badge.ok{background:#1e5c34;color:#b6f5cb;}
.badge.warn{background:#5c4a1e;color:#f5e0b6;}
.badge.off{background:#3a3a3a;color:#bbb;}
.err{color:#f5a3a3;font-size:.9em;min-height:1.2em;}
.msg{color:#b6f5cb;font-size:.9em;min-height:1.2em;}
.levelctl{display:flex;align-items:center;gap:12px;}
.levelctl .val{font-size:1.4em;font-weight:600;min-width:70px;text-align:center;}
a.logout{color:#9fb0bd;font-size:.85em;text-decoration:none;float:right;}
)CSS";

static const char LOGIN_PAGE_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Water Controller Setup</title><style>%CSS%</style></head>
<body><div class="wrap">
<h1>Water Controller</h1>
<div class="card">
<form id="f">
<label for="pw">Settings Password</label>
<input type="password" id="pw" name="password" autofocus>
<div class="err" id="err"></div>
<button type="submit">Log In</button>
</form>
</div>
</div>
<script>
document.getElementById('f').addEventListener('submit', function(e){
  e.preventDefault();
  var pw = document.getElementById('pw').value;
  fetch('/login', {method:'POST', body:new URLSearchParams({password:pw})})
    .then(function(r){
      if (r.ok) { location.reload(); return; }
      if (r.status === 429) { return r.json().then(function(j){ throw new Error('Too many attempts. Try again in ' + j.retryAfterSec + 's.'); }); }
      throw new Error('Incorrect password.');
    })
    .catch(function(e){ document.getElementById('err').textContent = e.message; });
});
</script>
</body></html>
)HTML";

static const char SETTINGS_PAGE_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Water Controller Settings</title><style>%CSS%</style></head>
<body><div class="wrap">
<h1>Water Controller <a class="logout" href="#" id="logout">Log out</a></h1>

<div class="card">
<div class="row"><span>WiFi</span><span class="v" id="s-wifi">-</span></div>
<div class="row"><span>Access point</span><span class="v" id="s-ap">-</span></div>
<div class="row"><span>Tank level</span><span class="v" id="s-level">-</span></div>
<div class="row"><span>Battery</span><span class="v" id="s-batt">-</span></div>
<div class="row"><span>Filling</span><span class="v" id="s-filling">-</span></div>
<div class="row"><span>Last heard</span><span class="v" id="s-heard">-</span></div>
</div>

<h2>Desired Water Level</h2>
<div class="card">
<div class="levelctl">
<button class="secondary" id="lvl-down">&minus; 0.1"</button>
<span class="val" id="lvl-val">-</span>
<button id="lvl-up">+ 0.1"</button>
</div>
<button class="secondary" id="freeze-btn" style="margin-top:14px;width:100%;">Freeze Protect: <span id="freeze-val">-</span></button>
</div>

<h2>WiFi Network</h2>
<div class="card">
<form id="wifi-form">
<label>SSID</label><input name="ssid" id="wifi-ssid">
<label>Password</label><input type="password" name="password" id="wifi-pass">
<div class="msg" id="wifi-msg"></div>
<button type="submit">Save &amp; Connect</button>
</form>
</div>

<h2>Carbon Cache Metrics</h2>
<div class="card">
<form id="carbon-form">
<label>Hostname</label><input name="host" id="carbon-host">
<label>Port</label><input name="port" id="carbon-port" type="number" min="1" max="65535">
<div class="msg" id="carbon-msg"></div>
<button type="submit">Save</button>
</form>
</div>

<h2>Device</h2>
<div class="card">
<button class="danger" id="reboot-btn">Reboot</button>
</div>

</div>
<script>
function post(url, data){
  return fetch(url, {method:'POST', body:new URLSearchParams(data||{})}).then(function(r){
    if (r.status === 401) { location.reload(); throw new Error('logged out'); }
    return r;
  });
}
function fmtSecs(s){
  if (s < 0) return 'never';
  if (s < 60) return s + 's ago';
  if (s < 3600) return Math.floor(s/60) + 'm ago';
  return Math.floor(s/3600) + 'h ago';
}
var wifiFormDirty = false, carbonFormDirty = false;
function refresh(){
  fetch('/api/status').then(function(r){
    if (r.status === 401) { location.reload(); return; }
    return r.json();
  }).then(function(j){
    if (!j) return;
    document.getElementById('s-wifi').textContent = j.wifiConnected ? (j.staSsid + ' (' + j.staIp + ')') : 'disconnected';
    document.getElementById('s-ap').textContent = j.apActive ? 'broadcasting' : 'off';
    document.getElementById('s-level').textContent = j.sensorWaterLevel >= 0 ? j.sensorWaterLevel.toFixed(1) + '" (' + j.sensorPercentage.toFixed(0) + '%)' : 'unknown';
    document.getElementById('s-batt').textContent = j.sensorVoltage >= 0 ? j.sensorVoltage.toFixed(2) + 'V' : 'unknown';
    document.getElementById('s-filling').textContent = j.isFilling ? 'yes' : (j.fillingPaused ? 'paused' : 'no');
    document.getElementById('s-heard').textContent = fmtSecs(j.lastHeardSecsAgo);
    document.getElementById('lvl-val').textContent = j.preferredWaterLevel.toFixed(1) + '"';
    document.getElementById('freeze-val').textContent = j.inFreezeProtect ? 'ON' : 'OFF';

    if (!wifiFormDirty) document.getElementById('wifi-ssid').value = j.staSsid || '';
    if (!carbonFormDirty) {
      document.getElementById('carbon-host').value = j.carbonHost || '';
      document.getElementById('carbon-port').value = j.carbonPort || '';
    }

    var msg = document.getElementById('wifi-msg');
    if (j.wifiTest === 'testing') msg.textContent = 'Testing connection...';
    else if (j.wifiTest === 'success') msg.textContent = 'Connected!';
    else if (j.wifiTest === 'failed') msg.textContent = 'Could not connect - reverted.';
    else msg.textContent = '';
  }).catch(function(){});
}
setInterval(refresh, 1000);
refresh();

document.getElementById('lvl-up').addEventListener('click', function(){ post('/api/level', {delta:'0.1'}).then(refresh); });
document.getElementById('lvl-down').addEventListener('click', function(){ post('/api/level', {delta:'-0.1'}).then(refresh); });
document.getElementById('freeze-btn').addEventListener('click', function(){ post('/api/freeze').then(refresh); });

document.getElementById('wifi-ssid').addEventListener('input', function(){ wifiFormDirty = true; });
document.getElementById('wifi-pass').addEventListener('input', function(){ wifiFormDirty = true; });
document.getElementById('wifi-form').addEventListener('submit', function(e){
  e.preventDefault();
  wifiFormDirty = false;
  post('/api/wifi', {ssid: document.getElementById('wifi-ssid').value, password: document.getElementById('wifi-pass').value});
});

document.getElementById('carbon-host').addEventListener('input', function(){ carbonFormDirty = true; });
document.getElementById('carbon-port').addEventListener('input', function(){ carbonFormDirty = true; });
document.getElementById('carbon-form').addEventListener('submit', function(e){
  e.preventDefault();
  carbonFormDirty = false;
  post('/api/carbon', {host: document.getElementById('carbon-host').value, port: document.getElementById('carbon-port').value}).then(function(){
    document.getElementById('carbon-msg').textContent = 'Saved.';
  });
});

document.getElementById('reboot-btn').addEventListener('click', function(){
  if (!confirm('Reboot the device now?')) return;
  post('/api/reboot');
});
document.getElementById('logout').addEventListener('click', function(e){
  e.preventDefault();
  post('/logout').then(function(){ location.reload(); });
});
</script>
</body></html>
)HTML";
