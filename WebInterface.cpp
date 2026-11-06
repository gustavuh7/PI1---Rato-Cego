/**
 * @file WebInterface.cpp
 * @brief Servidor Web embarcado no ESP32 (Core 0) com Dashboard visual HTML5/Canvas
 *        para iniciar, parar, resetar e monitorar o robô dentro do labirinto sem fios.
 */

#include "WebInterface.h"
#include <WiFi.h>
#include <WebServer.h>

static WebServer s_server(80);

// =============================================================================
// PÁGINA HTML DO PAINEL DE CONTROLE VISUAL (EMBARCADA NA FLASH PROGMEM)
// =============================================================================
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Rato Cego ESP32 - Painel do Labirinto</title>
<style>
  :root {
    --bg: #0f172a; --card: #1e293b; --border: #334155;
    --text: #f8fafc; --muted: #94a3b8;
    --green: #22c55e; --red: #ef4444; --blue: #3b82f6; --amber: #f59e0b;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: system-ui, -apple-system, sans-serif; }
  body { background: var(--bg); color: var(--text); padding: 16px; max-width: 1100px; margin: 0 auto; }
  header { display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 12px; margin-bottom: 16px; padding-bottom: 12px; border-bottom: 1px solid var(--border); }
  h1 { font-size: 1.35rem; display: flex; align-items: center; gap: 10px; }
  .badge { padding: 6px 12px; border-radius: 999px; font-weight: 700; font-size: 0.85rem; letter-spacing: 0.5px; }
  .st-wait { background: rgba(245,158,11,0.2); color: var(--amber); border: 1px solid var(--amber); }
  .st-run  { background: rgba(34,197,94,0.2); color: var(--green); border: 1px solid var(--green); }
  .st-win  { background: rgba(59,130,246,0.25); color: #60a5fa; border: 1px solid var(--blue); }
  .grid { display: grid; grid-template-columns: 1fr 1.2fr; gap: 16px; }
  @media (max-width: 800px) { .grid { grid-template-columns: 1fr; } }
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 12px; padding: 16px; }
  .card h2 { font-size: 1rem; color: var(--muted); text-transform: uppercase; letter-spacing: 0.6px; margin-bottom: 14px; }
  .btn-row { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 14px; }
  button { border: none; border-radius: 10px; padding: 16px 14px; font-size: 1.05rem; font-weight: 700; cursor: pointer; color: #fff; transition: transform 0.1s, opacity 0.2s; display: flex; align-items: center; justify-content: center; gap: 8px; }
  button:active { transform: scale(0.97); }
  .btn-start { background: var(--green); box-shadow: 0 4px 14px rgba(34,197,94,0.35); }
  .btn-stop  { background: var(--red);   box-shadow: 0 4px 14px rgba(239,68,68,0.35); }
  .btn-reset { background: var(--blue);  grid-column: span 2; padding: 12px; font-size: 0.95rem; }
  .cfg-row { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 12px; }
  label { font-size: 0.82rem; color: var(--muted); display: block; margin-bottom: 4px; }
  select, input { width: 100%; padding: 9px 10px; border-radius: 8px; border: 1px solid var(--border); background: #0f172a; color: var(--text); font-size: 0.95rem; }
  .stats { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; margin-top: 12px; }
  .stat-box { background: #0f172a; border: 1px solid var(--border); border-radius: 8px; padding: 10px 12px; }
  .stat-label { font-size: 0.76rem; color: var(--muted); }
  .stat-val { font-size: 1.15rem; font-weight: 700; margin-top: 2px; }
  .sensors { display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px; margin-top: 12px; }
  .sens-item { background: #0f172a; border-radius: 8px; padding: 8px 10px; border: 1px solid var(--border); display: flex; justify-content: space-between; align-items: center; }
  .wall-on  { color: var(--red); font-weight: 700; }
  .wall-off { color: var(--green); font-weight: 700; }
  canvas { width: 100%; max-width: 480px; aspect-ratio: 1/1; background: #090d16; border-radius: 8px; border: 1px solid var(--border); display: block; margin: 0 auto; }
  .legend { display: flex; gap: 14px; justify-content: center; flex-wrap: wrap; margin-top: 10px; font-size: 0.82rem; color: var(--muted); }
  .dot { width: 10px; height: 10px; border-radius: 50%; display: inline-block; margin-right: 5px; }
</style>
</head>
<body>
<header>
  <h1>🐭 Rato Cego ESP32 — Controle do Labirinto</h1>
  <span id="statusBadge" class="badge st-wait">AGUARDANDO INÍCIO</span>
</header>

<div class="grid">
  <div class="card">
    <h2>Comando de Partida e Parada</h2>
    <div class="btn-row">
      <button class="btn-start" onclick="sendStart()">▶ INICIAR ROBÔ</button>
      <button class="btn-stop"  onclick="sendStop()">⏹ PARAR AGORA</button>
    </div>

    <div class="cfg-row">
      <div>
        <label>Canto Inicial</label>
        <select id="cornerSel">
          <option value="0">Inferior Esquerdo (0,0)</option>
          <option value="1">Inferior Direito (M-1,0)</option>
        </select>
      </div>
      <div>
        <label>Dimensão do Labirinto (MxM)</label>
        <input id="sizeInp" type="number" min="2" max="18" value="18">
      </div>
    </div>
    <button class="btn-reset" onclick="sendReset()">↺ APLICAR CONFIGURAÇÃO E ZERAR MAPA</button>

    <div class="stats">
      <div class="stat-box">
        <div class="stat-label">Posição Atual &rarr; Alvo</div>
        <div class="stat-val" id="posVal">(0, 0) &rarr; (17, 17)</div>
      </div>
      <div class="stat-box">
        <div class="stat-label">Orientação Global (Heading)</div>
        <div class="stat-val" id="headVal">NORTE (+Y)</div>
      </div>
      <div class="stat-box" style="grid-column: span 2;">
        <div class="stat-label">Frente Virtual Ativa (Sem Giro de 180°)</div>
        <div class="stat-val" id="frontVal" style="color:#38bdf8;">NORMAL — Frente Física (GPIO 34)</div>
      </div>
    </div>

    <h2 style="margin-top:16px;">Sensores Virtuais FC-51 (ADC ESP32)</h2>
    <div class="sensors">
      <div class="sens-item"><span>⬅ Esquerda: <b id="sLeft">0</b></span><span id="wLeft" class="wall-off">LIVRE</span></div>
      <div class="sens-item"><span>⬆ Frente: <b id="sFront">0</b></span><span id="wFront" class="wall-off">LIVRE</span></div>
      <div class="sens-item"><span>➡ Direita: <b id="sRight">0</b></span><span id="wRight" class="wall-off">LIVRE</span></div>
      <div class="sens-item"><span>⬇ Traseira: <b id="sRear">0</b></span><span id="wRear" class="wall-off">LIVRE</span></div>
    </div>
  </div>

  <div class="card">
    <h2>Mapa Matricial em Tempo Real (<span id="mazeDimLbl">18x18</span>)</h2>
    <canvas id="mazeCanvas" width="540" height="540"></canvas>
    <div class="legend">
      <span><i class="dot" style="background:#3b82f6"></i>Robô Atual</span>
      <span><i class="dot" style="background:#22c55e"></i>Célula Alvo</span>
      <span><i class="dot" style="background:#1e3a5f"></i>Célula Visitada</span>
      <span><i class="dot" style="background:#f43f5e"></i>Parede Detectada</span>
    </div>
  </div>
</div>

<script>
const HEADINGS = ["NORTE (+Y)", "LESTE (+X)", "SUL (-Y)", "OESTE (-X)"];

async function sendStart() {
  await fetch('/api/start', { method: 'POST' });
  fetchStatus();
}
async function sendStop() {
  await fetch('/api/stop', { method: 'POST' });
  fetchStatus();
}
async function sendReset() {
  const c = document.getElementById('cornerSel').value;
  const s = document.getElementById('sizeInp').value;
  await fetch(`/api/reset?corner=${c}&size=${s}`, { method: 'POST' });
  fetchStatus();
}

function setWallUI(idVal, idSt, raw, wall) {
  document.getElementById(idVal).textContent = raw;
  const el = document.getElementById(idSt);
  el.textContent = wall ? 'PAREDE' : 'LIVRE';
  el.className = wall ? 'wall-on' : 'wall-off';
}

function drawMaze(d) {
  const cv = document.getElementById('mazeCanvas');
  const ctx = cv.getContext('2d');
  const M = d.size || 18;
  const pad = 12;
  const cell = (cv.width - pad * 2) / M;

  ctx.clearRect(0, 0, cv.width, cv.height);

  for (let x = 0; x < M; x++) {
    for (let y = 0; y < M; y++) {
      const val = d.maze[x * M + y] || 0;
      const px = pad + x * cell;
      const py = pad + (M - 1 - y) * cell;

      if (val & 0x10) {
        ctx.fillStyle = '#172554';
        ctx.fillRect(px + 1, py + 1, cell - 2, cell - 2);
      }

      if (x === d.tx && y === d.ty) {
        ctx.fillStyle = 'rgba(34,197,94,0.35)';
        ctx.fillRect(px + 2, py + 2, cell - 4, cell - 4);
      }

      ctx.strokeStyle = '#1e293b';
      ctx.lineWidth = 1;
      ctx.strokeRect(px, py, cell, cell);

      ctx.strokeStyle = '#f43f5e';
      ctx.lineWidth = 3;
      ctx.beginPath();
      if (val & 0x01) { ctx.moveTo(px, py); ctx.lineTo(px + cell, py); }
      if (val & 0x02) { ctx.moveTo(px + cell, py); ctx.lineTo(px + cell, py + cell); }
      if (val & 0x04) { ctx.moveTo(px, py + cell); ctx.lineTo(px + cell, py + cell); }
      if (val & 0x08) { ctx.moveTo(px, py); ctx.lineTo(px, py + cell); }
      ctx.stroke();
    }
  }

  const rx = pad + (d.x + 0.5) * cell;
  const ry = pad + (M - 1 - d.y + 0.5) * cell;
  ctx.fillStyle = d.inv ? '#f59e0b' : '#3b82f6';
  ctx.beginPath();
  ctx.arc(rx, ry, cell * 0.32, 0, Math.PI * 2);
  ctx.fill();
}

async function fetchStatus() {
  try {
    const r = await fetch('/api/status');
    const d = await r.json();

    const b = document.getElementById('statusBadge');
    if (d.done) { b.textContent = '🏆 LABIRINTO CONCLUÍDO!'; b.className = 'badge st-win'; }
    else if (d.run) { b.textContent = '🟢 EM MOVIMENTO NO LABIRINTO'; b.className = 'badge st-run'; }
    else { b.textContent = '⏸ PARADO / AGUARDANDO INÍCIO'; b.className = 'badge st-wait'; }

    document.getElementById('posVal').innerHTML = `(${d.x}, ${d.y}) &rarr; (${d.tx}, ${d.ty})`;
    document.getElementById('headVal').textContent = HEADINGS[d.h] || 'NORTE';
    document.getElementById('mazeDimLbl').textContent = `${d.size}x${d.size}`;

    const fv = document.getElementById('frontVal');
    if (d.inv) {
      fv.textContent = 'INVERTIDA — Traseira Física é a Frente (GPIO 35)';
      fv.style.color = '#f59e0b';
    } else {
      fv.textContent = 'NORMAL — Frente Física Ativa (GPIO 34)';
      fv.style.color = '#38bdf8';
    }

    setWallUI('sLeft',  'wLeft',  d.sl, d.wl);
    setWallUI('sFront', 'wFront', d.sf, d.wf);
    setWallUI('sRight', 'wRight', d.sr, d.wr);
    setWallUI('sRear',  'wRear',  d.sb, d.wb);

    drawMaze(d);
  } catch (e) {}
}

setInterval(fetchStatus, 250);
fetchStatus();
</script>
</body>
</html>
)rawliteral";

// =============================================================================
// ROTAS HTTP DO SERVIDOR
// =============================================================================
static void addCorsHeaders(void)
{
    s_server.sendHeader("Access-Control-Allow-Origin", "*");
    s_server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
}

static void handleRoot(void)
{
    addCorsHeaders();
    s_server.send_P(200, "text/html", INDEX_HTML);
}

static void handleStatus(void)
{
    const NavigationState_t* st = Navigation_GetState();
    VirtualSensors_t vs;
    Navigation_ReadVirtualSensors(&vs);

    String json;
    json.reserve(1600);
    json += "{";
    json += "\"x\":" + String(st->x) + ",";
    json += "\"y\":" + String(st->y) + ",";
    json += "\"tx\":" + String(st->target_x) + ",";
    json += "\"ty\":" + String(st->target_y) + ",";
    json += "\"size\":" + String(st->maze_size) + ",";
    json += "\"h\":" + String((int)st->heading) + ",";
    json += "\"inv\":" + String(st->inverted_front ? "true" : "false") + ",";
    json += "\"run\":" + String(st->is_running ? "true" : "false") + ",";
    json += "\"done\":" + String(st->maze_completed ? "true" : "false") + ",";
    json += "\"sl\":" + String(vs.raw_left) + ",";
    json += "\"sf\":" + String(vs.raw_front) + ",";
    json += "\"sr\":" + String(vs.raw_right) + ",";
    json += "\"sb\":" + String(vs.raw_rear) + ",";
    json += "\"wl\":" + String(vs.wall_left ? "true" : "false") + ",";
    json += "\"wf\":" + String(vs.wall_front ? "true" : "false") + ",";
    json += "\"wr\":" + String(vs.wall_right ? "true" : "false") + ",";
    json += "\"wb\":" + String(vs.wall_rear ? "true" : "false") + ",";
    json += "\"maze\":[";

    uint8_t M = st->maze_size;
    for (uint8_t x = 0; x < M; x++)
    {
        for (uint8_t y = 0; y < M; y++)
        {
            if (x > 0 || y > 0) json += ",";
            json += String(st->maze[x][y]);
        }
    }
    json += "]}";

    addCorsHeaders();
    s_server.send(200, "application/json", json);
}

static void handleStart(void)
{
    Navigation_Start();
    addCorsHeaders();
    s_server.send(200, "application/json", "{\"ok\":true,\"action\":\"start\"}");
}

static void handleStop(void)
{
    Navigation_Stop();
    addCorsHeaders();
    s_server.send(200, "application/json", "{\"ok\":true,\"action\":\"stop\"}");
}

static void handleReset(void)
{
    Navigation_Stop();

    StartCorner_t corner = START_BOTTOM_LEFT;
    uint8_t size = MAZE_MAX_SIZE;

    if (s_server.hasArg("corner"))
    {
        corner = (s_server.arg("corner").toInt() == 1) ? START_BOTTOM_RIGHT : START_BOTTOM_LEFT;
    }
    if (s_server.hasArg("size"))
    {
        int s = s_server.arg("size").toInt();
        if (s >= 2 && s <= MAZE_MAX_SIZE) size = (uint8_t)s;
    }

    Navigation_Init(corner, size, HEADING_NORTH);
    addCorsHeaders();
    s_server.send(200, "application/json", "{\"ok\":true,\"action\":\"reset\"}");
}

// Tarefa dedicada no Core 0 para responder imediatamente aos botões Web
static void webServerTask(void* pvParameters)
{
    (void)pvParameters;
    for (;;)
    {
        s_server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void WebInterface_Init(void)
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL);

    s_server.on("/",          HTTP_GET,  handleRoot);
    s_server.on("/api/status", HTTP_GET,  handleStatus);
    s_server.on("/api/start",  HTTP_POST, handleStart);
    s_server.on("/api/stop",   HTTP_POST, handleStop);
    s_server.on("/api/reset",  HTTP_POST, handleReset);

    s_server.begin();

    Serial.println("======================================================");
    Serial.printf("[WIFI AP] Rede criada: %s (Senha: %s)\n", WIFI_AP_SSID, WIFI_AP_PASSWORD);
    Serial.print("[WEB UI]  Acesse no celular/PC: http://");
    Serial.println(WiFi.softAPIP());
    Serial.println("======================================================");

    // Executa o servidor HTTP no Core 0 do ESP32 para nunca travar com o loop de movimento
    xTaskCreatePinnedToCore(
        webServerTask,
        "WebUI_Task",
        4096,
        NULL,
        1,
        NULL,
        0
    );
}

