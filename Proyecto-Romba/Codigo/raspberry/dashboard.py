"""Panel de planeación y prueba motorizada supervisada."""
import time
from route_execution import simulate
from flask import Flask, Response, jsonify, render_template_string, request

PAGE=r'''<!doctype html><html lang="es"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Roomba - Navegación y visión (pruebas)</title>
<style>body{font:15px system-ui;background:#101827;color:#f1f5f9;max-width:1250px;margin:auto;padding:18px}.layout{display:grid;grid-template-columns:1fr 1fr;gap:16px}.panel{background:#1f2937;padding:16px;border-radius:12px}img,canvas{width:100%;background:#374151;border-radius:8px}button,select{padding:10px;border:0;border-radius:7px;cursor:pointer}button{background:#0d9488;color:white}button.stop{background:#b91c1c}pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:12px}#map{cursor:crosshair}.muted{color:#cbd5e1}#notice{color:#fbbf24}@media(max-width:750px){.layout{grid-template-columns:1fr}}</style>
<h1>Roomba · Cámara, detección y planificadores</h1><p id="notice">Sin HC-SR04. Exploración reactiva experimental con botón mantenido. Las rutas son vistas previas. Usa una parada física de motores; no probar cerca de desniveles.</p>
<div class="panel"><label>Algoritmo <select id="planner"><option>BFS</option><option>DFS</option><option>UCS</option><option>GREEDY</option><option>ASTAR</option></select></label>
<button onclick="start()">Planificar ruta (sin motores)</button> <button onclick="pause()">Pausar</button> <button class="stop" onclick="stop()">Detener</button>
<div style="margin-top:12px"><strong>Prueba de motores: mantener pulsado (se detiene al soltar). Verifica físicamente la zona frontal antes de pulsar. Evasión experimental: parada a 25 cm y giros cortos. Verifica físicamente TODO el espacio alrededor del robot antes y durante el permiso; suelta si hay riesgo.</strong><p><button class="hold" data-dir="startup">Arranque supervisado (2 s)</button> <button class="hold" data-dir="auto">Explorar y esquivar (mantener; espacio lateral revisado)</button> <button class="hold" data-dir="forward">Adelante</button> <button class="hold" data-dir="left">Girar izquierda</button> <button class="hold" data-dir="right">Girar derecha</button> <button class="hold" data-dir="back">Atrás</button></p></div><p style="color:#fbbf24"><label><input type="checkbox" id="visualtest" onchange="visualTest(this.checked)">
PRUEBA CONTROLADA: ignorar advertencias visuales durante avance manual (máx. 15 mm/s).
Solo en suelo plano y despejado, con corte físico de motores. No activa autonomía.</label></p>
<label style="margin-left:15px"><input type="checkbox" id="demo" onchange="demo(this.checked)"> Escenario DEMO (sintético)</label><p id="message"></p></div>
<p id="motionlive" style="font-weight:bold;color:#fbbf24"></p><div class="layout"><div class="panel"><h2>Cámara USB</h2><img src="/video"><h3>Corredor y barreras naranjas (estimación)</h3><img src="/video/segmentation"><pre id="nearinfo"></pre><p><button onclick="post('/api/floor-reference')">Capturar referencia SIN obstáculo</button> <button onclick="post('/api/floor-reference/clear')">Borrar referencia</button></p><p class="muted">Referencia de escena estática: coloca la cámara fija y retira el obstáculo antes de capturar. Rojo = cambios respecto a referencia; no es detección de distancia.</p><img src="/video/diagnostic"><p id="decision" style="font-weight:bold;color:#fbbf24"></p><pre id="floorstatus"></pre><h3>V6 · Movimiento de cámara (diagnóstico)</h3><p class="muted">Mueve el robot manualmente con motores desconectados. Si se detecta movimiento, la referencia fija queda invalidada hasta capturar una nueva. No mide distancias ni habilita motores.</p><pre id="cameramotion"></pre><h3>V6.1 · Diferencias tras compensar movimiento</h3><p class="muted">Rojo: residuo visual entre fotogramas alineados; puede ser perspectiva, sombras u objetos. NO confirma obstáculos ni espacio libre.</p><img src="/video/residual"><pre id="motionresidual"></pre><h3>V6.3 · Seguimiento predictivo de regiones</h3><p class="muted">ID N = candidato nuevo; ID P = persistente. La persistencia no confirma obstáculos. Sin control de motores.</p><pre id="regiontracking"></pre><h3>V6.4 · Segmentación del suelo</h3><button onclick="post('/api/segmentation/calibrate')">Calibrar apariencia del suelo despejado</button><p class="muted">Verde: apariencia similar al suelo; rojo: diferencia. No certifica espacio libre.</p><img src="/video/segmentation"><pre id="segmentation"></pre><h3>Integración visual con BFS / DFS / UCS / Greedy / A*</h3><pre id="visualplanning"></pre><h3>Detecciones (candidatas, no confirmadas)</h3><pre id="detections"></pre></div>
<div class="panel"><h2>V6.8.3 · Mapa visual experimental y rutas</h2><p class="muted">Haz clic en una celda clara para fijar una meta. Gris: desconocido; rojo: posible obstáculo; claro: apariencia de suelo; azul: ruta experimental. <button onclick="post('/api/map/reset')">Reiniciar mapa y posición</button> <a href="/api/map" download="mapa_v68.json" style="color:#93c5fd">Guardar mapa JSON</a></p><canvas id="map" width="500" height="500"></canvas><p id="mapmsg"></p></div>
<div class="panel"><h2>Simulación de conducción y evasión (sin motores)</h2><button onclick="runSimulation()">Ejecutar algoritmo seleccionado</button><canvas id="simmap" width="500" height="500"></canvas><p id="simmsg"></p></div><div class="panel"><h2>Trayectoria visual</h2><canvas id="trail" width="500" height="500"></canvas><p id="quality"></p></div>
<div class="panel"><h2>Estado</h2><pre id="status"></pre></div></div>
<script>
let mapData=null;
async function runSimulation(){let msg=document.getElementById('simmsg');msg.textContent='Calculando simulación';try{let r=await fetch('/api/simulate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({algorithm:document.getElementById('planner').value})});let m=await r.json();if(!r.ok)throw Error(m.error);let c=document.getElementById('simmap'),ctx=c.getContext('2d'),n=m.cells.length,z=c.width/n;ctx.clearRect(0,0,c.width,c.height);m.cells.forEach((row,y)=>row.forEach((v,x)=>{ctx.fillStyle=v===2?'#dc665b':v===1?'#e5e7eb':'#6b7280';ctx.fillRect(x*z,y*z,Math.ceil(z),Math.ceil(z))}));ctx.strokeStyle='#2563eb';ctx.lineWidth=2;ctx.beginPath();m.route.forEach(([y,x],i)=>{if(i)ctx.lineTo((x+.5)*z,(y+.5)*z);else ctx.moveTo((x+.5)*z,(y+.5)*z)});ctx.stroke();ctx.strokeStyle='#059669';ctx.lineWidth=3;ctx.beginPath();m.trajectory.forEach(([x,y],i)=>{let px=(n/2+x/.05+.5)*z,py=(n/2-y/.05+.5)*z;if(i)ctx.lineTo(px,py);else ctx.moveTo(px,py)});ctx.stroke();msg.textContent=m.algorithm+' | '+m.state+' | '+m.steps+' pasos simulados (sin motores)'}catch(e){msg.textContent=String(e)}}

async function post(url,data={}){try{let r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)}),j=await r.json();document.getElementById('message').textContent=j.message||j.error||JSON.stringify(j)}catch(e){document.getElementById('message').textContent=e}}
let activeHold=null, holdTimer=null;
function release(){if(holdTimer)clearInterval(holdTimer);holdTimer=null;activeHold=null;post('/api/manual',{direction:'release'})}
document.querySelectorAll('.hold').forEach(b=>{
 b.addEventListener('pointerdown',e=>{e.preventDefault();if(activeHold)release();activeHold=b.dataset.dir;b.setPointerCapture(e.pointerId);post('/api/manual',{direction:activeHold});holdTimer=setInterval(()=>{if(activeHold)post('/api/manual',{direction:activeHold})},100)});
 ['pointerup','pointercancel','lostpointercapture'].forEach(t=>b.addEventListener(t,release));
});
window.addEventListener('blur',release);
document.addEventListener('visibilitychange',()=>{if(document.hidden)release()});
function start(){post('/api/start',{planner:document.getElementById('planner').value})}
function pause(){release();post('/api/pause')}
function stop(){release();post('/api/stop')}
function visualTest(enabled){release();post('/api/visual-test',{enabled})}
function demo(enabled){post('/api/demo',{enabled})}
document.getElementById('planner').onchange=e=>post('/api/planner',{planner:e.target.value});
document.getElementById('map').onclick=e=>{if(!mapData)return;let rect=e.target.getBoundingClientRect();let c=Math.min(mapData.n-1,Math.max(0,Math.floor((e.clientX-rect.left)/rect.width*mapData.n))),r=Math.min(mapData.n-1,Math.max(0,Math.floor((e.clientY-rect.top)/rect.height*mapData.n)));post('/api/goal',{row:r,col:c})};
function draw(){fetch('/api/status').then(r=>r.json()).then(s=>{document.getElementById('motionlive').textContent='Movimiento: '+s.motion_state+' | Odometría: '+(s.visual_odometry||{}).quality+' | Umbral: '+(s.visual_odometry||{}).flow_threshold_px+' px';document.getElementById('nearinfo').textContent=JSON.stringify({proximity:(s.floor_segmentation||{}).proximity,orange:(s.floor_segmentation||{}).orange_barriers,contrast:s.contrast_near_candidates,historial:s.corridor_history,camera_forward:s.camera_forward,camera_avoid:s.camera_avoid},null,2);document.getElementById('status').textContent=JSON.stringify(s,null,2);document.getElementById('floorstatus').textContent=JSON.stringify(s.floor_reference||{},null,2);document.getElementById('decision').textContent='Evaluación visual: '+((s.floor_reference||{}).decision||'SIN REFERENCIA')+' (solo diagnóstico, sin motores)';document.getElementById('cameramotion').textContent=JSON.stringify(s.camera_motion||{},null,2);document.getElementById('motionresidual').textContent=JSON.stringify(s.motion_residual||{},null,2);document.getElementById('segmentation').textContent=JSON.stringify(s.floor_segmentation||{},null,2);document.getElementById('regiontracking').textContent=JSON.stringify(s.region_tracking||{},null,2);document.getElementById('visualplanning').textContent=JSON.stringify(s.visual_planning||{},null,2);document.getElementById('detections').textContent=JSON.stringify(s.detections||[],null,2);document.getElementById('planner').value=s.planner;document.getElementById('demo').checked=s.preview.demo;document.getElementById('visualtest').checked=!!s.visual_test_override;}).catch(()=>{});
fetch('/api/map').then(r=>r.json()).then(m=>{mapData=m;let c=document.getElementById('map'),cx=c.getContext('2d'),z=c.width/m.n;cx.clearRect(0,0,500,500);m.cells.forEach((row,r)=>row.forEach((a,col)=>{cx.fillStyle=a===2?'#dc665b':a===1?'#e5e7eb':'#6b7280';cx.fillRect(col*z,r*z,Math.ceil(z),Math.ceil(z))}));if(m.route){cx.strokeStyle='#2563eb';cx.lineWidth=Math.max(2,z*.7);cx.beginPath();m.route.forEach(([r,col],i)=>{let x=(col+.5)*z,y=(r+.5)*z;if(i)cx.lineTo(x,y);else cx.moveTo(x,y)});cx.stroke()}if(m.goal){let [r,col]=m.goal;cx.fillStyle='#f59e0b';cx.beginPath();cx.arc((col+.5)*z,(r+.5)*z,6,0,7);cx.fill()}if(m.robot){let [r,col]=m.robot;cx.fillStyle='#0d9488';cx.beginPath();cx.arc((col+.5)*z,(r+.5)*z,6,0,7);cx.fill()}document.getElementById('mapmsg').textContent=m.message+' | '+m.mapping.state+' | observaciones: '+m.mapping.frames}).catch(()=>{});
fetch('/api/visual_path').then(r=>r.json()).then(v=>{let c=document.getElementById('trail'),cx=c.getContext('2d');cx.clearRect(0,0,500,500);let p=v.trajectory||[];cx.strokeStyle='#38bdf8';cx.lineWidth=3;cx.beginPath();p.forEach((xy,i)=>{let x=250+xy[0]*130,y=250-xy[1]*130;if(i)cx.lineTo(x,y);else cx.moveTo(x,y)});cx.stroke();document.getElementById('quality').textContent='Calidad: '+v.quality+' | escala no calibrada'}).catch(()=>{})}
setInterval(draw,1000);draw();
</script>'''

def make_app(controller):
    app=Flask(__name__)
    @app.post('/api/simulate')
    def simulate_route():
        data=request.get_json(silent=True) or {}
        try: return jsonify(simulate(data.get('algorithm','ASTAR')))
        except ValueError as exc: return jsonify(error=str(exc)),400
    @app.get('/')
    def index():return render_template_string(PAGE)
    @app.post('/api/segmentation/calibrate')
    def segmentation_calibrate():
        ok,msg=controller.calibrate_segmentation()
        return jsonify(ok=ok,message=msg),200 if ok else 400
    @app.get('/video/segmentation')
    def segmentation_video():
        def frames():
            while True:
                jpg=controller.segmentation_jpeg()
                if jpg: yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+jpg+b'\r\n'
                time.sleep(.25)
        return Response(frames(),mimetype='multipart/x-mixed-replace; boundary=frame')
    @app.get('/video/residual')
    def residual_video():
        def frames():
            while True:
                jpg=controller.motion_residual_jpeg()
                if jpg:
                    yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+jpg+b'\r\n'
                time.sleep(.15)
        return Response(frames(),mimetype='multipart/x-mixed-replace; boundary=frame')
    @app.get('/api/status')
    def status():return jsonify(controller.status())
    @app.post('/api/map/reset')
    def reset_map():
        ok,msg=controller.reset_mapping()
        return jsonify(ok=ok,message=msg),200 if ok else 400
    @app.get('/api/map')
    def grid():return jsonify(controller.map_snapshot())
    @app.get('/api/visual_path')
    def path():return jsonify(controller.visual_path())
    @app.post('/api/visual-test')
    def visual_test():
        data=request.get_json(silent=True) or {}
        ok,msg=controller.set_visual_test_override(data.get('enabled') is True)
        return jsonify(ok=ok,message=msg),200 if ok else 400

    @app.post('/api/manual')
    def manual():
        data=request.get_json(silent=True) or {}
        ok,msg=controller.manual_drive(data.get('direction'))
        return jsonify(ok=ok,message=msg),200 if ok else 400
    @app.post('/api/start')
    def start():
        data=request.get_json(silent=True) or {}
        ok,msg=controller.start(data.get('planner','BFS'))
        return jsonify(ok=ok,message=msg),200 if ok else 400
    @app.post('/api/planner')
    def planner():
        data=request.get_json(silent=True) or {}
        ok,msg=controller.set_planner(data.get('planner'))
        return jsonify(ok=ok,message=msg),200 if ok else 400
    @app.post('/api/goal')
    def goal():
        data=request.get_json(silent=True) or {}
        try:r,c=int(data['row']),int(data['col'])
        except (KeyError,ValueError,TypeError):return jsonify(ok=False,error='Celda inválida'),400
        ok,msg=controller.set_goal(r,c)
        return jsonify(ok=ok,message=msg),200 if ok else 400
    @app.post('/api/demo')
    def demo():
        data=request.get_json(silent=True) or {}
        ok,msg=controller.set_demo(data.get('enabled') is True)
        return jsonify(ok=ok,message=msg)
    @app.post('/api/pause')
    def pause():controller.pause();return jsonify(ok=True,message='Pausado. Motores detenidos')
    @app.post('/api/stop')
    def stop():controller.stop();return jsonify(ok=True,message='Detenido. Motores detenidos')
    @app.post('/api/floor-reference')
    def floor_reference():
        ok,msg=controller.calibrate_floor()
        return jsonify(ok=ok,message=msg),200 if ok else 400

    @app.post('/api/floor-reference/clear')
    def floor_reference_clear():
        ok,msg=controller.clear_floor()
        return jsonify(ok=ok,message=msg)

    @app.get('/video/diagnostic')
    def video_diagnostic():
        def gen():
            while True:
                frame=controller.annotated_camera_jpeg()
                if frame:yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+frame+b'\r\n'
                time.sleep(.25)
        return Response(gen(),mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.get('/video')
    def video():
        def gen():
            while True:
                frame=controller.camera_jpeg()
                if frame:yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+frame+b'\r\n'
                time.sleep(.12)
        return Response(gen(),mimetype='multipart/x-mixed-replace; boundary=frame')
    return app
