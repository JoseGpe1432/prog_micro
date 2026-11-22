"""Panel local Flask: seleccion, inicio, pausa, parada enclavada y camara MJPEG."""
import threading
import time
from flask import Flask, Response, jsonify, request, render_template_string

PAGE = r"""<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Roomba | Control</title><style>body{font-family:system-ui;background:#111827;color:#f9fafb;max-width:1100px;margin:20px auto;padding:0 15px}header{display:flex;align-items:center;justify-content:space-between;gap:10px;flex-wrap:wrap}.layout{display:grid;grid-template-columns:1.4fr 1fr;gap:18px}.panel{background:#1f2937;border:1px solid #374151;padding:16px;border-radius:12px;margin-bottom:16px}img{width:100%;background:#030712;border-radius:8px;min-height:200px;object-fit:contain}button,select{font:inherit;border:0;border-radius:8px;padding:11px 15px;margin:4px;background:#374151;color:white;cursor:pointer}button.start{background:#047857}button.stop{background:#b91c1c}button.pause{background:#b45309}button:disabled{opacity:.5}pre{white-space:pre-wrap;word-break:break-word}canvas{width:100%;image-rendering:pixelated;background:#374151;border-radius:8px}@media(max-width:800px){.layout{grid-template-columns:1fr}}</style></head><body><header><h1>Control de Roomba</h1><strong id="mode">DETENIDO</strong></header><div class="layout"><section><div class="panel"><h2>Cámara en vivo</h2><img src="/video" alt="Vista de la cámara"><p>La cámara identifica posibles residuos; la distancia frontal viene del HC-SR04.</p></div><div class="panel"><h2>Mapa estimado</h2><canvas id="map" width="400" height="400"></canvas><p>Gris: desconocido · claro: libre · rojo: obstáculo · azul: robot. Mapa aproximado, no SLAM completo.</p></div></section><section><div class="panel"><h2>Navegación</h2><label>Algoritmo <select id="planner"><option>BFS</option><option>DFS</option><option>UCS</option><option value="GREEDY">Greedy</option><option value="ASTAR">A*</option></select></label><p><button class="start" onclick="act('start')">Iniciar</button><button class="pause" onclick="act('pause')">Pausar</button><button class="stop" onclick="act('stop')">DETENER</button></p><p>Para reanudar tras DETENER, reinicia el programa desde la terminal.</p></div><div class="panel"><h2>Telemetría</h2><pre id="status">Esperando datos...</pre></div></section></div><script>async function act(a){if(a==='stop'&&!confirm('¿Detener y bloquear el robot hasta reiniciar el programa?'))return;let r=await fetch('/api/'+a,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({planner:document.getElementById('planner').value})});let d=await r.json();if(!r.ok)alert(d.error||'Error');refresh()}async function refresh(){try{let s=await(await fetch('/api/status')).json();document.getElementById('mode').textContent=s.mode;document.getElementById('planner').value=s.planner;document.getElementById('planner').disabled=s.mode==='RUNNING'||s.mode==='STOPPED';document.getElementById('status').textContent=JSON.stringify(s,null,2);let m=await(await fetch('/api/map')).json();let c=document.getElementById('map'),ctx=c.getContext('2d'),n=m.n,sz=c.width/n;ctx.clearRect(0,0,c.width,c.height);m.cells.forEach((row,r)=>row.forEach((v,col)=>{ctx.fillStyle=v===2?'#dc665b':v===1?'#e5e7eb':'#6b7280';ctx.fillRect(col*sz,r*sz,Math.ceil(sz),Math.ceil(sz))}));if(m.robot){let [r,col]=m.robot;ctx.fillStyle='#2563eb';ctx.beginPath();ctx.arc((col+.5)*sz,(r+.5)*sz,Math.max(3,sz),0,7);ctx.fill()}}catch(e){document.getElementById('status').textContent='Sin conexión con el programa: '+e}}setInterval(refresh,700);refresh()</script></body></html>"""

def make_app(controller):
    app = Flask(__name__)
    @app.get('/')
    def index(): return render_template_string(PAGE)
    @app.get('/api/status')
    def status(): return jsonify(controller.status())
    @app.get('/api/map')
    def grid(): return jsonify(controller.map_snapshot())
    @app.post('/api/start')
    def start():
        data=request.get_json(silent=True) or {}
        ok,msg=controller.start(data.get('planner','BFS'))
        return jsonify({'ok':ok,'error':None if ok else msg}), 200 if ok else 409
    @app.post('/api/pause')
    def pause():
        controller.pause(); return jsonify({'ok':True})
    @app.post('/api/stop')
    def stop():
        controller.stop(); return jsonify({'ok':True})
    @app.get('/video')
    def video():
        def gen():
            while True:
                frame=controller.camera_jpeg()
                if frame:
                    yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+frame+b'\r\n'
                time.sleep(0.12)
        return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')
    return app
