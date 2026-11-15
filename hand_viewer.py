#!/usr/bin/env python3
"""
hand_viewer.py - live 3D view of both forearms + hands around the Coil Pro, in your browser.

Reads the hand solver's solved hands (OpenXR joints) and draws them at two fixed spots,
so you only see what the fingers and wrists are doing. Runs on the laptop, so no cloud lag.

  1. rkk-hand-solver --emit-openxr --listen 0.0.0.0:12277      (already running for Isaac)
  2. python hand_viewer.py
  3. open http://localhost:8765

In the page: drag = orbit, scroll = zoom, C or the button = calibrate "forward"
(hold both hands flat, fingers pointing forward, then press C).
Standard library only.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

GROUP = "joints_openxr"
STATE = {"hands": {}, "connected": False, "coil": {}, "coil_connected": False}
LOCK = threading.Lock()


def recv_exact(s, n):
    b = bytearray()
    while len(b) < n:
        c = s.recv(n - len(b))
        if not c:
            raise ConnectionError("closed")
        b += c
    return bytes(b)


def rgmp_loop(host, port):
    layouts = {}   # device_id -> dict(hand, gid, names, offs, wrist_idx, radii)
    while True:
        try:
            s = socket.create_connection((host, port), timeout=5)
            s.settimeout(None)
            with LOCK:
                STATE["connected"] = True
            print(f"[viewer] connected to solver {host}:{port}", flush=True)
            while True:
                prefix, n = struct.unpack("<II", recv_exact(s, 8))
                p = recv_exact(s, n)
                if prefix == 1:
                    d = json.loads(p)
                    if d.get("device_type") != "solved_hand":
                        continue
                    for gid, g in enumerate(d.get("groups", [])):
                        if g.get("name") != GROUP:
                            continue
                        names, offs, off = [], [], 16
                        for st in g["streams"]:
                            size = 8 if st["data_type"].lstrip().startswith(("DOUBLE", "INT64", "UINT64")) else 4
                            for k in re.findall(r"\d+", "".join(re.findall(r"\[([^\]]*)\]", st["data_type"])) or "1"):
                                size *= int(k)
                            if st.get("measure_type") == "TRANSFORM" and size == 28:
                                names.append(st["target_frame"].removeprefix("xr_"))
                                offs.append(off)
                            off += size
                        radii = {e["target_frame"].removeprefix("xr_"): e.get("value", 0.008)
                                 for e in d.get("static_data") or [] if e.get("custom_label") == "joint_radius"}
                        hand = (d.get("device_info") or {}).get("hand", f"dev{d['device_id']}")
                        if "wrist" in names:
                            layouts[d["device_id"]] = dict(hand=hand, gid=gid, names=names, offs=offs,
                                                           wrist=names.index("wrist"),
                                                           radii=[float(radii.get(nm, 0.008)) for nm in names])
                            print(f"[viewer] {hand} hand ({len(names)} joints)", flush=True)
                elif prefix == 2:
                    dev, gid = struct.unpack_from("<II", p, 0)
                    L = layouts.get(dev)
                    if not L or L["gid"] != gid:
                        continue
                    pts = [struct.unpack_from("<3f", p, o) for o in L["offs"]]
                    w = pts[L["wrist"]]
                    rel = [[round(x - w[0], 4), round(y - w[1], 4), round(z - w[2], 4)] for x, y, z in pts]
                    with LOCK:
                        h = STATE["hands"].setdefault(L["hand"], {"n": 0})
                        h.update(names=L["names"], radii=L["radii"], pos=rel, t=time.time())
                        h["n"] += 1
                elif prefix == 3:
                    dev = struct.unpack_from("<I", p, 0)[0]
                    L = layouts.pop(dev, None)
                    if L:
                        with LOCK:
                            STATE["hands"].pop(L["hand"], None)
        except (OSError, ConnectionError) as e:
            with LOCK:
                STATE["connected"] = False
                STATE["hands"].clear()
            print(f"[viewer] solver {host}:{port}: {e} - is rkk-hand-solver running? retrying...", flush=True)
            layouts.clear()
            time.sleep(1.0)


def _qmul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by, aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw, aw*bw - ax*bx - ay*by - az*bz)


def _qrot(q, v):
    r = _qmul(_qmul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return r[:3]


def _qnorm(q):
    n = sum(c * c for c in q) ** 0.5 or 1.0
    return tuple(c / n for c in q)


def coil_loop(host, port):
    """Read the raw glove stream (rokoko-sdk or replay_csv.py): arm pose vs Coil Pro and vs hub
    -> absolute hub (hand) pose and arm-sensor pose in the Coil Pro frame."""
    layouts = {}   # device_id -> dict
    while True:
        try:
            s = socket.create_connection((host, port), timeout=5)
            s.settimeout(None)
            with LOCK:
                STATE["coil_connected"] = True
            print(f"[viewer] reading Coil poses from {host}:{port}", flush=True)
            while True:
                prefix, n = struct.unpack("<II", recv_exact(s, 8))
                p = recv_exact(s, n)
                if prefix == 1:
                    d = json.loads(p)
                    if not str(d.get("device_type", "")).startswith("Smartgloves"):
                        continue
                    hand = (d.get("device_info") or {}).get("hand")
                    for gid, g in enumerate(d.get("groups", [])):
                        if g.get("name") != "default":
                            continue
                        off, L = 16, {"hand": hand, "gid": gid, "coil": None, "hub": None, "flags": None, "bits": {}}
                        for st in g["streams"]:
                            size = 8 if st["data_type"].lstrip().startswith(("DOUBLE", "INT64", "UINT64")) else 4
                            for k in re.findall(r"\d+", "".join(re.findall(r"\[([^\]]*)\]", st["data_type"])) or "1"):
                                size *= int(k)
                            if st.get("target_frame") == "arm_sensor":
                                ref, mt = st.get("reference_frame"), st.get("measure_type")
                                if mt == "TRANSFORM" and ref == "coil_pro":
                                    L["coil"] = off
                                elif mt == "TRANSFORM" and ref == "hub":
                                    L["hub"] = off
                                elif mt == "STATUS_FLAGS" and ref == "coil_pro":
                                    L["flags"] = off
                                    L["bits"] = {v: int(k) for k, v in (st.get("bit_mapping") or {}).items()}
                            off += size
                        if L["coil"] is not None and L["hub"] is not None and hand:
                            layouts[d["device_id"]] = L
                elif prefix == 2:
                    dev, gid = struct.unpack_from("<II", p, 0)
                    L = layouts.get(dev)
                    if not L or L["gid"] != gid:
                        continue
                    ac = struct.unpack_from("<7f", p, L["coil"])      # arm in coil
                    ah = struct.unpack_from("<7f", p, L["hub"])       # arm in hub
                    ok = not (ac[0] == 0 and ac[1] == 0 and ac[2] == 0)
                    if L["flags"] is not None:
                        f = struct.unpack_from("<I", p, L["flags"])[0]
                        for name in ("is_coil_pro_present", "emf_flip_decided"):
                            if name in L["bits"] and not (f >> L["bits"][name]) & 1:
                                ok = False
                    if not ok:
                        with LOCK:
                            c = STATE["coil"].get(L["hand"])
                            if c:
                                c["ok"] = False
                                c["t"] = time.time()
                        continue
                    q_ac, q_ah = _qnorm(ac[3:7]), _qnorm(ah[3:7])
                    q_hub = _qnorm(_qmul(q_ac, (-q_ah[0], -q_ah[1], -q_ah[2], q_ah[3])))   # hub in coil
                    r = _qrot(q_hub, ah[:3])
                    p_hub = (ac[0] - r[0], ac[1] - r[1], ac[2] - r[2])
                    with LOCK:
                        STATE["coil"][L["hand"]] = {
                            "ok": True, "t": time.time(),
                            "hub_p": [round(v, 4) for v in p_hub], "hub_q": [round(v, 5) for v in q_hub],
                            "arm_p": [round(v, 4) for v in ac[:3]], "arm_q": [round(v, 5) for v in q_ac]}
        except (OSError, ConnectionError):
            with LOCK:
                STATE["coil_connected"] = False
            time.sleep(1.0)


PAGE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>Glove hands v3 (Coil world)</title>
<style>
 html,body{margin:0;height:100%;background:#0d1014;color:#d8dde3;font:14px system-ui,sans-serif;overflow:hidden}
 canvas{display:block;width:100vw;height:100vh;cursor:grab}
 #hud{position:fixed;top:12px;left:14px;line-height:1.6;text-shadow:0 1px 2px #000}
 #hud b{color:#fff} .dim{color:#8a94a0}
 button{background:#2b6cb0;color:#fff;border:0;border-radius:6px;padding:6px 11px;font-size:13px;cursor:pointer;margin:6px 6px 0 0}
 button:hover{background:#3182ce}
</style></head><body>
<canvas id="c"></canvas>
<div id="hud">
 <div><b>Glove hands v3 · Coil world</b> <span id="conn" class="dim">connecting…</span></div>
 <div id="rates" class="dim"></div>
 <div id="coil" class="dim"></div>
 <button id="bh">Focus hands (H)</button><button id="bo">Overview (O)</button><button id="bm">Colors (M)</button>
 <div class="dim">drag = orbit · right-drag / shift-drag = pan · scroll = zoom · C = calibrate forward (only without Coil)</div>
</div>
<script>
const cv = document.getElementById('c'), ctx = cv.getContext('2d');
const FINGERS = ['thumb','index','middle','ring','little'];
const COLORS = {thumb:'#f6ad55', index:'#68d391', middle:'#63b3ed', ring:'#b794f4', little:'#fc8181'};
const GREY = '#aab3bf', GREY_D = '#8e98a6';
let data = {hands:{}, connected:false, coil:{}}, mono = true, yaw = {}, needCal = true;
let cam = {theta:0.9, phi:0.45, dist:2.6, target:[0,0,0]}, framed = false;

function resize(){ cv.width = innerWidth*devicePixelRatio; cv.height = innerHeight*devicePixelRatio; }
addEventListener('resize', resize); resize();

// ---------- math ----------
const add=(a,b)=>[a[0]+b[0],a[1]+b[1],a[2]+b[2]], sub=(a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const mul=(a,k)=>[a[0]*k,a[1]*k,a[2]*k], dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const norm=a=>{const n=Math.hypot(a[0],a[1],a[2])||1; return [a[0]/n,a[1]/n,a[2]/n];};
function qrot(q,v){ // q = [x,y,z,w]
  const [x,y,z,w]=q, t=mul(cross([x,y,z],v),2);
  return add(add(v, mul(t,w)), cross([x,y,z],t));
}
// Coil Pro frame (+z points down) -> view frame (Y up)
const C2V = c => [c[0], -c[2], c[1]];

// ---------- camera ----------
let drag = null;
cv.addEventListener('contextmenu', e => e.preventDefault());
cv.addEventListener('mousedown', e => drag = {x:e.clientX, y:e.clientY, t:cam.theta, p:cam.phi, tg:cam.target.slice(),
  pan: e.button===2 || e.shiftKey});
addEventListener('mouseup', () => drag = null);
addEventListener('mousemove', e => { if(!drag) return;
  const dx=e.clientX-drag.x, dy=e.clientY-drag.y;
  if (drag.pan){ const B = basis(); const k = cam.dist*0.0015;
    cam.target = add(add(drag.tg, mul(B.r, -dx*k)), mul(B.u, dy*k)); }
  else { cam.theta = drag.t - dx*0.008; cam.phi = Math.max(-1.45, Math.min(1.45, drag.p + dy*0.008)); }});
cv.addEventListener('wheel', e => { cam.dist = Math.max(0.2, Math.min(8, cam.dist*Math.exp(e.deltaY*0.001))); e.preventDefault(); }, {passive:false});
addEventListener('keydown', e => { const k=e.key.toLowerCase();
  if (k==='h') focusHands(); if (k==='o') overview(); if (k==='m') mono=!mono; if (k==='c') needCal = true; });
document.getElementById('bh').onclick = () => focusHands();
document.getElementById('bo').onclick = () => overview();
document.getElementById('bm').onclick = () => mono = !mono;

function basis(){
  const ct=Math.cos(cam.theta), st=Math.sin(cam.theta), cp=Math.cos(cam.phi), sp=Math.sin(cam.phi);
  const off=[cam.dist*cp*st, cam.dist*sp, cam.dist*cp*ct], eye=add(cam.target, off), f=norm(mul(off,-1));
  const r=norm([-f[2],0,f[0]]), u=cross(r,f);
  return {eye,f,r,u};
}
let B = basis();
function project(p){
  const d = sub(p, B.eye), z = dot(d,B.f), F = Math.min(cv.width, cv.height)*1.1;
  return {x: cv.width/2 + dot(d,B.r)/z*F, y: cv.height/2 - dot(d,B.u)/z*F, z, s: F/z};
}

// ---------- hands ----------
const WRIST_FROM_HUB = [0, 0.06, 0.02];    // glove frame: +Y toward wrist, +Z palm side (m)
const FOREARM = 0.25;                      // arm sensor -> elbow along +Y of the arm sensor (m)
function idx(h, n){ return h.names ? h.names.indexOf(n) : -1; }
function handWorld(side){
  const c = (data.coil||{})[side]; if (!c || !c.hub_q) return null;
  const q = c.hub_q;
  const gx = C2V(qrot(q,[1,0,0])), gy = C2V(qrot(q,[0,1,0])), gz = C2V(qrot(q,[0,0,1]));
  const wrist = C2V(add(c.hub_p, qrot(q, WRIST_FROM_HUB)));
  const elbow = C2V(add(c.arm_p, qrot(c.arm_q, [0, FOREARM, 0.02])));
  // anatomical target frame: u = toward fingers, v = toward the index side, w = u x v
  const u = mul(gy,-1), v = side==='right' ? mul(gx,-1) : gx, w = cross(u, v);
  return {wrist, elbow, u, v, w, ok: c.ok, palmDown: mul(gz,1)};
}
function placeJoints(side, h, W){
  const iw = idx(h,'wrist'), im = idx(h,'middle_proximal'), ii = idx(h,'index_proximal'), il = idx(h,'little_proximal');
  if (iw<0||im<0||ii<0||il<0) return null;
  const P = h.pos, base = P[iw];
  const us = norm(sub(P[im], base)); let lat = sub(P[ii], P[il]);
  const vs = norm(sub(lat, mul(us, dot(lat,us)))), ws = cross(us, vs);
  return P.map(p => { const d = sub(p, base);
    return add(W.wrist, add(add(mul(W.u, dot(d,us)), mul(W.v, dot(d,vs))), mul(W.w, dot(d,ws)))); });
}
// fallback without Coil: fixed spots, yaw-calibrated (old behaviour)
const SLOT = {left:[-0.16,0,0], right:[0.16,0,0]};
function calibrateYaw(){
  for (const [side,h] of Object.entries(data.hands)){ const i = idx(h,'middle_proximal'); if (i<0) continue;
    const f = h.pos[i]; yaw[side] = Math.atan2(f[0], -f[2]); }
  needCal = false;
}
function placeFixed(side, p){
  const a = yaw[side]||0, c = Math.cos(a), s = Math.sin(a), o = SLOT[side]||[0,0,0];
  return [c*p[0] + s*p[2] + o[0], p[1] + o[1], -s*p[0] + c*p[2] + o[2]];
}
function bonesOf(names){
  const b = [], ix = n => names.indexOf(n);
  for (const f of FINGERS){
    const ch = ['wrist', f+'_metacarpal', f+'_proximal', f+'_intermediate', f+'_distal', f+'_tip'].map(ix).filter(i=>i>=0);
    for (let k=0;k<ch.length-1;k++) b.push([ch[k], ch[k+1], f]);
  }
  const pr = ['index_proximal','middle_proximal','ring_proximal','little_proximal'].map(ix).filter(i=>i>=0);
  for (let k=0;k<pr.length-1;k++) b.push([pr[k], pr[k+1], 'palm']);     // knuckle line
  return b;
}
function coilMode(){ return Object.values(data.coil||{}).some(c => c.hub_q); }
function focusHands(){
  const ws = ['left','right'].map(handWorld).filter(Boolean).map(w => w.wrist);
  if (ws.length){ cam.target = mul(ws.reduce(add,[0,0,0]), 1/ws.length); cam.dist = 1.1; }
}
function overview(){
  const ws = ['left','right'].map(handWorld).filter(Boolean).map(w => w.wrist);
  const pts = [[0,0,0], ...ws]; cam.target = mul(pts.reduce(add,[0,0,0]), 1/pts.length);
  const span = Math.max(0.6, ...pts.map(p => Math.hypot(...sub(p, cam.target))));
  cam.dist = span*3.2;
}

// ---------- drawing ----------
function cubeFaces(){
  const hx=0.105, hy=0.105, hz=0.115;   // Coil Pro is about 21 x 21 x 23 cm
  const V = []; for (const x of [-hx,hx]) for (const y of [-hy,hy]) for (const z of [-hz,hz]) V.push(C2V([x,y,z]));
  const F = [[0,1,3,2],[4,5,7,6],[0,1,5,4],[2,3,7,6],[0,2,6,4],[1,3,7,5]];
  return F.map(f => f.map(i => V[i]));
}
function draw(){
  B = basis();
  ctx.fillStyle = '#0d1014'; ctx.fillRect(0,0,cv.width,cv.height);
  const items = [];
  const world = coilMode();
  const floorY = world ? -0.75 : -0.12;    // Coil Pro sits on a stand about 0.75 m above the floor
  // grid
  const ext = world ? 3 : 0.5, step = world ? 0.1 : 0.05;
  for (let g=-ext; g<=ext+1e-6; g+=step){
    const major = Math.abs(Math.round(g/(step*5))*(step*5) - g) < 1e-6;
    const cx = world ? cam.target[0] : 0, cz = world ? cam.target[2] : 0;
    const gx = Math.round(cx/step)*step, gz = Math.round(cz/step)*step;
    for (const [a,b] of [[[gx+g,floorY,gz-ext],[gx+g,floorY,gz+ext]], [[gx-ext,floorY,gz+g],[gx+ext,floorY,gz+g]]]){
      const A=project(a), Bp=project(b); if (A.z<0.05||Bp.z<0.05) continue;
      ctx.strokeStyle = major ? '#2c3540' : '#1a2028'; ctx.lineWidth = devicePixelRatio;
      ctx.beginPath(); ctx.moveTo(A.x,A.y); ctx.lineTo(Bp.x,Bp.y); ctx.stroke();
    }
  }
  if (world){
    for (const f of cubeFaces()){ const P = f.map(project); if (P.some(p=>p.z<0.05)) continue;
      items.push({z: P.reduce((s,p)=>s+p.z,0)/4, face:P}); }
    // stand under the Coil
    { const A = project([0,-0.115,0]), Bp = project([0,floorY,0]);
      if (A.z>0.05 && Bp.z>0.05) items.push({z:(A.z+Bp.z)/2+0.05, bone:[A,Bp], c:'#2a3038', w:0.025}); }
    // ring on top + axes
    const ring = []; for (let k=0;k<=24;k++){ const a=k/24*2*Math.PI; ring.push(project(C2V([0.035*Math.cos(a),0.035*Math.sin(a),-0.116]))); }
    items.push({z: project(C2V([0,0,-0.116])).z - 0.01, poly: ring, c:'#9ae6b4'});
    for (const [ax,col] of [[[0.18,0,0],'#f56565'],[[0,0.18,0],'#48bb78'],[[0,0,-0.18],'#4299e1']]){
      const A=project(C2V([0,0,0])), Bp=project(C2V(ax)); items.push({z:Bp.z-0.02, line:[A,Bp], c:col});
    }
  }
  for (const side of ['left','right']){
    const h = data.hands[side];
    const W = world ? handWorld(side) : null;
    let J = null;
    if (W){
      const E = project(W.elbow), Wr = project(W.wrist);
      const fc = W.ok ? GREY_D : '#5a6370';
      items.push({z:(E.z+Wr.z)/2+0.02, bone:[E,Wr], c:fc, w:0.05});
      items.push({z:E.z, joint:E, r:0.03, c:fc});
      const L = project(add(W.elbow,[0,0.08,0])); items.push({z:-1, label:[L, side + (W.ok ? '' : ' (no Coil signal)')]});
      if (h && h.pos) J = placeJoints(side, h, W);
    } else if (h && h.pos){
      if (needCal) calibrateYaw();
      J = h.pos.map(p => placeFixed(side, p));
      const L = project(placeFixed(side,[0,0.09,0.03])); items.push({z:-1, label:[L, side]});
    }
    if (J){
      const P = J.map(project);
      for (const [i,j,f] of bonesOf(h.names))
        items.push({z:(P[i].z+P[j].z)/2, bone:[P[i],P[j]], c: mono ? GREY : (COLORS[f]||'#cbd5e0'), w: f==='palm'?0.012:0.011});
      h.pos.forEach((_,i) => { const nm = h.names[i];
        items.push({z:P[i].z, joint:P[i], r: nm==='wrist'?0.012:0.0075,
          c: mono ? '#c3cad4' : (COLORS[FINGERS.find(f=>nm.startsWith(f))] || '#e2e8f0')}); });
    }
  }
  items.sort((a,b) => b.z - a.z);
  for (const it of items){
    if (it.face){ const P=it.face; ctx.beginPath(); ctx.moveTo(P[0].x,P[0].y); for (const p of P.slice(1)) ctx.lineTo(p.x,p.y);
      ctx.closePath(); ctx.fillStyle='#16191e'; ctx.fill(); ctx.strokeStyle='#3b4652'; ctx.lineWidth=devicePixelRatio; ctx.stroke(); }
    else if (it.poly){ const P=it.poly; if (P.some(p=>p.z<0.05)) continue; ctx.beginPath(); ctx.moveTo(P[0].x,P[0].y);
      for (const p of P.slice(1)) ctx.lineTo(p.x,p.y); ctx.strokeStyle=it.c; ctx.lineWidth=2*devicePixelRatio; ctx.stroke(); }
    else if (it.line){ const [A,Bp]=it.line; if (A.z<0.05||Bp.z<0.05) continue; ctx.strokeStyle=it.c; ctx.lineWidth=2*devicePixelRatio;
      ctx.beginPath(); ctx.moveTo(A.x,A.y); ctx.lineTo(Bp.x,Bp.y); ctx.stroke(); }
    else if (it.bone){ const [A,Bp]=it.bone; if (A.z<0.05||Bp.z<0.05) continue;
      ctx.strokeStyle=it.c; ctx.lineWidth=Math.max(2, it.w*(A.s+Bp.s)/2); ctx.lineCap='round';
      ctx.beginPath(); ctx.moveTo(A.x,A.y); ctx.lineTo(Bp.x,Bp.y); ctx.stroke(); }
    else if (it.joint){ const J=it.joint; if (J.z<0.05) continue;
      ctx.fillStyle=it.c; ctx.beginPath(); ctx.arc(J.x,J.y,Math.max(2,it.r*J.s),0,7); ctx.fill(); }
  }
  for (const it of items) if (it.label){ const [L,t]=it.label; if (L.z<0.05) continue;
    ctx.fillStyle='#8a94a0'; ctx.font=`${12*devicePixelRatio}px system-ui`; ctx.textAlign='center'; ctx.fillText(t.toUpperCase(), L.x, L.y); }
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);

// ---------- data ----------
const es = new EventSource('/stream');
let counts = {}, lastRate = performance.now();
es.onmessage = ev => {
  data = JSON.parse(ev.data);
  if (!framed && coilMode() && ['left','right'].some(s => (data.coil[s]||{}).hub_q)){ overview(); framed = true; }
  const now = performance.now();
  if (now - lastRate > 1000){
    const parts = Object.entries(data.hands).map(([s,h]) => {
      const r = ((h.n - (counts[s]||h.n)) / ((now-lastRate)/1000)).toFixed(0); counts[s]=h.n; return `${s}: ${r} Hz`; });
    document.getElementById('rates').textContent = parts.join(' · ') || 'no hands yet - is rkk-hand-solver running?';
    lastRate = now;
  }
  const cs = ['left','right'].map(sd => { const c=(data.coil||{})[sd];
    return `${sd}: ${c ? (c.ok ? 'tracking' : 'no signal (holding last pose)') : 'waiting'}`; });
  document.getElementById('coil').textContent = coilMode() ? `Coil Pro world · ${cs.join(' · ')}`
    : 'no Coil Pro data - hands shown at fixed spots (press C with hands flat, fingers forward)';
  const c = document.getElementById('conn');
  c.textContent = data.connected ? 'connected to solver' : 'waiting for rkk-hand-solver…';
  c.style.color = data.connected ? '#68d391' : '#f6ad55';
};
es.onerror = () => { const c=document.getElementById('conn'); c.textContent='viewer server stopped?'; c.style.color='#fc8181'; };
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    with LOCK:
                        now = time.time()
                        hands = {k: {kk: v[kk] for kk in ("names", "radii", "pos", "n")}
                                 for k, v in STATE["hands"].items() if now - v.get("t", 0) < 1.0 and "pos" in v}
                        coil = {k: {kk: v[kk] for kk in ("ok", "hub_p", "hub_q", "arm_p", "arm_q") if kk in v}
                                for k, v in STATE["coil"].items() if now - v["t"] < 2.0 and "hub_p" in v}
                        msg = json.dumps({"connected": STATE["connected"], "hands": hands, "coil": coil},
                                         separators=(",", ":"))
                    self.wfile.write(f"data: {msg}\n\n".encode())
                    self.wfile.flush()
                    time.sleep(1 / 60)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                return
        else:
            self.send_error(404)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1", help="hand solver host (default: this laptop)")
    ap.add_argument("--port", type=int, default=12277, help="hand solver port")
    ap.add_argument("--http", type=int, default=8765, help="web page port")
    ap.add_argument("--coil-port", type=int, default=12276,
                    help="driver / replay_csv.py port for Coil Pro arm poses (0 = off, hands stay fixed)")
    a = ap.parse_args()
    threading.Thread(target=rgmp_loop, args=(a.host, a.port), daemon=True).start()
    if a.coil_port:
        threading.Thread(target=coil_loop, args=(a.host, a.coil_port), daemon=True).start()
    class Server(ThreadingHTTPServer):
        # Windows: exclusive bind, so an old viewer can't keep answering on the same port.
        # Elsewhere: normal reuse, so a restart right after Ctrl+C works.
        allow_reuse_address = os.name != "nt"

        def server_bind(self):
            if os.name == "nt" and hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            super().server_bind()
    try:
        srv = Server(("127.0.0.1", a.http), Handler)
    except OSError:
        print(f"[viewer] port {a.http} is busy - an older hand_viewer is probably still running. "
              f"Close it, or run: python hand_viewer.py --http {a.http + 1}", flush=True)
        return
    srv.daemon_threads = True
    print(f"[viewer] open http://localhost:{a.http}  (Ctrl+C to stop)", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
