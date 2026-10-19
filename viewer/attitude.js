// Real-time 3D attitude view of the 3U CubeSat (three.js).
//
// The spacecraft sits at the origin. Two display frames:
//   LVLH      X = along-track, Y = zenith (Earth below), Z = -orbit normal
//   Inertial  ECI axes (Z up), stars fixed
// Earth is drawn with its true angular size as seen from the satellite, and
// is rotated by GMST so the continents under the spacecraft are correct.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const EARTH_TEX = 'https://cdn.jsdelivr.net/npm/three-globe@2.31.0/example/img/earth-blue-marble.jpg';
const R_E = 6378.1363;
const S = 6;          // display units per metre of spacecraft
const D_EARTH = 900;  // display distance to Earth's centre

// ---- 3x3 helpers (row-major arrays)
const mul = (A, B) => A.map((row, i) => [0, 1, 2].map(j => row[0] * B[0][j] + row[1] * B[1][j] + row[2] * B[2][j]));
const mulv = (A, v) => [A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2], A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2], A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2]];
const unit = v => { const n = Math.hypot(v[0], v[1], v[2]) || 1; return [v[0] / n, v[1] / n, v[2] / n]; };
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
function quatToMat([w, x, y, z]) {
  return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
          [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
          [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]];
}
function toM4(M, pos = [0, 0, 0], s = 1) {
  return new THREE.Matrix4().set(
    M[0][0] * s, M[0][1] * s, M[0][2] * s, pos[0],
    M[1][0] * s, M[1][1] * s, M[1][2] * s, pos[1],
    M[2][0] * s, M[2][1] * s, M[2][2] * s, pos[2],
    0, 0, 0, 1);
}
const V3 = a => new THREE.Vector3(a[0], a[1], a[2]);
const ECI_TO_THREE = [[1, 0, 0], [0, 0, 1], [0, -1, 0]];
const ECEF_TO_TEX_T = [[1, 0, 0], [0, 0, -1], [0, 1, 0]];  // three sphere local -> ECEF

function cellTexture(front) {
  const c = document.createElement('canvas');
  c.width = 128; c.height = 512;
  const g = c.getContext('2d');
  g.fillStyle = front ? '#1b2a4a' : '#14203a';
  g.fillRect(0, 0, 128, 512);
  const n = 7, pad = 6, h = (512 - pad * (n + 1)) / n;
  for (let i = 0; i < n; i++) {
    const y = pad + i * (h + pad);
    const grd = g.createLinearGradient(0, y, 128, y + h);
    grd.addColorStop(0, '#1d3f8f'); grd.addColorStop(1, '#0f2459');
    g.fillStyle = grd;
    g.fillRect(pad, y, 128 - 2 * pad, h);
    g.strokeStyle = 'rgba(200,210,230,.55)'; g.lineWidth = 1;
    for (let k = 1; k < 6; k++) { g.beginPath(); g.moveTo(pad, y + k * h / 6); g.lineTo(128 - pad, y + k * h / 6); g.stroke(); }
    g.beginPath(); g.moveTo(64, y); g.lineTo(64, y + h); g.stroke();
  }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 4;
  return t;
}

function labelSprite(text, color) {
  const c = document.createElement('canvas');
  c.width = 256; c.height = 64;
  const g = c.getContext('2d');
  g.font = 'bold 34px monospace';
  g.fillStyle = color; g.textAlign = 'center'; g.textBaseline = 'middle';
  g.shadowColor = '#000'; g.shadowBlur = 6;
  g.fillText(text, 128, 32);
  const m = new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthTest: false, transparent: true });
  const s = new THREE.Sprite(m);
  s.scale.set(1.6, 0.4, 1);
  s.renderOrder = 10;
  return s;
}

function glowTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 128;
  const g = c.getContext('2d');
  const grd = g.createRadialGradient(64, 64, 0, 64, 64, 64);
  grd.addColorStop(0, 'rgba(255,255,240,1)');
  grd.addColorStop(0.15, 'rgba(255,240,180,.9)');
  grd.addColorStop(0.4, 'rgba(255,200,80,.25)');
  grd.addColorStop(1, 'rgba(255,160,0,0)');
  g.fillStyle = grd; g.fillRect(0, 0, 128, 128);
  return new THREE.CanvasTexture(c);
}

export class AttitudeView {
  constructor(el) {
    this.el = el;
    this.frame = 'lvlh';
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(2, window.devicePixelRatio));
    el.appendChild(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x000000);
    this.camera = new THREE.PerspectiveCamera(45, 1, 0.05, 30000);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.minDistance = 1.2;
    this.controls.maxDistance = 400;
    this.resetCamera();

    this.scene.add(new THREE.AmbientLight(0xffffff, 0.12));
    this.sunLight = new THREE.DirectionalLight(0xffffff, 2.6);
    this.scene.add(this.sunLight, this.sunLight.target);
    this.earthShine = new THREE.DirectionalLight(0x88aaff, 0.4);
    this.scene.add(this.earthShine, this.earthShine.target);

    // Stars (fixed in ECI)
    const sg = new THREE.BufferGeometry(), pts = [];
    let seed = 7;
    const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
    for (let i = 0; i < 2500; i++) {
      const u = rnd() * 2 - 1, th = rnd() * Math.PI * 2, s = Math.sqrt(1 - u * u);
      pts.push(20000 * s * Math.cos(th), 20000 * s * Math.sin(th), 20000 * u);
    }
    sg.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
    this.stars = new THREE.Points(sg, new THREE.PointsMaterial({ color: 0xffffff, size: 1.6, sizeAttenuation: false }));
    this.stars.matrixAutoUpdate = false;
    this.scene.add(this.stars);

    // Earth
    const tex = new THREE.TextureLoader().setCrossOrigin('anonymous').load(EARTH_TEX);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.anisotropy = 8;
    this.earth = new THREE.Mesh(new THREE.SphereGeometry(1, 128, 96),
      new THREE.MeshPhongMaterial({ map: tex, color: 0xffffff, shininess: 6, specular: 0x222233 }));
    this.earth.matrixAutoUpdate = false;
    this.scene.add(this.earth);
    this.atmo = new THREE.Mesh(new THREE.SphereGeometry(1.015, 96, 64),
      new THREE.MeshBasicMaterial({ color: 0x4d8cff, transparent: true, opacity: 0.12, side: THREE.BackSide, blending: THREE.AdditiveBlending, depthWrite: false }));
    this.atmo.matrixAutoUpdate = false;
    this.scene.add(this.atmo);

    // Sun disc
    this.sun = new THREE.Sprite(new THREE.SpriteMaterial({ map: glowTexture(), transparent: true, depthWrite: false, blending: THREE.AdditiveBlending }));
    this.sun.scale.set(2600, 2600, 1);
    this.scene.add(this.sun);

    // Direction arrows (world frame, from the spacecraft)
    const mk = (color, label) => {
      const a = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 3.2, color, 0.35, 0.18);
      a.line.material.linewidth = 2;
      const l = labelSprite(label, '#' + new THREE.Color(color).getHexString());
      this.scene.add(a, l);
      return { a, l };
    };
    this.arrows = {
      sun: mk(0xffd400, 'SUN'),
      nadir: mk(0x00e5ff, 'NADIR'),
      vel: mk(0x33ff66, 'VEL'),
      gs: mk(0xff00ff, 'GS'),
    };

    this.sat = new THREE.Group();
    this.sat.matrixAutoUpdate = false;
    this.scene.add(this.sat);

    new ResizeObserver(() => this.resize()).observe(el);
    this.resize();
  }

  resetCamera() {
    this.camera.position.set(-5.5, 3.2, 7.5);
    this.controls.target.set(0, 0, 0);
    this.controls.update();
  }

  setFrame(f) { this.frame = f; }

  resize() {
    const w = this.el.clientWidth || 600, h = this.el.clientHeight || 450;
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = '100%';
    this.renderer.domElement.style.height = '100%';
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  build(meta) {
    this.meta = meta;
    while (this.sat.children.length) this.sat.remove(this.sat.children[0]);
    const P = meta.plates;
    const body = new THREE.Group();
    // Structure: box from the six body plates
    const w = 2 * P[0].hu, l = 2 * P[0].hv;  // +X plate: hu along Y (width), hv along Z (length)
    const box = new THREE.Mesh(new THREE.BoxGeometry(w * S, w * S, l * S),
      new THREE.MeshStandardMaterial({ color: 0x9a9da3, metalness: 0.75, roughness: 0.38 }));
    body.add(box);
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(box.geometry), new THREE.LineBasicMaterial({ color: 0x222222 }));
    body.add(edges);
    // Rails
    const railMat = new THREE.MeshStandardMaterial({ color: 0x202020, metalness: 0.4, roughness: 0.6 });
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) {
      const r = new THREE.Mesh(new THREE.BoxGeometry(0.0085 * S, 0.0085 * S, l * S * 1.02), railMat);
      r.position.set(sx * (w / 2) * S, sy * (w / 2) * S, 0);
      body.add(r);
    }
    // Imager aperture on the imager axis face, antenna patch on the antenna face
    const faceCenter = ax => [ax[0] * w / 2 * S, ax[1] * w / 2 * S, ax[2] * l / 2 * S].map((v, i) => v + ax[i] * 0.002 * S);
    const orient = (obj, ax) => obj.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), V3(ax));
    const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.03 * S, 0.034 * S, 0.012 * S, 40),
      new THREE.MeshStandardMaterial({ color: 0x050505, metalness: 0.2, roughness: 0.15 }));
    orient(lens, meta.axes.imager);
    lens.position.set(...faceCenter(meta.axes.imager));
    body.add(lens);
    const glass = new THREE.Mesh(new THREE.CircleGeometry(0.024 * S, 40), new THREE.MeshStandardMaterial({ color: 0x3355aa, metalness: 1, roughness: 0.05 }));
    glass.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), V3(meta.axes.imager));
    glass.position.set(...faceCenter(meta.axes.imager).map((v, i) => v + meta.axes.imager[i] * 0.0065 * S));
    body.add(glass);
    const patch = new THREE.Mesh(new THREE.BoxGeometry(0.036 * S, 0.004 * S, 0.036 * S),
      new THREE.MeshStandardMaterial({ color: 0xc9a227, metalness: 0.9, roughness: 0.3 }));
    orient(patch, meta.axes.antenna);
    const pc = faceCenter(meta.axes.antenna);
    const off = Math.abs(meta.axes.antenna[2]) > 0.5 ? [0.028 * S, 0.028 * S, 0] : [0, 0, 0.1 * S];
    patch.position.set(pc[0] + off[0], pc[1] + off[1], pc[2] + off[2]);
    body.add(patch);
    this.sat.add(body);

    // Deployable panels (plates 6..9), front and back faces. Two copies share the
    // materials: deployed pose and stowed pose (flat on the body, before deployment).
    this.panelMats = [];
    const frontTex = cellTexture(true), backTex = cellTexture(false);
    const hasBack = meta.faces.some(f => f.name.includes('back'));
    for (let p = 6; p < P.length; p++) {
      const mFront = new THREE.MeshStandardMaterial({ map: frontTex, metalness: 0.35, roughness: 0.35, emissive: 0x000000, side: THREE.FrontSide });
      const mBack = new THREE.MeshStandardMaterial(hasBack ? { map: backTex, metalness: 0.35, roughness: 0.4, emissive: 0x000000 }
                                                            : { color: 0x3a3a38, metalness: 0.2, roughness: 0.8 });
      this.panelMats.push({ front: mFront, back: hasBack ? mBack : null, mFront, mBack });
    }
    const makePanels = (plates) => {
      const grp = new THREE.Group();
      plates.forEach((pl, idx) => {
        const src = P[6 + idx];
        const g = new THREE.PlaneGeometry(2 * src.hv * S, 2 * src.hu * S);  // x along v (width), y along u (length)
        const { mFront, mBack } = this.panelMats[idx];
        const n = pl.n, u = pl.u, v = pl.v;
        const front = new THREE.Mesh(g, mFront);
        front.matrixAutoUpdate = false;
        front.matrix.makeBasis(V3(v).negate(), V3(u), V3(n)).setPosition(V3(pl.c).multiplyScalar(S).add(V3(n).multiplyScalar(0.0025 * S)));
        const back = new THREE.Mesh(g, mBack);
        back.matrixAutoUpdate = false;
        back.matrix.makeBasis(V3(v), V3(u), V3(n).negate()).setPosition(V3(pl.c).multiplyScalar(S).add(V3(n).multiplyScalar(-0.0025 * S)));
        const frame = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(2 * src.hv * S, 2 * src.hu * S, 0.004 * S)),
          new THREE.LineBasicMaterial({ color: 0x777777 }));
        frame.matrixAutoUpdate = false;
        frame.matrix.makeBasis(V3(v).negate(), V3(u), V3(n)).setPosition(V3(pl.c).multiplyScalar(S));
        grp.add(front, back, frame);
      });
      this.sat.add(grp);
      return grp;
    };
    this.deployedPanels = makePanels(P.slice(6));
    this.stowedPanels = meta.stowed_plates ? makePanels(meta.stowed_plates) : null;
    if (this.stowedPanels) this.stowedPanels.visible = false;

    // Magnetorquer dipole (shown while it is non-zero)
    this.mtqArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1, 0xff66cc, 0.2, 0.1);
    this.mtqArrow.visible = false;
    this.sat.add(this.mtqArrow);

    // Body axes triad
    const ax = (d, c) => new THREE.ArrowHelper(V3(d), new THREE.Vector3(), 0.12 * S, c, 0.03 * S, 0.018 * S);
    this.sat.add(ax([1, 0, 0], 0xff4444), ax([0, 1, 0], 0x44ff44), ax([0, 0, 1], 0x4488ff));

    // Imager field of view and antenna beam
    const cone = (halfDeg, len, color, axis, faceOff) => {
      const r = len * Math.tan(halfDeg * Math.PI / 180);
      const g = new THREE.ConeGeometry(r, len, 48, 1, true);
      g.translate(0, -len / 2, 0);  // apex at origin, opening toward -Y
      const m = new THREE.Mesh(g, new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.16, side: THREE.DoubleSide, depthWrite: false, blending: THREE.AdditiveBlending }));
      m.quaternion.setFromUnitVectors(new THREE.Vector3(0, -1, 0), V3(axis));
      m.position.copy(V3(axis).multiplyScalar(faceOff));
      m.visible = false;
      this.sat.add(m);
      return m;
    };
    const halfFov = Math.atan((meta.swath_km || 120) / 2 / (meta.altitude_km || 500)) * 180 / Math.PI;
    this.fovCone = cone(halfFov, 60, 0x00ff66, meta.axes.imager, l / 2 * S);
    this.beamCone = cone(35, 4.5, 0xff00ff, meta.axes.antenna, l / 2 * S);
    this.boresight = new THREE.ArrowHelper(V3(meta.axes.imager), V3(meta.axes.imager).multiplyScalar(l / 2 * S), 2.4, 0xff5555, 0.25, 0.12);
    this.sat.add(this.boresight);
  }

  // s: interpolated sample from app.js
  update(s) {
    if (!this.meta) return;
    const rh = unit(s.r);
    const vh = unit(s.v);
    const hh = unit(cross(s.r, s.v));
    let W;
    if (this.frame === 'lvlh') {
      const along = unit(cross(hh, rh));
      W = [along, rh, hh.map(x => -x)];
    } else W = ECI_TO_THREE;

    const Rq = quatToMat(s.q);
    this.sat.matrix.copy(toM4(mul(W, Rq)));
    this.sat.matrixWorldNeedsUpdate = true;

    // Earth: true angular size, rotated by GMST
    const rn = Math.hypot(...s.r);
    const Rv = D_EARTH * R_E / rn;
    const ec = mulv(W, rh.map(x => -x * D_EARTH));
    const cg = Math.cos(s.gmst), sgm = Math.sin(s.gmst);
    const Rz = [[cg, -sgm, 0], [sgm, cg, 0], [0, 0, 1]];
    const Me = mul(mul(W, Rz), ECEF_TO_TEX_T);
    this.earth.matrix.copy(toM4(Me, ec, Rv));
    this.atmo.matrix.copy(toM4(Me, ec, Rv));
    this.stars.matrix.copy(toM4(W));

    const sunW = mulv(W, s.sun);
    this.sunLight.position.copy(V3(sunW).multiplyScalar(1000));
    this.sunLight.intensity = 2.6 * s.sunfrac;
    this.sun.position.copy(V3(sunW).multiplyScalar(14000));
    this.sun.visible = s.sunfrac > 0.02;
    const nadW = mulv(W, rh.map(x => -x));
    this.earthShine.position.copy(V3(nadW).multiplyScalar(1000));
    this.earthShine.intensity = 0.1 + 0.5 * Math.max(0, dot(rh, s.sun));

    const place = (o, dir, len, show = true) => {
      o.a.visible = o.l.visible = show;
      if (!show) return;
      const d = V3(dir).normalize();
      o.a.setDirection(d);
      o.a.setLength(len, 0.35, 0.18);
      o.l.position.copy(d.multiplyScalar(len + 0.45));
    };
    place(this.arrows.sun, sunW, 3.4);
    place(this.arrows.nadir, nadW, 3.0);
    place(this.arrows.vel, mulv(W, vh), 2.6);
    place(this.arrows.gs, mulv(W, s.gsDir), 3.0, s.gsEl > 0);

    // Panel glow proportional to each face's power
    const faces = this.meta.faces;
    for (let f = 0; f < faces.length; f++) {
      const pnl = faces[f].panel;
      if (pnl < 0) continue;
      const m = faces[f].name.includes('back') ? this.panelMats[pnl].back : this.panelMats[pnl].front;
      if (!m) continue;
      const frac = Math.min(1, (s.faceP[f] || 0) / (s.facePmax || 1));
      m.emissive.setRGB(0.02 * frac, 0.12 * frac, 0.38 * frac);
    }
    this.fovCone.visible = s.imager === 2;
    if (this.stowedPanels) {
      this.stowedPanels.visible = s.deployed === false;
      this.deployedPanels.visible = s.deployed !== false;
    }
    const mm = s.mtq ? Math.hypot(...s.mtq) : 0;
    this.mtqArrow.visible = mm > 1e-4;
    if (mm > 1e-4) {
      this.mtqArrow.setDirection(V3(s.mtq).normalize());
      this.mtqArrow.setLength(0.6 + 6 * mm, 0.2, 0.1);  // 0.3 A m^2 -> ~2.4 units
    }
    this.beamCone.visible = s.mode === 2 && s.gsEl > 0;
  }

  render() {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
