// 3D orbit view (three.js). Units: Earth radii. ECI is drawn with Z up.
// All orbit geometry is uploaded once per simulation run; each frame only moves
// the satellite, rotates the Earth by GMST and changes the trail's draw range.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const R_E = 6378.1363;
const EARTH_TEX = 'https://cdn.jsdelivr.net/npm/three-globe@2.31.0/example/img/earth-blue-marble.jpg';
const M = [[1, 0, 0], [0, 0, 1], [0, -1, 0]];           // ECI -> three (Z up becomes Y up)
const P_T = [[1, 0, 0], [0, 0, -1], [0, 1, 0]];         // three sphere local -> ECEF
const toThree = (v, s = 1) => new THREE.Vector3(v[0] * s, v[2] * s, -v[1] * s);
const mul = (A, B) => A.map(row => [0, 1, 2].map(j => row[0] * B[0][j] + row[1] * B[1][j] + row[2] * B[2][j]));

function label(text, color) {
  const c = document.createElement('canvas');
  c.width = 256; c.height = 64;
  const g = c.getContext('2d');
  g.font = 'bold 30px monospace'; g.fillStyle = color; g.textAlign = 'center'; g.textBaseline = 'middle';
  g.shadowColor = '#000'; g.shadowBlur = 6;
  g.fillText(text, 128, 32);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthTest: false, transparent: true }));
  s.scale.set(0.5, 0.125, 1);
  s.renderOrder = 10;
  return s;
}
function glow(color) {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d');
  const grd = g.createRadialGradient(32, 32, 0, 32, 32, 32);
  grd.addColorStop(0, '#fff'); grd.addColorStop(0.25, color); grd.addColorStop(1, 'rgba(0,0,0,0)');
  g.fillStyle = grd; g.fillRect(0, 0, 64, 64);
  return new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthTest: false, transparent: true, blending: THREE.AdditiveBlending }));
}

export class OrbitView {
  constructor(el) {
    this.el = el;
    this.follow = false;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(2, window.devicePixelRatio));
    el.appendChild(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x000000);
    this.camera = new THREE.PerspectiveCamera(40, 1, 0.005, 500);
    this.camera.position.set(3.2, 1.9, 3.2);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.minDistance = 1.15;
    this.controls.maxDistance = 30;

    this.scene.add(new THREE.AmbientLight(0xffffff, 0.07));
    this.sunLight = new THREE.DirectionalLight(0xffffff, 2.4);
    this.scene.add(this.sunLight, this.sunLight.target);

    // stars
    const pts = [];
    let seed = 11;
    const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);
    for (let i = 0; i < 3000; i++) {
      const u = rnd() * 2 - 1, th = rnd() * Math.PI * 2, s = Math.sqrt(1 - u * u);
      pts.push(200 * s * Math.cos(th), 200 * u, 200 * s * Math.sin(th));
    }
    const sg = new THREE.BufferGeometry();
    sg.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
    this.scene.add(new THREE.Points(sg, new THREE.PointsMaterial({ color: 0xffffff, size: 1.3, sizeAttenuation: false })));

    // Earth (rotated by GMST each frame)
    const tex = new THREE.TextureLoader().setCrossOrigin('anonymous').load(EARTH_TEX);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.anisotropy = 8;
    this.earth = new THREE.Mesh(new THREE.SphereGeometry(1, 128, 96),
      new THREE.MeshPhongMaterial({ map: tex, shininess: 8, specular: 0x222233 }));
    this.earth.matrixAutoUpdate = false;
    this.scene.add(this.earth);
    const atmo = new THREE.Mesh(new THREE.SphereGeometry(1.012, 96, 64),
      new THREE.MeshBasicMaterial({ color: 0x4d8cff, transparent: true, opacity: 0.1, side: THREE.BackSide, blending: THREE.AdditiveBlending, depthWrite: false }));
    this.scene.add(atmo);

    // equatorial plane
    this.eq = new THREE.Mesh(new THREE.RingGeometry(1.0, 1.9, 96), new THREE.MeshBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.08, side: THREE.DoubleSide, depthWrite: false }));
    this.eq.rotation.x = -Math.PI / 2;
    this.eq.visible = false;
    this.scene.add(this.eq);

    // Sun direction
    this.sunArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1.9, 0xffd400, 0.1, 0.05);
    this.sunLabel = label('SUN', '#ffd400');
    this.scene.add(this.sunArrow, this.sunLabel);

    // satellite
    this.sat = glow('#ffffff');
    this.sat.scale.set(0.09, 0.09, 1);
    this.satCore = new THREE.Mesh(new THREE.SphereGeometry(0.012, 16, 12), new THREE.MeshBasicMaterial({ color: 0xffffff }));
    this.satLabel = label('CubeSat', '#ffffff');
    this.scene.add(this.sat, this.satCore, this.satLabel);

    // link line
    const lg = new THREE.BufferGeometry();
    lg.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
    this.link = new THREE.Line(lg, new THREE.LineBasicMaterial({ color: 0xff00ff }));
    this.link.frustumCulled = false;
    this.scene.add(this.link);

    new ResizeObserver(() => this.resize()).observe(el);
    this.resize();
  }

  resize() {
    const w = this.el.clientWidth, h = this.el.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = '100%';
    this.renderer.domElement.style.height = '100%';
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  setData(D) {
    this.D = D;
    const { c, n } = D;
    if (this.full) { this.scene.remove(this.full, this.trail); this.full.geometry.dispose(); this.trail.geometry.dispose(); }
    const pos = new Float32Array(n * 3), col = new Float32Array(n * 3);
    const tmp = new THREE.Color();
    for (let i = 0; i < n; i++) {
      pos[3 * i] = c.x[i] / R_E; pos[3 * i + 1] = c.z[i] / R_E; pos[3 * i + 2] = -c.y[i] / R_E;
      tmp.set(D.colors[i]);
      col[3 * i] = tmp.r; col[3 * i + 1] = tmp.g; col[3 * i + 2] = tmp.b;
    }
    const posAttr = new THREE.BufferAttribute(pos, 3);
    const fg = new THREE.BufferGeometry();
    fg.setAttribute('position', posAttr);
    this.full = new THREE.Line(fg, new THREE.LineBasicMaterial({ color: 0x00e5ff, transparent: true, opacity: 0.18 }));
    const tg = new THREE.BufferGeometry();
    tg.setAttribute('position', posAttr);
    tg.setAttribute('color', new THREE.BufferAttribute(col, 3));
    this.trail = new THREE.Line(tg, new THREE.LineBasicMaterial({ vertexColors: true, linewidth: 2 }));
    this.full.frustumCulled = this.trail.frustumCulled = false;
    this.scene.add(this.full, this.trail);
    this.per = Math.round(D.m.period_s / D.dt);

    // ground station on the Earth (child of the Earth mesh, so it rotates with it)
    if (this.gs) this.earth.remove(this.gs, this.gsLabel);
    const la = D.m.gs.lat * Math.PI / 180, lo = D.m.gs.lon * Math.PI / 180;
    const ecef = [Math.cos(la) * Math.cos(lo), Math.cos(la) * Math.sin(lo), Math.sin(la)];
    const local = new THREE.Vector3(ecef[0], ecef[2], -ecef[1]);  // ECEF -> sphere local
    this.gs = new THREE.Mesh(new THREE.SphereGeometry(0.018, 12, 8), new THREE.MeshBasicMaterial({ color: 0xff00ff }));
    this.gs.position.copy(local.clone().multiplyScalar(1.005));
    this.gsLabel = label(D.m.gs.name, '#ff66ff');
    this.gsLabel.position.copy(local.clone().multiplyScalar(1.12));
    this.earth.add(this.gs, this.gsLabel);
  }

  setOptions(o) {
    if (this.full) this.full.visible = o.full;
    this.sunArrow.visible = this.sunLabel.visible = o.sun;
    this.eq.visible = o.eq;
    this.follow = o.follow;
  }

  update(s) {
    if (!this.D) return;
    const cg = Math.cos(s.gmst), sg = Math.sin(s.gmst);
    const R = mul(mul(M, [[cg, -sg, 0], [sg, cg, 0], [0, 0, 1]]), P_T);
    this.earth.matrix.set(R[0][0], R[0][1], R[0][2], 0, R[1][0], R[1][1], R[1][2], 0, R[2][0], R[2][1], R[2][2], 0, 0, 0, 0, 1);
    this.earth.matrixWorldNeedsUpdate = true;

    const p = toThree(s.r, 1 / R_E);
    this.sat.position.copy(p);
    this.satCore.position.copy(p);
    this.satLabel.position.copy(p.clone().multiplyScalar(1.06));
    this.sat.material.color.set(s.color);

    const a = Math.max(0, s.i - this.per);
    this.trail.geometry.setDrawRange(a, s.i - a + 1);

    const sun = toThree(s.sun);
    this.sunLight.position.copy(sun.clone().multiplyScalar(50));
    this.sunLight.intensity = 2.4;
    this.sunArrow.setDirection(sun);
    this.sunLabel.position.copy(sun.clone().multiplyScalar(2.05));

    const lp = this.link.geometry.attributes.position;
    this.link.visible = s.gsEl > 0;
    if (this.link.visible) {
      const g = this.gs.getWorldPosition(new THREE.Vector3());
      lp.setXYZ(0, p.x, p.y, p.z); lp.setXYZ(1, g.x, g.y, g.z); lp.needsUpdate = true;
      this.link.material.color.set(s.linkRate > 0 ? 0xff00ff : 0x552255);
    }

    if (this.follow) {
      const d = this.camera.position.clone().sub(this.controls.target);
      this.controls.target.lerp(p, 0.2);
      this.camera.position.copy(this.controls.target.clone().add(d));
    } else if (this.controls.target.lengthSq() > 1e-6) {
      const d = this.camera.position.clone().sub(this.controls.target);
      this.controls.target.lerp(new THREE.Vector3(), 0.2);
      this.camera.position.copy(this.controls.target.clone().add(d));
    }
  }

  render() {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
