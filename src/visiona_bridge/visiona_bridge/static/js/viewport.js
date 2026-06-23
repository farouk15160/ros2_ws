/**
 * Visiona Robotics Studio — 3D DH arm viewport (Three.js)
 */
(function () {
  const JOINT_COLORS = [0x00c2ff, 0x00ffae, 0x7a5fff, 0xffa63e, 0xff4e63, 0x8a96a3];

  class RobotViewport {
    constructor(container) {
      this.container = container;
      this.dh = null;
      this.scene = new THREE.Scene();
      this.scene.background = new THREE.Color(0x0b0d10);
      this.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 10);
      this.camera.position.set(0.8, 0.6, 0.9);
      this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
      this.renderer.setPixelRatio(window.devicePixelRatio || 1);
      container.innerHTML = "";
      container.appendChild(this.renderer.domElement);

      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.target.set(0.15, 0.2, 0.25);

      const grid = new THREE.GridHelper(1.2, 24, 0x1a3a4a, 0x111418);
      grid.position.y = 0;
      this.scene.add(grid);

      const light = new THREE.DirectionalLight(0xffffff, 1.1);
      light.position.set(1, 2, 1);
      this.scene.add(light);
      this.scene.add(new THREE.AmbientLight(0x404060, 0.6));

      this.linkMeshes = [];
      this.jointMeshes = [];
      this.eeMarker = null;
      this._anim = () => {
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
        requestAnimationFrame(this._anim);
      };
      this._anim();
      window.addEventListener("resize", () => this._resize());
      this._resize();
    }

    async init() {
      const resp = await fetch("/api/kinematics");
      const data = await resp.json();
      this.dh = data.dh_params || [];
      this._buildArm();
    }

    _resize() {
      const w = this.container.clientWidth || 400;
      const h = this.container.clientHeight || 280;
      this.camera.aspect = w / h;
      this.camera.updateProjectionMatrix();
      this.renderer.setSize(w, h);
    }

    _buildArm() {
      this.linkMeshes.forEach((m) => this.scene.remove(m));
      this.jointMeshes.forEach((m) => this.scene.remove(m));
      this.linkMeshes = [];
      this.jointMeshes = [];
      if (this.eeMarker) this.scene.remove(this.eeMarker);

      const mat = (i) =>
        new THREE.MeshStandardMaterial({
          color: JOINT_COLORS[i % JOINT_COLORS.length],
          metalness: 0.35,
          roughness: 0.45,
          emissive: JOINT_COLORS[i % JOINT_COLORS.length],
          emissiveIntensity: 0.08,
        });

      for (let i = 0; i < 4; i++) {
        const sphere = new THREE.Mesh(new THREE.SphereGeometry(0.018, 16, 16), mat(i));
        this.jointMeshes.push(sphere);
        this.scene.add(sphere);
        const cyl = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 1, 12), mat(i));
        cyl.visible = false;
        this.linkMeshes.push(cyl);
        this.scene.add(cyl);
      }
      this.eeMarker = new THREE.Mesh(
        new THREE.SphereGeometry(0.022, 16, 16),
        new THREE.MeshStandardMaterial({ color: 0x00ffae, emissive: 0x00ffae, emissiveIntensity: 0.4 })
      );
      this.scene.add(this.eeMarker);
      this.updateJoints([90, 90, 90, 90, 90, 0]);
    }

    _fk(jointsDeg) {
      if (!this.dh || this.dh.length === 0) return [[0, 0, 0]];
      const q = jointsDeg.slice(0, 4).map((d) => (d * Math.PI) / 180);
      let t = new THREE.Matrix4().identity();
      const pts = [new THREE.Vector3(0, 0, 0)];
      for (let i = 0; i < this.dh.length; i++) {
        const [a, alpha, d, offset] = this.dh[i];
        const theta = q[i] + offset;
        const ct = Math.cos(theta), st = Math.sin(theta);
        const ca = Math.cos(alpha), sa = Math.sin(alpha);
        const m = new THREE.Matrix4().set(
          ct, -st * ca, st * sa, a * ct,
          st, ct * ca, -ct * sa, a * st,
          0, sa, ca, d,
          0, 0, 0, 1
        );
        t = t.clone().multiply(m);
        const p = new THREE.Vector3();
        p.setFromMatrixPosition(t);
        pts.push(p);
      }
      return pts;
    }

    updateJoints(jointsDeg) {
      if (!this.dh) return;
      const pts = this._fk(jointsDeg);
      for (let i = 0; i < this.jointMeshes.length; i++) {
        if (pts[i]) this.jointMeshes[i].position.copy(pts[i]);
        if (pts[i + 1]) {
          const a = pts[i], b = pts[i + 1];
          const cyl = this.linkMeshes[i];
          const mid = a.clone().add(b).multiplyScalar(0.5);
          const dir = b.clone().sub(a);
          const len = dir.length();
          cyl.scale.set(1, len, 1);
          cyl.position.copy(mid);
          cyl.visible = len > 0.001;
          if (len > 0.001) {
            cyl.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
          }
        }
      }
      if (this.eeMarker && pts.length) this.eeMarker.position.copy(pts[pts.length - 1]);
    }
  }

  window.initRobotViewport = async function (containerId) {
    const el = document.getElementById(containerId);
    if (!el || typeof THREE === "undefined") return null;
    const vp = new RobotViewport(el);
    await vp.init();
    window.updateRobotModel = (anglesRad) => {
      const deg = anglesRad.map((r) => (r * 180) / Math.PI);
      while (deg.length < 6) deg.push(0);
      vp.updateJoints(deg);
    };
    return vp;
  };
})();
