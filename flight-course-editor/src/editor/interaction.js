// Mouse / touch interaction: orbit camera, ray-pick control points and gates,
// drag them on a plane, and report edits back to the kernel-driven controller.
// Orbit controls are disabled while dragging a scene object so the camera
// doesn't fight the edit.

import * as THREE from 'three';
import { OrbitControls } from '../../vendor/OrbitControls.js';

export class Interaction {
  constructor(courseScene, dom, handlers) {
    this.scene = courseScene;
    this.dom = dom;
    this.handlers = handlers; // {onDrag(index|gate, pos, end), onSelect(ref)}
    this.raycaster = new THREE.Raycaster();
    this.pointer = new THREE.Vector2();
    this.dragPlane = new THREE.Plane();
    this.dragging = null;
    this.selected = null;

    this.orbit = new OrbitControls(courseScene.camera, dom);
    this.orbit.enableDamping = true;
    this.orbit.dampingFactor = 0.08;
    this.orbit.maxDistance = 1200;

    dom.addEventListener('pointerdown', (e) => this._down(e));
    dom.addEventListener('pointermove', (e) => this._move(e));
    window.addEventListener('pointerup', (e) => this._up(e));
  }

  _setPointer(e) {
    const rect = this.dom.getBoundingClientRect();
    this.pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    this.pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
  }

  _pick(e) {
    this._setPointer(e);
    this.raycaster.setFromCamera(this.pointer, this.scene.camera);
    const hits = this.raycaster.intersectObjects(this.scene.dragMeshes, true);
    if (!hits.length) return null;
    // Walk up to the object carrying userData.kind.
    let obj = hits[0].object;
    while (obj && !obj.userData.kind) obj = obj.parent;
    return obj ? { obj, point: hits[0].point } : null;
  }

  _down(e) {
    const hit = this._pick(e);
    if (!hit) { this.selected = null; this.handlers.onSelect?.(null); return; }
    e.preventDefault();
    this.dom.setPointerCapture?.(e.pointerId);
    const { obj, point } = hit;
    this.selected = obj;
    this.handlers.onSelect?.(obj.userData);
    // Drag plane faces the camera and passes through the picked point.
    const normal = new THREE.Vector3();
    this.scene.camera.getWorldDirection(normal);
    this.dragPlane.setFromNormalAndCoplanarPoint(normal, point);
    this.dragging = { obj, kind: obj.userData.kind, index: obj.userData.index, moved: false };
    this.orbit.enabled = false;
  }

  _move(e) {
    if (!this.dragging) {
      this.dom.style.cursor = this._pick(e) ? 'grab' : 'default';
      return;
    }
    this._setPointer(e);
    this.raycaster.setFromCamera(this.pointer, this.scene.camera);
    const target = new THREE.Vector3();
    if (!this.raycaster.ray.intersectPlane(this.dragPlane, target)) return;
    this.dragging.moved = true;
    const pos = [target.x, target.y, target.z];
    this.dragging.obj.position.copy(target);
    this.handlers.onDrag?.(this.dragging.kind, this.dragging.index, pos, false);
  }

  _up(e) {
    if (!this.dragging) return;
    const d = this.dragging;
    this.dragging = null;
    this.orbit.enabled = true;
    if (d.moved) {
      const pos = [d.obj.position.x, d.obj.position.y, d.obj.position.z];
      this.handlers.onDrag?.(d.kind, d.index, pos, true);
    }
  }

  // Programmatic focus (clicking a violation row flies the camera to an s).
  focusOn(pos) {
    const target = new THREE.Vector3(...pos);
    const dir = new THREE.Vector3(0.7, 0.5, 0.8).normalize();
    this.scene.camera.position.copy(target).add(dir.multiplyScalar(60));
    this.orbit.target.copy(target);
    this.orbit.update();
  }
}
