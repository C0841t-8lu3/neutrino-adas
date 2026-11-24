const assert = require('node:assert/strict');
const { readFileSync } = require('node:fs');
const { join } = require('node:path');
const test = require('node:test');
const { loadModule } = require('./helpers/module.cjs');

const fsdSource = readFileSync(join(__dirname, '../js/adas/FsdVisualizer.js'), 'utf8');

// Lightweight Three.js WebGPU mock for deterministic Node tests
function createThreeMock() {
  class BufferAttribute {
    constructor(array, itemSize) {
      this.array = array;
      this.itemSize = itemSize;
      this.count = array ? array.length / itemSize : 0;
      this.needsUpdate = false;
    }
    setUsage(usage) {
      this.usage = usage;
      return this;
    }
  }

  class BufferGeometry {
    constructor() {
      this.attributes = {};
      this.index = null;
      this.drawRange = { start: 0, count: Infinity };
    }
    setAttribute(name, attr) {
      this.attributes[name] = attr;
      return this;
    }
    setIndex(index) {
      this.index = index;
      return this;
    }
    setDrawRange(start, count) {
      this.drawRange = { start, count };
    }
    dispose() {}
  }

  class PlaneGeometry extends BufferGeometry {
    constructor(width, height) {
      super();
      this.width = width;
      this.height = height;
    }
    rotateX() { return this; }
  }

  class RingGeometry extends BufferGeometry {
    constructor(innerRadius, outerRadius, thetaSegments) {
      super();
      this.innerRadius = innerRadius;
      this.outerRadius = outerRadius;
    }
    rotateX() { return this; }
  }

  class MeshBasicNodeMaterial {
    constructor(params = {}) {
      Object.assign(this, params);
      this.color = {
        hex: params.color || 0xffffff,
        setHex(h) { this.hex = h; }
      };
    }
    dispose() {}
  }

  class Mesh {
    constructor(geometry, material) {
      this.geometry = geometry;
      this.material = material;
      this.visible = true;
      this.position = {
        x: 0, y: 0, z: 0,
        set(x, y, z) { this.x = x; this.y = y; this.z = z; }
      };
      this.rotation = { x: 0, y: 0, z: 0 };
    }
  }

  return {
    BufferAttribute,
    BufferGeometry,
    PlaneGeometry,
    RingGeometry,
    MeshBasicNodeMaterial,
    Mesh,
    DoubleSide: 2,
    DynamicDrawUsage: 35048
  };
}

test('FsdVisualizer initializes pre-allocated buffers and maintains ZERO-GC buffer identity over frames', () => {
  const threeMock = createThreeMock();
  const { FsdVisualizer } = loadModule(fsdSource, {
    globals: { Float32Array, Uint16Array, Math },
    imports: { 'three/webgpu': threeMock }
  });

  const mockScene = {
    add: () => {},
    remove: () => {}
  };

  const visualizer = new FsdVisualizer(mockScene, {});
  assert.ok(visualizer.ribbonPositions instanceof Float32Array);
  assert.ok(visualizer.ribbonColors instanceof Float32Array);
  assert.ok(visualizer.ribbonIndices instanceof Uint16Array);

