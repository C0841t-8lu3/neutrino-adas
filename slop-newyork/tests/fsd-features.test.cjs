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
