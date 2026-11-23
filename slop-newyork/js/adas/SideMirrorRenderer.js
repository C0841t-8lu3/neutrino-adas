// js/adas/SideMirrorRenderer.js
// ============================================================================
// Photorealistic 2D Picture-in-Picture Side-View Mirror Projection Renderer
// Renders rear perspective, approaching vehicles, distance callouts & BSM border
// ============================================================================

export class SideMirrorRenderer {
  renderSideMirror(ctx, canvas, car, isLeft, bsmActive, trafficSystem = null) {
    if (!ctx || !canvas || !car) return;

    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);

    // 1. Sky and Asphalt Ground Perspective
    const horizonY = h * 0.44;
    const skyGrad = ctx.createLinearGradient(0, 0, 0, horizonY);
    skyGrad.addColorStop(0, '#0c1626');
    skyGrad.addColorStop(1, '#1e293b');
    ctx.fillStyle = skyGrad;
    ctx.fillRect(0, 0, w, horizonY);

    const roadGrad = ctx.createLinearGradient(0, horizonY, 0, h);
    roadGrad.addColorStop(0, '#1a222d');
    roadGrad.addColorStop(1, '#0f141c');
    ctx.fillStyle = roadGrad;
    ctx.fillRect(0, horizonY, w, h - horizonY);

    // 2. Road Lanes & Perspective Lines
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1.5;
    // Vanishing point shifted slightly outwards to reflect mirror angle
    const vpX = isLeft ? w * 0.38 : w * 0.62;

    // Curbs
    ctx.beginPath();
    ctx.moveTo(vpX - 40, horizonY);
    ctx.lineTo(0, h);
    ctx.moveTo(vpX + 40, horizonY);
    ctx.lineTo(w, h);
    ctx.stroke();

    // Dashed center lane markings
    ctx.strokeStyle = '#e2e8f0';
    ctx.lineWidth = 2.0;
    ctx.setLineDash([8, 8]);
    ctx.beginPath();
    ctx.moveTo(vpX, horizonY);
    ctx.lineTo(isLeft ? w * 0.25 : w * 0.75, h);
    ctx.stroke();
    ctx.setLineDash([]);

    // 3. Project Traffic Vehicles onto Mirror Glass
    const px = car.position.x;
    const pz = car.position.z;

    const traffic = [];
    if (trafficSystem?.cars) {
      for (const c of trafficSystem.cars) {
        if (Number.isFinite(c.x) && Number.isFinite(c.z)) traffic.push(c);
      }
    }
    if (trafficSystem?.buses) {
      for (const b of trafficSystem.buses) {
        if (Number.isFinite(b.x) && Number.isFinite(b.z)) traffic.push({ ...b, isBus: true });
      }
    }

    // Mirror orientation: facing rearward with slight outward deflection
    const mirrorAngleOffset = isLeft ? -0.12 : 0.12;
    const mirrorSin = Math.sin(car.heading + Math.PI + mirrorAngleOffset);
    const mirrorCos = Math.cos(car.heading + Math.PI + mirrorAngleOffset);

    for (const v of traffic) {
      const dx = v.x - px;
      const dz = v.z - pz;

      // Distance along mirror optical axis (rearward)
      const mirrorDepth = dx * mirrorSin + dz * mirrorCos;
      // Lateral distance in mirror view plane
      const mirrorLat = dx * mirrorCos - dz * mirrorSin;

      // Render vehicles behind ego car up to 55m
      if (mirrorDepth > 2.0 && mirrorDepth < 55.0) {
        const perspective = 70.0 / mirrorDepth;
        const screenX = vpX + (mirrorLat * perspective * 2.8);
        const screenY = horizonY + (1.2 * perspective * 1.8);

        const carW = (v.isBus ? 3.2 : 2.0) * perspective;
        const carH = (v.isBus ? 3.0 : 1.6) * perspective;

        if (screenX > -carW && screenX < w + carW && screenY > horizonY && screenY < h + carH) {
          // Approaching Vehicle Body
          const distColor = mirrorDepth < 10.0 ? '#ef4444' : mirrorDepth < 22.0 ? '#f59e0b' : '#38bdf8';
          ctx.fillStyle = distColor;
          ctx.strokeStyle = '#f8fafc';
          ctx.lineWidth = 1.0;

          // Vehicle 3D block
          ctx.fillRect(screenX - carW * 0.5, screenY - carH, carW, carH);
          ctx.strokeRect(screenX - carW * 0.5, screenY - carH, carW, carH);

          // Headlights or Taillights
          ctx.fillStyle = '#fef08a';
          const lightR = Math.max(1.5, carW * 0.12);
          ctx.beginPath();
          ctx.arc(screenX - carW * 0.35, screenY - carH * 0.3, lightR, 0, Math.PI * 2);
          ctx.arc(screenX + carW * 0.35, screenY - carH * 0.3, lightR, 0, Math.PI * 2);
          ctx.fill();

          // Distance callout
          if (mirrorDepth < 30.0) {
            ctx.fillStyle = '#ffffff';
            ctx.font = 'bold 9px monospace';
            ctx.textAlign = 'center';
            ctx.fillText(`${mirrorDepth.toFixed(0)}m`, screenX, screenY - carH - 3);
          }
        }
      }
    }

    // 4. Subtle Convex Mirror Vignette & Reflection Glare
    const grad = ctx.createRadialGradient(w * 0.5, h * 0.5, w * 0.3, w * 0.5, h * 0.5, w * 0.6);
    grad.addColorStop(0, 'rgba(0,0,0,0)');
    grad.addColorStop(1, 'rgba(0,0,0,0.45)');
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, w, h);

    // Convex mirror curved grid lines
    ctx.strokeStyle = 'rgba(255,255,255,0.06)';
    ctx.lineWidth = 1.0;
    ctx.beginPath();
    ctx.arc(w * 0.5, h * 0.5, w * 0.45, 0, Math.PI * 2);
    ctx.stroke();

    // 5. Blind-Spot Warning Border Alert Pulse
    if (bsmActive) {
      ctx.strokeStyle = 'rgba(239, 68, 68, 0.85)';
      ctx.lineWidth = 3.5;
      ctx.strokeRect(2, 2, w - 4, h - 4);
    }
  }
}
