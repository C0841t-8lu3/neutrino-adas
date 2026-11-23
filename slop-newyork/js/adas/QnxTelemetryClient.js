// js/adas/QnxTelemetryClient.js
// ============================================================================
// High-Speed WebSocket Telemetry Link to QNX 8.0 RTOS Safety Kernel Bridge
// Encapsulates socket lifecycle, JSON message framing, and reconnect semantics
// ============================================================================

export class QnxTelemetryClient {
  constructor({ url = 'ws://127.0.0.1:8089/', onResponse = null, onConnectionChange = null } = {}) {
    this.url = url;
    this.onResponse = onResponse;
    this.onConnectionChange = onConnectionChange;

    this.ws = null;
    this.connected = false;
    this.reconnectTimer = null;
    this.explicitClose = false;
  }

  connect() {
    this.explicitClose = false;
    if (typeof WebSocket === 'undefined') return;

    try {
      if (this.ws) {
        this.ws.onopen = null;
        this.ws.onmessage = null;
        this.ws.onclose = null;
        this.ws.onerror = null;
        try { this.ws.close(); } catch (_) {}
      }

      this.ws = new WebSocket(this.url);

      this.ws.onopen = () => {
        this.connected = true;
        console.log('[QnxTelemetryClient] WebSocket Bridge Connected to QNX RTOS on VM!');
        if (this.onConnectionChange) {
          this.onConnectionChange(true, 'QNX 8.0 RTOS Safety Kernel Online (60Hz Socket)');
        }
      };

      this.ws.onmessage = (event) => {
        try {
          const res = JSON.parse(event.data);
          if (this.onResponse) {
            this.onResponse(res);
          }
        } catch (e) {
          console.warn('[QnxTelemetryClient] JSON parse error:', e);
        }
      };

      this.ws.onclose = () => {
        const wasConnected = this.connected;
        this.connected = false;
        if (this.onConnectionChange) {
          this.onConnectionChange(false, 'QNX Host Disconnected. Reconnecting...');
        }
        if (!this.explicitClose) {
          this.scheduleReconnect();
        }
      };

      this.ws.onerror = () => {
        this.connected = false;
      };
    } catch (e) {
      console.warn('[QnxTelemetryClient] WebSocket init error:', e);
      if (!this.explicitClose) {
        this.scheduleReconnect();
      }
    }
  }

  scheduleReconnect() {
    if (this.reconnectTimer) return;
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      if (!this.explicitClose) {
        this.connect();
      }
    }, 2000);
  }

  send(telemetry) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(telemetry));
      return true;
    }
    return false;
  }

  disconnect() {
    this.explicitClose = true;
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    if (this.ws) {
      try { this.ws.close(); } catch (_) {}
      this.ws = null;
    }
    this.connected = false;
  }

  isConnected() {
    return this.connected;
  }
}
