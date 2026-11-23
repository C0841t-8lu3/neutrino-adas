// js/adas/HornAudioEngine.js
// ============================================================================
// Acoustic Dual-Tone Horn Engine using Web Audio API
// Synthesizes authentic European/American dual-reed car horn (410 Hz + 505 Hz)
// ============================================================================

export class HornAudioEngine {
  constructor() {
    this.audioCtx = null;
    this.lastHornPlayTime = 0;
  }

  initAudio() {
    if (!this.audioCtx && typeof window !== 'undefined') {
      const AudioContextClass = window.AudioContext || window.webkitAudioContext;
      if (AudioContextClass) {
        this.audioCtx = new AudioContextClass();
      }
    }
    if (this.audioCtx && this.audioCtx.state === 'suspended') {
      this.audioCtx.resume();
    }
  }

  playHornSound(duration = 0.35) {
    try {
      this.initAudio();
      if (!this.audioCtx) return;

      const now = this.audioCtx.currentTime;
      if (now - this.lastHornPlayTime < 0.25) return;
      this.lastHornPlayTime = now;

      // Authentic dual-reed car horn frequencies: F4 (410 Hz) & B4 (505 Hz)
      const freq1 = 410;
      const freq2 = 505;

      const osc1 = this.audioCtx.createOscillator();
      const osc2 = this.audioCtx.createOscillator();
      const gain = this.audioCtx.createGain();
      const filter = this.audioCtx.createBiquadFilter();

      osc1.type = 'sawtooth';
      osc2.type = 'sawtooth';
      osc1.frequency.setValueAtTime(freq1, now);
      osc2.frequency.setValueAtTime(freq2, now);

      filter.type = 'bandpass';
      filter.frequency.setValueAtTime(1400, now);
      filter.Q.setValueAtTime(2.2, now);

      gain.gain.setValueAtTime(0.01, now);
      gain.gain.exponentialRampToValueAtTime(0.45, now + 0.03);
      gain.gain.setValueAtTime(0.40, now + duration - 0.06);
      gain.gain.exponentialRampToValueAtTime(0.001, now + duration);

      osc1.connect(filter);
      osc2.connect(filter);
      filter.connect(gain);
      gain.connect(this.audioCtx.destination);

      osc1.start(now);
      osc2.start(now);
      osc1.stop(now + duration);
      osc2.stop(now + duration);
    } catch (e) {
      console.warn('[HornAudioEngine] Horn sound failed:', e);
    }
  }
}
