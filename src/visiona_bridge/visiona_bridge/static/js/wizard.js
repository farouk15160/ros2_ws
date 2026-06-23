/**
 * URAF Setup Wizard — PLAN.md §17 multi-step flow
 */
(function () {
  const STEPS = [
    { id: 'welcome', title: 'Welcome', desc: 'Select setup mode' },
    { id: 'hardware', title: 'Hardware', desc: 'Connect your robot' },
    { id: 'discovery', title: 'Discovery', desc: 'Scan and identify' },
    { id: 'confirm', title: 'Confirm', desc: 'Verify robot profile' },
    { id: 'visualize', title: 'Visualize', desc: 'Live 3D model' },
    { id: 'calibrate', title: 'Calibrate', desc: 'Camera & joints' },
    { id: 'complete', title: 'Complete', desc: 'Ready to operate' },
  ];

  let state = { step: 0, mode: 'guided', discovery: {}, profile_match: {}, confirmed: false };

  function el(id) { return document.getElementById(id); }

  function renderStepNav() {
    const nav = el('wizard-step-nav');
    if (!nav) return;
    nav.innerHTML = STEPS.map((s, i) =>
      `<div class="wizard-step-item ${i === state.step ? 'active' : ''} ${i < state.step ? 'done' : ''}" data-step="${i}">
        <span class="wizard-step-num">${i + 1}</span>
        <span class="wizard-step-label">${s.title}</span>
      </div>`
    ).join('');
  }

  function showPanel(stepIdx) {
    STEPS.forEach((s, i) => {
      const panel = el('wizard-panel-' + s.id);
      if (panel) panel.style.display = i === stepIdx ? 'block' : 'none';
    });
    const prog = el('wizard-progress-bar');
    if (prog) prog.style.width = ((stepIdx + 1) / STEPS.length * 100) + '%';
    const title = el('wizard-step-title');
    if (title) title.textContent = STEPS[stepIdx].title + ' — ' + STEPS[stepIdx].desc;
    renderStepNav();
  }

  function saveState(patch) {
    return fetch('/api/uraf/wizard', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(patch || {}),
    }).then(r => r.json()).then(d => {
      if (d.state) { state = d.state; showPanel(state.step); }
      return d;
    }).catch(() => null);
  }

  function loadState() {
    return fetch('/api/uraf/wizard').then(r => r.json()).then(d => {
      state = d.state || state;
      const modeSel = el('uraf-mode');
      if (modeSel && state.mode) modeSel.value = state.mode;
      showPanel(state.step || 0);
      return state;
    }).catch(() => { showPanel(0); });
  }

  window.wizardNext = function () {
    const mode = el('uraf-mode')?.value || 'guided';
    if (state.step === 0) {
      saveState({ step: 1, mode }).then(() => {
        if (mode === 'auto') runAutoSetup();
      });
      return;
    }
    if (state.step === 1) {
      runDiscovery();
      saveState({ step: 2 });
      return;
    }
    if (state.step === 2) {
      saveState({ step: 3 });
      return;
    }
    if (state.step === 3) {
      saveState({ step: 4, confirmed: true });
      return;
    }
    if (state.step === 4) {
      saveState({ step: 5 });
      return;
    }
    if (state.step === 5) {
      generateUrafUrdf();
      saveState({ step: 6, complete: true });
      return;
    }
    saveState({ step: Math.min(state.step + 1, STEPS.length - 1) });
  };

  window.wizardBack = function () {
    saveState({ step: Math.max(0, state.step - 1) });
  };

  window.wizardReset = function () {
    fetch('/api/uraf/wizard/reset', { method: 'POST' }).then(() => loadState());
  };

  function runAutoSetup() {
    const log = el('wizard-discovery-log');
    if (log) log.innerHTML = '<div style="color:#00c2ff;">Auto setup running…</div>';
    fetch('/api/uraf/discovery', { method: 'POST' })
      .then(() => fetch('/api/uraf/community/lookup', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ profile_id: 'visiona_v1' }),
      }))
      .then(() => fetch('/api/uraf/generate_urdf', { method: 'POST' }))
      .then(() => saveState({ step: 6, complete: true, mode: 'auto' }))
      .catch(e => { if (typeof logMessage === 'function') logMessage('error', String(e)); });
  }

  window.onUrafDiscovery = function (data) {
    state.discovery = data;
    const log = el('wizard-discovery-log') || el('uraf-discovery-log');
    if (typeof renderDiscovery === 'function' && log) renderDiscovery(data);
    saveState({ discovery: data });
  };

  window.onUrafCommunityMatch = function (data) {
    state.profile_match = data;
    const box = el('wizard-confirm-body');
    if (box) box.textContent = JSON.stringify(data, null, 2);
    saveState({ profile_match: data });
  };

  document.addEventListener('DOMContentLoaded', () => {
    if (el('wizard-step-nav')) loadState();
  });
})();
