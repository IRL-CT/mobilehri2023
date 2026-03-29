/**
 * app.js — Dance Studio main application.
 *
 * Flow:
 *   1. Generate or load a dance
 *   2. Play → executes on robot in Gazebo
 *   3. 2D view shows live robot position from odometry
 */

import { Stage } from './stage.js';
import { Timeline } from './timeline.js';

const $ = (sel) => document.querySelector(sel);

const dom = {
    canvas:        $('#stage-canvas'),
    btnGenerate:   $('#btn-generate'),
    btnPlay:       $('#btn-play'),
    btnStop:       $('#btn-stop'),
    btnSave:       $('#btn-save'),
    loadDropdown:  $('#load-dropdown'),
    aiPrompt:      $('#ai-prompt'),
    timeDisplay:   $('#time-display'),
    sliderEnergy:  $('#slider-energy'),
    energyValue:   $('#energy-value'),
    seqTitle:      $('#seq-title'),
    seqMood:       $('#seq-mood'),
    seqTempo:      $('#seq-tempo'),
    seqDescription:$('#seq-description'),
    loadingOverlay:$('#loading-overlay'),
    timelineBar:   $('#timeline-bar'),
    rosStatus:     $('#ros-status'),
};

let stage;
let timeline;

const state = {
    sequence: null,
    stage: { width: 8.0, depth: 6.0 },
    overrides: { energyScale: 1.0 },
    executing: false,
};


// ── Init ────────────────────────────────────────────────────────────────────

async function init() {
    stage = new Stage(dom.canvas);
    timeline = new Timeline(dom.timelineBar);

    setupEventListeners();
    if (typeof ResizeObserver !== 'undefined') {
        new ResizeObserver(() => stage.resize()).observe(dom.canvas.parentElement);
    }

    stage.render();
    listDances();
    checkRosStatus();
    connectPositionWs();

    // Render loop for live position
    (function loop() {
        requestAnimationFrame(loop);
        stage.render();
    })();
}


// ── Event Listeners ─────────────────────────────────────────────────────────

function setupEventListeners() {
    dom.btnGenerate.addEventListener('click', () => generateDance());
    dom.btnPlay.addEventListener('click', () => playDance());
    dom.btnStop.addEventListener('click', () => stopDance());
    $('#btn-reset-robot').addEventListener('click', () => resetRobot());
    $('#btn-view-prompt').addEventListener('click', () => viewPrompt());
    $('#btn-close-prompt').addEventListener('click', () => {
        document.getElementById('prompt-modal').classList.add('hidden');
    });

    dom.sliderEnergy.addEventListener('input', (e) => {
        const val = parseFloat(e.target.value);
        dom.energyValue.textContent = val.toFixed(2);
        state.overrides.energyScale = val;
    });

    dom.btnSave.addEventListener('click', () => {
        const name = prompt('Enter a name for this choreography:');
        if (name && name.trim()) saveDance(name.trim());
    });
    dom.loadDropdown.addEventListener('change', (e) => {
        if (e.target.value) loadDance(e.target.value);
    });

    window.addEventListener('resize', () => stage.resize());
}


// ── Generate ────────────────────────────────────────────────────────────────

async function generateDance() {
    const description = dom.aiPrompt.value.trim();
    if (!description) { alert('Please enter a dance description.'); return; }

    showLoading(true);
    try {
        const res = await fetch('/api/generate', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                description,
                stage_width: state.stage.width,
                stage_depth: state.stage.depth,
            }),
        });
        if (!res.ok) throw new Error(await res.text());
        const data = await res.json();
        state.sequence = data;
        updateSequenceInfo(data);
        printSequence(data);
    } catch (e) {
        alert('Generation failed: ' + e.message);
    } finally {
        showLoading(false);
    }
}


// ── Play / Stop ─────────────────────────────────────────────────────────────

async function playDance() {
    if (!state.sequence) { alert('No sequence loaded.'); return; }
    if (state.executing) return;

    stage.clearLiveTrail();

    try {
        dom.btnPlay.disabled = true;
        dom.btnPlay.textContent = '⏳ Running...';
        state.executing = true;
        dom.rosStatus.className = 'status-dot executing';

        const res = await fetch('/api/execute', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                sequence: state.sequence,
                energy_scale: state.overrides.energyScale,
            }),
        });
        if (!res.ok) {
            const err = await res.json();
            throw new Error(err.detail || `HTTP ${res.status}`);
        }

        const poll = setInterval(async () => {
            try {
                const sr = await fetch('/api/ros_status');
                const st = await sr.json();
                if (!st.executing) {
                    clearInterval(poll);
                    onDone();
                }
            } catch (_) {}
        }, 500);
    } catch (e) {
        alert('Play failed: ' + e.message);
        onDone();
    }
}

function onDone() {
    state.executing = false;
    dom.btnPlay.disabled = false;
    dom.btnPlay.textContent = '▶ Play';
    dom.rosStatus.className = 'status-dot connected';
}

async function stopDance() {
    try { await fetch('/api/stop', { method: 'POST' }); } catch (_) {}
    onDone();
}

async function resetRobot() {
    try {
        await fetch('/api/reset', { method: 'POST' });
        stage.clearLiveTrail();
    } catch (e) { alert('Reset failed: ' + e.message); }
}


// ── Save / Load ─────────────────────────────────────────────────────────────

async function saveDance(name) {
    if (!state.sequence) { alert('No sequence to save.'); return; }
    try {
        const res = await fetch(`/api/dances/${encodeURIComponent(name)}`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(state.sequence),
        });
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        listDances();
    } catch (e) { alert('Save failed: ' + e.message); }
}

async function loadDance(name) {
    try {
        const res = await fetch(`/api/dances/${encodeURIComponent(name)}`);
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        const data = await res.json();
        state.sequence = data;
        updateSequenceInfo(data);
        printSequence(data);
        stage.clearLiveTrail();
    } catch (e) { alert('Load failed: ' + e.message); }
}

async function listDances() {
    try {
        const res = await fetch('/api/dances');
        const data = await res.json();
        dom.loadDropdown.innerHTML = '<option value="">Load...</option>';
        for (const name of data.dances || []) {
            const opt = document.createElement('option');
            opt.value = name;
            opt.textContent = name;
            dom.loadDropdown.appendChild(opt);
        }
    } catch (_) {}
}


// ── UI Helpers ──────────────────────────────────────────────────────────────

function updateSequenceInfo(seq) {
    dom.seqTitle.textContent = seq.title || '--';
    dom.seqMood.textContent = seq.mood || '--';
    dom.seqTempo.textContent = seq.tempo_bpm ? `${seq.tempo_bpm} BPM` : '--';
}

function printSequence(seq) {
    console.group(`Dance: ${seq.title || 'Untitled'} (${seq.mood}, ${seq.tempo_bpm} BPM)`);
    console.log(seq.description || '');

    let text = '';
    for (const phrase of (seq.phrases || [])) {
        const intent = phrase.intent ? ` — ${phrase.intent}` : '';
        text += `[${phrase.name}]${intent}\n`;
        console.group(`[${phrase.name}]${intent}`);
        for (const m of (phrase.motifs || [])) {
            const p = m.params && Object.keys(m.params).length > 0
                ? ' ' + JSON.stringify(m.params) : '';
            const note = m.annotation ? `  # ${m.annotation}` : '';
            text += `  ${m.move}${p}  e=${m.energy} t=${m.texture}${note}\n`;
            console.log(`${m.move}${p}  e=${m.energy} t=${m.texture}${note}`);
        }
        if (phrase.gap_after) text += `  ... pause ${phrase.gap_after}s\n`;
        console.groupEnd();
    }
    console.groupEnd();
    dom.seqDescription.textContent = text || seq.description || '';
}

async function viewPrompt() {
    const description = dom.aiPrompt.value.trim() || 'A playful dance';
    try {
        const res = await fetch('/api/prompt', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                description,
                stage_width: state.stage.width,
                stage_depth: state.stage.depth,
            }),
        });
        if (!res.ok) { alert('Prompt fetch failed: ' + await res.text()); return; }
        const data = await res.json();
        const modal = document.getElementById('prompt-modal');
        const text = document.getElementById('prompt-text');
        if (!modal || !text) {
            console.log('System prompt:', data.system_prompt);
            alert('Prompt logged to console');
            return;
        }
        text.textContent = `=== SYSTEM PROMPT ===\n\n${data.system_prompt}\n\n=== USER PROMPT ===\n\n${data.user_prompt}`;
        modal.classList.remove('hidden');
    } catch (e) { alert('Failed: ' + e.message); }
}

function showLoading(visible) {
    dom.loadingOverlay.classList.toggle('hidden', !visible);
}


// ── ROS Connection ──────────────────────────────────────────────────────────

async function checkRosStatus() {
    try {
        const res = await fetch('/api/ros_status');
        const data = await res.json();
        dom.rosStatus.className = data.connected ? 'status-dot connected' : 'status-dot disconnected';
    } catch (_) {
        dom.rosStatus.className = 'status-dot disconnected';
    }
}

function connectPositionWs() {
    const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    const url = `${protocol}//${location.host}/ws/position`;

    function connect() {
        const ws = new WebSocket(url);
        ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (data.error) return;
            stage.setLivePosition(data.x, data.y, data.theta);
            if (state.executing) {
                dom.timeDisplay.textContent = `x=${data.x.toFixed(2)} y=${data.y.toFixed(2)}`;
            }
        };
        ws.onclose = () => setTimeout(connect, 3000);
        ws.onerror = () => ws.close();
    }
    connect();
}


// ── Bootstrap ───────────────────────────────────────────────────────────────

window._debug = { get state() { return state; }, get stage() { return stage; } };
document.addEventListener('DOMContentLoaded', init);
