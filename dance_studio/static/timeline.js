/**
 * Timeline - Playback timeline and move segment display.
 *
 * Renders colored segments for each move in the choreography and
 * provides a cursor showing the current playback position.
 */

const PHRASE_COLORS = [
    '#569cd6', '#4ec9b0', '#c586c0', '#6a9955',
    '#dcdcaa', '#ce9178', '#b5cea8', '#9cdcfe',
];

export class Timeline {
    constructor(container) {
        /** @type {HTMLElement} */
        this.container = container;

        /** @type {HTMLElement} */
        this.segmentsEl = container.querySelector('#timeline-segments') || container;

        /** @type {HTMLElement} */
        this.cursorEl = container.querySelector('#timeline-cursor');

        this.totalDuration = 0;
        this.moveBoundaries = [];
        this._phraseColorMap = {};
        this._phraseCounter = 0;
        this._segments = [];
    }

    /* ------------------------------------------------------------------ */
    /*  Public API                                                         */
    /* ------------------------------------------------------------------ */

    /**
     * Build the timeline from move boundaries.
     * @param {Array} moveBoundaries - [{name, phrase, t_start, t_end}, ...]
     * @param {number} totalDuration - total sequence duration in seconds
     */
    setData(moveBoundaries, totalDuration) {
        this.moveBoundaries = moveBoundaries || [];
        this.totalDuration = totalDuration || 0;

        // Reset
        this.segmentsEl.innerHTML = '';
        this._segments = [];
        this._phraseColorMap = {};
        this._phraseCounter = 0;

        if (this.totalDuration <= 0 || this.moveBoundaries.length === 0) {
            return;
        }

        for (const mb of this.moveBoundaries) {
            const duration = (mb.t_end || 0) - (mb.t_start || 0);
            if (duration <= 0) continue;

            const fraction = duration / this.totalDuration;
            const color = this._colorForPhrase(mb.phrase || mb.name || '');

            const seg = document.createElement('div');
            seg.className = 'timeline-segment';
            seg.style.flex = `${fraction} 0 0`;
            seg.style.backgroundColor = color;
            seg.title = `${mb.name || 'move'} (${mb.t_start.toFixed(1)}s - ${mb.t_end.toFixed(1)}s)`;

            // Show text label if segment is wide enough (> 5% of total)
            if (fraction > 0.05) {
                seg.textContent = mb.name || '';
            }

            seg.dataset.tStart = mb.t_start;
            seg.dataset.tEnd = mb.t_end;

            this.segmentsEl.appendChild(seg);
            this._segments.push({ el: seg, t_start: mb.t_start, t_end: mb.t_end });
        }
    }

    /**
     * Update the cursor position and highlight the current segment.
     * @param {number} currentTime - playback time in seconds
     */
    update(currentTime) {
        if (this.totalDuration <= 0) return;

        // Move cursor
        const containerRect = this.segmentsEl.getBoundingClientRect();
        const parentRect = this.container.getBoundingClientRect();
        const segLeft = containerRect.left - parentRect.left;
        const segWidth = containerRect.width;

        const frac = Math.min(currentTime / this.totalDuration, 1);
        const cursorLeft = segLeft + frac * segWidth;

        if (this.cursorEl) {
            this.cursorEl.style.left = `${cursorLeft}px`;
        }

        // Highlight current segment
        for (const s of this._segments) {
            if (currentTime >= s.t_start && currentTime < s.t_end) {
                s.el.classList.add('current');
                s.el.classList.remove('dim');
            } else {
                s.el.classList.remove('current');
                s.el.classList.add('dim');
            }
        }
    }

    /* ------------------------------------------------------------------ */
    /*  Private                                                            */
    /* ------------------------------------------------------------------ */

    _colorForPhrase(phrase) {
        if (this._phraseColorMap[phrase] !== undefined) {
            return this._phraseColorMap[phrase];
        }
        const color = PHRASE_COLORS[this._phraseCounter % PHRASE_COLORS.length];
        this._phraseColorMap[phrase] = color;
        this._phraseCounter++;
        return color;
    }
}
