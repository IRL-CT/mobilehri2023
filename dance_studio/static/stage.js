/**
 * Stage - Canvas rendering module for the Dance Studio.
 *
 * Draws a top-down view of the dance stage including grid, boundary,
 * path trace, move labels, and the robot triangle.
 *
 * World coordinate convention:
 *   +x = forward  (up on screen)
 *   +y = left      (left on screen)
 */

export class Stage {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');

        // Stage dimensions in metres (matching stage.sdf: 6m x 8m)
        // forward = world +x (Gazebo X), lateral = world +y (Gazebo Y)
        this.stageForward = 6.0;  // forward/backward span
        this.stageLateral = 8.0;  // left/right span

        // Derived after resize — start with canvas's default size
        this.canvasW = canvas.width;
        this.canvasH = canvas.height;
        this.pixelsPerMeter = Math.min(canvas.width / this.stageLateral, canvas.height / this.stageForward) * 0.85;
        this.offsetX = canvas.width / 2;
        this.offsetY = canvas.height / 2;

        // Live robot position from ROS (green dot)
        this._livePos = null;
        this._liveTrail = [];
        this._liveOrigin = null;  // first odom reading becomes (0,0)

        this.resize();
    }

    setLivePosition(x, y, theta) {
        // Treat first received position as origin
        if (this._liveOrigin === null) {
            this._liveOrigin = { x, y, theta };
        }
        // Transform to stage-relative coords
        const ox = x - this._liveOrigin.x;
        const oy = y - this._liveOrigin.y;
        const ot = theta - this._liveOrigin.theta;

        this._livePos = { x: ox, y: oy, theta: ot };
        const last = this._liveTrail.length > 0 ? this._liveTrail[this._liveTrail.length - 1] : null;
        if (!last || Math.hypot(ox - last.x, oy - last.y) > 0.01) {
            this._liveTrail.push({ x: ox, y: oy });
            if (this._liveTrail.length > 2000) this._liveTrail = this._liveTrail.slice(-2000);
        }
    }

    clearLiveTrail() {
        this._liveTrail = [];
        this._liveOrigin = null;  // re-zero on next position
    }

    /* ------------------------------------------------------------------ */
    /*  Sizing                                                             */
    /* ------------------------------------------------------------------ */

    /** Call when the container size changes. */
    resize() {
        const container = this.canvas.parentElement;
        if (!container) return;

        // Use container's actual rendered size
        const w = container.clientWidth;
        const h = container.clientHeight;

        if (w < 10 || h < 10) {
            setTimeout(() => this.resize(), 200);
            return;
        }

        const dpr = window.devicePixelRatio || 1;

        // Fill the container
        this.canvas.width = Math.round(w * dpr);
        this.canvas.height = Math.round(h * dpr);
        this.canvas.style.width = w + 'px';
        this.canvas.style.height = h + 'px';

        this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        this._recalcScale(w, h);
        console.log(`Stage: ${w}x${h}px, scale=${this.pixelsPerMeter.toFixed(1)}px/m`);
    }

    /** Set stage dimensions (metres) and recalculate. */
    setDimensions(forward, lateral) {
        this.stageForward = forward;
        this.stageLateral = lateral;
        this.resize();
    }

    _recalcScale(canvasW, canvasH) {
        this.canvasW = canvasW;
        this.canvasH = canvasH;
        // After 90° CCW rotation: forward maps to canvas width, lateral to height
        this.pixelsPerMeter =
            Math.min(canvasW / this.stageForward, canvasH / this.stageLateral) * 0.85;
        this.offsetX = canvasW / 2;
        this.offsetY = canvasH / 2;
    }

    /* ------------------------------------------------------------------ */
    /*  Coordinate transform                                               */
    /* ------------------------------------------------------------------ */

    /**
     * Convert world coords to canvas pixel coords.
     *   world +x (forward) -> canvas up  (negative canvas Y)
     *   world +y (left)    -> canvas left (negative canvas X)
     */
    worldToCanvas(wx, wy) {
        // Rotated 90° CCW: old (cx, cy) → new (-cy, cx) around center
        const rawCx = this.offsetX + wx * this.pixelsPerMeter;
        const rawCy = this.offsetY - wy * this.pixelsPerMeter;
        // 90° CCW rotation around canvas center
        const cx = this.offsetX - (rawCy - this.offsetY);
        const cy = this.offsetY + (rawCx - this.offsetX);
        return [cx, cy];
    }

    /* ------------------------------------------------------------------ */
    /*  Rendering                                                          */
    /* ------------------------------------------------------------------ */

    /**
     * Main render call.
     * @param {Object} trajectory  - API trajectory object with .frames and .move_boundaries
     * @param {number} currentFrameIndex - index into trajectory.frames
     */
    render() {
        const ctx = this.ctx;
        ctx.clearRect(0, 0, this.canvasW, this.canvasH);

        this.drawGrid();
        this.drawBoundary();

        // --- Live robot position (green) from ROS — always drawn ---
        this._drawLive(ctx);

        // --- Position text ---
        const pos = this._livePos;
        if (pos) {
            ctx.save();
            ctx.fillStyle = '#4ec9b0';
            ctx.font = '11px Consolas, Monaco, monospace';
            ctx.textAlign = 'left';
            ctx.textBaseline = 'top';
            ctx.fillText(
                `x=${pos.x.toFixed(2)}m  y=${pos.y.toFixed(2)}m  \u03b8=${(pos.theta * 180 / Math.PI).toFixed(0)}\u00b0`,
                8, this.canvasH - 16
            );
            ctx.restore();
        }
    }

    /* ------------------------------------------------------------------ */
    /*  Helpers                                                            */
    /* ------------------------------------------------------------------ */

    drawGrid() {
        const ctx = this.ctx;
        const step = 0.5; // metres
        const halfF = this.stageForward / 2;   // forward half-extent
        const halfL = this.stageLateral / 2;    // lateral half-extent

        ctx.save();
        ctx.strokeStyle = 'rgba(255,255,255,0.06)';
        ctx.lineWidth = 1;

        // Horizontal lines on canvas (constant forward value, vary lateral)
        for (let fwd = -halfF; fwd <= halfF + 0.001; fwd += step) {
            const [x0, y0] = this.worldToCanvas(fwd, -halfL);
            const [x1, y1] = this.worldToCanvas(fwd, halfL);
            ctx.beginPath();
            ctx.moveTo(x0, y0);
            ctx.lineTo(x1, y1);
            ctx.stroke();
        }

        // Vertical lines on canvas (constant lateral value, vary forward)
        for (let lat = -halfL; lat <= halfL + 0.001; lat += step) {
            const [x0, y0] = this.worldToCanvas(-halfF, lat);
            const [x1, y1] = this.worldToCanvas(halfF, lat);
            ctx.beginPath();
            ctx.moveTo(x0, y0);
            ctx.lineTo(x1, y1);
            ctx.stroke();
        }

        // Origin cross-hair (brighter)
        ctx.strokeStyle = 'rgba(255,255,255,0.15)';
        const [ox, oy] = this.worldToCanvas(0, 0);
        ctx.beginPath();
        ctx.moveTo(ox - 8, oy);
        ctx.lineTo(ox + 8, oy);
        ctx.moveTo(ox, oy - 8);
        ctx.lineTo(ox, oy + 8);
        ctx.stroke();

        ctx.restore();
    }

    drawBoundary() {
        const ctx = this.ctx;
        const halfF = this.stageForward / 2;
        const halfL = this.stageLateral / 2;

        const [x0, y0] = this.worldToCanvas(halfF, halfL);
        const [x1, y1] = this.worldToCanvas(-halfF, -halfL);

        ctx.save();
        ctx.strokeStyle = 'rgba(233,69,96,0.35)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([6, 4]);
        ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
        ctx.setLineDash([]);
        ctx.restore();
    }

    _drawLive(ctx) {
        // Draw live trail (green)
        if (this._liveTrail.length > 1) {
            ctx.beginPath();
            const [sx, sy] = this.worldToCanvas(this._liveTrail[0].x, this._liveTrail[0].y);
            ctx.moveTo(sx, sy);
            for (let i = 1; i < this._liveTrail.length; i++) {
                const [px, py] = this.worldToCanvas(this._liveTrail[i].x, this._liveTrail[i].y);
                ctx.lineTo(px, py);
            }
            ctx.strokeStyle = 'rgba(78, 201, 176, 0.6)';
            ctx.lineWidth = 2;
            ctx.stroke();
        }
        // Draw live robot (green triangle)
        if (this._livePos) {
            const [cx, cy] = this.worldToCanvas(this._livePos.x, this._livePos.y);
            const size = 0.15 * this.pixelsPerMeter;
            const canvasAngle = -this._livePos.theta - Math.PI;
            ctx.save();
            ctx.translate(cx, cy);
            ctx.rotate(canvasAngle);
            ctx.beginPath();
            ctx.moveTo(0, -size);
            ctx.lineTo(-size * 0.65, size * 0.6);
            ctx.lineTo(size * 0.65, size * 0.6);
            ctx.closePath();
            ctx.fillStyle = '#4ec9b0';
            ctx.fill();
            ctx.strokeStyle = 'rgba(255,255,255,0.5)';
            ctx.lineWidth = 1;
            ctx.stroke();
            ctx.restore();
        }
    }

}
