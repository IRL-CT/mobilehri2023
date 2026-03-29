/**
 * robot3d.js — Three.js 3D visualization using URDF loader.
 *
 * Loads the robot model from a URDF file via the urdf-loader library,
 * making it generalizable to any robot platform.
 */

import * as THREE from 'three';
import URDFLoader from 'urdf-loader';

export class Robot3DView {
    constructor(container, urdfPath = 'chair_robot.urdf') {
        this._container = container;
        this._urdfPath = urdfPath;
        this._scene = null;
        this._camera = null;
        this._renderer = null;
        this._robot = null;
        this._pathLine = null;
        this._stageWidth = 6.0;
        this._stageDepth = 4.0;
        this._loaded = false;

        this._init();
    }

    _init() {
        // Scene
        this._scene = new THREE.Scene();
        this._scene.background = new THREE.Color(0x1e1e1e);

        // Camera
        this._camera = new THREE.PerspectiveCamera(50, 1, 0.1, 100);
        this._camera.position.set(0, 4, 3);
        this._camera.lookAt(0, 0, 0);

        // Renderer
        this._renderer = new THREE.WebGLRenderer({ antialias: true });
        this._renderer.setPixelRatio(window.devicePixelRatio);
        this._container.appendChild(this._renderer.domElement);

        // Lighting
        const ambient = new THREE.AmbientLight(0x808080);
        this._scene.add(ambient);
        const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
        dirLight.position.set(3, 5, 3);
        this._scene.add(dirLight);
        const dirLight2 = new THREE.DirectionalLight(0x8888ff, 0.3);
        dirLight2.position.set(-3, 3, -2);
        this._scene.add(dirLight2);

        // Stage floor + grid + boundary
        this._createStage();

        // Robot container group (will hold URDF or fallback)
        this._robotGroup = new THREE.Group();
        this._scene.add(this._robotGroup);

        // Path trace
        const pathMat = new THREE.LineBasicMaterial({ color: 0x569cd6, linewidth: 2 });
        this._pathLine = new THREE.Line(new THREE.BufferGeometry(), pathMat);
        this._scene.add(this._pathLine);

        // Load URDF
        this._loadURDF();

        this.resize();
        this._setupControls();

        // Continuous render loop (needed for orbit controls to feel responsive)
        const animate = () => {
            requestAnimationFrame(animate);
            this._renderer.render(this._scene, this._camera);
        };
        animate();
    }

    async _loadURDF() {
        try {
            const loader = new URDFLoader();
            loader.packages = '';

            const res = await fetch(this._urdfPath);
            if (!res.ok) throw new Error(`Failed to fetch URDF: ${res.status}`);
            const urdfText = await res.text();

            this._robot = loader.parse(urdfText);
            // URDF is Z-up, Three.js is Y-up.
            // Also rotate so URDF +x (forward) aligns with Three.js -z (our forward).
            // Apply as: first rotate -90 around X (Z-up → Y-up), then -90 around Y (face -z).
            this._robot.rotation.set(-Math.PI / 2, -Math.PI / 2, 0, 'YXZ');
            this._applyMaterials(this._robot);
            this._robotGroup.add(this._robot);
            this._loaded = true;
            console.log('URDF loaded:', Object.keys(this._robot.joints).length, 'joints,',
                        Object.keys(this._robot.links).length, 'links');
            this.render();
        } catch (e) {
            console.warn('URDF loader failed, using fallback:', e.message);
            this._robot = this._createFallbackRobot();
            this._robotGroup.add(this._robot);
            this._loaded = true;
            this.render();
        }
    }

    _applyMaterials(urdfRoot) {
        // Walk all links/meshes and apply colors from URDF material names
        const colorMap = {
            'chair_mat':  0x3a3a3a,
            'wheel_mat':  0x555555,
            'caster_mat': 0x888888,
        };
        const defaultColor = 0x4a4a4a;

        urdfRoot.traverse((child) => {
            if (child.isMesh) {
                const matName = child.material && child.material.name;
                const color = colorMap[matName] || defaultColor;
                child.material = new THREE.MeshStandardMaterial({
                    color: color,
                    roughness: 0.7,
                    metalness: 0.3,
                });
            }
        });
    }

    _createFallbackRobot() {
        // Minimal fallback if URDF fails to load
        const group = new THREE.Group();
        const mat = new THREE.MeshStandardMaterial({ color: 0x555555 });

        // Simple box for the body
        const body = new THREE.Mesh(new THREE.BoxGeometry(0.4, 0.5, 0.4), mat);
        body.position.y = 0.3;
        group.add(body);

        // Direction arrow
        const arrow = new THREE.Mesh(
            new THREE.ConeGeometry(0.04, 0.08, 4),
            new THREE.MeshStandardMaterial({ color: 0x569cd6 })
        );
        arrow.rotation.x = -Math.PI / 2;
        arrow.position.set(0, 0.55, -0.15);
        group.add(arrow);

        return group;
    }

    _createStage() {
        // Floor
        const floorGeo = new THREE.PlaneGeometry(this._stageWidth, this._stageDepth);
        const floorMat = new THREE.MeshStandardMaterial({ color: 0x2a2a2a, roughness: 0.9 });
        const floor = new THREE.Mesh(floorGeo, floorMat);
        floor.rotation.x = -Math.PI / 2;
        this._scene.add(floor);

        // Grid
        const gridSize = Math.max(this._stageWidth, this._stageDepth);
        const grid = new THREE.GridHelper(gridSize, gridSize * 2, 0x444444, 0x333333);
        this._scene.add(grid);

        // Stage boundary
        const edgeGeo = new THREE.EdgesGeometry(
            new THREE.BoxGeometry(this._stageWidth, 0.01, this._stageDepth)
        );
        const edgeMat = new THREE.LineBasicMaterial({ color: 0x569cd6 });
        const edge = new THREE.LineSegments(edgeGeo, edgeMat);
        edge.position.y = 0.005;
        this._scene.add(edge);
    }

    _setupControls() {
        let isDragging = false;
        let prevX = 0, prevY = 0;
        let orbitAngle = 0;
        let orbitPitch = 0.8;
        let orbitDist = 5;

        const updateCamera = () => {
            this._camera.position.x = Math.sin(orbitAngle) * Math.cos(orbitPitch) * orbitDist;
            this._camera.position.y = Math.sin(orbitPitch) * orbitDist;
            this._camera.position.z = Math.cos(orbitAngle) * Math.cos(orbitPitch) * orbitDist;
            this._camera.lookAt(0, 0.2, 0);
        };

        this._container.addEventListener('mousedown', (e) => {
            isDragging = true;
            prevX = e.clientX;
            prevY = e.clientY;
        });
        window.addEventListener('mousemove', (e) => {
            if (!isDragging) return;
            orbitAngle += (e.clientX - prevX) * 0.01;
            orbitPitch = Math.max(0.1, Math.min(1.4, orbitPitch + (e.clientY - prevY) * 0.01));
            prevX = e.clientX;
            prevY = e.clientY;
            updateCamera();
        });
        window.addEventListener('mouseup', () => { isDragging = false; });
        this._container.addEventListener('wheel', (e) => {
            e.preventDefault();
            orbitDist = Math.max(2, Math.min(12, orbitDist + e.deltaY * 0.005));
            updateCamera();
        }, { passive: false });

        updateCamera();
    }

    setDimensions(width, depth) {
        this._stageWidth = width;
        this._stageDepth = depth;
    }

    resize() {
        const w = this._container.clientWidth;
        const h = this._container.clientHeight;
        if (w === 0 || h === 0) return;
        this._camera.aspect = w / h;
        this._camera.updateProjectionMatrix();
        this._renderer.setSize(w, h);
    }

    /**
     * Update robot position and render.
     * World coords: x=forward, y=left → Three.js: -z=forward, -x=left
     */
    update(frame, pathFrames) {
        if (!frame || !this._loaded) return;

        // Position the robot group
        this._robotGroup.position.set(-frame.y, 0, -frame.x);
        this._robotGroup.rotation.y = frame.theta;

        // Update path trace
        if (pathFrames && pathFrames.length > 1) {
            const points = pathFrames.map(f => new THREE.Vector3(-f.y, 0.01, -f.x));
            this._pathLine.geometry.dispose();
            this._pathLine.geometry = new THREE.BufferGeometry().setFromPoints(points);
        }

        this._renderer.render(this._scene, this._camera);
    }

    render() {
        this._renderer.render(this._scene, this._camera);
    }
}
