import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import * as CANNON from 'cannon-es';

// --- Configuration ---
const config = {
    frameCount: 6,
    frameSize: 1.5,      // meters
    beamThickness: 0.04, // meters 
    damping: 0.1,        // Reduced default damping
    connectorLength: 0.2,
    baseWeight: 500,     // Fixed internal base weight
    weightReduction: 0.65, // Optimized stability point
    weightScale: 0.64,
    
    // CHEAT FACTORS
    frameDensity: 100,   
    weightDensity: 100000, 
};

// --- Globals ---
let scene, camera, renderer, controls;
let world;
let meshes = [];
let bodies = [];
let constraints = [];
let lastCallTime;
let dragPlane;

// Materials
const matAluminum = new THREE.MeshStandardMaterial({ 
    color: 0xcccccc, 
    roughness: 0.4, 
    metalness: 0.8 
});
const matSteel = new THREE.MeshStandardMaterial({ 
    color: 0x3e2723, 
    roughness: 0.7, 
    metalness: 0.4
});
const matConnector = new THREE.MeshStandardMaterial({
    color: 0x888888,
    roughness: 0.5,
    metalness: 0.5
});
const matWeightConn = new THREE.MeshStandardMaterial({
    color: 0x2e7d32, 
    roughness: 0.5,
    metalness: 0.5
});

// Physics Materials
const physMatAluminum = new CANNON.Material('aluminum');
const physMatGround = new CANNON.Material('ground');
const contactMat = new CANNON.ContactMaterial(physMatGround, physMatAluminum, {
    friction: 0.5,
    restitution: 0.1
});

// Collision Groups
const GROUP_FRAME = 1;
const GROUP_WEIGHT = 2;
const GROUP_GROUND = 4;

// --- Initialization ---
function init() {
    const container = document.getElementById('canvas-container');
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x87CEEB);
    scene.fog = new THREE.Fog(0x87CEEB, 10, 100);

    camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.1, 100);
    // Isometric-ish view: Up and Back
    camera.position.set(-10, 10, 10);
    camera.lookAt(0, 0, 0);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    // Shadows disabled
    renderer.shadowMap.enabled = false; 
    container.appendChild(renderer.domElement);

    // Lights
    const ambLight = new THREE.AmbientLight(0x404040, 2);
    scene.add(ambLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 5);
    dirLight.position.set(-15, 20, 10);
    dirLight.castShadow = false; // Disabled
    scene.add(dirLight);

    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.target.set(0, 6, 0);
    controls.enablePan = false; // Disable pan for simpler mobile interaction
    controls.touches = {
        ONE: THREE.TOUCH.ROTATE,
        TWO: THREE.TOUCH.DOLLY_PAN
    }; 

    world = new CANNON.World();
    world.gravity.set(0, -9.82, 0);
    world.broadphase = new CANNON.SAPBroadphase(world);
    world.solver.iterations = 50; 
    world.addContactMaterial(contactMat);

    // Ground
    const groundGeo = new THREE.PlaneGeometry(50, 50);
    const groundMat = new THREE.MeshStandardMaterial({ color: 0x455A64 });
    const groundMesh = new THREE.Mesh(groundGeo, groundMat);
    groundMesh.rotation.x = -Math.PI / 2;
    groundMesh.receiveShadow = false;
    scene.add(groundMesh);

    const groundBody = new CANNON.Body({
        mass: 0,
        shape: new CANNON.Plane(),
        material: physMatGround,
        collisionFilterGroup: GROUP_GROUND,
        collisionFilterMask: GROUP_FRAME | GROUP_WEIGHT
    });
    groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(groundBody);

    setupDragControls();
    rebuildSculpture();
    setupUI();

    lastCallTime = performance.now();
    animate();
}

function createCompositeBody(yPivot, frameSize, weightScaleFactor, isStatic, weightIndex) {
    const thickness = config.beamThickness;
    const halfSize = frameSize / 2;
    const connHeight = config.connectorLength;
    
    // --- VISUAL SIZE ---
    const frameInner = frameSize - 2*thickness;
    const baseVisualSize = frameInner * config.weightScale;
    const idx = Math.max(0, weightIndex);
    let weightSide = baseVisualSize * Math.pow(config.weightReduction, idx);
    
    if (weightSide > frameInner - 0.05) { 
        weightSide = frameInner - 0.05;
    }
    
    // --- MASS CALCULATIONS ---
    const frameVolume = 4 * frameSize * thickness * thickness;
    const frameMass = frameVolume * config.frameDensity;
    
    const weightVolume = Math.pow(weightSide, 3);
    let weightMass = weightVolume * config.weightDensity;

    let totalMass = frameMass;
    if (weightIndex >= 0) totalMass += weightMass;

    // --- CoM Calculation ---
    const frameCenterY = connHeight/2 + frameSize/2 - thickness/2;
    const weightCenterY = -frameSize/2; 
    
    let comY = frameMass * frameCenterY;
    if (weightIndex >= 0) {
        comY += weightMass * weightCenterY;
    }
    comY /= totalMass;
    
    const comOffset = new CANNON.Vec3(0, comY, 0);

    const body = new CANNON.Body({
        mass: isStatic ? 0 : totalMass,
        material: physMatAluminum,
        position: new CANNON.Vec3(0, yPivot + comY, 0),
        linearDamping: 0.01,
        angularDamping: config.damping,
        collisionFilterGroup: GROUP_FRAME,
        collisionFilterMask: GROUP_FRAME | GROUP_GROUND
    });
    
    function addShape(shape, pivotRelPos) {
        const offset = new CANNON.Vec3().copy(pivotRelPos).vsub(comOffset);
        body.addShape(shape, offset);
    }

    // Frame Shapes
    addShape(new CANNON.Box(new CANNON.Vec3(halfSize, thickness/2, thickness/2)), new CANNON.Vec3(0, frameCenterY + halfSize - thickness/2, 0)); 
    addShape(new CANNON.Box(new CANNON.Vec3(halfSize, thickness/2, thickness/2)), new CANNON.Vec3(0, frameCenterY - halfSize + thickness/2, 0)); 
    addShape(new CANNON.Box(new CANNON.Vec3(thickness/2, halfSize - thickness, thickness/2)), new CANNON.Vec3(-halfSize + thickness/2, frameCenterY, 0)); 
    addShape(new CANNON.Box(new CANNON.Vec3(thickness/2, halfSize - thickness, thickness/2)), new CANNON.Vec3(halfSize - thickness/2, frameCenterY, 0)); 
    
    // Connector UP
    addShape(new CANNON.Box(new CANNON.Vec3(thickness/2, connHeight/4, thickness/2)), new CANNON.Vec3(0, connHeight/4, 0));

    // Weight Shapes
    if (weightIndex >= 0) {
        addShape(new CANNON.Box(new CANNON.Vec3(weightSide/2, weightSide/2, weightSide/2)), new CANNON.Vec3(0, weightCenterY, 0));
        
        const weightTopY = weightCenterY + weightSide/2;
        const connLen = -weightTopY;
        if (connLen > 0) {
            addShape(new CANNON.Box(new CANNON.Vec3(thickness/4, connLen/2, thickness/4)), new CANNON.Vec3(0, weightTopY + connLen/2, 0));
        }
    }

    world.addBody(body);
    
    // --- Visual Mesh ---
    const group = new THREE.Group();
    
    function createBox(w, h, d, mat, pivotRelX, pivotRelY, pivotRelZ) {
        const geo = new THREE.BoxGeometry(w, h, d);
        const mesh = new THREE.Mesh(geo, mat);
        mesh.position.set(pivotRelX, pivotRelY - comY, pivotRelZ);
        mesh.castShadow = false;
        mesh.receiveShadow = false;
        return mesh;
    }

    group.add(createBox(frameSize, thickness, thickness, matAluminum, 0, frameCenterY + halfSize - thickness/2, 0)); 
    group.add(createBox(frameSize, thickness, thickness, matAluminum, 0, frameCenterY - halfSize + thickness/2, 0)); 
    group.add(createBox(thickness, frameSize - 2*thickness, thickness, matAluminum, -halfSize + thickness/2, frameCenterY, 0)); 
    group.add(createBox(thickness, frameSize - 2*thickness, thickness, matAluminum, halfSize - thickness/2, frameCenterY, 0)); 
    
    group.add(createBox(thickness, connHeight/2, thickness, matConnector, 0, connHeight/4, 0));

    if (weightIndex >= 0) {
        group.add(createBox(weightSide, weightSide, weightSide, matSteel, 0, weightCenterY, 0));
        
        const weightTopY = weightCenterY + weightSide/2;
        const connLen = -weightTopY;
        if (connLen > 0) {
            group.add(createBox(thickness/2, connLen, thickness/2, matWeightConn, 0, weightTopY + connLen/2, 0));
        }
    }

    scene.add(group);
    
    return { body, mesh: group, comY };
}

function rebuildSculpture() {
    bodies.forEach(b => world.removeBody(b));
    meshes.forEach(m => scene.remove(m));
    constraints.forEach(c => world.removeConstraint(c));
    bodies = [];
    meshes = [];
    constraints = [];

    const pedestalHeight = 2.0;
    const thick = config.beamThickness;
    
    // 1. Base Pillar
    const baseBody = new CANNON.Body({
        mass: 0,
        position: new CANNON.Vec3(0, pedestalHeight/2, 0),
        material: physMatAluminum
    });
    baseBody.addShape(new CANNON.Box(new CANNON.Vec3(0.2, pedestalHeight/2, 0.2)));
    world.addBody(baseBody);
    bodies.push(baseBody);

    const baseMesh = new THREE.Mesh(new THREE.BoxGeometry(0.4, pedestalHeight, 0.4), new THREE.MeshStandardMaterial({ color: 0x333333 }));
    baseMesh.position.y = pedestalHeight/2;
    baseMesh.castShadow = false;
    scene.add(baseMesh);
    meshes.push(baseMesh);

    // 2. Frame 0
    const halfSize = config.frameSize/2;
    const frame0CenterY = pedestalHeight + halfSize;
    const frame0TopBeamY = frame0CenterY + halfSize - thick/2;

    const f0Body = new CANNON.Body({ mass: 0, position: new CANNON.Vec3(0, frame0CenterY, 0) });
    f0Body.addShape(new CANNON.Box(new CANNON.Vec3(halfSize, thick/2, thick/2)), new CANNON.Vec3(0, halfSize - thick/2, 0)); 
    f0Body.addShape(new CANNON.Box(new CANNON.Vec3(halfSize, thick/2, thick/2)), new CANNON.Vec3(0, -halfSize + thick/2, 0)); 
    f0Body.addShape(new CANNON.Box(new CANNON.Vec3(thick/2, halfSize - thick, thick/2)), new CANNON.Vec3(-halfSize + thick/2, 0, 0)); 
    f0Body.addShape(new CANNON.Box(new CANNON.Vec3(thick/2, halfSize - thick, thick/2)), new CANNON.Vec3(halfSize - thick/2, 0, 0)); 
    world.addBody(f0Body);
    bodies.push(f0Body);
    
    const f0Group = new THREE.Group();
    f0Group.position.y = frame0CenterY;
    function addBox(w,h,d,mat,y) {
        const m = new THREE.Mesh(new THREE.BoxGeometry(w,h,d), mat);
        m.position.y = y; m.castShadow=false; f0Group.add(m);
        return m;
    }
    function addBoxX(w,h,d,mat,x) {
        const m = new THREE.Mesh(new THREE.BoxGeometry(w,h,d), mat);
        m.position.x = x; m.castShadow=false; f0Group.add(m);
    }
    addBox(config.frameSize, thick, thick, matAluminum, halfSize - thick/2);
    addBox(config.frameSize, thick, thick, matAluminum, -halfSize + thick/2);
    addBoxX(thick, config.frameSize - 2*thick, thick, matAluminum, -halfSize + thick/2);
    addBoxX(thick, config.frameSize - 2*thick, thick, matAluminum, halfSize - thick/2);
    scene.add(f0Group);
    meshes.push(f0Group);

    let prevBody = f0Body;
    let prevPivotLocal = new CANNON.Vec3(0, halfSize - thick/2, 0);
    
    let currentPivotY = frame0TopBeamY; 

    // 3. Moving Bodies (1 to N)
    for (let i = 1; i < config.frameCount; i++) {
        const weightIndex = i - 1;
        
        const obj = createCompositeBody(currentPivotY, config.frameSize, 1.0, false, weightIndex);
        bodies.push(obj.body);
        meshes.push(obj.mesh);
        
        const pivotB = new CANNON.Vec3(0, -obj.comY, 0);
        
        const c = new CANNON.HingeConstraint(prevBody, obj.body, {
            pivotA: prevPivotLocal,
            pivotB: pivotB,
            axisA: new CANNON.Vec3(1, 0, 0),
            axisB: new CANNON.Vec3(1, 0, 0),
            collideConnected: false
        });
        c.enableMotor = false;
        c.limits = [-Math.PI/1.2, Math.PI/1.2];
        
        world.addConstraint(c);
        constraints.push(c);
        
        prevBody = obj.body;
        const distToNextPivot = config.connectorLength/2 + config.frameSize/2 - config.beamThickness/2 + halfSize - config.beamThickness/2;
        currentPivotY += distToNextPivot;
        prevPivotLocal = new CANNON.Vec3(0, distToNextPivot - obj.comY, 0);
    }

    // 4. Top Cap
    const capIndex = config.frameCount - 1;
    const frameInner = config.frameSize - 2*thick;
    const baseVisualSize = frameInner * config.weightScale;
    let weightSide = baseVisualSize * Math.pow(config.weightReduction, capIndex);
    if (weightSide > frameInner - 0.05) weightSide = frameInner - 0.05;
    
    const capWeightMass = Math.pow(weightSide, 3) * config.weightDensity;
    const weightCenterY = -config.frameSize/2;
    const capComY = weightCenterY;
    
    const capBody = new CANNON.Body({
        mass: capWeightMass,
        material: physMatAluminum,
        position: new CANNON.Vec3(0, currentPivotY + capComY, 0),
        linearDamping: 0.01,
        angularDamping: config.damping,
        collisionFilterGroup: GROUP_FRAME,
        collisionFilterMask: GROUP_FRAME | GROUP_GROUND
    });

    capBody.addShape(new CANNON.Box(new CANNON.Vec3(weightSide/2, weightSide/2, weightSide/2)), new CANNON.Vec3(0, 0, 0));
    
    const weightTopY = weightCenterY + weightSide/2;
    const connLen = -weightTopY;
    if (connLen > 0) {
        const y = weightTopY + connLen/2 - weightCenterY;
        capBody.addShape(new CANNON.Box(new CANNON.Vec3(thick/4, connLen/2, thick/4)), new CANNON.Vec3(0, y, 0));
    }
    
    const vaneY = thick*2 - weightCenterY;
    capBody.addShape(new CANNON.Box(new CANNON.Vec3(thick, thick*2, thick)), new CANNON.Vec3(0, vaneY, 0)); 

    world.addBody(capBody);
    bodies.push(capBody);

    const capGroup = new THREE.Group();
    function createBox(w, h, d, mat, y) {
        const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
        m.position.y = y - capComY;
        m.castShadow=false; capGroup.add(m);
    }
    createBox(weightSide, weightSide, weightSide, matSteel, weightCenterY);
    if (connLen > 0) {
        createBox(thick/2, connLen, thick/2, matWeightConn, weightTopY + connLen/2);
    }
    createBox(thick*2, thick*4, thick*2, matAluminum, thick*2);
    scene.add(capGroup);
    meshes.push(capGroup);
    
    const cCap = new CANNON.HingeConstraint(prevBody, capBody, {
        pivotA: prevPivotLocal,
        pivotB: new CANNON.Vec3(0, -capComY, 0),
        axisA: new CANNON.Vec3(1, 0, 0),
        axisB: new CANNON.Vec3(1, 0, 0),
        collideConnected: false
    });
    world.addConstraint(cCap);
    constraints.push(cCap);
}

function createBox(w, h, d, mat, x, y, z) {
    const geo = new THREE.BoxGeometry(w, h, d);
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(x, y, z);
    mesh.castShadow = false;
    mesh.receiveShadow = false;
    return mesh;
}

function setupDragControls() {
    const raycaster = new THREE.Raycaster();
    const mouse = new THREE.Vector2();
    const planeGeo = new THREE.PlaneGeometry(100, 100);
    const planeMat = new THREE.MeshBasicMaterial({ visible: false });
    dragPlane = new THREE.Mesh(planeGeo, planeMat);
    scene.add(dragPlane);

    let isDragging = false;
    let jointBody, constrainedBody, mouseConstraint;
    let touchId = null;

    function getRayIntersection(clientX, clientY) {
        const rect = renderer.domElement.getBoundingClientRect();
        mouse.x = ((clientX - rect.left) / rect.width) * 2 - 1;
        mouse.y = -((clientY - rect.top) / rect.height) * 2 + 1;
        raycaster.setFromCamera(mouse, camera);
        return raycaster.intersectObjects(meshes, true);
    }

    function startDrag(clientX, clientY) {
        const intersects = getRayIntersection(clientX, clientY);
        if (intersects.length > 0) {
            controls.enabled = false;
            isDragging = true;
            let group = intersects[0].object;
            while(group.parent && group.parent.type !== 'Scene') group = group.parent;
            const index = meshes.indexOf(group);
            if (index > 1) {
                constrainedBody = bodies[index];
                const point = intersects[0].point;
                const localPoint = new CANNON.Vec3(point.x, point.y, point.z).vsub(constrainedBody.position);
                jointBody = new CANNON.Body({ mass: 0 });
                jointBody.position.set(point.x, point.y, point.z);
                world.addBody(jointBody);
                mouseConstraint = new CANNON.PointToPointConstraint(constrainedBody, localPoint, jointBody, new CANNON.Vec3(0,0,0));
                world.addConstraint(mouseConstraint);
                dragPlane.position.copy(point);
                dragPlane.quaternion.copy(camera.quaternion);
                return true;
            }
        }
        return false;
    }

    function updateDrag(clientX, clientY) {
        if (!isDragging) return;
        const rect = renderer.domElement.getBoundingClientRect();
        mouse.x = ((clientX - rect.left) / rect.width) * 2 - 1;
        mouse.y = -((clientY - rect.top) / rect.height) * 2 + 1;
        raycaster.setFromCamera(mouse, camera);
        const intersects = raycaster.intersectObject(dragPlane);
        if (intersects.length > 0 && jointBody) {
            jointBody.position.set(intersects[0].point.x, intersects[0].point.y, intersects[0].point.z);
        }
    }

    function endDrag() {
        controls.enabled = true;
        isDragging = false;
        touchId = null;
        if (mouseConstraint) { world.removeConstraint(mouseConstraint); mouseConstraint = null; }
        if (jointBody) { world.removeBody(jointBody); jointBody = null; }
    }

    // Mouse events
    window.addEventListener('mousedown', (e) => {
        if (e.button !== 0) return;
        startDrag(e.clientX, e.clientY);
    });

    window.addEventListener('mousemove', (e) => {
        updateDrag(e.clientX, e.clientY);
    });

    window.addEventListener('mouseup', () => {
        endDrag();
    });

    // Touch events
    window.addEventListener('touchstart', (e) => {
        if (e.touches.length === 1 && touchId === null) {
            const touch = e.touches[0];
            const started = startDrag(touch.clientX, touch.clientY);
            if (started) {
                touchId = touch.identifier;
                e.preventDefault();
            }
        }
    }, { passive: false });

    window.addEventListener('touchmove', (e) => {
        if (touchId !== null && isDragging) {
            const touch = Array.from(e.touches).find(t => t.identifier === touchId);
            if (touch) {
                updateDrag(touch.clientX, touch.clientY);
                e.preventDefault();
            }
        }
    }, { passive: false });

    window.addEventListener('touchend', (e) => {
        if (touchId !== null) {
            const touch = Array.from(e.changedTouches).find(t => t.identifier === touchId);
            if (touch) {
                endDrag();
                e.preventDefault();
            }
        }
    }, { passive: false });

    window.addEventListener('touchcancel', (e) => {
        if (touchId !== null) {
            endDrag();
            e.preventDefault();
        }
    }, { passive: false });
}

function setupUI() {
    const inputs = [
        { id: 'frameCount', key: 'frameCount', needsRebuild: true },
        { id: 'frameSize', key: 'frameSize', needsRebuild: true },
        { id: 'weightReduction', key: 'weightReduction', needsRebuild: true },
        { id: 'weightScale', key: 'weightScale', needsRebuild: true },
        { id: 'damping', key: 'damping', needsRebuild: true }
    ];

    inputs.forEach(item => {
        const el = document.getElementById(item.id);
        if (!el) return;
        const valEl = document.getElementById(item.id + 'Val');
        
        el.addEventListener('input', (e) => {
            const val = parseFloat(e.target.value);
            config[item.key] = val;
            if(valEl) valEl.textContent = val;
            if (item.needsRebuild) rebuildSculpture();
        });
    });

    document.getElementById('resetBtn').addEventListener('click', rebuildSculpture);
    
    // Controls toggle
    const controlsToggle = document.getElementById('controlsToggle');
    const controls = document.getElementById('controls');
    
    if (controlsToggle && controls) {
        const icon = controlsToggle.querySelector('.toggle-icon');
        
        // Update icon based on initial state (controls visible by default)
        if (icon && !controls.classList.contains('controls-collapsed')) {
            icon.textContent = '✕';
        }
        
        controlsToggle.addEventListener('click', () => {
            controls.classList.toggle('controls-collapsed');
            if (icon) {
                icon.textContent = controls.classList.contains('controls-collapsed') ? '☰' : '✕';
            }
        });
    }
    
    window.addEventListener('resize', () => {
        const container = document.getElementById('canvas-container');
        camera.aspect = container.clientWidth / container.clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(container.clientWidth, container.clientHeight);
    });
}

function animate() {
    requestAnimationFrame(animate);
    const now = performance.now();
    const dt = (now - lastCallTime) / 1000;
    lastCallTime = now;

    world.step(1/60, dt, 5);

    for (let i = 0; i < meshes.length; i++) {
        const b = bodies[i];
        const m = meshes[i];
        m.position.copy(b.position);
        m.quaternion.copy(b.quaternion);
    }

    controls.update();
    renderer.render(scene, camera);
}

window.onload = init;