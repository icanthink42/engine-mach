// Get canvas and context
const canvas = document.getElementById('particleCanvas');
const ctx = canvas.getContext('2d');

// Physical dimensions
const PHYSICAL_WIDTH_METERS = 5; // Width of the simulation in meters

// Initialize walls at the top level
let topWall;
let bottomWall;

// Shock detection
const shockPoints = new Map(); // Map of x positions to arrays of {type, timestamp} events
const SHOCK_WINDOW = 100; // Number of events to track per position
const SHOCK_THRESHOLD = 1; // Number of consistent transitions needed to draw shock
const SHOCK_DISTANCE = metersToPixels(0.1); // 10cm in pixels for grouping
const SHOCK_LIFETIME = 300; // Shock lifetime in milliseconds
const SPAWN_INTERVAL = 0.01; // Base spawn interval in milliseconds

// Convert between physical and screen coordinates
function metersToPixels(meters) {
    return (meters / PHYSICAL_WIDTH_METERS) * canvas.width;
}

function pixelsToMeters(pixels) {
    return (pixels / canvas.width) * PHYSICAL_WIDTH_METERS;
}

// Flow control variables
let soundSpeed = 343; // Default speed of sound in air at room temperature
let initialVelocity = 100;
let timeScale = 0.01; // Default 1% time scale (0.1% to 2% range)

// Get control elements
const soundSpeedSlider = document.getElementById('soundSpeed');
const velocitySlider = document.getElementById('initialVelocity');
const timeScaleSlider = document.getElementById('timeScale');
const soundValue = document.getElementById('soundValue');
const velocityValue = document.getElementById('velocityValue');
const timeScaleValue = document.getElementById('timeScaleValue');

// Update control values
soundSpeedSlider.addEventListener('input', (e) => {
    soundSpeed = parseFloat(e.target.value);
    soundValue.textContent = soundSpeed.toFixed(0);
});

velocitySlider.addEventListener('input', (e) => {
    initialVelocity = parseFloat(e.target.value);
    velocityValue.textContent = initialVelocity.toFixed(0);
});

timeScaleSlider.addEventListener('input', (e) => {
    timeScale = parseFloat(e.target.value) / 100; // Convert percentage to decimal
    timeScaleValue.textContent = e.target.value;
});

// Set canvas size to window size
function resizeCanvas() {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight;
    // Reinitialize control points when canvas is resized
    if (topWall) {
        topWall.initializePoints();
        bottomWall.initializePoints();
    }
}

// calculate velocity based purely on the compressible flow equation (m² - 1)dv/v = da/a
function calculateVelocity(x1, x2, y1, y2, V1) {
    // Convert pixel coordinates to meters and calculate circular areas
    const y2_meters = pixelsToMeters(y2);
    const y2_next_meters = pixelsToMeters(bottomWall.getYAtX(x2));

    // Calculate areas in square meters (assuming circular cross-section)
    const A1 = y2_meters * y2_meters * Math.PI;
    const A2 = y2_next_meters * y2_next_meters * Math.PI;

    const dA = A2 - A1;
    const dA_A = dA / A1;

    // Calculate current Mach number from velocity
    const M = V1 / soundSpeed;
    const one_minus_M1 = 1 - M * M;

    // Near Mach 1, avoid division by zero
    if (Math.abs(one_minus_M1) < 0.01) {
        return initialVelocity;
    }

    const dV = -dA_A * V1 / one_minus_M1;
    return V1 + dV * 0.05;
}

// Spline class for wall generation
class Spline {
    constructor(isTop) {
        this.isTop = isTop;
        this.dragging = false;
        this.dragPointIndex = -1;
        this.initializePoints();
        // Array to store velocity measurements for each control point
        this.velocityMeasurements = this.points ? this.points.map(() => []) : [];
        this.splineCoeffs = null;
    }

    initializePoints() {
        this.points = [];
        const numPoints = 5;
        const baseY = this.isTop ? canvas.height * 0.2 : canvas.height * 0.8;

        for (let i = 0; i < numPoints; i++) {
            const x = (canvas.width / (numPoints - 1)) * i;
            const y = baseY + Math.sin(i * Math.PI / 2) * 20;
            this.points.push({ x, y });
        }

        // Reset velocity measurements and spline
        this.velocityMeasurements = this.points.map(() => []);
        this.initializeSpline();
    }

    initializeSpline() {
        // Sort points by x coordinate
        this.points.sort((a, b) => a.x - b.x);

        // Extract x and y coordinates
        const xs = this.points.map(p => p.x);
        const ys = this.points.map(p => p.y);

        // Create cubic spline interpolation
        this.splineCoeffs = numeric.spline(xs, ys);
    }

    // Calculate average velocity at a control point
    getAverageVelocity(pointIndex) {
        const measurements = this.velocityMeasurements[pointIndex];
        if (measurements.length === 0) return 0;

        const sum = measurements.reduce((a, b) => a + b, 0);
        return sum / measurements.length;
    }

    // Record velocity measurement for a point
    recordVelocity(x, velocity) {
        // Find the closest control point
        let closestIndex = 0;
        let closestDist = Math.abs(this.points[0].x - x);

        for (let i = 1; i < this.points.length; i++) {
            const dist = Math.abs(this.points[i].x - x);
            if (dist < closestDist) {
                closestDist = dist;
                closestIndex = i;
            }
        }

        // Only record if particle is close enough to the control point
        if (closestDist < 50) {
            const measurements = this.velocityMeasurements[closestIndex];
            measurements.push(velocity);
            // Keep only last 50 measurements
            if (measurements.length > 50) {
                measurements.shift();
            }
        }
    }

    getYAtX(x) {
        // Initialize spline if needed
        if (!this.splineCoeffs) {
            this.initializeSpline();
        }

        // Handle out of bounds
        if (x <= this.points[0].x) return this.points[0].y;
        if (x >= this.points[this.points.length - 1].x) return this.points[this.points.length - 1].y;

        // Use cubic spline interpolation
        return this.splineCoeffs.at(x);
    }

    // Get spline points using Catmull-Rom interpolation
    getSplinePoints() {
        // Initialize spline if needed
        if (!this.splineCoeffs) {
            this.initializeSpline();
        }

        const points = [];
        const numPoints = 200; // Higher resolution for smoother curve

        // Get x range
        const xStart = this.points[0].x;
        const xEnd = this.points[this.points.length - 1].x;
        const dx = (xEnd - xStart) / (numPoints - 1);

        // Generate points along the spline
        for (let i = 0; i < numPoints; i++) {
            const x = xStart + i * dx;
            const y = this.splineCoeffs.at(x);
            points.push({ x, y });
        }

        return points;
    }

    draw() {
        // Draw spline
        const splinePoints = this.getSplinePoints();
        ctx.beginPath();
        ctx.moveTo(splinePoints[0].x, splinePoints[0].y);
        for (let i = 1; i < splinePoints.length; i++) {
            ctx.lineTo(splinePoints[i].x, splinePoints[i].y);
        }
        ctx.strokeStyle = 'white';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Draw control points and velocity measurements
        this.points.forEach((point, i) => {
            // Draw control point
            ctx.beginPath();
            ctx.arc(point.x, point.y, 5, 0, Math.PI * 2);
            ctx.fillStyle = 'red';
            ctx.fill();

            // Draw velocity measurement
            const avgVel = this.getAverageVelocity(i);
            if (avgVel > 0) {
                ctx.font = '12px Arial';
                ctx.fillStyle = 'white';
                ctx.textAlign = 'center';
                const displayVel = avgVel.toFixed(0);
                const yOffset = this.isTop ? -15 : 25; // Position text above/below point
                ctx.fillText(`${displayVel} m/s`, point.x, point.y + yOffset);
            }
        });
    }

    handleMouseDown(e) {
        const rect = canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;

        this.points.forEach((point, i) => {
            const dx = point.x - mouseX;
            const dy = point.y - mouseY;
            if (dx * dx + dy * dy < 100) {
                this.dragging = true;
                this.dragPointIndex = i;
            }
        });
    }

    handleMouseMove(e) {
        if (!this.dragging) return;

        const rect = canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;

        // Update point position
        this.points[this.dragPointIndex].x = mouseX;
        this.points[this.dragPointIndex].y = mouseY;

        // Update mirrored wall's corresponding point
        const mirroredWall = this.isTop ? bottomWall : topWall;
        const mirrorY = canvas.height - mouseY;
        mirroredWall.points[this.dragPointIndex].x = mouseX;
        mirroredWall.points[this.dragPointIndex].y = mirrorY;

        // Reinitialize splines after point movement
        this.initializeSpline();
        mirroredWall.initializeSpline();
    }

    handleMouseUp() {
        this.dragging = false;
        this.dragPointIndex = -1;
    }
}

// Particle class
class Particle {
    constructor() {
        this.reset();
    }

    initialSpawn() {
        this.size = Math.random() * 2 + 1;
        // Spread particles across the entire width initially
        this.x = Math.random() * canvas.width;
        // Fixed normalized position between walls (0 = top wall, 1 = bottom wall)
        this.normalizedY = Math.random();
        this.setupVelocity();
    }

    reset() {
        this.size = Math.random() * 2 + 1;
        // Reset to left side
        this.x = 0;
        // Keep same normalized Y position between walls
        if (!this.hasOwnProperty('normalizedY')) {
            this.normalizedY = Math.random();
        }
        this.setupVelocity();
    }

    setupVelocity() {
        this.speedX = initialVelocity;
        this.opacity = Math.random() * 0.5 + 0.5;
        this.updateColor();
    }

    updateColor() {
        // Calculate local Mach number based on velocity relative to sound speed
        const localMach = this.speedX / soundSpeed;

        // Red for supersonic, white for subsonic
        if (localMach > 1) {
            this.color = `rgba(255, 50, 50, ${this.opacity})`;
        } else {
            this.color = `rgba(255, 255, 255, ${this.opacity})`;
        }
    }

    update() {
        if (topWall && bottomWall) {
            const lookAheadMeters = 0.1; // Look ahead 10cm
            const lookAheadPixels = metersToPixels(lookAheadMeters);
            const topY = topWall.getYAtX(this.x);
            const bottomY = bottomWall.getYAtX(this.x);

            // Calculate actual Y position based on normalized position
            this.y = topY + (this.normalizedY * (bottomY - topY));

            // Store previous Mach state
            const prevMach = this.speedX / soundSpeed;

            // Update speedX based on the compressible flow equation (in m/s)
            this.speedX = calculateVelocity(this.x, this.x + lookAheadPixels, topY, bottomY, this.speedX);

            // Check for Mach transition
            const newMach = this.speedX / soundSpeed;
            if ((prevMach < 1 && newMach >= 1) || (prevMach >= 1 && newMach < 1)) {
                console.log(`Mach transition at x=${this.x}, prevMach=${prevMach}, newMach=${newMach}`);
                // Round x to nearest SHOCK_DISTANCE
                const roundedX = Math.round(this.x / SHOCK_DISTANCE) * SHOCK_DISTANCE;

                // Get or create transition array for this position
                if (!shockPoints.has(roundedX)) {
                    shockPoints.set(roundedX, []);
                }
                const transitions = shockPoints.get(roundedX);

                // Add transition type and timestamp
                transitions.push({
                    type: newMach >= 1 ? 1 : -1,
                    timestamp: performance.now()
                });

                // Keep only last SHOCK_WINDOW events and those within lifetime
                const now = performance.now();
                const updatedTransitions = transitions.filter(t =>
                    now - t.timestamp < SHOCK_LIFETIME
                ).slice(-SHOCK_WINDOW);
                shockPoints.set(roundedX, updatedTransitions);
            }

            // Convert velocity from m/s to pixels/frame
            const dt = (1/60) * timeScale; // Scale time step by time scale factor
            const dxMeters = this.speedX * dt;
            const dxPixels = metersToPixels(dxMeters);

            // Record velocity for both walls
            topWall.recordVelocity(this.x, this.speedX);
            bottomWall.recordVelocity(this.x, this.speedX);

            // Update particle color based on local velocity
            this.updateColor();

            // Update x position
            this.x += dxPixels;
        } else {
            // If no walls, use default movement
            const dt = (1/60) * timeScale;
            const dxMeters = this.speedX * dt;
            const dxPixels = metersToPixels(dxMeters);
            this.x += dxPixels;
        }
    }

    draw() {
        ctx.beginPath();
        ctx.fillStyle = this.color;
        ctx.arc(this.x, this.y, this.size, 0, Math.PI * 2);
        ctx.fill();
    }
}

// Initialize everything
function init() {
    // Create walls
    topWall = new Spline(true);
    bottomWall = new Spline(false);

    // Event listeners for wall interaction
    canvas.addEventListener('mousedown', (e) => {
        topWall.handleMouseDown(e);
        bottomWall.handleMouseDown(e);
    });

    canvas.addEventListener('mousemove', (e) => {
        topWall.handleMouseMove(e);
        bottomWall.handleMouseMove(e);
    });

    canvas.addEventListener('mouseup', () => {
        topWall.handleMouseUp();
        bottomWall.handleMouseUp();
    });

    // Create particle array
    let particles = [];

    // Spawn rate control
    let lastSpawnTime = 0;

    // Animation loop
    function animate(timestamp) {
        // Scale spawn interval inversely with time scale
        const spawnInterval = SPAWN_INTERVAL / timeScale;

        // Spawn new particles at time-scaled rate
        if (timestamp - lastSpawnTime > spawnInterval) {
            const p = new Particle();
            p.reset(); // Start from left side
            particles.push(p);
            lastSpawnTime = timestamp;
        }

        ctx.fillStyle = 'rgba(0, 0, 0, 0.1)';
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        // Draw walls
        topWall.draw();
        bottomWall.draw();

        // Draw shocks
        ctx.lineWidth = 3;
        const now = performance.now();

        // Clean up old shock points
        for (const [x, transitions] of shockPoints.entries()) {
            if (transitions.length === 0 ||
                now - transitions[transitions.length - 1].timestamp >= SHOCK_LIFETIME) {
                shockPoints.delete(x);
            }
        }

        for (const [x, transitions] of shockPoints.entries()) {
            // Count consistent transitions
            const superCount = transitions.filter(t => t.type === 1).length;
            const subCount = transitions.filter(t => t.type === -1).length;

            // Draw shock if enough consistent transitions in either direction
            console.log(`Shock point at x=${x}: super=${superCount}, sub=${subCount}`);
            if (superCount >= SHOCK_THRESHOLD || subCount >= SHOCK_THRESHOLD) {
                const topY = topWall.getYAtX(x);
                const bottomY = bottomWall.getYAtX(x);

                // Calculate opacity based on most recent transition
                const latestTime = Math.max(...transitions.map(t => t.timestamp));
                const age = now - latestTime;
                const opacity = Math.max(0, 1 - (age / SHOCK_LIFETIME));

                ctx.beginPath();
                ctx.moveTo(x, topY);
                ctx.lineTo(x, bottomY);
                ctx.strokeStyle = superCount >= SHOCK_THRESHOLD ?
                    `rgba(255, 50, 50, ${opacity})` :
                    `rgba(50, 50, 255, ${opacity})`;
                ctx.stroke();
            }
        }
        ctx.lineWidth = 2;

        // Update particles and remove those that go off screen or are too slow
        particles = particles.filter(particle => {
            particle.update();
            particle.draw();
            // Remove if off screen or velocity too low
            return particle.x <= canvas.width && Math.abs(particle.speedX) >= 0.001;
        });

        requestAnimationFrame(animate);
    }

    // Start animation
    animate();
}

// Initialize slider displays
function initializeSliderDisplays() {
    // Trigger input events to update display values
    soundSpeedSlider.dispatchEvent(new Event('input'));
    velocitySlider.dispatchEvent(new Event('input'));
    timeScaleSlider.dispatchEvent(new Event('input'));
}

// Set up canvas and start
resizeCanvas();
window.addEventListener('resize', resizeCanvas);
initializeSliderDisplays();
init();