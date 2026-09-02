package com.malviys;

import processing.core.PApplet;
import processing.core.PVector;

import java.util.Arrays;
import java.util.stream.IntStream;

/**
 * A data-oriented two-dimensional n-body simulation.
 *
 * <p>Body properties are kept in parallel arrays so the same property for all
 * bodies is contiguous in memory. Pairwise forces are calculated in parallel;
 * every worker writes to private acceleration arrays that are reduced before
 * the integration phase.</p>
 */
public class SimulationMT extends PApplet {
    private static final float GRAVITY = 9.8f;
    private static final float FRAME_TIME = 1.0f;
    private static final int DEFAULT_ORBITER_COUNT = 1_000;
    private static final int DEFAULT_CAPACITY = DEFAULT_ORBITER_COUNT + 1;
    private static final float MINIMUM_RADIUS = 2.0f;
    private static final int PARALLEL_BODY_THRESHOLD = 128;

    private float[] locXArray;
    private float[] locYArray;
    private float[] vecXArray;
    private float[] vecYArray;
    private float[] accXArray;
    private float[] accYArray;
    private float[] massArray;
    private float[] radiusArray;
    private float[][] workerAccXArrays;
    private float[][] workerAccYArrays;

    private final int workerCount;
    private int bodyCount;

    public SimulationMT() {
        this(DEFAULT_CAPACITY);
    }

    public SimulationMT(int initialCapacity) {
        this(initialCapacity, Runtime.getRuntime().availableProcessors());
    }

    SimulationMT(int initialCapacity, int workerCount) {
        if (initialCapacity < 0) {
            throw new IllegalArgumentException("Initial capacity cannot be negative");
        }
        if (workerCount <= 0) {
            throw new IllegalArgumentException("Worker count must be positive");
        }

        this.workerCount = workerCount;
        locXArray = new float[initialCapacity];
        locYArray = new float[initialCapacity];
        vecXArray = new float[initialCapacity];
        vecYArray = new float[initialCapacity];
        accXArray = new float[initialCapacity];
        accYArray = new float[initialCapacity];
        massArray = new float[initialCapacity];
        radiusArray = new float[initialCapacity];
        workerAccXArrays = new float[workerCount][initialCapacity];
        workerAccYArrays = new float[workerCount][initialCapacity];
    }

    @Override
    public void settings() {
        size(800, 600);
    }

    @Override
    public void setup() {
        if (bodyCount == 0) {
            createOrbitalSystem(DEFAULT_ORBITER_COUNT);
        }
    }

    @Override
    public void draw() {
        background(225);
        run();

        fill(255, 0, 0);
        noStroke();
        for (int i = 0; i < bodyCount; i++) {
            float diameter = radiusArray[i] * 2.0f;
            ellipse(locXArray[i], locYArray[i], diameter, diameter);
        }

        fill(0);
        rect(5, 5, 90, 40);
        fill(255);
        text("FPS: " + nf(frameRate, 2, 1), 10, 20);
        text("Bodies: " + bodyCount, 10, 40);
    }

    /** Adds a body to the simulation. */
    public int addBody(
            float locationX,
            float locationY,
            float mass,
            float velocityX,
            float velocityY
    ) {
        requirePositiveFinite(mass, "Mass");
        requireFinite(locationX, "Location X");
        requireFinite(locationY, "Location Y");
        requireFinite(velocityX, "Velocity X");
        requireFinite(velocityY, "Velocity Y");

        ensureCapacity(bodyCount + 1);
        int index = bodyCount++;

        massArray[index] = mass;
        radiusArray[index] = radiusFromMass(mass);
        locXArray[index] = locationX;
        locYArray[index] = locationY;
        vecXArray[index] = velocityX;
        vecYArray[index] = velocityY;
        accXArray[index] = 0.0f;
        accYArray[index] = 0.0f;

        return index;
    }

    /** Advances every body by one rendered-frame interval. */
    public void run() {
        calcForce();
        applyForce();
    }

    /**
     * Calculates pairwise gravitational force from one shared position state.
     * Each pair is visited once and receives equal, opposite force.
     */
    private void calcForce() {
        int activeWorkerCount = bodyCount >= PARALLEL_BODY_THRESHOLD
                ? Math.min(workerCount, bodyCount)
                : 1;

        for (int worker = 0; worker < activeWorkerCount; worker++) {
            Arrays.fill(workerAccXArrays[worker], 0, bodyCount, 0.0f);
            Arrays.fill(workerAccYArrays[worker], 0, bodyCount, 0.0f);
        }

        if (activeWorkerCount == 1) {
            calculateWorkerForces(0, 1);
        } else {
            IntStream.range(0, activeWorkerCount)
                    .parallel()
                    .forEach(worker -> calculateWorkerForces(worker, activeWorkerCount));
        }

        // Parallel workers never share writable state. Combine their partial
        // accelerations only after the parallel stream has joined.
        for (int body = 0; body < bodyCount; body++) {
            float accelerationX = 0.0f;
            float accelerationY = 0.0f;
            for (int worker = 0; worker < activeWorkerCount; worker++) {
                accelerationX += workerAccXArrays[worker][body];
                accelerationY += workerAccYArrays[worker][body];
            }
            accXArray[body] = accelerationX;
            accYArray[body] = accelerationY;
        }
    }

    /** Calculates a strided share of unique body pairs into worker-local arrays. */
    private void calculateWorkerForces(int worker, int activeWorkerCount) {
        float[] workerAccX = workerAccXArrays[worker];
        float[] workerAccY = workerAccYArrays[worker];

        // Striding spreads the expensive low first-index iterations across workers.
        for (int first = worker; first < bodyCount; first += activeWorkerCount) {
            PVector firstLocation = new PVector(locXArray[first], locYArray[first]);
            float firstMass = massArray[first];
            float firstRadius = radiusArray[first];

            for (int second = first + 1; second < bodyCount; second++) {
                PVector secondLocation = new PVector(locXArray[second], locYArray[second]);
                PVector displacement = PVector.sub(secondLocation, firstLocation);

                // Softening makes the force finite when bodies overlap and limits
                // unstable acceleration during very close encounters.
                float softening = (firstRadius + radiusArray[second]) * 0.5f;
                float softenedDistanceSquared = displacement.magSq() + softening * softening;
                float softenedDistanceCubed = softenedDistanceSquared
                        * (float) Math.sqrt(softenedDistanceSquared);

                float firstAccelerationScale = GRAVITY * massArray[second]
                        / softenedDistanceCubed;
                float secondAccelerationScale = GRAVITY * firstMass
                        / softenedDistanceCubed;
                PVector firstAcceleration = displacement.copy().mult(firstAccelerationScale);
                PVector secondAcceleration = displacement.copy().mult(-secondAccelerationScale);

                workerAccX[first] += firstAcceleration.x;
                workerAccY[first] += firstAcceleration.y;
                workerAccX[second] += secondAcceleration.x;
                workerAccY[second] += secondAcceleration.y;
            }
        }
    }

    /** Applies acceleration with semi-implicit Euler integration. */
    private void applyForce() {
        for (int i = 0; i < bodyCount; i++) {
            vecXArray[i] += accXArray[i] * FRAME_TIME;
            vecYArray[i] += accYArray[i] * FRAME_TIME;
            locXArray[i] += vecXArray[i] * FRAME_TIME;
            locYArray[i] += vecYArray[i] * FRAME_TIME;
            accXArray[i] = 0.0f;
            accYArray[i] = 0.0f;
        }
    }

    private void createOrbitalSystem(int orbiterCount) {
        float centralMass = 200.0f;
        float orbiterMass = 0.1f;
        float centerX = width / 2.0f;
        float centerY = height / 2.0f;

        addBody(centerX, centerY, centralMass, 0.0f, 0.0f);

        for (int i = 0; i < orbiterCount; i++) {
            float progress = orbiterCount == 1 ? 0.0f : (float) i / (orbiterCount - 1);
            float orbitalRadius = lerp(100.0f, 180.0f, progress);
            float angle = TWO_PI * i / orbiterCount;
            float speed = circularOrbitSpeed(centralMass, orbitalRadius);

            addBody(
                    centerX + cos(angle) * orbitalRadius,
                    centerY + sin(angle) * orbitalRadius,
                    orbiterMass,
                    -sin(angle) * speed,
                    cos(angle) * speed
            );
        }
    }

    public float circularOrbitSpeed(float centralMass, float orbitalRadius) {
        requirePositiveFinite(centralMass, "Central mass");
        requirePositiveFinite(orbitalRadius, "Orbital radius");
        return (float) Math.sqrt(GRAVITY * centralMass / orbitalRadius);
    }

    public int size() {
        return bodyCount;
    }

    public float getMass(int index) {
        checkBodyIndex(index);
        return massArray[index];
    }

    public float getRadius(int index) {
        checkBodyIndex(index);
        return radiusArray[index];
    }

    public PVector getLocation(int index) {
        checkBodyIndex(index);
        return new PVector(locXArray[index], locYArray[index]);
    }

    public PVector getVelocity(int index) {
        checkBodyIndex(index);
        return new PVector(vecXArray[index], vecYArray[index]);
    }

    public PVector getAcceleration(int index) {
        checkBodyIndex(index);
        return new PVector(accXArray[index], accYArray[index]);
    }

    private static float radiusFromMass(float mass) {
        return Math.max(MINIMUM_RADIUS, (float) Math.sqrt(mass));
    }

    private void ensureCapacity(int requiredCapacity) {
        if (requiredCapacity <= locXArray.length) {
            return;
        }

        int newCapacity = Math.max(requiredCapacity, locXArray.length * 2);
        locXArray = Arrays.copyOf(locXArray, newCapacity);
        locYArray = Arrays.copyOf(locYArray, newCapacity);
        vecXArray = Arrays.copyOf(vecXArray, newCapacity);
        vecYArray = Arrays.copyOf(vecYArray, newCapacity);
        accXArray = Arrays.copyOf(accXArray, newCapacity);
        accYArray = Arrays.copyOf(accYArray, newCapacity);
        massArray = Arrays.copyOf(massArray, newCapacity);
        radiusArray = Arrays.copyOf(radiusArray, newCapacity);
        for (int worker = 0; worker < workerCount; worker++) {
            workerAccXArrays[worker] = Arrays.copyOf(workerAccXArrays[worker], newCapacity);
            workerAccYArrays[worker] = Arrays.copyOf(workerAccYArrays[worker], newCapacity);
        }
    }

    private void checkBodyIndex(int index) {
        if (index < 0 || index >= bodyCount) {
            throw new IndexOutOfBoundsException("Body index: " + index);
        }
    }

    private static void requirePositiveFinite(float value, String property) {
        if (!Float.isFinite(value) || value <= 0.0f) {
            throw new IllegalArgumentException(property + " must be positive and finite");
        }
    }

    private static void requireFinite(float value, String property) {
        if (!Float.isFinite(value)) {
            throw new IllegalArgumentException(property + " must be finite");
        }
    }

    public static void main(String[] args) {
        PApplet.main(SimulationMT.class, args);
    }
}
