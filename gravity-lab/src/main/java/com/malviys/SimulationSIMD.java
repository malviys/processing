package com.malviys;

import processing.core.PApplet;
import processing.core.PVector;

import java.util.Arrays;

/**
 * A data-oriented two-dimensional n-body simulation.
 *
 * <p>Body properties are kept in parallel arrays so the same property for all
 * bodies is contiguous in memory. {@link PVector} is used as the computation
 * wrapper for force, acceleration, velocity, and position; calculated values
 * are written back to their respective arrays after every operation.</p>
 */
public class SimulationSIMD extends PApplet {
    private static final float G = 9.8f;
    private static final float FRAME_TIME = 1.0f;
    private static final int PHYSICS_SUBSTEPS = 16;
    private static final int DEFAULT_ORBITER_COUNT = 1_000;
    private static final int DEFAULT_CAPACITY = DEFAULT_ORBITER_COUNT + 1;
    private static final float MINIMUM_RADIUS = 2.0f;

    private float[] locXArray;
    private float[] locYArray;
    private float[] vecXArray;
    private float[] vecYArray;
    private float[] accXArray;
    private float[] accYArray;
    private float[] massArray;
    private float[] radiusArray;

    private int bodyCount;

    public SimulationSIMD() {
        this(DEFAULT_CAPACITY);
    }

    public SimulationSIMD(int initialCapacity) {
        if (initialCapacity < 0) {
            throw new IllegalArgumentException("Initial capacity cannot be negative");
        }

        locXArray = new float[initialCapacity];
        locYArray = new float[initialCapacity];
        vecXArray = new float[initialCapacity];
        vecYArray = new float[initialCapacity];
        accXArray = new float[initialCapacity];
        accYArray = new float[initialCapacity];
        massArray = new float[initialCapacity];
        radiusArray = new float[initialCapacity];
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
    public int addBody(float mass, float radius, PVector location, PVector velocity) {
        requirePositiveFinite(mass, "Mass");
        requirePositiveFinite(radius, "Radius");
        requireFinite(location, "Location");
        requireFinite(velocity, "Velocity");

        ensureCapacity(bodyCount + 1);
        int index = bodyCount++;

        massArray[index] = mass;
        radiusArray[index] = radius;
        locXArray[index] = location.x;
        locYArray[index] = location.y;
        vecXArray[index] = velocity.x;
        vecYArray[index] = velocity.y;
        accXArray[index] = 0.0f;
        accYArray[index] = 0.0f;

        return index;
    }

    /** Advances every body by one rendered-frame interval. */
    public void run() {
        for (int step = 0; step < PHYSICS_SUBSTEPS; step++) {
            calcForce();
            applyForce();
        }
    }

    /**
     * Calculates pairwise gravitational force from one shared position state.
     * Each pair is visited once and receives equal, opposite force.
     */
    private void calcForce() {
        Arrays.fill(accXArray, 0, bodyCount, 0.0f);
        Arrays.fill(accYArray, 0, bodyCount, 0.0f);

        for (int first = 0; first < bodyCount; first++) {
            PVector firstLocation = new PVector(locXArray[first], locYArray[first]);

            for (int second = first + 1; second < bodyCount; second++) {
                PVector secondLocation = new PVector(locXArray[second], locYArray[second]);
                PVector displacement = PVector.sub(secondLocation, firstLocation);

                // Softening makes the force finite when bodies overlap and limits
                // unstable acceleration during very close encounters.
                float softening = (radiusArray[first] + radiusArray[second]) * 0.5f;
                float softenedDistanceSquared = displacement.magSq() + softening * softening;
                float softenedDistanceCubed = softenedDistanceSquared
                        * (float) Math.sqrt(softenedDistanceSquared);

                float forceScale = G * massArray[first] * massArray[second]
                        / softenedDistanceCubed;
                PVector forceOnFirst = displacement.copy().mult(forceScale);
                PVector accelerationOfFirst = forceOnFirst.copy().div(massArray[first]);
                PVector accelerationOfSecond = forceOnFirst.copy().mult(-1.0f)
                        .div(massArray[second]);

                accXArray[first] += accelerationOfFirst.x;
                accYArray[first] += accelerationOfFirst.y;
                accXArray[second] += accelerationOfSecond.x;
                accYArray[second] += accelerationOfSecond.y;
            }
        }
    }

    /** Applies acceleration with semi-implicit Euler integration. */
    private void applyForce() {
        float timeStep = FRAME_TIME / PHYSICS_SUBSTEPS;

        for (int i = 0; i < bodyCount; i++) {
            PVector acceleration = new PVector(accXArray[i], accYArray[i]);
            PVector velocity = new PVector(vecXArray[i], vecYArray[i]);
            velocity.add(PVector.mult(acceleration, timeStep));

            PVector location = new PVector(locXArray[i], locYArray[i]);
            location.add(PVector.mult(velocity, timeStep));

            vecXArray[i] = velocity.x;
            vecYArray[i] = velocity.y;
            locXArray[i] = location.x;
            locYArray[i] = location.y;
        }
    }

    private void createOrbitalSystem(int orbiterCount) {
        float centralMass = 200.0f;
        float orbiterMass = 0.1f;
        float centerX = width / 2.0f;
        float centerY = height / 2.0f;

        addBody(
                centralMass,
                radiusFromMass(centralMass),
                new PVector(centerX, centerY),
                new PVector()
        );

        for (int i = 0; i < orbiterCount; i++) {
            float progress = orbiterCount == 1 ? 0.0f : (float) i / (orbiterCount - 1);
            float orbitalRadius = lerp(100.0f, 180.0f, progress);
            float angle = TWO_PI * i / orbiterCount;
            float speed = circularOrbitSpeed(centralMass, orbitalRadius);

            PVector location = new PVector(
                    centerX + cos(angle) * orbitalRadius,
                    centerY + sin(angle) * orbitalRadius
            );
            PVector velocity = new PVector(
                    -sin(angle) * speed,
                    cos(angle) * speed
            );

            addBody(
                    orbiterMass,
                    radiusFromMass(orbiterMass),
                    location,
                    velocity
            );
        }
    }

    public float circularOrbitSpeed(float centralMass, float orbitalRadius) {
        requirePositiveFinite(centralMass, "Central mass");
        requirePositiveFinite(orbitalRadius, "Orbital radius");
        return (float) Math.sqrt(G * centralMass / orbitalRadius);
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

    private static void requireFinite(PVector vector, String property) {
        if (vector == null
                || !Float.isFinite(vector.x)
                || !Float.isFinite(vector.y)
                || !Float.isFinite(vector.z)) {
            throw new IllegalArgumentException(property + " must be a finite vector");
        }
    }

    public static void main(String[] args) {
        PApplet.main(SimulationSIMD.class, args);
    }
}
