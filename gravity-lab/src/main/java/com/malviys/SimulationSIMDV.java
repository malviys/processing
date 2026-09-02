package com.malviys;

import jdk.incubator.vector.FloatVector;
import jdk.incubator.vector.VectorOperators;
import jdk.incubator.vector.VectorSpecies;
import processing.core.PApplet;
import processing.core.PVector;

import java.util.Arrays;

/**
 * A structure-of-arrays n-body simulation accelerated with the incubating
 * Vector API.
 */
public class SimulationSIMDV extends PApplet {
    private static final VectorSpecies<Float> SPECIES = FloatVector.SPECIES_PREFERRED;
    private static final float GRAVITY = 9.8f;
    private static final float FRAME_TIME = 1.0f;
    private static final int DEFAULT_ORBITER_COUNT = 1_000;
    private static final int DEFAULT_CAPACITY = DEFAULT_ORBITER_COUNT + 1;
    private static final float MINIMUM_RADIUS = 2.0f;

    private float[] locationX;
    private float[] locationY;
    private float[] velocityX;
    private float[] velocityY;
    private float[] accelerationX;
    private float[] accelerationY;
    private float[] mass;
    private float[] radius;
    private int bodyCount;

    public SimulationSIMDV() {
        this(DEFAULT_CAPACITY);
    }

    public SimulationSIMDV(int initialCapacity) {
        if (initialCapacity < 0) {
            throw new IllegalArgumentException("Initial capacity cannot be negative");
        }

        locationX = new float[initialCapacity];
        locationY = new float[initialCapacity];
        velocityX = new float[initialCapacity];
        velocityY = new float[initialCapacity];
        accelerationX = new float[initialCapacity];
        accelerationY = new float[initialCapacity];
        mass = new float[initialCapacity];
        radius = new float[initialCapacity];
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
            float diameter = radius[i] * 2.0f;
            ellipse(locationX[i], locationY[i], diameter, diameter);
        }

        fill(0);
        rect(5, 5, 130, 55);
        fill(255);
        text("FPS: " + nf(frameRate, 2, 1), 10, 20);
        text("Bodies: " + bodyCount, 10, 40);
        text("SIMD lanes: " + SPECIES.length(), 10, 55);
    }

    public int addBody(float bodyMass, float bodyRadius, PVector location, PVector velocity) {
        requirePositiveFinite(bodyMass, "Mass");
        requirePositiveFinite(bodyRadius, "Radius");
        requireFinite(location, "Location");
        requireFinite(velocity, "Velocity");

        ensureCapacity(bodyCount + 1);
        int index = bodyCount++;
        this.locationX[index] = location.x;
        this.locationY[index] = location.y;
        this.velocityX[index] = velocity.x;
        this.velocityY[index] = velocity.y;
        this.accelerationX[index] = 0.0f;
        this.accelerationY[index] = 0.0f;
        this.mass[index] = bodyMass;
        this.radius[index] = bodyRadius;
        return index;
    }

    public int addBody(PVector location, float bodyMass, PVector velocity) {
        return addBody(bodyMass, radiusFromMass(bodyMass), location, velocity);
    }

    /** Advances the simulation by one full-frame step. */
    public void run() {
        calculateAccelerations();
        integrate();
    }

    /**
     * Vectorizes the inner pair loop. Each lane represents a different second
     * body interacting with the scalar first body.
     */
    private void calculateAccelerations() {
        Arrays.fill(accelerationX, 0, bodyCount, 0.0f);
        Arrays.fill(accelerationY, 0, bodyCount, 0.0f);

        int laneCount = SPECIES.length();
        FloatVector gravityVector = FloatVector.broadcast(SPECIES, GRAVITY);
        FloatVector halfVector = FloatVector.broadcast(SPECIES, 0.5f);

        for (int first = 0; first < bodyCount; first++) {
            FloatVector firstX = FloatVector.broadcast(SPECIES, locationX[first]);
            FloatVector firstY = FloatVector.broadcast(SPECIES, locationY[first]);
            FloatVector firstRadius = FloatVector.broadcast(SPECIES, radius[first]);
            FloatVector secondAccelerationScaleNumerator = FloatVector.broadcast(
                    SPECIES,
                    GRAVITY * mass[first]
            );
            FloatVector accumulatedFirstX = FloatVector.zero(SPECIES);
            FloatVector accumulatedFirstY = FloatVector.zero(SPECIES);

            int second = first + 1;
            int vectorEnd = second + SPECIES.loopBound(bodyCount - second);
            for (; second < vectorEnd; second += laneCount) {
                FloatVector displacementX = FloatVector.fromArray(
                        SPECIES, locationX, second
                ).sub(firstX);
                FloatVector displacementY = FloatVector.fromArray(
                        SPECIES, locationY, second
                ).sub(firstY);
                FloatVector softening = FloatVector.fromArray(SPECIES, radius, second)
                        .add(firstRadius)
                        .mul(halfVector);

                FloatVector distanceSquared = displacementX.mul(displacementX)
                        .add(displacementY.mul(displacementY))
                        .add(softening.mul(softening));
                FloatVector distanceCubed = distanceSquared.mul(
                        distanceSquared.lanewise(VectorOperators.SQRT)
                );

                FloatVector firstScale = FloatVector.fromArray(SPECIES, mass, second)
                        .mul(gravityVector)
                        .div(distanceCubed);
                FloatVector secondScale = secondAccelerationScaleNumerator
                        .div(distanceCubed);
                FloatVector firstAccelerationX = displacementX.mul(firstScale);
                FloatVector firstAccelerationY = displacementY.mul(firstScale);

                accumulatedFirstX = accumulatedFirstX.add(firstAccelerationX);
                accumulatedFirstY = accumulatedFirstY.add(firstAccelerationY);

                FloatVector.fromArray(SPECIES, accelerationX, second)
                        .sub(displacementX.mul(secondScale))
                        .intoArray(accelerationX, second);
                FloatVector.fromArray(SPECIES, accelerationY, second)
                        .sub(displacementY.mul(secondScale))
                        .intoArray(accelerationY, second);
            }

            accelerationX[first] += accumulatedFirstX.reduceLanes(VectorOperators.ADD);
            accelerationY[first] += accumulatedFirstY.reduceLanes(VectorOperators.ADD);

            // Only the bodies that do not fill a complete vector reach this loop.
            for (; second < bodyCount; second++) {
                applyScalarPair(first, second);
            }
        }
    }

    private void applyScalarPair(int first, int second) {
        float displacementX = locationX[second] - locationX[first];
        float displacementY = locationY[second] - locationY[first];
        float softening = (radius[first] + radius[second]) * 0.5f;
        float distanceSquared = displacementX * displacementX
                + displacementY * displacementY
                + softening * softening;
        float distanceCubed = distanceSquared * (float) Math.sqrt(distanceSquared);
        float firstScale = GRAVITY * mass[second] / distanceCubed;
        float secondScale = GRAVITY * mass[first] / distanceCubed;

        accelerationX[first] += displacementX * firstScale;
        accelerationY[first] += displacementY * firstScale;
        accelerationX[second] -= displacementX * secondScale;
        accelerationY[second] -= displacementY * secondScale;
    }

    /** Vectorized semi-implicit Euler integration with a scalar tail. */
    private void integrate() {
        int vectorEnd = SPECIES.loopBound(bodyCount);
        FloatVector frameTime = FloatVector.broadcast(SPECIES, FRAME_TIME);
        int i = 0;

        for (; i < vectorEnd; i += SPECIES.length()) {
            FloatVector nextVelocityX = FloatVector.fromArray(SPECIES, velocityX, i)
                    .add(FloatVector.fromArray(SPECIES, accelerationX, i).mul(frameTime));
            FloatVector nextVelocityY = FloatVector.fromArray(SPECIES, velocityY, i)
                    .add(FloatVector.fromArray(SPECIES, accelerationY, i).mul(frameTime));
            FloatVector nextLocationX = FloatVector.fromArray(SPECIES, locationX, i)
                    .add(nextVelocityX.mul(frameTime));
            FloatVector nextLocationY = FloatVector.fromArray(SPECIES, locationY, i)
                    .add(nextVelocityY.mul(frameTime));

            nextVelocityX.intoArray(velocityX, i);
            nextVelocityY.intoArray(velocityY, i);
            nextLocationX.intoArray(locationX, i);
            nextLocationY.intoArray(locationY, i);
        }

        for (; i < bodyCount; i++) {
            velocityX[i] += accelerationX[i] * FRAME_TIME;
            velocityY[i] += accelerationY[i] * FRAME_TIME;
            locationX[i] += velocityX[i] * FRAME_TIME;
            locationY[i] += velocityY[i] * FRAME_TIME;
        }
    }

    private void createOrbitalSystem(int orbiterCount) {
        float centralMass = 200.0f;
        float orbiterMass = 0.1f;
        float centerX = width / 2.0f;
        float centerY = height / 2.0f;

        addBody(new PVector(centerX, centerY), centralMass, new PVector());

        for (int i = 0; i < orbiterCount; i++) {
            float progress = orbiterCount == 1 ? 0.0f : (float) i / (orbiterCount - 1);
            float orbitalRadius = lerp(100.0f, 180.0f, progress);
            float angle = TWO_PI * i / orbiterCount;
            float speed = circularOrbitSpeed(centralMass, orbitalRadius);

            addBody(
                    new PVector(
                            centerX + cos(angle) * orbitalRadius,
                            centerY + sin(angle) * orbitalRadius
                    ),
                    orbiterMass,
                    new PVector(-sin(angle) * speed, cos(angle) * speed)
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

    public int vectorLaneCount() {
        return SPECIES.length();
    }

    public float getMass(int index) {
        checkBodyIndex(index);
        return mass[index];
    }

    public float getRadius(int index) {
        checkBodyIndex(index);
        return radius[index];
    }

    public PVector getLocation(int index) {
        checkBodyIndex(index);
        return new PVector(locationX[index], locationY[index]);
    }

    public PVector getVelocity(int index) {
        checkBodyIndex(index);
        return new PVector(velocityX[index], velocityY[index]);
    }

    public PVector getAcceleration(int index) {
        checkBodyIndex(index);
        return new PVector(accelerationX[index], accelerationY[index]);
    }

    private void ensureCapacity(int requiredCapacity) {
        if (requiredCapacity <= locationX.length) {
            return;
        }

        int doubledCapacity = locationX.length <= Integer.MAX_VALUE / 2
                ? Math.max(1, locationX.length * 2)
                : Integer.MAX_VALUE;
        int newCapacity = Math.max(requiredCapacity, doubledCapacity);
        locationX = Arrays.copyOf(locationX, newCapacity);
        locationY = Arrays.copyOf(locationY, newCapacity);
        velocityX = Arrays.copyOf(velocityX, newCapacity);
        velocityY = Arrays.copyOf(velocityY, newCapacity);
        accelerationX = Arrays.copyOf(accelerationX, newCapacity);
        accelerationY = Arrays.copyOf(accelerationY, newCapacity);
        mass = Arrays.copyOf(mass, newCapacity);
        radius = Arrays.copyOf(radius, newCapacity);
    }

    private void checkBodyIndex(int index) {
        if (index < 0 || index >= bodyCount) {
            throw new IndexOutOfBoundsException("Body index: " + index);
        }
    }

    private static float radiusFromMass(float bodyMass) {
        requirePositiveFinite(bodyMass, "Mass");
        return Math.max(MINIMUM_RADIUS, (float) Math.sqrt(bodyMass));
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
        PApplet.main(SimulationSIMDV.class, args);
    }
}
