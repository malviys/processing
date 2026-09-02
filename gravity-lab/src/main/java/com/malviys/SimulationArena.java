package com.malviys;

import processing.core.PApplet;
import processing.core.PVector;

import java.lang.foreign.Arena;
import java.lang.foreign.MemoryLayout;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.SequenceLayout;

import static java.lang.foreign.ValueLayout.JAVA_FLOAT;

/**
 * An off-heap, data-oriented two-dimensional n-body simulation.
 *
 * <p>Particle fields use a structure-of-arrays layout inside one contiguous
 * native-memory block. Byte offsets divide that block into float sequences for
 * location, velocity, acceleration, mass, and radius.</p>
 */
public class SimulationArena extends PApplet implements AutoCloseable {
    private static final float GRAVITY = 9.8f;
    private static final float FRAME_TIME = 1.0f;
    private static final int DEFAULT_ORBITER_COUNT = 1_000;
    private static final int DEFAULT_CAPACITY = DEFAULT_ORBITER_COUNT + 1;
    private static final float MINIMUM_RADIUS = 2.0f;
    private static final int COMPONENT_COUNT = 8;
    private static final int LOCATION_X_COMPONENT = 0;
    private static final int LOCATION_Y_COMPONENT = 1;
    private static final int VELOCITY_X_COMPONENT = 2;
    private static final int VELOCITY_Y_COMPONENT = 3;
    private static final int ACCELERATION_X_COMPONENT = 4;
    private static final int ACCELERATION_Y_COMPONENT = 5;
    private static final int MASS_COMPONENT = 6;
    private static final int RADIUS_COMPONENT = 7;

    private Arena stateArena;
    private SequenceLayout componentSequenceLayout;
    private SequenceLayout stateBlockLayout;
    private MemorySegment stateBlock;
    private MemorySegment locationXState;
    private MemorySegment locationYState;
    private MemorySegment velocityXState;
    private MemorySegment velocityYState;
    private MemorySegment accelerationXState;
    private MemorySegment accelerationYState;
    private MemorySegment massState;
    private MemorySegment radiusState;
    private int capacity;
    private int bodyCount;
    private boolean closed;

    public SimulationArena() {
        this(DEFAULT_CAPACITY);
    }

    public SimulationArena(int initialCapacity) {
        if (initialCapacity < 0) {
            throw new IllegalArgumentException("Initial capacity cannot be negative");
        }

        allocateStateSequences(initialCapacity);
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
            float diameter = read(radiusState, i) * 2.0f;
            ellipse(read(locationXState, i), read(locationYState, i), diameter, diameter);
        }

        fill(0);
        rect(5, 5, 90, 40);
        fill(255);
        text("FPS: " + nf(frameRate, 2, 1), 10, 20);
        text("Bodies: " + bodyCount, 10, 40);
    }

    /** Adds a particle with an explicitly chosen drawing/collision radius. */
    public int addBody(float mass, float radius, PVector location, PVector velocity) {
        ensureOpen();
        requirePositiveFinite(mass, "Mass");
        requirePositiveFinite(radius, "Radius");
        requireFinite(location, "Location");
        requireFinite(velocity, "Velocity");

        ensureCapacity(bodyCount + 1);
        int index = bodyCount++;
        write(locationXState, index, location.x);
        write(locationYState, index, location.y);
        write(velocityXState, index, velocity.x);
        write(velocityYState, index, velocity.y);
        write(accelerationXState, index, 0.0f);
        write(accelerationYState, index, 0.0f);
        write(massState, index, mass);
        write(radiusState, index, radius);
        return index;
    }

    /** Adds a particle whose radius is derived from its mass. */
    public int addBody(PVector location, float mass, PVector velocity) {
        return addBody(mass, radiusFromMass(mass), location, velocity);
    }

    /** Advances every particle by one rendered-frame interval. */
    public void run() {
        ensureOpen();
        calculateAccelerations();
        integrate(FRAME_TIME);
    }

    /**
     * Visits each pair once and applies equal, opposite gravitational force.
     * Acceleration values live in their own off-heap component sequences.
     */
    private void calculateAccelerations() {
        for (int i = 0; i < bodyCount; i++) {
            write(accelerationXState, i, 0.0f);
            write(accelerationYState, i, 0.0f);
        }

        for (int first = 0; first < bodyCount; first++) {
            float firstX = read(locationXState, first);
            float firstY = read(locationYState, first);
            float firstMass = read(massState, first);
            float firstRadius = read(radiusState, first);

            for (int second = first + 1; second < bodyCount; second++) {
                float displacementX = read(locationXState, second) - firstX;
                float displacementY = read(locationYState, second) - firstY;

                // Softening keeps acceleration finite during close encounters.
                float softening = (firstRadius + read(radiusState, second)) * 0.5f;
                float distanceSquared = displacementX * displacementX
                        + displacementY * displacementY
                        + softening * softening;
                float distanceCubed = distanceSquared * (float) Math.sqrt(distanceSquared);

                float firstScale = GRAVITY * read(massState, second) / distanceCubed;
                float secondScale = GRAVITY * firstMass / distanceCubed;

                add(accelerationXState, first, displacementX * firstScale);
                add(accelerationYState, first, displacementY * firstScale);
                add(accelerationXState, second, -displacementX * secondScale);
                add(accelerationYState, second, -displacementY * secondScale);
            }
        }
    }

    /** Integrates velocity and position using semi-implicit Euler. */
    private void integrate(float timeStep) {
        for (int i = 0; i < bodyCount; i++) {
            float velocityX = read(velocityXState, i)
                    + read(accelerationXState, i) * timeStep;
            float velocityY = read(velocityYState, i)
                    + read(accelerationYState, i) * timeStep;

            write(velocityXState, i, velocityX);
            write(velocityYState, i, velocityY);
            add(locationXState, i, velocityX * timeStep);
            add(locationYState, i, velocityY * timeStep);
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
        ensureOpen();
        return bodyCount;
    }

    public float getMass(int index) {
        checkBodyIndex(index);
        return read(massState, index);
    }

    public float getRadius(int index) {
        checkBodyIndex(index);
        return read(radiusState, index);
    }

    public PVector getLocation(int index) {
        checkBodyIndex(index);
        return new PVector(read(locationXState, index), read(locationYState, index));
    }

    public PVector getVelocity(int index) {
        checkBodyIndex(index);
        return new PVector(read(velocityXState, index), read(velocityYState, index));
    }

    public PVector getAcceleration(int index) {
        checkBodyIndex(index);
        return new PVector(read(accelerationXState, index), read(accelerationYState, index));
    }

    /** Exposes the sequence layout shared by all component slices. */
    public SequenceLayout getParticleSequenceLayout() {
        ensureOpen();
        return componentSequenceLayout;
    }

    private void ensureCapacity(int requiredCapacity) {
        if (requiredCapacity <= capacity) {
            return;
        }

        int doubledCapacity = capacity <= Integer.MAX_VALUE / 2
                ? Math.max(1, capacity * 2)
                : Integer.MAX_VALUE;
        allocateStateSequences(Math.max(requiredCapacity, doubledCapacity));
    }

    /** Allocates one block and divides it into contiguous component slices. */
    private void allocateStateSequences(int newCapacity) {
        SequenceLayout newComponentLayout = MemoryLayout.sequenceLayout(
                newCapacity,
                JAVA_FLOAT
        );
        SequenceLayout newBlockLayout = MemoryLayout.sequenceLayout(
                COMPONENT_COUNT,
                newComponentLayout
        );
        long componentBytes = newComponentLayout.byteSize();
        long activeBytes = Math.multiplyExact((long) bodyCount, JAVA_FLOAT.byteSize());

        Arena newArena = Arena.ofShared();
        MemorySegment newBlock = newArena.allocate(newBlockLayout);
        MemorySegment newLocationX = componentSlice(
                newBlock, componentBytes, LOCATION_X_COMPONENT
        );
        MemorySegment newLocationY = componentSlice(
                newBlock, componentBytes, LOCATION_Y_COMPONENT
        );
        MemorySegment newVelocityX = componentSlice(
                newBlock, componentBytes, VELOCITY_X_COMPONENT
        );
        MemorySegment newVelocityY = componentSlice(
                newBlock, componentBytes, VELOCITY_Y_COMPONENT
        );
        MemorySegment newAccelerationX = componentSlice(
                newBlock, componentBytes, ACCELERATION_X_COMPONENT
        );
        MemorySegment newAccelerationY = componentSlice(
                newBlock, componentBytes, ACCELERATION_Y_COMPONENT
        );
        MemorySegment newMass = componentSlice(newBlock, componentBytes, MASS_COMPONENT);
        MemorySegment newRadius = componentSlice(newBlock, componentBytes, RADIUS_COMPONENT);

        if (stateBlock != null) {
            copy(locationXState, newLocationX, activeBytes);
            copy(locationYState, newLocationY, activeBytes);
            copy(velocityXState, newVelocityX, activeBytes);
            copy(velocityYState, newVelocityY, activeBytes);
            copy(accelerationXState, newAccelerationX, activeBytes);
            copy(accelerationYState, newAccelerationY, activeBytes);
            copy(massState, newMass, activeBytes);
            copy(radiusState, newRadius, activeBytes);
        }

        Arena oldArena = stateArena;
        stateArena = newArena;
        componentSequenceLayout = newComponentLayout;
        stateBlockLayout = newBlockLayout;
        stateBlock = newBlock;
        locationXState = newLocationX;
        locationYState = newLocationY;
        velocityXState = newVelocityX;
        velocityYState = newVelocityY;
        accelerationXState = newAccelerationX;
        accelerationYState = newAccelerationY;
        massState = newMass;
        radiusState = newRadius;
        capacity = newCapacity;

        closeArena(oldArena);
    }

    private static MemorySegment componentSlice(
            MemorySegment block,
            long componentBytes,
            int componentIndex
    ) {
        long offset = Math.multiplyExact(componentBytes, componentIndex);
        return block.asSlice(offset, componentBytes);
    }

    private static void copy(MemorySegment source, MemorySegment destination, long bytes) {
        MemorySegment.copy(source, 0, destination, 0, bytes);
    }

    private static float read(MemorySegment state, int index) {
        return state.getAtIndex(JAVA_FLOAT, index);
    }

    private static void write(MemorySegment state, int index, float value) {
        state.setAtIndex(JAVA_FLOAT, index, value);
    }

    private static void add(MemorySegment state, int index, float value) {
        write(state, index, read(state, index) + value);
    }

    private static void closeArena(Arena arena) {
        if (arena != null) {
            arena.close();
        }
    }

    private void checkBodyIndex(int index) {
        ensureOpen();
        if (index < 0 || index >= bodyCount) {
            throw new IndexOutOfBoundsException("Body index: " + index);
        }
    }

    private void ensureOpen() {
        if (closed) {
            throw new IllegalStateException("Simulation arena is closed");
        }
    }

    private static float radiusFromMass(float mass) {
        requirePositiveFinite(mass, "Mass");
        return Math.max(MINIMUM_RADIUS, (float) Math.sqrt(mass));
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

    /** Releases all native memory owned by the simulation. */
    @Override
    public void close() {
        if (!closed) {
            closed = true;
            closeArena(stateArena);
        }
    }

    /** Processing invokes this when its drawing surface is disposed. */
    @Override
    public void dispose() {
        close();
        super.dispose();
    }

    public static void main(String[] args) {
        PApplet.main(SimulationArena.class, args);
    }
}
