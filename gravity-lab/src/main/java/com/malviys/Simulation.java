package com.malviys;

import processing.core.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Processing sketch and physics engine for a collection of gravitational bodies.
 */
public class Simulation extends PApplet {
    // These are simulation units rather than real-world SI units.
    private static final float GRAVITATIONAL_CONSTANT = 9.8f;
    private static final float FRAME_TIME = 1.0f;
    private static final float MINIMUM_ORBIT_RADIUS = 100.0f;
    private static final float MAXIMUM_ORBIT_RADIUS = 180.0f;
    private static final int DEFAULT_ORBITING_BODY_COUNT = 1_000;

    private final List<Body> bodies;
    private int orbitingBodyCount = DEFAULT_ORBITING_BODY_COUNT;

    /** Constructor used by Processing when launching this class as a sketch. */
    public Simulation() {
        this(DEFAULT_ORBITING_BODY_COUNT + 1L);
    }

    public Simulation(long particleCount) {
        if (particleCount < 0 || particleCount > Integer.MAX_VALUE) {
            throw new IllegalArgumentException("Particle count is outside the supported range");
        }

        this.bodies = new ArrayList<>((int) particleCount);
    }

    /** Processing calls settings before setup to create the drawing surface. */
    @Override
    public void settings() {
        size(800, 600);
    }

    /** Builds a central mass and a configurable set of initially circular orbiters. */
    @Override
    public void setup() {
        background(0);

        // Keep explicitly added bodies when this engine is embedded in another sketch.
        if (!bodies.isEmpty()) {
            return;
        }

        float centralMass = 200.0f;
        float orbitingMass = 0.1f;
        float centerX = width / 2.0f;
        float centerY = height / 2.0f;

        addBody(
                new PVector(centerX, centerY),
                centralMass,
                new PVector()
        );

        // Spread positions by both angle and radius. Velocity is perpendicular
        // to the radius vector, which provides the tangential circular motion.
        for (int i = 0; i < orbitingBodyCount; i++) {
            float radiusProgress = orbitingBodyCount == 1
                    ? 0.0f
                    : (float) i / (orbitingBodyCount - 1);
            float radius = lerp(MINIMUM_ORBIT_RADIUS, MAXIMUM_ORBIT_RADIUS, radiusProgress);
            float angle = TWO_PI * i / orbitingBodyCount;
            float orbitSpeed = circularOrbitSpeed(centralMass, radius);

            PVector location = new PVector(
                    centerX + cos(angle) * radius,
                    centerY + sin(angle) * radius
            );
            PVector velocity = new PVector(
                    -sin(angle) * orbitSpeed,
                    cos(angle) * orbitSpeed
            );

            addBody(location, orbitingMass, velocity);
        }
    }

    /** Advances physics once and redraws the complete scene. */
    @Override
    public void draw() {
        background(225);
        fill(255, 0, 0);

        run();
        for (Body body : bodies) {
            PVector location = body.getLocation();
            float diameter = body.getRadius() * 2.0f;
            ellipse(location.x, location.y, diameter, diameter);
        }

        fill(0);
        rect(5, 5, 75, 40);
        fill(255);
        text("FPS: " + nf(frameRate, 2, 1), 10, 20);
        text("Bodies: " + bodies.size(), 10, 40);
    }

    /** Adds a body and returns its stable index in this simulation. */
    public int addBody(PVector location, float mass, PVector velocity) {
        this.bodies.add(new Body(location, mass, velocity));
        return this.bodies.size() - 1;
    }

    /** Advances the simulation by one frame. */
    public void run() {
        // Force phase: positions remain unchanged throughout this loop.
        for (int i = 0; i < this.bodies.size(); i++) {
            for (int j = 0; j < this.bodies.size(); j++) {
                if (i != j) {
                    this.bodies.get(i).applyGravity(
                            this.bodies.get(j),
                            GRAVITATIONAL_CONSTANT
                    );
                }
            }
        }

        // Integration phase: apply the accumulated acceleration to every body.
        for (Body body : this.bodies) {
            body.update(FRAME_TIME);
        }
    }

    /**
     * Returns the ideal tangential speed for a small body in a circular orbit
     * around a dominant central mass: v = sqrt(GM/r).
     */
    public float circularOrbitSpeed(float centralMass, float orbitalRadius) {
        if (!Float.isFinite(centralMass) || centralMass <= 0) {
            throw new IllegalArgumentException("Central mass must be a positive, finite value");
        }
        if (!Float.isFinite(orbitalRadius) || orbitalRadius <= 0) {
            throw new IllegalArgumentException("Orbital radius must be a positive, finite value");
        }

        return (float) Math.sqrt(GRAVITATIONAL_CONSTANT * centralMass / orbitalRadius);
    }

    public int size() {
        return this.bodies.size();
    }

    public PVector getLocation(int index) {
        return getBody(index).getLocation().copy();
    }

    public PVector getVelocity(int index) {
        return getBody(index).getVelocity().copy();
    }

    public float getMass(int index) {
        return getBody(index).getMass();
    }

    public float getRadius(int index) {
        return getBody(index).getRadius();
    }

    private Body getBody(int index) {
        if (index < 0 || index >= bodies.size()) {
            throw new IndexOutOfBoundsException("Body index: " + index);
        }
        return bodies.get(index);
    }

    public static void main(String[] args) {
        PApplet.main(Simulation.class, args);
    }

    /** A body with position, velocity, accumulated acceleration, and mass. */
    private static final class Body {
        private static final float MINIMUM_RADIUS = 2.0f;

        private final PVector location;
        private final PVector velocity;
        private final PVector acceleration;
        private final float mass;

        private Body(PVector location, float mass, PVector velocity) {
            if (!Float.isFinite(mass) || mass <= 0) {
                throw new IllegalArgumentException("Mass must be a positive, finite value");
            }

            this.location = location.copy();
            this.velocity = velocity.copy();
            this.mass = mass;
            this.acceleration = new PVector();
        }

        private void applyGravity(Body other, float gravitationalConstant) {
            PVector displacement = PVector.sub(other.location, this.location);

            // Plummer-style softening removes the 1/r^2 singularity. Using body
            // radii for epsilon also prevents near-overlaps from creating huge kicks.
            float softening = (this.getRadius() + other.getRadius()) * 0.5f;
            float softenedDistanceSquared = displacement.magSq() + softening * softening;
            float softenedDistanceCubed = softenedDistanceSquared
                    * (float) Math.sqrt(softenedDistanceSquared);

            // a = G * otherMass * displacement / (distance^2 + epsilon^2)^(3/2)
            displacement.mult(gravitationalConstant * other.mass / softenedDistanceCubed);
            this.acceleration.add(displacement);
        }

        private void update(float timeStep) {
            // Semi-implicit Euler: update velocity before position. It conserves
            // orbital energy better than explicit Euler at the same timestep.
            this.velocity.x += this.acceleration.x * timeStep;
            this.velocity.y += this.acceleration.y * timeStep;
            this.velocity.z += this.acceleration.z * timeStep;

            this.location.x += this.velocity.x * timeStep;
            this.location.y += this.velocity.y * timeStep;
            this.location.z += this.velocity.z * timeStep;

            this.acceleration.mult(0);
        }

        private PVector getLocation() {
            return location;
        }

        private PVector getVelocity() {
            return velocity;
        }

        private float getMass() {
            return mass;
        }

        private float getRadius() {
            // In 2D, sqrt(mass) makes visual area approximately proportional to mass.
            return Math.max(MINIMUM_RADIUS, (float) Math.sqrt(this.mass));
        }
    }
}
