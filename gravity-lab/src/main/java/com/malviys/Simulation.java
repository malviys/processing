package com.malviys;

import processing.core.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Advances a collection of gravitational bodies by one rendered-frame interval.
 * Physics is calculated in smaller substeps to make close encounters more stable.
 */
public class Simulation {
    // These are simulation units rather than real-world SI units.
    private static final float GRAVITATIONAL_CONSTANT = 0.98f;
    private static final int PHYSICS_SUBSTEPS = 16;
    private static final float FRAME_TIME = 1.0f;

    private final List<Body> bodies;

    public Simulation(long particleCount) {
        if (particleCount < 0 || particleCount > Integer.MAX_VALUE) {
            throw new IllegalArgumentException("Particle count is outside the supported range");
        }

        this.bodies = new ArrayList<>((int) particleCount);
    }

    public void addBody(Body body) {
        this.bodies.add(body);
    }

    public void addBody(Body ...bodies) {
        this.bodies.addAll(Arrays.asList(bodies));
    }

    /**
     * Advances the simulation by one frame. Every substep first accumulates all
     * accelerations from the same position state, then moves all bodies together.
     */
    public void run() {
        float timeStep = FRAME_TIME / PHYSICS_SUBSTEPS;

        for (int step = 0; step < PHYSICS_SUBSTEPS; step++) {
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
                body.update(timeStep);
            }
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

    public List<Body> getBodies() {
        return Collections.unmodifiableList(this.bodies);
    }
}

/** A body with position, velocity, accumulated acceleration, and mass. */
class Body {
    private static final float MINIMUM_RADIUS = 2.0f;

    private PVector location;

    private PVector velocity;

    private PVector acceleration;

    private float mass;

    public Body(PVector location, float mass, PVector velocity) {
        if (!Float.isFinite(mass) || mass <= 0) {
            throw new IllegalArgumentException("Mass must be a positive, finite value");
        }

        this.location = location;
        this.velocity = velocity;
        this.mass = mass;
        this.acceleration = new PVector(0, 0);
    }

    public void applyGravity(Body other, float gravitationalConstant) {
        var displacement = PVector.sub(other.location, this.location);

        // Plummer-style softening removes the 1/r^2 singularity. Using body
        // radii for epsilon also prevents near-overlaps from creating huge kicks.
        var softening = (this.getRadius() + other.getRadius()) * 0.5f;
        var softenedDistanceSquared = displacement.magSq() + softening * softening;
        var softenedDistanceCubed = softenedDistanceSquared * (float) Math.sqrt(softenedDistanceSquared);

        // a = G * otherMass * displacement / (distance^2 + epsilon^2)^(3/2)
        displacement.mult(gravitationalConstant * other.mass / softenedDistanceCubed);
        this.acceleration.add(displacement);
    }

    public void update(float timeStep) {
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

    public PVector getLocation() {
        return location;
    }

    public float getRadius() {
        // In 2D, sqrt(mass) makes visual area approximately proportional to mass.
        return Math.max(MINIMUM_RADIUS, (float) Math.sqrt(this.mass));
    }
}
