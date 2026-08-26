package com.malviys;

import processing.core.PApplet;
import processing.core.PVector;

/** Processing sketch responsible for creating and drawing the orbital system. */
public class MySketch extends PApplet {
    private static final float MINIMUM_ORBIT_RADIUS = 100;
    private static final float MAXIMUM_ORBIT_RADIUS = 180;

    Simulation simulation;

    // Number of orbiters; the central body is added separately. Runtime grows
    // with the square of this value because every body attracts every other body.
    int orbitingBodyCount = 10_00;

    /** Processing calls settings before setup to create the drawing surface. */
    public void settings() {
        size(800, 600);
    }

    /** Builds a central mass and a configurable set of initially circular orbiters. */
    public void setup() {
        background(0);
        float centralMass = 200;
        float orbitingMass = 0.1f;

        simulation = new Simulation(orbitingBodyCount + 1L);
        float centerX = width / 2.0f;
        float centerY = height / 2.0f;

        simulation.addBody(new Body(
                new PVector(centerX, centerY),
                centralMass,
                new PVector()
        ));

        // Spread positions by both angle and radius. Velocity is perpendicular
        // to the radius vector, which provides the tangential circular motion.
        for (int i = 0; i < orbitingBodyCount; i++) {
            float radiusProgress = orbitingBodyCount == 1
                    ? 0
                    : (float) i / (orbitingBodyCount - 1);
            float radius = lerp(MINIMUM_ORBIT_RADIUS, MAXIMUM_ORBIT_RADIUS, radiusProgress);
            float angle = TWO_PI * i / orbitingBodyCount;
            float orbitSpeed = simulation.circularOrbitSpeed(centralMass, radius);

            var location = new PVector(
                    centerX + cos(angle) * radius,
                    centerY + sin(angle) * radius
            );
            var velocity = new PVector(
                    -sin(angle) * orbitSpeed,
                    cos(angle) * orbitSpeed
            );

            simulation.addBody(new Body(location, orbitingMass, velocity));
        }
    }

    /** Advances physics once and redraws the complete scene. */
    public void draw() {
        background(225);
        fill(255, 0, 0);

        simulation.run();
        simulation.getBodies().forEach(p -> {
            var loc = p.getLocation();
            var diameter = p.getRadius() * 2;
            ellipse(loc.x, loc.y, diameter, diameter);
        });

        fill(0);
        rect(5, 5, 75, 40);
        fill(255);
        text("FPS: " + nf(frameRate, 2, 1), 10, 20);
        text("Bodies: " + simulation.getBodies().size(), 10, 40);
    }

    public static void main(String[] args) {
        PApplet.main(MySketch.class, args);
    }
}
