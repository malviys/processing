package com.malviys;

import org.junit.jupiter.api.Test;
import processing.core.PVector;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SimulationTest {
    @Test
    void circularOrbitSpeedUsesSimulationGravity() {
        var simulation = new Simulation(2);

        assertEquals(1.4f, simulation.circularOrbitSpeed(200, 100), 0.0001f);
    }

    @Test
    void bodyRadiusScalesWithSquareRootOfMass() {
        var body = new Body(new PVector(), 100, new PVector());

        assertEquals(10, body.getRadius());
    }

    @Test
    void initialOrbitalSystemDoesNotEjectBodies() {
        float[] orbitalRadii = {100, 120, 130, 140, 150, 160, 170, 180};
        float centralMass = 200;
        var simulation = new Simulation(orbitalRadii.length + 1);
        simulation.addBody(new Body(new PVector(400, 300), centralMass, new PVector()));

        for (int i = 0; i < orbitalRadii.length; i++) {
            float radius = orbitalRadii[i];
            float angle = (float) (Math.PI * 2 * i / orbitalRadii.length);
            float speed = simulation.circularOrbitSpeed(centralMass, radius);
            simulation.addBody(new Body(
                    new PVector(
                            400 + (float) Math.cos(angle) * radius,
                            300 + (float) Math.sin(angle) * radius
                    ),
                    0.1f,
                    new PVector(
                            -(float) Math.sin(angle) * speed,
                            (float) Math.cos(angle) * speed
                    )
            ));
        }

        for (int frame = 0; frame < 1_000; frame++) {
            simulation.run();
        }

        for (Body body : simulation.getBodies()) {
            var location = body.getLocation();
            assertTrue(location.x >= 0 && location.x <= 800);
            assertTrue(location.y >= 0 && location.y <= 600);
            assertFinite(location);
        }
    }

    @Test
    void overlappingBodiesRemainFinite() {
        var firstBody = new Body(new PVector(10, 10), 10, new PVector());
        var secondBody = new Body(new PVector(10, 10), 20, new PVector());
        var simulation = new Simulation(2);
        simulation.addBody(firstBody, secondBody);

        simulation.run();

        assertFinite(firstBody.getLocation());
        assertFinite(secondBody.getLocation());
    }

    private static void assertFinite(PVector vector) {
        assertTrue(Float.isFinite(vector.x));
        assertTrue(Float.isFinite(vector.y));
        assertTrue(Float.isFinite(vector.z));
    }
}
