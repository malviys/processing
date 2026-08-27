package com.malviys;

import org.junit.jupiter.api.Test;
import processing.core.PVector;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SimulationTest {
    @Test
    void circularOrbitSpeedUsesSimulationGravity() {
        var simulation = new Simulation(2);

        assertEquals(4.427189f, simulation.circularOrbitSpeed(200, 100), 0.0001f);
    }

    @Test
    void bodyRadiusScalesWithSquareRootOfMass() {
        var simulation = new Simulation(1);
        int body = simulation.addBody(new PVector(), 100, new PVector());

        assertEquals(10, simulation.getRadius(body));
    }

    @Test
    void initialOrbitalSystemDoesNotEjectBodies() {
        float[] orbitalRadii = {100, 120, 130, 140, 150, 160, 170, 180};
        float centralMass = 200;
        var simulation = new Simulation(orbitalRadii.length + 1);
        simulation.addBody(new PVector(400, 300), centralMass, new PVector());

        for (int i = 0; i < orbitalRadii.length; i++) {
            float radius = orbitalRadii[i];
            float angle = (float) (Math.PI * 2 * i / orbitalRadii.length);
            float speed = simulation.circularOrbitSpeed(centralMass, radius);
            simulation.addBody(
                    new PVector(
                            400 + (float) Math.cos(angle) * radius,
                            300 + (float) Math.sin(angle) * radius
                    ),
                    0.1f,
                    new PVector(
                            -(float) Math.sin(angle) * speed,
                            (float) Math.cos(angle) * speed
                    )
            );
        }

        for (int frame = 0; frame < 1_000; frame++) {
            simulation.run();
        }

        for (int body = 0; body < simulation.size(); body++) {
            var location = simulation.getLocation(body);
            assertTrue(location.x >= 0 && location.x <= 800);
            assertTrue(location.y >= 0 && location.y <= 600);
            assertFinite(location);
        }
    }

    @Test
    void overlappingBodiesRemainFinite() {
        var simulation = new Simulation(2);
        int firstBody = simulation.addBody(new PVector(10, 10), 10, new PVector());
        int secondBody = simulation.addBody(new PVector(10, 10), 20, new PVector());

        simulation.run();

        assertFinite(simulation.getLocation(firstBody));
        assertFinite(simulation.getLocation(secondBody));
    }

    @Test
    void arraySimulationMatchesObjectSimulation() {
        var simulation = new Simulation(9);
        var arraySimulation = new SimulationSIMD(9);
        float centralMass = 200.0f;

        addToBoth(
                simulation,
                arraySimulation,
                new PVector(400, 300),
                centralMass,
                new PVector()
        );

        for (int i = 0; i < 8; i++) {
            float radius = 100.0f + i * 10.0f;
            float angle = (float) (Math.PI * 2.0 * i / 8.0);
            float speed = simulation.circularOrbitSpeed(centralMass, radius);
            addToBoth(
                    simulation,
                    arraySimulation,
                    new PVector(
                            400 + (float) Math.cos(angle) * radius,
                            300 + (float) Math.sin(angle) * radius
                    ),
                    0.1f,
                    new PVector(
                            -(float) Math.sin(angle) * speed,
                            (float) Math.cos(angle) * speed
                    )
            );
        }

        for (int frame = 0; frame < 1_000; frame++) {
            simulation.run();
            arraySimulation.run();
        }

        assertEquals(simulation.size(), arraySimulation.size());
        for (int body = 0; body < simulation.size(); body++) {
            PVector expectedLocation = simulation.getLocation(body);
            PVector actualLocation = arraySimulation.getLocation(body);
            PVector expectedVelocity = simulation.getVelocity(body);
            PVector actualVelocity = arraySimulation.getVelocity(body);

            assertEquals(expectedLocation.x, actualLocation.x, 0.0f);
            assertEquals(expectedLocation.y, actualLocation.y, 0.0f);
            assertEquals(expectedVelocity.x, actualVelocity.x, 0.0f);
            assertEquals(expectedVelocity.y, actualVelocity.y, 0.0f);
            assertEquals(simulation.getMass(body), arraySimulation.getMass(body), 0.0f);
            assertEquals(simulation.getRadius(body), arraySimulation.getRadius(body), 0.0f);
        }
    }

    private static void addToBoth(
            Simulation simulation,
            SimulationSIMD arraySimulation,
            PVector location,
            float mass,
            PVector velocity
    ) {
        simulation.addBody(location, mass, velocity);
        arraySimulation.addBody(location, mass, velocity);
    }

    private static void assertFinite(PVector vector) {
        assertTrue(Float.isFinite(vector.x));
        assertTrue(Float.isFinite(vector.y));
        assertTrue(Float.isFinite(vector.z));
    }
}
