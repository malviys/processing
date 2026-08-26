# Particle Simulation

An interactive 2D gravitational particle simulation built with Java and
[Processing](https://processing.org/). A central body and a configurable number
of smaller bodies move under their mutual gravity.

## Running the project

The Maven project is configured for Java 24.

1. Import the repository as a Maven project in IntelliJ IDEA or another Java IDE.
2. Select a Java 24 JDK.
3. Run `com.malviys.MySketch.main()` from the `particle-simulation` module.

Run the automated checks from the repository root:

```shell
./mvnw -pl particle-simulation -am test
```

## Configuration

The main visual configuration is near the top of `MySketch.java`:

```java
private static final float MINIMUM_ORBIT_RADIUS = 100;
private static final float MAXIMUM_ORBIT_RADIUS = 180;

int orbitingBodyCount = 10_00;
```

`orbitingBodyCount` does not include the central body. Therefore, a value of
`1000` creates 1001 total bodies.

The initial masses are configured in `setup()`:

```java
float centralMass = 200;
float orbitingMass = 0.1f;
```

The orbiters are intentionally much lighter than the center. This makes the
central mass dominate the system and reduces chaotic scattering between
orbiters.

## Initial orbital system

The central body starts at the center of the `800 x 600` canvas with zero
velocity. Orbiting bodies are distributed across:

- angles from `0` through `2π`;
- radii between `MINIMUM_ORBIT_RADIUS` and `MAXIMUM_ORBIT_RADIUS`.

For an orbital position with angle `θ` and radius `r`, its location is:

```text
x = centerX + cos(θ) * r
y = centerY + sin(θ) * r
```

The ideal circular speed around a dominant central mass is:

```text
v = sqrt(G * centralMass / r)
```

Velocity points perpendicular to the radius:

```text
vx = -sin(θ) * v
vy =  cos(θ) * v
```

This gives each body tangential motion instead of sending it directly toward or
away from the center.

## Gravity calculation

For each body, the simulation adds the acceleration caused by every other body.
The gravitational constant is `G = 0.98` in simulation units.

Plain Newtonian gravity contains a division by `distance²`. At zero distance,
that becomes infinite and can produce `NaN` values. Very small distances can
also create enormous acceleration and throw bodies off screen.

This project uses softened gravity:

```text
acceleration = G * otherMass * displacement
               / (distance² + epsilon²)^(3/2)
```

The softening length `epsilon` is half the sum of the two bodies' visual radii.
At long distances the equation behaves like ordinary gravity. At short
distances it smoothly limits the attraction and remains finite, including when
two bodies overlap exactly.

The accelerated body's own mass is absent from this equation because it cancels
when gravitational force is divided by that body's mass:

```text
F = G * firstMass * secondMass / distance²
a = F / firstMass
```

## Simulation step

One call to `Simulation.run()` represents one rendered frame. It divides that
frame into 16 physics substeps:

```text
substep time = 1 / 16
```

Each substep has two phases:

1. Calculate and accumulate gravity for every body without moving anything.
2. Update all velocities and positions together.

Separating these phases matters. If a body moved immediately after calculating
its force, later bodies would see a mixture of old and new positions. The result
would depend on list order and would not preserve force symmetry.

Position and velocity use semi-implicit Euler integration:

```text
velocity = velocity + acceleration * dt
position = position + velocity * dt
```

Updating velocity first is generally more stable for orbital motion than
explicit Euler integration. Acceleration is reset after each substep because it
will be recalculated from the next set of positions.

## Body size

Visual radius is calculated from mass:

```text
radius = max(2, sqrt(mass))
```

The square root keeps a heavy body from becoming excessively large and makes its
2D area approximately proportional to mass. The minimum radius keeps very light
orbiters visible.

## Performance

The current force calculation compares every body with every other body, making
its complexity:

```text
O(substeps * bodyCount²)
```

With 1001 total bodies and 16 substeps, one rendered frame performs about 16
million directed force calculations. Reduce `orbitingBodyCount` if the frame
rate is too low. For much larger systems, a Barnes-Hut quadtree would reduce the
approximate force calculation to `O(n log n)`.

## Current limitations

- Bodies do not collide, bounce, or merge; softened gravity only prevents force
  singularities.
- There are no canvas boundaries. A sufficiently disturbed body can leave the
  visible area.
- The units are designed for visualization and are not SI units.
- Semi-implicit Euler is stable enough for this demonstration but still
  accumulates numerical error over long runs.
- Circular speed assumes the center dominates and does not include every
  orbiter's gravitational perturbation.

## Tests

`SimulationTest` verifies that:

- circular speed uses the configured gravity constant;
- visual radius scales with mass;
- the initial orbital arrangement remains on screen for 1000 frames;
- overlapping bodies remain finite instead of producing `NaN`.
