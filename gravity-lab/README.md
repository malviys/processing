# Gravity Lab

An interactive 2D gravitational particle simulation built with Java 24 and
[Processing](https://processing.org/). The project contains several versions of
the same n-body simulation for comparing object-oriented, data-oriented,
off-heap, multithreaded, and SIMD implementations.

## Requirements

- JDK 24
- Maven 3.9 or newer
- An IDE that can run Processing `PApplet` classes, such as IntelliJ IDEA

`SimulationSIMDV` uses the incubating Vector API. The Maven compiler and test
configuration in `pom.xml` already enable `jdk.incubator.vector`.

## Build

Run from this directory:

```shell
mvn compile
```

The JDK prints a warning when compiling or running incubator modules. This is
expected for the Vector API.

## Simulation implementations

| Main class | Storage and execution model |
| --- | --- |
| `Simulation` | Reference implementation using a list of body objects and `PVector` values |
| `SimulationSIMD` | Structure-of-arrays using ordinary `float[]` calculations |
| `SimulationMT` | Structure-of-arrays with parallel pair calculations and worker-local acceleration arrays |
| `SimulationArena` | Off-heap structure-of-arrays stored in one contiguous Arena allocation |
| `SimulationSIMDV` | Structure-of-arrays with explicit SIMD calculations using the incubator Vector API |
| `SimulationGPU` | Experimental Processing shader implementation |

Run the desired class's `main()` method from the IDE. For example:

```text
com.malviys.Simulation.main()
com.malviys.SimulationArena.main()
com.malviys.SimulationSIMDV.main()
```

Add this VM option when running `SimulationSIMDV`:

```text
--add-modules jdk.incubator.vector
```

## Initial orbital system

The default scene contains one central body and 1,000 orbiters on an `800 x
600` canvas. The central body has a mass of `200`, and every orbiter has a mass
of `0.1`.

Orbiters are distributed between radii `100` and `180`. For angle `theta` and
orbital radius `r`, the initial position is:

```text
x = centerX + cos(theta) * r
y = centerY + sin(theta) * r
```

The circular-orbit speed around the central mass is:

```text
speed = sqrt(G * centralMass / r)
```

Velocity is perpendicular to the radius:

```text
vx = -sin(theta) * speed
vy =  cos(theta) * speed
```

The simulation uses `G = 9.8` in visualization units.

## Gravity and integration

Every unique body pair receives equal and opposite gravitational effects. The
calculation uses softened gravity so overlapping or closely passing bodies do
not produce infinite acceleration:

```text
acceleration = G * otherMass * displacement
               / (distanceSquared + epsilonSquared)^(3/2)
```

The softening length is half the sum of the two visual radii.

One call to `run()` performs one force calculation and one full-frame
semi-implicit Euler update. There are no physics substeps:

```text
velocity = velocity + acceleration * frameTime
position = position + velocity * frameTime
```

All forces are calculated from the same position state before any body moves.

## Arena memory layout

`SimulationArena` allocates one large native-memory block through a Java
`Arena`. The block contains eight contiguous float sequences:

```text
[locationX]
[locationY]
[velocityX]
[velocityY]
[accelerationX]
[accelerationY]
[mass]
[radius]
```

For capacity `N`, each component occupies `N * Float.BYTES`. A component's byte
offset is therefore:

```text
componentOffset = componentIndex * N * Float.BYTES
```

`MemorySegment.asSlice()` turns each region into a component segment. When the
simulation outgrows its capacity, it allocates a larger block, copies the active
values component by component, and closes the Arena that owned the old block.

## Vector API implementation

`SimulationSIMDV` uses `FloatVector.SPECIES_PREFERRED`, allowing the JVM to
select the preferred SIMD width for the current processor.

For each first body, the force loop:

1. Broadcasts its position, radius, and mass across the SIMD lanes.
2. Loads a contiguous batch of second bodies from the component arrays.
3. Calculates displacement, softening, distance, and acceleration lane-wise.
4. Reduces the first body's lane results and writes the opposite acceleration
   back to the second bodies.
5. Uses a scalar loop only for the final bodies that do not fill a complete
   vector.

Velocity and position integration are also vectorized. The sketch displays the
selected SIMD lane count alongside its FPS and body count.

## Body size

Visual radius is calculated from mass when a radius is not supplied explicitly:

```text
radius = max(2, sqrt(mass))
```

The square root makes 2D area approximately proportional to mass, while the
minimum radius keeps light orbiters visible.

## Performance

The exact pairwise force calculation has `O(bodyCount²)` time complexity.
The data-oriented variants visit each unique pair once, resulting in about
500,000 pair interactions per frame for 1,001 bodies.

- `SimulationMT` distributes pair ranges across CPU workers.
- `SimulationSIMDV` processes several second bodies per SIMD operation.
- `SimulationArena` keeps component values contiguous in off-heap memory.

These approaches improve execution and memory behavior but do not change the
quadratic algorithm. A spatial approximation such as Barnes-Hut would be needed
to reduce the algorithmic complexity for much larger systems.

## Current limitations

- Bodies do not collide, bounce, or merge.
- There are no canvas boundaries.
- The units are intended for visualization and are not SI units.
- Semi-implicit Euler accumulates numerical error over long runs.
- Circular speed assumes the central body dominates the system.

## Tests

`SimulationTest` checks circular-orbit speed, radius scaling, long-running orbit
stability, and finite results for overlapping bodies.
