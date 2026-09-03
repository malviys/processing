# Gravity Lab

An interactive 2D gravitational particle simulation built with Java 24 and
[Processing](https://processing.org/). The project contains several versions of
the same n-body simulation for comparing object-oriented, data-oriented,
off-heap, multithreaded, and SIMD implementations.

## Why Gravity Lab exists

Gravity Lab is a Java, JVM, and hardware performance experiment presented as a
visual simulation. It asks how far a straightforward implementation can be
pushed by changing how the application represents data and uses the platform,
rather than immediately replacing its algorithm.

The exact pairwise gravity calculation has `O(n²)` time complexity. That is the
algorithm chosen for this case study, not the defining idea of the wider
project. A hierarchical or approximate gravity algorithm could reduce the
complexity, but it would answer a different question. Gravity Lab deliberately
retains the pairwise calculation so the effects of Java features, JVM
optimization, memory layout, parallel execution, and CPU behavior can be
observed against the same workload.

The long-term challenge is to explore whether this simple calculation can be
pushed toward particle counts on the order of one million. That is an
aspirational performance target, not a claim that every implementation already
supports it. Frames per second and particle count provide visible measurements
of progress; they are not the purpose of the project by themselves.

## What the variants investigate

The implementations explore several complementary questions:

- What is the runtime cost of a conventional object graph compared with flat
  primitive data?
- How do structure-of-arrays layouts and sequential access affect CPU cache
  locality and prefetching?
- When does dividing the pairwise work across Java threads overcome the cost of
  coordination and per-worker state?
- How can the Vector API expose SIMD work while allowing the JVM to select a
  suitable vector width for the current processor?
- Can Arena-backed native memory provide a predictable contiguous layout and
  different lifetime-management trade-offs from ordinary heap objects?
- How does HotSpot turn the same Java workload into optimized machine code for
  the host CPU?
- How can a Java/Processing application interact with GPU shaders as a step
  toward moving suitable work to the GPU?

HotSpot's tiered compilation is part of that investigation. C1 compiles and
profiles methods quickly, while C2 spends more effort optimizing frequently
executed code and producing machine code specialized for the current CPU
architecture. The resulting performance is a collaboration between application
design, JVM decisions, and processor behavior rather than a property of the
source code alone.

## Data layout and CPU locality

Gravity Lab compares designs that make different demands on the memory system.
A linked structure or object-rich model can require the CPU to follow references
to data scattered throughout the heap. This pointer chasing can cause cache
misses and makes prefetching and vectorization more difficult.

Sequential primitive arrays and contiguous Arena allocations instead place
related streams of values in predictable regions. This helps the CPU fetch data
in cache-line-sized blocks and gives both the JVM and processor clearer
opportunities for SIMD execution. An `ArrayList` makes its references
contiguous, but the objects behind those references may still be scattered, so
it does not provide the same layout as flat primitive values.

These comparisons are examples of data-oriented design: organizing an
application around how its hot data is consumed, while preserving the behavior
of the original solution.

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
The data-oriented variants visit each unique pair once. For `N` particles, one
frame therefore evaluates `N × (N - 1) / 2` unique particle pairs. Each pair
interaction then performs several arithmetic, memory-access, and force-update
operations, so the pair count is a measure of work rather than a literal CPU
instruction count.

On a 60 Hz display, at most 60 distinct frames can be presented each second.
Sustaining that rate gives the application about **16.67 milliseconds per
frame**. The force calculation, integration, and rendering all have to fit
inside that budget. See the [FPS Visualizer][fps-visualizer] for a visual
comparison and the [FPS Calculator][fps-calculator] for the `1000 / FPS`
frame-time relationship.

The quadratic growth becomes visible quickly:

| Particles | Pair interactions per frame | Pair interactions per second at 60 FPS |
| ---: | ---: | ---: |
| 1,000 | 499,500 | 29,970,000 |
| 10,000 | 49,995,000 | 2,999,700,000 |
| 100,000 | 4,999,950,000 | 299,997,000,000 |
| 1,000,000 | 499,999,500,000 | 29,999,970,000,000 |

The default system contains 1,001 bodies and evaluates 500,500 unique pairs per
frame. Increasing the particle count by a factor of ten increases the pairwise
work by approximately a factor of one hundred.

### Simplified CPU throughput model

The following estimate assumes a processor sustains **one billion abstract work
units per second** and that one frame requires `n²` such units. This is a
deliberately simple model, not a claim that every CPU core performs one billion
complete gravity interactions per second. A real interaction contains multiple
machine instructions and memory accesses, while SIMD and multiple cores may
execute some of that work in parallel.

| Particles | `n²` work units per frame | Estimated compute time | Compute-limited rate | Visible rate on 60 Hz |
| ---: | ---: | ---: | ---: | ---: |
| 100 | 10,000 | 0.01 ms | 100,000 FPS | Up to 60 FPS |
| 500 | 250,000 | 0.25 ms | 4,000 FPS | Up to 60 FPS |
| 1,000 | 1,000,000 | 1 ms | 1,000 FPS | Up to 60 FPS |
| 2,000 | 4,000,000 | 4 ms | 250 FPS | Up to 60 FPS |
| 4,000 | 16,000,000 | 16 ms | 62.5 FPS | Up to 60 FPS |
| 5,000 | 25,000,000 | 25 ms | 40 FPS | Up to 40 FPS |
| 10,000 | 100,000,000 | 100 ms | 10 FPS | Up to 10 FPS |
| 100,000 | 10,000,000,000 | 10 seconds | 0.1 FPS | Up to 0.1 FPS |
| 1,000,000 | 1,000,000,000,000 | 1,000 seconds | 0.001 FPS | Up to 0.001 FPS |

Under this model, approximately 4,000 particles consume almost the entire
16.67 ms frame budget before integration and rendering are considered. Doubling
the particle count quadruples the work, so a compute-bound frame rate falls to
roughly one quarter. For example, 40 FPS becoming 10 FPS is a fourfold slowdown,
or a 75% decrease in frame rate.

These figures illustrate quadratic scaling; they are not measured Gravity Lab
benchmarks. Real results depend on rendering costs, JVM warm-up, cache behavior,
memory bandwidth, SIMD width, thread scheduling, processor architecture, and
the implementation being tested. Measurements from the running application
remain the source of truth for a particular machine.

- `SimulationMT` distributes pair ranges across CPU workers.
- `SimulationSIMDV` processes several second bodies per SIMD operation.
- `SimulationArena` keeps component values contiguous in off-heap memory.

These approaches improve execution and memory behavior but do not change the
quadratic algorithm. At very large counts, especially near one million
particles, hardware limits make a CPU-only exact calculation unlikely to fit
within a real-time frame budget. The experiment may therefore need to move the
parallel force calculation to the GPU, where many particle interactions can be
evaluated concurrently. Doing so changes the execution hardware but still does
not remove the `O(n²)` growth or guarantee 60 FPS. A GPU may move the practical
limit much further, but quadratic growth eventually catches up with any fixed
amount of compute throughput.

A spatial approximation such as Barnes-Hut would be another path for much
larger systems because it changes the algorithmic complexity. Gravity Lab keeps
that option separate from its initial investigation: first learn how far the
original calculation can be pushed through Java, JVM, CPU, memory-layout, and
GPU techniques, then identify the point at which the algorithm itself must be
improved.

## Current limitations

- Bodies do not collide, bounce, or merge.
- There are no canvas boundaries.
- The units are intended for visualization and are not SI units.
- Semi-implicit Euler accumulates numerical error over long runs.
- Circular speed assumes the central body dominates the system.

## Tests

`SimulationTest` checks circular-orbit speed, radius scaling, long-running orbit
stability, and finite results for overlapping bodies.

[fps-visualizer]: https://www.techcompare.app/fps-visualizer
[fps-calculator]: https://alathasiba.com/en/calculator/fps-calculator
