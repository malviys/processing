# Processing Experiments

This repository is a laboratory for exploring application performance with
Java, the JVM, and the hardware beneath them. The goal is to understand how far
a straightforward solution can be pushed by using the platform well before
replacing it with a different algorithm.

## Why this repository exists

Choosing a better algorithm is often the largest possible optimization, but it
is not the only kind of optimization. An application is also shaped by its data
layout, memory-access patterns, concurrency model, generated machine code, and
the processor that executes that code.

The experiments in this repository therefore keep a problem and its semantics
recognizable while trying different Java and JVM facilities. An `O(n²)`
calculation is one useful workload for this investigation, not a requirement
for every experiment in the repository. The broader question is:

> How can Java code and data be designed so that the JVM can optimize them well
> and the CPU can execute them efficiently, without first inventing a new
> algorithm?

## What the experiments explore

The work spans three connected layers:

- **Java and application design:** object-oriented and data-oriented models,
  primitive storage, multithreading, the Vector API, and Arena-backed native
  memory.
- **JVM execution:** runtime profiling, tiered compilation, and the transition
  from quickly compiled code to highly optimized, architecture-specific machine
  code. In HotSpot, C1 supports fast compilation and profiling while C2 performs
  more aggressive optimization of hot code.
- **CPU execution:** cache lines, locality, prefetching, SIMD instructions, and
  processor-level behavior such as instruction scheduling and reordering.

These layers influence one another. The JVM can only generate effective vector
instructions when a loop and its data are suitable, and a fast instruction
sequence can still spend most of its time waiting for scattered memory.

## Designing data for the processor

The repository also examines how data-structure choices affect the hardware. A
linked list, for example, requires pointer chasing between nodes that may be
scattered across memory. Flat, sequential data gives the CPU a better chance to
load useful cache lines, prefetch upcoming values, and process several values at
once.

An `ArrayList` stores its references contiguously, but the referenced objects
may still be spread throughout the heap. Primitive arrays, structure-of-arrays
layouts, and contiguous native-memory regions make the locality trade-off more
explicit. The aim is not to declare one representation universally best, but to
learn when a representation matches the way a workload is processed.

## Experiments

- [Gravity Lab](gravity-lab/README.md) uses an exact pairwise particle
  simulation to compare object-based, primitive, multithreaded, Arena-backed,
  SIMD, and experimental GPU approaches. It uses particle count and frames per
  second as visible measurements while preserving the same fundamental
  calculation.

Each experiment has its own README for implementation details, requirements,
and observations. This document describes the shared motivation rather than
duplicating those details.
