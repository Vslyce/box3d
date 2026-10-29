# Reusable moving-mesh support

This fork carries kinematic-mesh fixes on top of Erin Catto's Box3D at
`336263320469e5e9febfe18697ab562488f7e97a` (2026-10-27, Dynamic Mover).
The changes originated in Esoterica's reproduction cases, then were adapted to
the current standalone library. No Engine, planet, renderer, or project asset is
required. Catto's mover work and the existing float/double build options remain available.

## What changes and why

- **Triangle cache coverage follows the mesh.** A stationary convex object can
  encounter different triangles when its supporting mesh moves. Cache validity
  is checked in mesh space. For kinematic meshes, relative positions are formed
  before conversion to float, preserving small motion at large world coordinates.
- **Triangle candidates cover relative motion.** Candidate bounds account for
  mesh translation and the intervening rotation arc, alongside the convex sweep.
  The opt-in query avoids a centroid-radius rejection that can skip a slowly
  approaching hull face. The normal stationary-mesh TOI fast path remains available.
- **Moving meshes wake eligible sleepers before contact generation.** Masks,
  collision groups, joint exclusions, sensors and custom filters are honored.
  Iteration remains valid when waking islands reallocates the solver arrays.
- **The impact query has explicit controls.** Shape-pair and triangle budgets
  replace the original world-population and fixed-timestep limits. Results include
  hit shape IDs and diagnostic status. Spatial pruning avoids a full Cartesian
  product of meshes and dynamic shapes; a pass over live bodies is still required.
- **Embedding policy is optional.** `include/box3d/config.h` retains upstream
  defaults. `examples/esoterica_config.h` demonstrates assertions and DLL macro
  mapping through the existing `BOX3D_USER_CONFIG` hook. Use the same configuration
  for the library and all consumers; enabling it requires no Engine headers.

The mesh-cache and wake fixes participate in ordinary world stepping. Applications
must explicitly use the query and subdivide whole-world steps to benefit from the
new impact scheduling. This does not add automatic mesh CCD to `b3World_Step`.

## Query contract

```c
b3MeshImpactQueryDef def = b3DefaultMeshImpactQueryDef();
def.timeStep = 1.0f / 240.0f;
def.maxPairs = 1024;
def.maxTriangles = 65536;
b3MeshImpactResult result;
b3MeshImpactQueryStatus status =
    b3World_QueryKinematicMeshImpactWithOptions(world, &def, &result);
```

The query covers enabled, non-sensor kinematic **mesh shapes** against dynamic
spheres, capsules and convex hulls. Frozen kinematic meshes are included, so a
moving convex shape can approach them. Static meshes use Box3D's existing CCD;
dynamic meshes, moving heightfields and mesh children inside compounds are outside
this query's scope. Mesh triangles retain their normal one-sided behavior.

| Status | Meaning |
| --- | --- |
| `b3_meshImpactComplete` | The result is usable. |
| `b3_meshImpactInvalidInput` | Invalid arguments/world ID or nonfinite predicted motion. |
| `b3_meshImpactWorldLocked` | Called during a step or a reentrant query. |
| `b3_meshImpactCapacityExceeded` | A pair/triangle budget or counter capacity was exhausted. |
| `b3_meshImpactUnsupportedShape` | An eligible dynamic candidate is not convex. |
| `b3_meshImpactFailed` | The underlying TOI calculation failed to converge. |

On every failure the caller's result remains unchanged; a partial hit is never
reported as a complete answer. On success, `fraction` is the earliest positive
TOI fraction found, or `1` with null IDs if none was found before the endpoint.
An exact-endpoint impact also leaves `fraction == 1`. Initial overlap is handled
by the contact solver; the existing centroid fallback can produce a later
positive fraction for an initially overlapping shape.

The default budgets are 1024 candidate shape pairs and 65536 candidate triangles.
Zero disables either limit. These count candidates after the relevant spatial
and collision filtering, including triangles later rejected by one-sidedness.
A triangle may perform a main and fallback TOI calculation. Budgets do **not**
bound body enumeration, tree node visits, custom-filter work, allocations inside
user callbacks, or elapsed time. The query itself allocates no memory, advances
no body and wakes no sleeper. Custom filters must not modify the world and may
be called again on later queries.

The older `b3World_QueryKinematicMeshImpact` function remains a compatibility
wrapper. It requires a positive `maxPairs`, has no triangle budget and returns
`false` on any failure, leaving both outputs untouched. Its timestep and
world-population restrictions have been removed; pair counts now describe
spatial candidates, rather than all potentially collidable pairs in the world.

## Integrating the prediction

Prediction uses a single free-motion integration over `timeStep`, including
current velocities, forces/gravity, torque, damping, motion locks and speed
limits. Sleeping dynamics start at zero velocity and include gravity as if
awakened. Rotation uses Box3D's endpoint quaternion interpolation. Contact and
joint impulses, gyroscopic torque and changes made by the application during
the interval are not predicted. It is an approximate scheduling aid, not a
guarantee against tunneling under arbitrary motion or a complete trajectory solver.

Start with a small fixed interval. Query it, advance the **whole world** to the
predicted fraction, then query the remainder using the updated state. Keep an
application-level subdivision limit and minimum-progress policy. A budget or
progress failure needs explicit handling: stop, raise the budget, or retry a
smaller interval. Never treat an incomplete query as clear space. Previously
completed partial steps are not rolled back. Each `b3World_Step` clears applied
forces, so persistent forces must be reapplied for each partial step. Smaller
steps can also change damping and contact response.

[`examples/moving_mesh.c`](examples/moving_mesh.c) demonstrates this loop on a
constant-velocity platform and reports failure with the remaining interval.
The loop's 240 Hz interval and subdivision limits are example policy, not
library constraints.

## Build and run

The regular upstream unit-test executable includes `MovingMeshTest`:

```sh
cmake -S . -B build/float -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=ON -DBOX3D_COMPILE_WARNING_AS_ERROR=ON
cmake --build build/float --config RelWithDebInfo --target test
cmake -S . -B build/double -DBOX3D_SAMPLES=OFF -DBOX3D_UNIT_TESTS=ON -DBOX3D_DOUBLE_PRECISION=ON -DBOX3D_COMPILE_WARNING_AS_ERROR=ON
cmake --build build/double --config RelWithDebInfo --target test
```

Run `build/<mode>/bin/RelWithDebInfo/test.exe` on Windows. Pass `MovingMeshTest`
to run only the new suite. For a single-configuration generator, configure with
`-DCMAKE_BUILD_TYPE=RelWithDebInfo` and run `build/<mode>/bin/test`.

The example also demonstrates embedding Box3D with `add_subdirectory`:

```sh
cmake -S examples -B build/example
cmake --build build/example --config Release
```

Run `build/example/bin/Release/moving_mesh.exe` on Windows, or
`build/example/bin/moving_mesh` for a single-configuration generator. Add
`-DBOX3D_EXAMPLE_ESOTERICA_CONFIG=ON` to exercise the optional configuration.
`-DBUILD_SHARED_LIBS=ON` and `-DBOX3D_DOUBLE_PRECISION=ON` are also supported.

## Validation and provenance

Local validation on 2026-10-28 used Visual Studio 18 2026, MSVC x64, with
warnings treated as errors:

| Build | Result |
| --- | --- |
| Float, RelWithDebInfo | All 29 test suites passed, including the 12 moving-mesh subtests. |
| Double, RelWithDebInfo | All 29 test suites passed. |
| Double, Debug with heavy validation | All 29 test suites passed. |
| Public C example, default float/static Release | Built and ran; one impact split. |
| Public C example, double/shared Release with optional config | Built and ran; one impact split. |

Existing determinism expectations were not changed. The graphical samples and
non-Windows toolchains were not run locally. The new suite is included in the
existing upstream test executable and therefore in its CI build matrix.

The new standalone C tests cover analytical translation/rotation impact times,
quaternion sign equivalence, moving triangle-cache coverage, spheres/capsules/hulls
approached by translating and rotating meshes, fast convex motion toward a frozen
mesh, supported sleeping bodies, 30 seconds of rotating-platform contact,
filtering, sensor exclusion, wake-up across many islands, relative velocities,
damping/forces/locks, earliest hit IDs, repeated read-only queries, transactional
budget failures, sparse IDs, and 2000 remote bodies under a one-pair budget.
Cache and query tests include common offsets of 10,000 units and, in double
mode, 100,000,000 units. These are bounded regression fixtures; they do not
establish arbitrary-scene performance or universal large-world accuracy.

The original imported snapshot remains on `codex/esoterica-box3d-changes` at
`1614ac7cb7f3072589c8b1fddbff284e37a4972c`, based on upstream `47d7f7c`.
This branch supersedes its Engine-specific configuration and limited query;
it is not a byte-identical vendor copy. Esoterica's vendored library and adapters
must be migrated separately if they are to use this revision.
