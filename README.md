# Mr. Golf: ball physics and 3D visualization

The C++17 physics library can run on its own. Stage 2 adds a Windows 3D demo
that follows a test shot from launch to rest and can export a fixed-rate MP4.

## Build and test

With a C++17 compiler, CMake 3.20 or newer, and a matching build tool:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

`golf_physics` is the library target. `physics_tests` is a small test runner
registered with CTest; its checks stay enabled in Release builds.
No test framework or network download is needed by CMake.

On Windows, `golf_demo` and `demo_tests` build by default. Use
`-DGOLF_BUILD_DEMO=OFF` to build only the physics library and its tests.
`demo_tests` checks the full shot, an output rate that does not divide the
physics rate, trajectory changes under debug overrides, and camera tracking.

## Debug preview and video

From the project root on Windows with a single-config build:

```powershell
Copy-Item configs/demo.cfg build/debug.cfg
.\build\golf_demo.exe --preview build/debug.cfg
.\build\golf_demo.exe --summary configs/demo.cfg
python tools/export_mp4.py --demo build/golf_demo.exe --config configs/demo.cfg --output outputs/stage2-golf-retuned.mp4
```

For a Visual Studio build, use `build/Debug/golf_demo.exe`. The preview uses
Windows OpenGL and GDI. MP4 export also requires Python 3 and `ffmpeg` with
`libx264` on `PATH`. The exporter leaves an existing output file untouched.

Edit and save the configuration passed to `--preview` to replay with new settings.
The integration test uses `configs/demo.cfg` as the baseline shot. The
preview holds the last frame until the next edit or until the window closes.
Each export uses one configuration snapshot. `--summary` runs without opening
a window and reports contact and rest times, apex, landing angle, and horizontal
distances. Carry ends at first contact. Run is the horizontal displacement from
that contact to the final position, including the bounces. Both use the first
recorded contact tick as an estimate. The video displays carry and run after landing.

The configuration uses `key = value` lines, with `#` comments and SI units.
The supplied example is editable, and omitted keys use defaults. In particular:

| Change | Keys |
| --- | --- |
| Terrain geometry | `terrain_height`, `slope_x`, `slope_z`, `hill_amplitude`, `hill_length`; set `hill_amplitude = 0` for a plane. |
| Ground response | `restitution`, `impact_braking`, `static_friction`, `dynamic_friction`, `rolling_resistance`, `spin_friction` |
| Wind and atmosphere | `wind_x`, `wind_y`, `wind_z`, `gravity_x`, `gravity_y`, `gravity_z`, `air_density` |
| Ball and solver | `mass`, `radius`, `drag`, `lift_slope`, `spin_decay`, `physics_dt`, `max_substep` |
| Launch and spin | `start_x`, `start_z`, `start_height`, `impulse_x/y/z`, `angular_impulse_x/y/z` (use a separate key for each axis) |
| Camera and export | `camera_x/y/z`, `camera_response`, `camera_fov`, `width`, `height`, `fps`, `max_duration`, `end_hold` |

`physics_dt` defaults to 1/240 s. The demo records the fixed-tick states, then
interpolates a read-only display sample at `frame_number / fps`. Changing
`fps` does not change the simulation or its stopping point. The terrain mesh
samples the physics heightfield. The ball is drawn at its physical radius;
the locator ring and height line help track it at a distance. The chase camera
smooths its target, leads moving shots, and moves closer for ground motion.
The video uses the same render path
as the preview and writes H.264 at 1280×720 and 30 fps by default.

This demo uses Windows' software OpenGL bitmap path so capture does not depend
on a visible or unoccluded window. High resolutions can render slower than real
time; the exported frame times still follow `fps`. The terrain mesh has a fixed
5 m sampling grid, so steep or tightly curved debug terrains can show a visible
gap between the mesh and the exact heightfield. A GPU backend and a finer mesh
near the ball are possible improvements if the preview needs higher fidelity.

The demo approximates a medium fairway using restitution `0.22`, impact braking
`0.55`, and rolling resistance `0.145`. Table 1 of the [USGA/R&A fairway
report](https://assets.randa.org/c42c7bf4-dca7-00ea-4f2e-373223f80f76/250b3e34-9172-4ad1-991f-9d5560cbf832/Proposed%20Bounce%20Model%20for%20Use%20in%20Evaluating%20Optimum%20Overall%20Distance.pdf)
reports about 14–34 m of bounce and roll at a 39° landing angle across several
turf tests. The demo lands at about 38.6° and runs 19.9 m. With the same material
on flat ground, checks at 27.4 m/s and about 1,800 rpm of backspin give 30.5,
18.4, and 8.7 m at 33°, 39°, and 45°, respectively. These are approximate
gameplay coefficients for the demonstrated conditions. Turf moisture, grass,
and landing conditions can change the run substantially.

## Using the library

```cpp
#include "physics/BallPhysics.hpp"

golf::BallPhysics physics;
golf::BallState ball;
auto terrain = golf::Terrain::plane(
    0.0, 0.0, 0.0, golf::materialFor(golf::SurfaceType::Fairway));
golf::Environment environment;
environment.windVelocity = {2.0, 0.0, 1.0};

physics.applyShot(ball, {
    {physics.ball().mass * 60.0, physics.ball().mass * 20.0, 0.0},
    {0.0, 0.0, physics.ball().inertia() * 250.0}
});
physics.step(ball, terrain, environment); // One 1/240 s tick, not a frame duration.
```

All quantities use SI units. Coordinates are right handed with +Y up; position
is the sphere centre. Spin uses radians/second and the right-hand rule. For a
ball moving along +X, +Z spin creates upward lift and rolling requires -Z spin.
`applyShot` adds impulses rather than resetting existing motion. A caller using
a custom radius must also initialize the centre at the corresponding height.

The caller owns each `BallState`, counts fixed ticks, and supplies terrain and
environment explicitly. The solver stores immutable parameters only; states can
be simulated independently. `phase` reports airborne/sliding/rolling/resting and
does not bypass force evaluation. Changes to wind, terrain, or material can wake
a stopped ball. Invalid data or exhausted integration budgets throw a standard
exception; a failed call leaves the caller's ball unchanged.

## Physical model

* **Sphere and impulses:** inertia `I = k*m*r*r`, with default `k = 2/5`.
  Linear and angular impulses give `delta-v = J/m`, `delta-omega = L/I`.
  Ball mass, radius, inertia factor, all aerodynamic coefficients, solver
  tolerances, and time/space step limits are constructor parameters.
* **Flight:** for relative air velocity `u = v - wind`, cross-sectional area
  `A = pi*r*r`, and `q = rho*|u|^2/2`, drag is `-q*A*Cd*unit(u)`.
  Magnus lift is `q*A*CL*unit(omega cross u)`, where
  `S = r*|omega cross u|/|u|^2` and `CL = min(liftSlope*S, CLmax)`.
  Zero airflow or axial spin creates no lift. Gravity is an external vector.
  Spin decays exponentially at rate `spinDecayPerDensity*rho`; in a vacuum
  spin is conserved. The drag equation follows the
  [NASA drag model](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/drag-equation/).
  The bounded lift curve and coefficients are tunable approximations, not fits
  to a particular ball. Spin, atmosphere, and wind affect golf-ball flight as
  described in [Stengel's flight study](https://collaborate.princeton.edu/en/publications/on-the-flight-of-a-golf-ball-in-the-vertical-plane/).
* **Impact:** a normal impulse uses a material restitution coefficient in
  `[0,1]`. Contact slip is `v_t + omega cross (-r*n)`. A tangential impulse uses
  effective inverse mass `1/m + r*r/I`, capped by Coulomb friction times normal
  impulse. This transfers momentum between translation and rotation. Below a
  configurable incoming normal speed, restitution is zero to avoid chatter.
  Optional `impactBraking` approximates the loss of translational speed to turf
  deformation. Before Coulomb friction it removes up to `impactBraking * |v_n|`
  from the tangential speed, capped at that speed. Omitted coefficients and the
  generic surface presets default to zero for this term.
* **Contact and rolling:** remove inward normal acceleration, integrate tangent
  forces, apply rolling resistance torque `mu_r*N*r`, then solve static/dynamic
  friction with the same effective mass. Under sufficient static friction,
  rolling down a plane gives `a = g_t/(1+k)` and rolling resistance gives
  deceleration `mu_r*N/[m*(1+k)]`. On frictionless ground, the resisting torque
  slows spin but cannot slow translation. A separate torque `mu_spin*N*r`
  removes spin about the normal. Small motion snaps to rest only when local
  drive can be held by both rolling resistance and static friction.

The material presets are deliberately adjustable starting points:

| Surface | Restitution | Static friction | Dynamic friction | Rolling resistance | Spin friction |
| --- | ---: | ---: | ---: | ---: | ---: |
| Green | 0.50 | 0.40 | 0.25 | 0.010 | 0.010 |
| Fairway | 0.45 | 0.45 | 0.30 | 0.025 | 0.015 |
| Rough | 0.25 | 0.65 | 0.50 | 0.080 | 0.040 |
| Sand | 0.10 | 0.80 | 0.65 | 0.180 | 0.080 |

Supply any `SurfaceMaterial` directly, or return different materials from the
terrain sampler. Coefficients must be finite and nonnegative, restitution at
most one, and dynamic friction at most static friction.

## Integration and terrain contract

The default fixed tick is 1/240 s. Internal substeps are at most 1/960 s and
limit estimated travel to 5 cm. Flight uses RK4, with exact exponential spin
decay at its stages. When a substep crosses the surface, 32 bisection iterations
find the impact time along the integrated path; the remaining time is simulated
after impact. Ground motion uses an impulse update and trapezoidal translation.
The configurable work budget bounds per-tick work and fails explicitly rather
than silently dropping simulation time. Cost depends on speed and contacts;
there are no dynamic allocations per normal tick beyond a caller's sampler.

`Terrain` returns height, upward normal, and material at `(x,z)`. Its normal is
normalized and must have `normal.y >= 0.1`. Height and normal must describe the
same deterministic smooth surface over the entire reachable region. Planes and
inclines are provided; custom smooth heightfields use a callback. The solver
uses local tangent distance `(centre.y-height)*normal.y-radius`, exact on planes.
Contact position is projected to this offset surface and contact velocities are
transported to its tangent.

Limits of this stage:

* General curved heightfields use a small-sphere local-plane approximation.
  Curvature-dependent normal loads and ballistic separation at hill crests are
  not modeled. Use gently curved terrain with features wider than the configured
  travel limit; arbitrary narrow ridges can be missed between samples.
* Vertical walls, cliffs, mesh edges, moving ground, water, cups, and ball-ball
  contacts are outside the terrain contract.
* Drag is constant-coefficient, lift is a bounded spin-ratio approximation,
  and spin decay is isotropic. Reynolds-number and dimple-specific calibration
  require measured coefficients.
* Turf deformation and sand penetration are approximated through coefficients.
  Rigid Coulomb impact does not reproduce every measured golf-ball bounce;
  [experimental bounce measurements](https://arxiv.org/abs/2302.02758) show
  additional slipping and spin-reversal behavior.
* Ground friction and contact geometry are integrated to substep accuracy;
  reducing the substep improves transitions. Bit-identical results are checked
  for the same build and inputs, not promised across compiler/CPU platforms.

## File responsibilities

* `src/core/Math.hpp`: value-only vector operations.
* `src/physics/BallState.hpp`: state, ball and aerodynamic parameters.
* `src/physics/BallPhysics.hpp/.cpp`: fixed ticks, forces, integration and contact progression.
* `src/physics/Collision.hpp/.cpp`: impact, slip and friction impulse calculations.
* `src/world/Terrain.hpp/.cpp`: geometry/material queries and editable material presets.
* `src/world/Environment.hpp`: gravity, wind and density data.
* `src/shot/ShotCommand.hpp`: physical impulse data only.
* `tests/PhysicsTests.cpp`: analytic, energy, contact, material, convergence,
  determinism, long-run stability and invalid-input checks.
* `src/rendering/Camera.*`, `Renderer.*`: chase camera and 3D scene with diagnostics.
* `src/demo/DebugConfig.*`, `Simulation.*`, `RunDemo.*`: test configuration,
  trajectory sampling, and preview or capture orchestration.
* `tools/export_mp4.py`: reads the offscreen frame stream and runs FFmpeg.

Dependencies follow `core -> world/shot data -> physics -> caller` in terms of
data provision. Rendering reads the ball state and terrain without running the
solver; the demo owns the connection between the two.
