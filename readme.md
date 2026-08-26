# Ballistic Rocketry 6

A 6DoF ballistic trajectory sim for testing flight controllers.

Copyright (c) 2026 Tyler Wiggins. All rights reserved.

This project and its contents are proprietary. No part of this repository may be reproduced,
distributed, transmitted, displayed, or used in any form or by any means without the prior
written permission of the copyright holder.

---

## 1. Dependencies

| what | why | fedora | debian/ubuntu |
| --- | --- | --- | --- |
| `g++` with C++20, `gcc` C11, `make` | everything | `gcc-c++ make` | `build-essential` |
| raylib | default renderer | `raylib-devel` | `libraylib-dev` |
| cmake, glfw | bgfx renderer only | `cmake glfw-devel` | `cmake libglfw3-dev` |
| git-lfs, unzip | bgfx earth textures only | `git-lfs unzip` | `git-lfs unzip` |

macOS and Windows (MinGW) link paths are already in the [Makefile](Makefile); only the
raylib/glfw packages differ.

## 2. Downloading

```sh
git clone <repo> && cd <dir_name>
git submodule update --init --recursive   # bgfx.cmake, only needed for `make bgfx`
```

## 3. Build

**raylib:**

```sh
make run
```

**bgfx:**

```sh
make bgfx-deps
make bgfx
make run-bgfx
```

`make bgfx-deps` only has to be run once.

Camera: `WASD` + `QE` to move, mouse to look, `shift` to boost, `F` to recenter on the vehicle,
`TAB` to cycle tracked rocket, `1`-`9` toggle HUD overlays.

On exit the sim writes `landing_errors.csv`.

---

## 4. Configuring the scenario (`config/sim.yaml`)

One YAML file drives the sim. Anything you leave out falls back to the default in the
table.

### Top level

| key | default | meaning |
| --- | --- | --- |
| `time_step` | `0.01` | integration step in seconds, and therefore the `fc_update` period |
| `step_delay` | `0.001` | real life sleep between steps, purely to slow the sim down for viewing |
| `rockets` | — | list, one entry per vehicle. All of them fly simultaneously and independently using the designated FC |

### Per rocket

| key | default | meaning |
| --- | --- | --- |
| `origin_lat` / `origin_lon` | `0.0` | launch site, degrees |
| `target_lat` / `target_lon` | `0.0` | aim point, degrees. given to the FC as `r_target_ecef` |
| `radius` | props default | tank radius (m) |
| `drag_coefficient` | props default | `Cd`. `0.0` disables drag |
| `stage` | — | list, in flight order: first entry is the booster |

### Per stage

Stage geometry is measured **from the leading edge (tip) of that stage, pointing aft**.

| key | units | meaning |
| --- | --- | --- |
| `dry_mass` | kg | fuel-less stage mass |
| `fuel_mass` | kg | propellant at ignition |
| `isp` | s | vacuum specific impulse |
| `isp_sea_level` | s | sea level Isp |
| `length` | m | tip to tail |
| `com_distance` | m | CoM from the tip with full tanks |
| `fuel_com_distance` | m | propellant column CoM from the tip, full (defaults to `com_distance`) |
| `fuel_length` | m | propellant column length, full |
| `max_thrust` | N | rated thrust |
| `engine_distance` | m | gimbal point from the tip (usually length) |
| `gimbal_range_deg` | deg | max nozzle deflection off the body axis |
| `rcs_max_moment` | N·m | 3 element `[x, y, z]` torque authority. Omit for no RCS (experimental feature) |

The number of stage entries defines the number of stages. Every stage you define is reported to the FC in `fc_vehicle.stages`.

Adding more vehicles is just another list entry as shown:

```yaml
rockets:
  - origin_lat: 48.209
    origin_lon: -101.406
    target_lat: 53.9
    target_lon: 43.3
    radius: 0.835
    drag_coefficient: 0.0
    stage:
      - id: 1
        dry_mass: 2292.0
        # ...

  - origin_lat: 45.0
    origin_lon: -100.0
    target_lat: 50.0
    target_lon: 40.0
    radius: 0.7
    drag_coefficient: 0.2
    stage:
      - id: 1
        # ...
```

Each rocket gets its own `fc_init` state, so the same controller code flies all of them without
sharing anything.

---

## 5. Plugging in your own flight controller

### Building with your own custom FC code

** **DISCLAIMER**: Ideally you remove your other dependencies and just focus on flight controller math/logic for testing with this program. Including other dependencies (such as embedded firmware libraries) is not suggested or supported and may cause unwanted behavior. 

The FC is selected at build time with `FC_SRC`:

```sh
make FC_SRC=src/fc/my_fc.c                          # single C file
make FC_SRC="src/fc/my_nav.cpp src/fc/my_guid.cpp"  # several files
make bgfx FC_SRC=src/fc/my_fc.c                     # same for the bgfx target
```

Default is `FC_SRC := src/fc/fc.cpp src/fc/stages.cpp` (my own epic controller).

Rules you should probably follow:

- Your code must define exactly three things: `fc_init`, `fc_update`, `fc_free`.
- C++ files are compiled into the main build. **C files are compiled separately**, so a `.c` controller has to live in `src/fc/` for `make` to find a rule for it. `.cpp` files can live anywhere.
- `fc_api.h` is the only header you need. It works from both C and C++ and
  already carries the vector/quaternion helpers, gravity model, ECI/ECEF conversions, and stage
  math (`fc_stage_burn_time`, `fc_stage_delta_v`, …). ((please read the API file completely))

### The three entry points

```c
void* fc_init(const fc_vehicle* vehicle, double t);  // once per rocket, return your state blob
void  fc_update(void* state, const fc_sensors* s);   // once per step
void  fc_free(void* state);                          // on vehicle destruction
```

`fc_init` gets the full vehicle spec (stages, launch point/attitude in ECI, target in ECEF,
`time_step`) and returns whatever pointer you want handed back each step.

### What you can read

`fc_sensors` provides mission time, `dt`, body frame accelerometer and gyro (both
carrying INS noise from ins.hpp), plus `g`, which is the sim's own gravity at
the true position. `g` is a debug tool, so you're a chud if you use it.

### What you can command

```c
fc_light_engine();  
fc_cutoff_engine();  
fc_burn_fraction(f);
fc_separate_stage(); 
fc_detonate();
fc_set_gimbal(q);   
fc_rcs_enable(1);    
fc_rcs_set_moment(m);
int stage = fc_active_stage();
```

Calls only record you intent to run that command. 
The sim applies them after `fc_update` returns, in a fixed order (burn/cutoff, separate, light, detonate, gimbal, RCS). Order of your calls does not matter. Gimbal and RCS settings persist across steps, and everything else is one shot.

### Frames

- Positions and velocities the sim hands you are **ECI**; the target is **ECEF**. Convert with `fc_ecef_to_eci(p, t)` / `fc_eci_to_ecef(p, t)` using mission time.
- Sensors are **body frame**. Body `+z` is the nose; `q_origin_eci` is the launch attitude.
- Gimbal quaternion is the nozzle relative to the body, identity q = straight aft.

### Example

```c
#include "fc/fc_api.h"
#include <stdlib.h>

typedef struct { const fc_vehicle* veh; fc_vec3 r, v; int lit; } my_fc;

void* fc_init(const fc_vehicle* vehicle, double t) {
    my_fc* s = calloc(1, sizeof(my_fc));
    s->veh = vehicle;
    s->r = vehicle->r_origin_eci;
    s->v = fc_surface_velocity_eci(vehicle->r_origin_eci);
    return s;
}

void fc_update(void* state, const fc_sensors* sen) {
    my_fc* s = (my_fc*)state;

    fc_vec3 a = fc_v3_add(fc_q_rotate(s->veh->q_origin_eci, sen->a_spec), fc_gravity_j2(s->r));
    s->v = fc_v3_add(s->v, fc_v3_scale(a, sen->dt));
    s->r = fc_v3_add(s->r, fc_v3_scale(s->v, sen->dt));

    if (!s->lit) { fc_light_engine(); s->lit = 1; }
}

void fc_free(void* state) { free(state); }
```

Build it with `make FC_SRC=src/fc/my_fc.c && ./program`.
