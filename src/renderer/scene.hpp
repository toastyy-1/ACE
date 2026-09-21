#pragma once

#include "render_types.hpp"
#include "rmath.hpp"
#include "../types.hpp"

// Backend-neutral *domain* description of the scene's high-level objects. The
// renderer computes these each frame from the sim (it owns the precision-
// critical ECI->view shift).

namespace renderer {

// Static rocket dimensions (metres). Changes at staging; the backend rebuilds
// its meshes when these move.
struct RocketDims {
    double length;
    double cm_dist;
    double radius;
    double engine_dist;
};

// Everything needed to draw the rocket for one frame. Transforms are already in
// view space (the backend must not redo the ECI->view shift, or float jitter
// returns). The backend owns the actual geometry/materials.
struct RocketFrame {
    RocketDims dims;        // current stack dimensions
    RMat4 hull;            // body-local -> view
    RMat4 bell;            // bell-local (gimballed about the engine pivot) -> view
    bool  firing;          // plume visible
    float thrust;          // [0,1] plume intensity
    float flick;           // flame flicker multiplier
    float air;             // [0,1] atmospheric density (1 = sea level, 0 = vacuum)
    float heating;         // [0,1] aerodynamic heating (ablation glow); ascent/reentry
    RVec3 vel_dir;         // unit travel direction, view space (windward = normal . vel_dir)
    RVec3 nozzle;          // bell exit, view space (km)
    RVec3 exhaust_dir;     // unit direction the plume travels, view space

    // Detonation (bgfx backend only; raylib ignores these). When `detonated` is
    // set the rocket has blown up: the backend draws the explosion at `center`
    // and no hull/plume, driving the animation off `det_time`.
    bool  detonated = false;   // this rocket has detonated
    float det_time  = 0.0f;    // seconds elapsed since detonation began
    RVec3 center{};            // rocket position, view space (km) -- explosion anchor
};

// Everything needed to draw the Earth for one frame. The backend owns the mesh,
// texture(s), and shader; the renderer supplies the placement and lighting.
struct EarthFrame {
    RMat4 model;           // ECI-metre sphere -> view
    RVec3 sun_dir;         // unit view-space direction TO the sun
    RVec3 center;          // view-space sphere centre (km)
    RVec3 cam_pos;         // view-space camera position (km)
    // `center` again in double. At ~6400 km a float only resolves ~0.5 m, which
    // is visible between the ground and a rocket sitting on it; the terrain
    // anchors its chunks from this instead.
    Vec3  center_km;
};

}
