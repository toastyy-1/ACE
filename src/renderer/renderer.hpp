#pragma once
#include "render_backend.hpp"
#include "../constants.hpp"
#include "../types.hpp"
#include "../sim/inc/rocket.hpp"   // RocketState (returned by primaryState / sim::get_state)
#include <vector>
#include <string>
#include <atomic>

namespace sim { class Sim; }

namespace renderer {

// Backend-agnostic renderer for providing orbit camera, the rocket,
// trajectory geometry, and telemetry.
class Renderer {
public:
    Renderer(RenderBackend& backend, const sim::Sim& s);
    ~Renderer();
    void Run();

    bool IsInitialized() const { return initialized_.load(std::memory_order_acquire); }

private:
    void HandleInput();
    void UpdateThrustLevels();
    void UpdateDetonations();
    RCamera BuildCamera() const;
    void DrawFrame(const RCamera& cam);
    void DrawEarth(const RCamera& cam, RVec3 earthC);
    void DrawECIAxes() const;
    void DrawBodyAxes() const;
    void DrawStateVectors() const;
    void DrawSurfaceMarkers() const;
    void DrawRocket() const;
    void DrawPredictedTrajectory() const;
    void DrawTrails() const;
    // Gather everything the HUD shows (telemetry, rocket labels, toggles).
    HudFrame BuildHud() const;
    // The default HUD, for backends that don't draw their own (DrawHud).
    void DrawTelemetry(const HudFrame& hud) const;
    // 2D overlay: the primary rocket's ID in a top-centre HUD, and floating ID
    // labels above every other rocket. Drawn after End3D.
    void DrawRocketLabels() const;
    // 2D overlay: number-key legend showing which overlays are on/off.
    void DrawOverlayLegend(const HudFrame& hud) const;

    // Append the current position of each rocket to its flown-path trail.
    void UpdateTrails();

    // Draw one rocket's hull + gimballed engine/plume at its ECI state. detTime is
    // the seconds elapsed since this rocket detonated, or < 0 if it hasn't; when
    // set, the backend draws the explosion instead of the intact rocket.
    void DrawOneRocket(const RocketState& st, float thrustLevel, double detTime) const;

    // Per-rocket Greek IDs, chosen by launch-origin latitude band (7.5° -> one of
    // 24 Greek letters α..ω) plus a 1-based ordinal within that band. Indexed to
    // match states_.
    std::vector<std::string> rocketIds() const;

    // The sim publishes every rocket's state; states_ is sampled once per frame.
    // primaryState() returns the currently selected rocket (Tab/Shift+Tab), or a
    // default RocketState when no rockets are live yet.
    RocketState primaryState() const;

    // ECI position (m) -> shifted render-scene position (km), with the rocket
    // held at the world origin. Differencing in double before the float cast
    // kills the jitter you'd get from float at Earth-radius magnitudes. The
    // reorientation (x,y,z)->(x,z,-y) is RotateX(-90 deg): it brings ECI +Z
    // (north) to view +Y (up) as a proper rotation (det +1), so winding/normals
    // survive. Must match rmath::viewBasis exactly.
    RVec3 ToView(const Vec3& eci_m) const {
        Vec3 v = ToViewKm(eci_m);
        return { float(v.x), float(v.y), float(v.z) };
    }
    // ToView kept in double, for the few consumers that need more than float
    // precision at planet scale (the terrain's chunk anchors).
    Vec3 ToViewKm(const Vec3& eci_m) const {
        Vec3 d = eci_m - p_ref_eci_;
        return { d.x * M_TO_KM, d.z * M_TO_KM, -d.y * M_TO_KM };
    }

    // The scene is shifted so this point (the primary rocket's ECI position) sits
    // at the world origin, keeping geometry inside float's precision sweet spot.
    // Sampled once per frame so camera, axes, rockets, and Earth all agree.
    Vec3 p_ref_eci_ { 0, 0, 0 };

    // All rocket states, sampled once per frame (sim::get_state). primary_ selects
    // the one the camera centres on and the telemetry panel describes; cycled with
    // Tab / Shift+Tab.
    std::vector<RocketState> states_;
    int  primary_ = 0;

    RenderBackend&   backend_;
    const sim::Sim&  sim_;

    // Set true after the first rendered frame (backend textures loaded). Read from
    // the sim thread via IsInitialized(), so it must be atomic.
    std::atomic<bool> initialized_{false};

    // Renderer-owned unit sphere (radius 1) for the surface markers.
    MeshHandle markerSphere_ = 0;

    float yaw   = 0.0f;
    float pitch = 0.3f;
    float dist  = 0.05f;  // 50 m: framed on the rocket; scroll out to see the Earth
    // Point the camera orbits (km, render-scene frame). Origin = the rocket; WASD
    // flies it across the surface so you can inspect terrain away from the rocket.
    RVec3 pivot_ { 0.0f, 0.0f, 0.0f };

    // Per-rocket exhaust plume state. The sim exposes no "firing" flag, so we
    // infer it from propellant being burned and smooth it into a level that drives
    // the plume. Indexed to match states_ (resized when the rocket count changes).
    std::vector<float>  thrustLvl_;   // [0,1] per rocket, ramps with the motor
    std::vector<double> prevFuel_;    // previous propellant sample (-1 = none yet)

    // Wall-clock time (backend_.Time()) at which each rocket was first observed with
    // detonation_active set; -1 = not detonated. The sim keeps publishing a detonated
    // rocket, so we latch the first sighting here and drive the explosion animation
    // off the elapsed time. Indexed to match states_ (reset when the count changes).
    std::vector<double> detStart_;

    // Flown-path history per rocket, in ECI metres (recorded regardless of whether
    // trails are shown, so toggling on reveals the full path). Indexed to match
    // states_; reset when the rocket count changes.
    std::vector<std::vector<Vec3>> trails_;

    // Overlay visibility, toggled by number keys 1-6 (see HandleInput). The
    // essentials default on; the debug helpers (axes + velocity vectors) share a
    // single toggle and default off so the initial view stays uncluttered.
    bool showPredicted_      = true;    // 1  predicted trajectory
    bool showTrails_         = true;    // 2  flown path
    bool showLabels_         = true;    // 3  rocket names
    bool showSurfaceMarkers_ = true;    // 4  targets / origin pins
    bool showTelemetry_      = true;    // 5  telemetry HUD
    bool showDebug_          = false;   // 6  body axes + state vectors + ECI axes
};

}
