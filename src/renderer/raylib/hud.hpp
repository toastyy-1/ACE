#pragma once

#include <raylib.h>
#include "../scene.hpp"
#include "ground.hpp"
#include <functional>
#include <string>
#include <vector>

namespace renderer {

// What the HUD needs from the backend besides the renderer's HudFrame.
struct HudView {
    int    width = 0, height = 0;
    double now = 0.0;                 // wall clock (s)
    int    fps = 0;
    bool   haveHeading = false;
    double headingDeg = 0.0;          // camera heading from local north
    const std::vector<WorldLabel>* labels = nullptr;
    std::function<ScreenPoint(const RVec3&)> project;   // view space -> screen pixels
};

// The raylib build's HUD, laid out like an aircraft multifunction display:
// boxed data blocks (cyan captions, white values), bar gauges, a gimbal
// indicator, and soft-key labels along the bottom edge.
class DisplayHud {
public:
    void Init();
    void Shutdown();
    void Draw(const HudFrame& hud, const HudView& view);

    // Plain text in the HUD font, for the renderer's fallback 2D path.
    void Text(const char* s, int x, int y, int size, RColor c) const;

private:
    struct Face { ::Font font{}; int size = 0; };
    const Face& face(int size) const;
    int  width(const char* s, int size) const;

    // Note when each vehicle leaves the pad.
    void track(const HudFrame& hud);

    void box(int x, int y, int w, int h, const char* title) const;
    void heading(const HudView& v) const;
    void flightData(const HudFrame& hud, const HudView& v);
    void gimbal(const HudFrame& hud, const HudView& v) const;
    void softKeys(const HudFrame& hud, const HudView& v) const;
    void corner(const HudFrame& hud, const HudView& v);
    void markers(const HudFrame& hud, const HudView& v) const;
    void worldLabels(const HudView& v) const;

    std::vector<Face> faces_;

    struct Track { bool lifted; };
    std::vector<Track> tracks_;

    // Propellant gauge reference for the selected rocket: the most it has held
    // since it was selected or last staged.
    int    fuelFor_ = -1;
    double fuelLength_ = 0.0, fuelMax_ = 0.0;

    // Sim speed relative to wall time, sampled twice a second.
    double rateMet_ = 0.0, rateWall_ = -1.0, rate_ = 0.0;
};

}
