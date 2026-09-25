#pragma once

#include "../render_backend.hpp"
#include <string>
#include <vector>

// Ground reference for the selected rocket: a dashed drop line from the rocket
// to the point beneath it, with its altitude, and a small cross on the ground.
// Also works out which way the camera faces (heading from local north) for
// the HUD's heading tape.

namespace renderer {

// Text pinned to a point in the 3D scene; the HUD projects and draws it.
struct WorldLabel {
    RVec3       pos;        // view space (km)
    std::string text;
    RColor      color;
};

class GroundMarks {
public:
    // The selected rocket sits at the view origin, so everything is placed from
    // the Earth's centre alone.
    void Draw(RenderBackend& b, const EarthFrame& f, const RCamera& cam,
              std::vector<WorldLabel>& labels);

    bool   HaveHeading() const { return haveHeading_; }
    double HeadingDeg() const  { return headingDeg_; }   // [0, 360), clockwise from north

private:
    std::vector<LineVertex> lines_;   // reused every frame
    bool   haveHeading_ = false;
    double headingDeg_  = 0.0;
};

}
