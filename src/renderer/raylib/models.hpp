#pragma once

#include "../render_backend.hpp"
#include <vector>

// raylib's rocket: a wireframe hull + gimballed bell, an exhaust plume of nested
// wire cones with Mach diamonds, and the detonation as expanding wire shells.
// Drawn through the backend interface; the backend turns every mesh into its
// outline (see wire.hpp). Separate from bgfx's model so each can evolve alone.

namespace renderer {

class RocketModel {
public:
    // Select (building on first sight) the hull/bell for these dimensions. One
    // RocketModel is shared by every rocket, so each distinct stage config is
    // built once and reused as the draw loop moves between rockets.
    void Ensure(RenderBackend& b, const RocketDims& dims);
    void Draw(RenderBackend& b, const RocketFrame& f) const;

private:
    void drawPlume(RenderBackend& b, const RocketFrame& f) const;
    void drawDetonation(RenderBackend& b, const RocketFrame& f) const;

    struct HullBell { RocketDims dims; MeshHandle hull; MeshHandle bell; };
    HullBell buildHullBell(RenderBackend& b, const RocketDims& d) const;

    std::vector<HullBell> cache_;   // one entry per distinct stack dimension set

    RocketDims dims_{};             // dims of the selected hull/bell
    MeshHandle hull_    = 0;        // selected by Ensure, drawn by Draw
    MeshHandle bell_    = 0;
    MeshHandle cone_    = 0;        // unit cone: plume layers
    MeshHandle diamond_ = 0;        // unit octahedron: Mach diamonds
    MeshHandle shell_   = 0;        // unit sphere: explosion shells
};

}
