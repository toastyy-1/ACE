#pragma once

#include "wire.hpp"
#include "../scene.hpp"

// Continent and island outlines over the wireframe globe, from Natural Earth's
// public-domain 50 m coastlines (converted by tools/coastline_to_dat.py). One
// static line mesh about the Earth's centre, faded in as the camera climbs:
// close to the ground the grid is the reference, from altitude this is.

namespace renderer {

class Coastline {
public:
    // Needs the GL context. Without the data file it warns and draws nothing.
    void Init();
    void Shutdown();

    void Draw(wire::Pipeline& p, const EarthFrame& f);

private:
    wire::GpuMesh mesh_;
};

}
