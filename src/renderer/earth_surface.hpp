#pragma once

#include "../types.hpp"

// The ground the sim launches rockets from and measures altitude against.
// Declared with no backend types so the sim builds against either renderer;
// each build links exactly one definition:
//   make       -> raylib/earth_surface.cpp  (a plain sphere of EARTH_RADIUS)
//   make bgfx  -> bgfx/earth_bump_map.cpp   (the bump map's terrain heights)

namespace renderer {

class EarthSurface {
public:
    static EarthSurface& Get();

    // Ground radius (m) under an ECEF position, or at a latitude/longitude (deg).
    double SurfaceRadius3D(const Vec3& r) const;
    double SurfaceRadius2D(double lat_deg, double lon_deg) const;
};

}
