#include "../earth_surface.hpp"
#include "../../constants.hpp"

// The raylib build has no terrain: the sim's ground is the same smooth sphere
// the wireframe globe draws.

namespace renderer {

EarthSurface& EarthSurface::Get() {
    static EarthSurface s;
    return s;
}

double EarthSurface::SurfaceRadius3D(const Vec3&) const { return EARTH_RADIUS; }

double EarthSurface::SurfaceRadius2D(double, double) const { return EARTH_RADIUS; }

}
