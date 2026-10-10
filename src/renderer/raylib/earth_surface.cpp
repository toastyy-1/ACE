#include "../earth_surface.hpp"
#include "../../planetary_constants.hpp"

// The raylib build has no terrain: the sim's ground is the same smooth sphere
// the wireframe globe draws.

namespace renderer {

EarthSurface& EarthSurface::Get() {
    static EarthSurface s;
    return s;
}

double EarthSurface::SurfaceRadius3D(const Vec3&) const { return planet::EARTH.radius; }

double EarthSurface::SurfaceRadius2D(double, double) const { return planet::EARTH.radius; }

}
