#include "terrain_lod.hpp"
#include <algorithm>
#include <cmath>

namespace renderer {

namespace {

constexpr double kQuarterPi = 0.78539816339744830962;

// Angle between two directions (any length). atan2 form: acos loses precision for
// the tiny angles the deepest chunks span.
double angleBetween(const Vec3& u, const Vec3& v) {
    return std::atan2(u.cross(v).mag(), u.dot(v));
}

Vec3 cubePoint(const Vec3& f, const Vec3& a, const Vec3& b, double angA, double angB) {
    return f + a * std::tan(angA) + b * std::tan(angB);
}

} // namespace

void TerrainLod::FaceBasis(int face, Vec3& f, Vec3& a, Vec3& b) {
    switch (face) {
    case 0:  f = { 1, 0, 0}; a = {0, 1, 0}; b = {0, 0, 1}; break;
    case 1:  f = {-1, 0, 0}; a = {0, 0, 1}; b = {0, 1, 0}; break;
    case 2:  f = { 0, 1, 0}; a = {0, 0, 1}; b = {1, 0, 0}; break;
    case 3:  f = { 0,-1, 0}; a = {1, 0, 0}; b = {0, 0, 1}; break;
    case 4:  f = { 0, 0, 1}; a = {1, 0, 0}; b = {0, 1, 0}; break;
    default: f = { 0, 0,-1}; a = {0, 1, 0}; b = {1, 0, 0}; break;
    }
}

void TerrainLod::Select(const TerrainLodCamera& cam, std::vector<TerrainChunk>& out) const {
    out.clear();
    for (int face = 0; face < 6; ++face)
        visit(cam, face, 0, 0.0, 0.0, kQuarterPi, out);
}

void TerrainLod::visit(const TerrainLodCamera& cam, int face, int level,
                       double a, double b, double half, std::vector<TerrainChunk>& out) const {
    Vec3 F, A, B;
    FaceBasis(face, F, A, B);

    TerrainChunk c;
    c.face  = face;
    c.level = level;
    c.a     = a;
    c.b     = b;
    c.half  = half;
    c.cube  = cubePoint(F, A, B, a, b);
    c.dir   = c.cube.unit();

    // Angular radius: the farthest corner from the centre direction.
    double ang = 0.0;
    for (int i = 0; i < 4; ++i) {
        double ca = a + ((i & 1) ? half : -half);
        double cb = b + ((i & 2) ? half : -half);
        ang = std::max(ang, angleBetween(c.dir, cubePoint(F, A, B, ca, cb)));
    }
    const double R = params_.radius;
    const double H = params_.max_elevation;
    c.edge_m = ang * R * std::sqrt(2.0);   // centre->corner is half the diagonal

    // Horizon: hidden if every point of it (up to R + H) is behind the sea-level
    // sphere. Only meaningful from above that sphere.
    const double D = cam.pos.mag();
    if (D > R) {
        double phi   = angleBetween(cam.pos, c.dir);
        double reach = std::acos(R / D) + std::acos(R / (R + H));
        if (phi - ang > reach) return;
    }

    // Bounding sphere over the chunk's angular extent and height range.
    Vec3   S   = c.dir * (R + 0.5 * H);
    double rad = (R + H) * 2.0 * std::sin(0.5 * ang) + 0.5 * H;
    Vec3   toS = S - cam.pos;
    double ds  = toS.mag();

    // View cone: skip if the sphere is entirely outside it.
    if (ds > rad) {
        double off = angleBetween(toS, cam.fwd);
        if (off - std::asin(rad / ds) > cam.cone_half) return;
    }

    double dist = std::max(0.0, ds - rad);
    c.distance_m = dist;
    bool split = level < params_.min_level ||
                 (level < params_.max_level && dist < params_.split_distance * c.edge_m);
    if (split) {
        double q = 0.5 * half;
        visit(cam, face, level + 1, a - q, b - q, q, out);
        visit(cam, face, level + 1, a + q, b - q, q, out);
        visit(cam, face, level + 1, a - q, b + q, q, out);
        visit(cam, face, level + 1, a + q, b + q, q, out);
        return;
    }
    out.push_back(c);
}

}
