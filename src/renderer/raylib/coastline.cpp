#include "coastline.hpp"
#include "theme.hpp"
#include "../../constants.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>

namespace renderer {

namespace {

// Relative to the repo root (the app runs from there).
const char* kPath = "src/renderer/assets/coastline_50m.dat";

// Longest straight piece drawn. Longer source segments are split along the
// great circle so the chord never sags far under the globe (~8 m at 20 km).
constexpr double kMaxSegment = 20e3;

// Camera altitudes (m) over which the outlines fade in.
constexpr double kFadeLow  = 5e3;
constexpr double kFadeHigh = 50e3;

// ECEF unit vector for a longitude/latitude in degrees (same convention as the
// sim's lat_lon_to_ecef).
Vec3 unitFromLonLat(double lon_deg, double lat_deg) {
    double lon = lon_deg * DEG_TO_RAD, lat = lat_deg * DEG_TO_RAD;
    return { std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat) };
}

template <typename T>
bool read(std::ifstream& in, T& v) {
    return (bool)in.read(reinterpret_cast<char*>(&v), sizeof v);
}

} // namespace

void Coastline::Init() {
    std::ifstream in(kPath, std::ios::binary);
    char magic[8];
    uint32_t version = 0, count = 0;
    if (!in || !in.read(magic, 8) || std::memcmp(magic, "ACECOAST", 8) != 0 ||
        !read(in, version) || version != 1 || !read(in, count)) {
        std::fprintf(stderr, "raylib: no coastlines (%s missing or unreadable)\n", kPath);
        return;
    }

    // Vertices on the sea-level sphere in body-frame metres; one line-list
    // index pair per piece. The globe mesh is placed by EarthFrame::model.
    std::vector<Vertex>   verts;
    std::vector<uint32_t> lines;
    auto vertex = [&](const Vec3& dir) {
        Vec3 p = dir * EARTH_RADIUS;
        verts.push_back({ { (float)p.x, (float)p.y, (float)p.z },
                          { (float)dir.x, (float)dir.y, (float)dir.z }, 0, 0, kWhite });
        return (uint32_t)(verts.size() - 1);
    };

    for (uint32_t l = 0; l < count; ++l) {
        uint32_t n = 0;
        if (!read(in, n)) break;
        Vec3     prev{};
        uint32_t prevIdx = 0;
        for (uint32_t i = 0; i < n; ++i) {
            float lon = 0, lat = 0;
            if (!read(in, lon) || !read(in, lat)) break;
            Vec3 dir = unitFromLonLat(lon, lat);
            if (i == 0) { prev = dir; prevIdx = vertex(dir); continue; }

            // Split along the great circle (slerp) into pieces of <= kMaxSegment.
            double ang    = std::atan2(prev.cross(dir).mag(), prev.dot(dir));
            int    pieces = std::max(1, (int)std::ceil(ang * EARTH_RADIUS / kMaxSegment));
            for (int k = 1; k <= pieces; ++k) {
                double t = (double)k / pieces;
                Vec3 d = ang < 1e-9 ? dir
                       : (prev * std::sin((1.0 - t) * ang) + dir * std::sin(t * ang)) * (1.0 / std::sin(ang));
                uint32_t idx = vertex(d.unit());
                lines.push_back(prevIdx);
                lines.push_back(idx);
                prevIdx = idx;
            }
            prev = dir;
        }
    }
    if (lines.empty()) return;
    mesh_ = wire::Upload(verts, {}, lines);
}

void Coastline::Shutdown() { wire::Destroy(mesh_); }

void Coastline::Draw(wire::Pipeline& p, const EarthFrame& f) {
    if (!mesh_.vao) return;

    const Vec3   eye = { f.cam_pos.x, f.cam_pos.y, f.cam_pos.z };
    const double alt = ((eye - f.center_km).mag() * KM_TO_M) - EARTH_RADIUS;
    const double fade = std::clamp((alt - kFadeLow) / (kFadeHigh - kFadeLow), 0.0, 1.0);
    if (fade <= 0.0) return;

    // Lift the outlines clear of the globe's own surface, which is built from
    // chords and so sits a little inside the true sphere; more from further out,
    // where the depth buffer is coarser.
    const double lift = 30.0 + alt * 1e-3;
    const RMat4 model = rmath::mul(f.model, rmath::scale((float)(1.0 + lift / EARTH_RADIUS)));

    wire::Shading s;
    s.tint = theme::withAlpha(theme::kCoast, (unsigned char)(fade * 255.0));
    p.Edges(mesh_, model, s);
}

}
