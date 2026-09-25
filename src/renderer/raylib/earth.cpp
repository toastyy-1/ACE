#include "earth.hpp"
#include "../../constants.hpp"

#include <cmath>

namespace renderer {

namespace {

// Squares per chunk edge. With kSplitDistance a chunk spans roughly a sixth to
// a third of the screen height, so the grid keeps a steady on-screen spacing.
constexpr int    kGrid          = 8;
constexpr double kSplitDistance = 3.0;
// Deepest quadtree level: ~40 m chunks (5 m squares), enough to read height and
// speed off the ground with the camera zoomed all the way in on a rocket.
constexpr int    kMaxLevel      = 18;
// Cached chunks beyond this are dropped once they've gone unused for kKeepFrames.
constexpr size_t   kMaxCached   = 4096;
constexpr uint64_t kKeepFrames  = 120;

const RColor kGridColor = { 50, 130, 175, 255 };

// Renderer view space is the body (ECEF) frame rotated +Z(north) -> +Y(up); see
// rmath::viewBasis / Renderer::ToView. These are that rotation and its inverse.
Vec3 viewToBody(const Vec3& v) { return { v.x, -v.z, v.y }; }
Vec3 bodyToView(const Vec3& b) { return { b.x, b.z, -b.y }; }

// Grid vertex at row i (along the face's B axis), column j (along A).
uint32_t gridIndex(int i, int j) { return (uint32_t)(i * (kGrid + 1) + j); }

// The grid's border as one loop; the skirt hangs a copy of it below the surface.
std::vector<uint32_t> borderLoop() {
    std::vector<uint32_t> b;
    for (int j = 0; j < kGrid; ++j) b.push_back(gridIndex(0, j));
    for (int i = 0; i < kGrid; ++i) b.push_back(gridIndex(i, kGrid));
    for (int j = kGrid; j > 0; --j) b.push_back(gridIndex(kGrid, j));
    for (int i = kGrid; i > 0; --i) b.push_back(gridIndex(i, 0));
    return b;
}

} // namespace

void WireEarth::Init() {
    TerrainLodParams p;
    p.radius         = EARTH_RADIUS;
    p.max_elevation  = 0.0;
    p.min_level      = 1;
    p.max_level      = kMaxLevel;
    p.split_distance = kSplitDistance;
    lod_.Configure(p);

    // Every chunk has the same topology: one index buffer for all of them.
    const uint32_t gridVerts = (kGrid + 1) * (kGrid + 1);
    std::vector<uint32_t> tris, lines;
    for (int i = 0; i < kGrid; ++i)
        for (int j = 0; j < kGrid; ++j) {
            uint32_t a = gridIndex(i, j),     b = gridIndex(i, j + 1);
            uint32_t c = gridIndex(i + 1, j), d = gridIndex(i + 1, j + 1);
            tris.insert(tris.end(), { a, c, b, b, c, d });
        }
    // Skirt: a wall from the border down to its copy. It fills the slivers where
    // a chunk meets a neighbour at a different level, so nothing behind the
    // globe shows through the cracks.
    std::vector<uint32_t> border = borderLoop();
    const uint32_t k = (uint32_t)border.size();
    for (uint32_t i = 0; i < k; ++i) {
        uint32_t a = border[i], b = border[(i + 1) % k];
        uint32_t sa = gridVerts + i, sb = gridVerts + (i + 1) % k;
        tris.insert(tris.end(), { a, sa, b, b, sa, sb });
    }
    // Grid lines along both axes (the skirt has none).
    for (int i = 0; i <= kGrid; ++i)
        for (int j = 0; j < kGrid; ++j) {
            lines.insert(lines.end(), { gridIndex(i, j), gridIndex(i, j + 1) });
            lines.insert(lines.end(), { gridIndex(j, i), gridIndex(j + 1, i) });
        }
    ebo_   = wire::UploadIndices(tris, lines);
    tris_  = (int)tris.size();
    lines_ = (int)lines.size();
}

void WireEarth::Shutdown() {
    for (auto& [key, c] : cache_) wire::Destroy(c.mesh);
    cache_.clear();
    wire::DestroyIndices(ebo_);
    ebo_ = 0;
}

const wire::GpuMesh& WireEarth::chunkMesh(const TerrainChunk& c) {
    // Chunks are identified by face, level, and position within the face.
    const double start = -0.25 * M_PI;
    uint64_t ia  = (uint64_t)std::llround((c.a - start) / (2.0 * c.half) - 0.5);
    uint64_t ib  = (uint64_t)std::llround((c.b - start) / (2.0 * c.half) - 0.5);
    uint64_t key = (uint64_t)c.face << 61 | (uint64_t)c.level << 54 | ia << 27 | ib;

    auto it = cache_.find(key);
    if (it != cache_.end()) {
        it->second.lastFrame = frame_;
        return it->second.mesh;
    }

    // Vertices relative to the chunk's centre on the sphere, computed in double
    // and only then narrowed to float, so they're exact to well under a
    // millimetre at any level. Normals point straight up (the body-frame dir).
    Vec3 F, A, B;
    TerrainLod::FaceBasis(c.face, F, A, B);
    const double R      = EARTH_RADIUS;
    const Vec3   anchor = c.dir * R;
    Vec3 dirs[(kGrid + 1) * (kGrid + 1)];
    for (int i = 0; i <= kGrid; ++i) {
        double tb = std::tan(c.b + (2.0 * i / kGrid - 1.0) * c.half);
        for (int j = 0; j <= kGrid; ++j) {
            double ta = std::tan(c.a + (2.0 * j / kGrid - 1.0) * c.half);
            dirs[gridIndex(i, j)] = (F + A * ta + B * tb).unit();
        }
    }
    auto push = [&](const Vec3& dir, double r) {
        Vec3 p = dir * r - anchor;
        scratch_.push_back({ { (float)p.x, (float)p.y, (float)p.z },
                             { (float)dir.x, (float)dir.y, (float)dir.z }, 0, 0, kWhite });
    };
    scratch_.clear();
    for (const Vec3& d : dirs) push(d, R);
    // Deep enough to cover the sag of the coarsest neighbour's straight edges.
    const double skirt = c.edge_m * 0.003 + 1.0;
    for (uint32_t b : borderLoop()) push(dirs[b], R - skirt);

    Cached& e = cache_[key];
    e.mesh      = wire::UploadShared(scratch_, ebo_, tris_, lines_);
    e.lastFrame = frame_;
    return e.mesh;
}

void WireEarth::evict() {
    if (cache_.size() <= kMaxCached) return;
    for (auto it = cache_.begin(); it != cache_.end();) {
        if (it->second.lastFrame + kKeepFrames < frame_) {
            wire::Destroy(it->second.mesh);
            it = cache_.erase(it);
        } else {
            ++it;
        }
    }
}

void WireEarth::Draw(wire::Pipeline& p, const EarthFrame& f, const RCamera& cam, float aspect) {
    ++frame_;

    // Camera in the body frame, relative to the planet centre (m), for LOD/culling.
    Vec3 camView = { cam.position.x, cam.position.y, cam.position.z };
    Vec3 fwdView = { cam.target.x - cam.position.x, cam.target.y - cam.position.y,
                     cam.target.z - cam.position.z };
    TerrainLodCamera lc;
    lc.pos = viewToBody(camView - f.center_km) * KM_TO_M;
    lc.fwd = viewToBody(fwdView).unit();
    double tanH  = std::tan(cam.fovy * 0.5 * DEG_TO_RAD);
    lc.cone_half = std::atan(tanH * std::sqrt(1.0 + (double)aspect * aspect));
    lod_.Select(lc, chunks_);

    // Each chunk is placed by its centre in view space, differenced in double
    // (the float view-space Earth centre is only good to ~0.5 m).
    const RMat4 basis = rmath::viewBasis((float)M_TO_KM);
    models_.clear();
    for (const TerrainChunk& c : chunks_) {
        Vec3 o = f.center_km + bodyToView(c.dir * EARTH_RADIUS) * M_TO_KM;
        models_.push_back(rmath::mul(rmath::translate({ (float)o.x, (float)o.y, (float)o.z }), basis));
    }

    // All the fills, then all the lines: the fill state is switched once.
    for (size_t i = 0; i < chunks_.size(); ++i) p.Fill(chunkMesh(chunks_[i]), models_[i]);
    wire::Shading s;
    s.tint = kGridColor;
    for (size_t i = 0; i < chunks_.size(); ++i) p.Edges(chunkMesh(chunks_[i]), models_[i], s);

    evict();
}

}
