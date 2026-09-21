#include "terrain.hpp"
#include "bgfx_util.hpp"
#include "earth_bump_map.hpp"
#include "../geometry.hpp"
#include "../../constants.hpp"

#include <algorithm>
#include <cmath>

namespace renderer {

namespace {

// Quads per chunk edge. With TerrainLodParams::split_distance this sets the
// on-screen quad size: a chunk is split once the camera is closer than
// split_distance edges, so quads stay under ~1/(kGrid * split_distance) rad.
constexpr int kGrid = 32;

// Deepest quadtree level. Earth chunks there are ~300 m (~10 m quads): the
// height map is 1.2 km/texel, so finer geometry would only re-trace the same
// bilinear surface. Ground detail below that comes from TerrainDetail.
constexpr int kMaxLevel = 15;

// Renderer view space is the body (ECEF) frame rotated +Z(north) -> +Y(up); see
// rmath::viewBasis / Renderer::ToView. These are that rotation and its inverse.
Vec3 viewToBody(const Vec3& v) { return { v.x, -v.z, v.y }; }
Vec3 bodyToView(const Vec3& b) { return { b.x, b.z, -b.y }; }

void setVec4(bgfx::UniformHandle u, float x, float y, float z, float w) {
    float d[4] = { x, y, z, w };
    bgfx::setUniform(u, d);
}

// Largest triplanar weight each body axis reaches over the chunk, with fs_earth's
// weighting (|n|^8, normalised). The weights follow the cube faces, so the
// extremes are at the corners (and the centre covers a chunk spanning an axis).
Vec3 triplanarReach(const TerrainChunk& c) {
    Vec3 F, A, B;
    TerrainLod::FaceBasis(c.face, F, A, B);
    Vec3 reach = { 0, 0, 0 };
    for (int i = 0; i < 5; ++i) {
        double da = i == 4 ? 0.0 : ((i & 1) ? c.half : -c.half);
        double db = i == 4 ? 0.0 : ((i & 2) ? c.half : -c.half);
        Vec3 n = (F + A * std::tan(c.a + da) + B * std::tan(c.b + db)).unit();
        Vec3 w = { std::pow(n.x, 8.0), std::pow(n.y, 8.0), std::pow(n.z, 8.0) };
        double sum = w.x + w.y + w.z;
        reach = { std::max(reach.x, w.x / sum), std::max(reach.y, w.y / sum), std::max(reach.z, w.z / sum) };
    }
    return reach;
}

} // namespace

void Terrain::Init(const bgfx::VertexLayout& layout) {
    if (bgfx::isValid(prog_)) return;

    TerrainLodParams p;
    p.radius        = EARTH_RADIUS;
    p.max_elevation = EarthBumpMap::kMaxElevation;
    p.max_level     = kMaxLevel;
    lod_.Configure(p);

    Mesh grid = geom::buildGrid(kGrid, /*skirt=*/true);
    vbh_ = bgfx::createVertexBuffer(bgfx::copy(grid.verts.data(), (uint32_t)(grid.verts.size() * sizeof(Vertex))), layout);
    ibh_ = bgfx::createIndexBuffer(bgfx::copy(grid.idx.data(), (uint32_t)(grid.idx.size() * sizeof(uint32_t))),
                                   BGFX_BUFFER_INDEX32);
    prog_ = bgfxutil::loadProgram("vs_terrain", "fs_earth");
    detail_.Create();

    s_color_       = bgfx::createUniform("s_color",       bgfx::UniformType::Sampler);
    s_bump_        = bgfx::createUniform("s_bump",        bgfx::UniformType::Sampler);
    s_night_       = bgfx::createUniform("s_night",       bgfx::UniformType::Sampler);
    s_rough_       = bgfx::createUniform("s_rough",       bgfx::UniformType::Sampler);
    s_cloud_       = bgfx::createUniform("s_cloud",       bgfx::UniformType::Sampler);
    s_detail_      = bgfx::createUniform("s_detail",      bgfx::UniformType::Sampler);
    u_depth_       = bgfx::createUniform("u_depth",       bgfx::UniformType::Vec4);
    u_sunDir_      = bgfx::createUniform("u_sunDir",      bgfx::UniformType::Vec4);
    u_earthCenter_ = bgfx::createUniform("u_earthCenter", bgfx::UniformType::Vec4);
    u_camPos_      = bgfx::createUniform("u_camPos",      bgfx::UniformType::Vec4);
    u_terrain_     = bgfx::createUniform("u_terrain",     bgfx::UniformType::Vec4);
    u_chunkCube_   = bgfx::createUniform("u_chunkCube",   bgfx::UniformType::Vec4);
    u_chunkA_      = bgfx::createUniform("u_chunkA",      bgfx::UniformType::Vec4);
    u_chunkB_      = bgfx::createUniform("u_chunkB",      bgfx::UniformType::Vec4);
    u_chunkTrig_   = bgfx::createUniform("u_chunkTrig",   bgfx::UniformType::Vec4);
    u_chunkOrigin_ = bgfx::createUniform("u_chunkOrigin", bgfx::UniformType::Vec4);
    u_chunkTex_    = bgfx::createUniform("u_chunkTex",    bgfx::UniformType::Vec4);
    u_chunkLod_    = bgfx::createUniform("u_chunkLod",    bgfx::UniformType::Vec4);
    u_detail_      = bgfx::createUniform("u_detail",      bgfx::UniformType::Vec4, TerrainDetail::kLayers);
    u_detailMask_  = bgfx::createUniform("u_detailMask",  bgfx::UniformType::Vec4);
}

void Terrain::Destroy() {
    if (bgfx::isValid(vbh_))  bgfx::destroy(vbh_);
    if (bgfx::isValid(ibh_))  bgfx::destroy(ibh_);
    if (bgfx::isValid(prog_)) bgfx::destroy(prog_);
    vbh_ = BGFX_INVALID_HANDLE; ibh_ = BGFX_INVALID_HANDLE; prog_ = BGFX_INVALID_HANDLE;
    detail_.Destroy();
    for (bgfx::UniformHandle* u : { &s_color_, &s_bump_, &s_night_, &s_rough_, &s_cloud_, &s_detail_,
                                    &u_depth_, &u_sunDir_, &u_earthCenter_, &u_camPos_, &u_terrain_,
                                    &u_chunkCube_, &u_chunkA_, &u_chunkB_, &u_chunkTrig_, &u_chunkOrigin_,
                                    &u_chunkTex_, &u_chunkLod_, &u_detail_, &u_detailMask_ }) {
        if (bgfx::isValid(*u)) bgfx::destroy(*u);
        *u = BGFX_INVALID_HANDLE;
    }
    chunks_.clear();
}

void Terrain::Draw(const EarthFrame& f, const RCamera& cam, float aspect, float far_plane,
                   const EarthTextures& tex, uint32_t bump_w, uint32_t bump_h) {
    if (!bgfx::isValid(prog_) || bump_w == 0 || bump_h == 0) return;
    const double R = lod_.Params().radius;

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

    // The height map's texel size sets each chunk's sampling mip.
    const double texel_m = TAU * R / bump_w;

    for (const TerrainChunk& c : chunks_) {
        setVec4(u_depth_, far_plane, 0, 0, 0);
        setVec4(u_sunDir_, f.sun_dir.x, f.sun_dir.y, f.sun_dir.z, 0);
        setVec4(u_earthCenter_, f.center.x, f.center.y, f.center.z, 0);
        setVec4(u_camPos_, f.cam_pos.x, f.cam_pos.y, f.cam_pos.z, 0);
        setVec4(u_terrain_, (float)R, (float)EarthBumpMap::kMaxElevation, (float)bump_w, (float)bump_h);

        Vec3 F, A, B;
        TerrainLod::FaceBasis(c.face, F, A, B);

        // Floating origin: the sea-level centre in view space, from doubles. The
        // shader only adds small offsets to it.
        Vec3 anchor_m = c.dir * R;
        Vec3 origin   = f.center_km + bodyToView(anchor_m) * M_TO_KM;

        // Height-map texel under the centre (same convention as EarthBumpMap),
        // split into integer + fraction so the shader's bilinear stays exact.
        double u     = std::atan2(c.dir.y, c.dir.x) / TAU + 0.5;
        double colat = std::atan2(std::hypot(c.dir.x, c.dir.y), c.dir.z);
        double tx = u * bump_w, ty = colat / M_PI * bump_h;
        double ix = std::floor(tx), iy = std::floor(ty);

        // Sample coarser mips where quads are bigger than a texel (no aliasing),
        // exact bilinear at lod 0 where they're smaller.
        double quad_m = R * 2.0 * c.half / kGrid;
        double lod    = std::max(0.0, std::log2(quad_m / texel_m));
        double skirt  = std::clamp(c.edge_m * 0.05, 20.0, 10000.0);

        setVec4(u_chunkCube_,   (float)c.cube.x, (float)c.cube.y, (float)c.cube.z, (float)c.cube.mag());
        setVec4(u_chunkA_,      (float)A.x, (float)A.y, (float)A.z, 0);
        setVec4(u_chunkB_,      (float)B.x, (float)B.y, (float)B.z, 0);
        setVec4(u_chunkTrig_,   (float)std::sin(c.a), (float)std::cos(c.a), (float)std::sin(c.b), (float)std::cos(c.b));
        setVec4(u_chunkOrigin_, (float)origin.x, (float)origin.y, (float)origin.z, (float)c.half);
        setVec4(u_chunkTex_,    (float)ix, (float)iy, (float)(tx - ix), (float)(ty - iy));
        setVec4(u_chunkLod_,    (float)lod, (float)skirt, 0, 0);

        // Detail layers: the anchor's position in tiles, wrapped (in double) to
        // [0,1). The shader adds the chunk-local offset, so the tiling is
        // continuous across chunks yet never sees a planet-sized coordinate.
        float det[TerrainDetail::kLayers * 4];
        for (int k = 0; k < TerrainDetail::kLayers; ++k) {
            double inv = 1.0 / TerrainDetail::kLayerPeriod[k];
            Vec3   p   = anchor_m * inv;
            det[k*4 + 0] = (float)(p.x - std::floor(p.x));
            det[k*4 + 1] = (float)(p.y - std::floor(p.y));
            det[k*4 + 2] = (float)(p.z - std::floor(p.z));
            det[k*4 + 3] = (float)inv;
        }
        bgfx::setUniform(u_detail_, det, TerrainDetail::kLayers);

        // Which detail work this chunk can need at all: the triplanar projections
        // with any weight on it, and the layers not yet faded out at its nearest
        // point (a prefix, since they run coarse -> fine). fs_earth branches on
        // these per draw, which keeps its fetches in uniform control flow.
        Vec3 reach  = triplanarReach(c);
        int  layers = 0;
        while (layers < TerrainDetail::kLayers &&
               c.distance_m < TerrainDetail::kFadeStart * TerrainDetail::kLayerPeriod[layers])
            ++layers;
        setVec4(u_detailMask_, reach.x > 0.02 ? 1.0f : 0.0f, reach.y > 0.02 ? 1.0f : 0.0f,
                reach.z > 0.02 ? 1.0f : 0.0f, (float)layers);

        bgfx::setTexture(0, s_color_,  tex.color);
        bgfx::setTexture(1, s_bump_,   tex.bump);
        bgfx::setTexture(2, s_night_,  tex.night);
        bgfx::setTexture(3, s_rough_,  tex.rough);
        bgfx::setTexture(4, s_cloud_,  tex.cloud);   // cloud shadows on the ground
        bgfx::setTexture(5, s_detail_, detail_.Texture());
        bgfx::setVertexBuffer(0, vbh_);
        bgfx::setIndexBuffer(ibh_);
        // No cull: the skirts face outward on some edges and inward on others.
        bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_WRITE_Z
                       | BGFX_STATE_DEPTH_TEST_LESS);
        bgfx::submit(0, prog_);
    }
}

}
