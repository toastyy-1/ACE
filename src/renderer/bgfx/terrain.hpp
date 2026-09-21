#pragma once

#include <bgfx/bgfx.h>
#include <vector>
#include "../scene.hpp"
#include "../terrain_lod.hpp"
#include "terrain_detail.hpp"

// The Earth's solid surface for the bgfx backend: a cube-sphere quadtree
// (TerrainLod) drawn as one shared grid per chunk with vs_terrain + fs_earth.
// One system covers every altitude: coarse chunks from orbit, ~10 m quads near
// the ground, and distance-tiered detail textures (TerrainDetail) below a few km.
//
// Geometry is the sim's own ground model: vs_terrain samples the same R8
// heights EarthBumpMap collides with, bilinearly, exactly as the sim does, so a
// rocket resting on the ground renders resting on it. Shading-only detail
// (fs_earth) never moves the surface.
//
// Owned by BgfxBackend (like RocketModel); the backend loads the global Earth
// maps and hands them in each frame.

namespace renderer {

// The global Earth maps fs_earth samples. Owned by the backend.
struct EarthTextures {
    bgfx::TextureHandle color, bump, night, rough, cloud;
};

class Terrain {
public:
    // `layout` must be the backend's Vertex layout (the chunk grid is a Mesh).
    void Init(const bgfx::VertexLayout& layout);
    void Destroy();

    // Submit the visible chunks to view 0. `aspect` and `far_plane` are the 3D
    // view's, `bump_w`/`bump_h` the height map size in texels.
    void Draw(const EarthFrame& f, const RCamera& cam, float aspect, float far_plane,
              const EarthTextures& tex, uint32_t bump_w, uint32_t bump_h);

    // Chunks drawn last frame (for diagnostics / tuning the split distance).
    size_t ChunkCount() const { return chunks_.size(); }

private:
    TerrainLod                lod_;
    TerrainDetail             detail_;
    std::vector<TerrainChunk> chunks_;   // reused every frame

    bgfx::VertexBufferHandle vbh_  = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle  ibh_  = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle      prog_ = BGFX_INVALID_HANDLE;

    // Samplers (fs_earth's slots 0-5).
    bgfx::UniformHandle s_color_ = BGFX_INVALID_HANDLE, s_bump_  = BGFX_INVALID_HANDLE,
                        s_night_ = BGFX_INVALID_HANDLE, s_rough_ = BGFX_INVALID_HANDLE,
                        s_cloud_ = BGFX_INVALID_HANDLE, s_detail_ = BGFX_INVALID_HANDLE;
    // Per frame.
    bgfx::UniformHandle u_depth_ = BGFX_INVALID_HANDLE, u_sunDir_ = BGFX_INVALID_HANDLE,
                        u_earthCenter_ = BGFX_INVALID_HANDLE, u_camPos_ = BGFX_INVALID_HANDLE,
                        u_terrain_ = BGFX_INVALID_HANDLE;
    // Per chunk (see vs_terrain.sc).
    bgfx::UniformHandle u_chunkCube_ = BGFX_INVALID_HANDLE, u_chunkA_ = BGFX_INVALID_HANDLE,
                        u_chunkB_ = BGFX_INVALID_HANDLE, u_chunkTrig_ = BGFX_INVALID_HANDLE,
                        u_chunkOrigin_ = BGFX_INVALID_HANDLE,
                        u_chunkTex_ = BGFX_INVALID_HANDLE, u_chunkLod_ = BGFX_INVALID_HANDLE,
                        u_detail_ = BGFX_INVALID_HANDLE, u_detailMask_ = BGFX_INVALID_HANDLE;
};

}
