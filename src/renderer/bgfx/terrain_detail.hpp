#pragma once

#include <bgfx/bgfx.h>

// Ground detail for the terrain close up (the KSP-style near/far texture swap).
// The global Earth maps top out at ~1.2 km/texel, so below a few km everything
// finer is supplied by one small tileable texture sampled at several tiling
// periods ("layers"). fs_earth fades each layer in by camera distance, so on
// the way down the ground gains detail in steps of kLayerPeriod instead of
// magnifying one blurry map.
//
// The texture holds one greyscale detail signal per ground material; fs_earth
// blends them by what the global colour/relief says is there:
//   r = soil / vegetation, g = rock, b = sand, a = snow
// It is generated procedurally at startup (tileable fBm/ridged noise), so no
// asset is needed. Authored textures with the same channel layout can replace
// Create() later without touching the shaders.

namespace renderer {

class TerrainDetail {
public:
    static constexpr int kLayers = 4;
    // Tiling period of each layer in metres, coarse -> fine. Terrain anchors each
    // layer per chunk. Must match u_detail's array size.
    static constexpr double kLayerPeriod[kLayers] = { 4096.0, 512.0, 64.0, 8.0 };
    // A layer fades in from kFadeStart periods of camera distance to a quarter of
    // that (hardcoded as 40 / 10 in fs_earth; keep in sync). Beyond kFadeStart it
    // contributes nothing, so Terrain skips it for chunks that far away.
    static constexpr double kFadeStart = 40.0;

    void Create();
    void Destroy();
    bgfx::TextureHandle Texture() const { return tex_; }

private:
    bgfx::TextureHandle tex_ = BGFX_INVALID_HANDLE;
};

}
