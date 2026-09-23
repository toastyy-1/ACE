#include "terrain_detail.hpp"
#include "bgfx_util.hpp"
#include "../../constants.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

namespace renderer {

namespace {

constexpr int kSize = 512;   // texels per side (power of two, for the mip chain)

uint32_t hashLattice(int x, int y, uint32_t seed) {
    uint32_t h = (uint32_t)x * 0x8da6b343u ^ (uint32_t)y * 0xd8163841u ^ seed * 0xcb1ab31fu;
    h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
    return h;
}

float fade(float t) { return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); }

// Gradient at lattice point (ix, iy), wrapped every `period` cells so the noise
// tiles, dotted with the offset from that point.
float corner(int ix, int iy, float dx, float dy, int period, uint32_t seed) {
    int wx = ((ix % period) + period) % period;
    int wy = ((iy % period) + period) % period;
    float ang = (float)(hashLattice(wx, wy, seed) & 0xffffu) * (float)(TAU / 65536.0);
    return std::cos(ang) * dx + std::sin(ang) * dy;
}

// Tileable Perlin gradient noise, x/y in lattice cells. Roughly [-0.7, 0.7].
float gradientNoise(float x, float y, int period, uint32_t seed) {
    int   x0 = (int)std::floor(x), y0 = (int)std::floor(y);
    float fx = x - (float)x0,      fy = y - (float)y0;
    float n00 = corner(x0,     y0,     fx,        fy,        period, seed);
    float n10 = corner(x0 + 1, y0,     fx - 1.0f, fy,        period, seed);
    float n01 = corner(x0,     y0 + 1, fx,        fy - 1.0f, period, seed);
    float n11 = corner(x0 + 1, y0 + 1, fx - 1.0f, fy - 1.0f, period, seed);
    float ux = fade(fx), uy = fade(fy);
    float nx0 = n00 + (n10 - n00) * ux;
    float nx1 = n01 + (n11 - n01) * ux;
    return nx0 + (nx1 - nx0) * uy;
}

// Fractal sum over octaves, u/v in [0,1) of the tile. Each octave doubles the
// lattice period, so every octave still tiles.
float fbm(float u, float v, int period, int octaves, float gain, uint32_t seed) {
    float sum = 0.0f, amp = 1.0f;
    for (int o = 0; o < octaves; ++o) {
        sum += amp * gradientNoise(u * period, v * period, period, seed + (uint32_t)o * 101u);
        period *= 2;
        amp *= gain;
    }
    return sum;
}

// Ridged multifractal: sharp crests and creases, reads as fractured rock.
float ridged(float u, float v, int period, int octaves, float gain, uint32_t seed) {
    float sum = 0.0f, amp = 1.0f;
    for (int o = 0; o < octaves; ++o) {
        float r = 1.0f - std::fabs(gradientNoise(u * period, v * period, period, seed + (uint32_t)o * 131u) * 1.4f);
        sum += amp * r * r;
        period *= 2;
        amp *= gain;
    }
    return sum;
}

// Soil / vegetation: clumpy mid-frequency mottling with darker patches (scrub,
// tree cover, field texture depending on the layer's scale).
float soilSignal(float u, float v) {
    float base   = fbm(u, v, 8, 6, 0.55f, 11u);
    float clumps = fbm(u, v, 16, 3, 0.5f, 23u);
    return base - 0.8f * std::max(clumps, 0.0f);
}

float rockSignal(float u, float v) {
    return ridged(u, v, 6, 6, 0.5f, 37u) + 0.3f * fbm(u, v, 32, 3, 0.5f, 41u);
}

// Sand: soft low-contrast dunes with wind ripples. The ripple frequency is an
// integer count per tile and its warp is tileable, so this still tiles.
float sandSignal(float u, float v) {
    float dunes  = fbm(u, v, 4, 5, 0.4f, 53u);
    float warp   = fbm(u, v, 4, 3, 0.5f, 59u);
    float ripple = std::sin((float)TAU * (v * 48.0f + warp * 3.0f));
    return dunes + 0.05f * ripple;
}

float snowSignal(float u, float v) {
    return fbm(u, v, 4, 5, 0.35f, 71u);
}

// Rescale to mean 0.5 with a common spread so each material's contrast is set in
// the shader, not by whatever range its noise happened to produce.
void normaliseChannel(std::vector<float>& ch) {
    double mean = 0.0, var = 0.0;
    for (float x : ch) mean += x;
    mean /= (double)ch.size();
    for (float x : ch) var += (x - mean) * (x - mean);
    double sd = std::sqrt(var / (double)ch.size());
    float  k  = sd > 1e-9 ? (float)(0.17 / sd) : 0.0f;
    for (float& x : ch) x = std::clamp(0.5f + (x - (float)mean) * k, 0.0f, 1.0f);
}

} // namespace

void TerrainDetail::Create() {
    if (bgfx::isValid(tex_)) return;

    const size_t n = (size_t)kSize * kSize;
    std::vector<float> soil(n), rock(n), sand(n), snow(n);
    // ~40 noise octaves per texel: split the rows across cores (each texel is
    // independent, so the result doesn't depend on the split).
    bgfxutil::parallelFor(kSize, [&](uint32_t y0, uint32_t y1) {
        for (int y = (int)y0; y < (int)y1; ++y) {
            for (int x = 0; x < kSize; ++x) {
                float  u = (x + 0.5f) / kSize, v = (y + 0.5f) / kSize;
                size_t i = (size_t)y * kSize + x;
                soil[i] = soilSignal(u, v);
                rock[i] = rockSignal(u, v);
                sand[i] = sandSignal(u, v);
                snow[i] = snowSignal(u, v);
            }
        }
    });
    normaliseChannel(soil);
    normaliseChannel(rock);
    normaliseChannel(sand);
    normaliseChannel(snow);

    // RGBA8 with a full box-filtered mip chain (wrapping, since it tiles).
    size_t total = 0;
    int    mips  = 0;
    for (int s = kSize; s >= 1; s /= 2) { total += (size_t)s * s * 4; ++mips; }
    const bgfx::Memory* mem = bgfx::alloc((uint32_t)total);
    uint8_t* lvl = mem->data;
    for (size_t i = 0; i < n; ++i) {
        lvl[i*4 + 0] = (uint8_t)(soil[i] * 255.0f + 0.5f);
        lvl[i*4 + 1] = (uint8_t)(rock[i] * 255.0f + 0.5f);
        lvl[i*4 + 2] = (uint8_t)(sand[i] * 255.0f + 0.5f);
        lvl[i*4 + 3] = (uint8_t)(snow[i] * 255.0f + 0.5f);
    }
    for (int s = kSize; s > 1; s /= 2) {
        const int d = s / 2;
        uint8_t* next = lvl + (size_t)s * s * 4;
        for (int y = 0; y < d; ++y) {
            for (int x = 0; x < d; ++x) {
                for (int c = 0; c < 4; ++c) {
                    int sum = lvl[((size_t)(2*y)     * s + 2*x)     * 4 + c]
                            + lvl[((size_t)(2*y)     * s + 2*x + 1) * 4 + c]
                            + lvl[((size_t)(2*y + 1) * s + 2*x)     * 4 + c]
                            + lvl[((size_t)(2*y + 1) * s + 2*x + 1) * 4 + c];
                    next[((size_t)y * d + x) * 4 + c] = (uint8_t)((sum + 2) / 4);
                }
            }
        }
        lvl = next;
    }

    tex_ = bgfx::createTexture2D(kSize, kSize, mips > 1, 1, bgfx::TextureFormat::RGBA8,
                                 BGFX_SAMPLER_MIN_ANISOTROPIC | BGFX_SAMPLER_MAG_ANISOTROPIC, mem);
    if (!bgfx::isValid(tex_)) std::fprintf(stderr, "TerrainDetail: texture creation failed\n");
}

void TerrainDetail::Destroy() {
    if (bgfx::isValid(tex_)) bgfx::destroy(tex_);
    tex_ = BGFX_INVALID_HANDLE;
}

}
