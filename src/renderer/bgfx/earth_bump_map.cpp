#include "earth_bump_map.hpp"
#include "bgfx_util.hpp"

#include <bimg/decode.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace renderer {

namespace {

const uint64_t kSamplerFlags = BGFX_SAMPLER_V_CLAMP | BGFX_SAMPLER_MIN_ANISOTROPIC | BGFX_SAMPLER_MAG_ANISOTROPIC;

long wrap_column(long x, long w) {
    x %= w;
    if (x < 0) x += w;
    return x;
}

long clamp_row(long y, long h) {
  long y_clamped = y >= h ? h - 1 : y;
  return y < 0 ? 0L : y_clamped;
}

}

void EarthBumpMap::Load(const char* path) {
    std::vector<uint8_t> data = bgfxutil::readFile(path);
    if (data.empty()) { std::fprintf(stderr, "EarthBumpMap: missing %s\n", path); return; }

    bx::AllocatorI* alloc = bgfxutil::allocator();
    bimg::ImageContainer* ic = bimg::imageParse(alloc, data.data(), (uint32_t)data.size());
    if (!ic) { std::fprintf(stderr, "EarthBumpMap: parse failed %s\n", path); return; }

    if (bimg::ImageMip mip; bimg::imageGetRawData(*ic, 0, 0, ic->m_data, ic->m_size, mip)) {
        w_ = mip.m_width;
        h_ = mip.m_height;
        heights_.resize((size_t)w_ * h_);
        const uint32_t kStrip = 512;
        for (uint32_t y = 0; y < h_; y += kStrip) {
            uint32_t hs = std::min(kStrip, h_ - y);
            const uint8_t* src = mip.m_data + (size_t)y * w_;
            uint8_t*       dst = heights_.data() + (size_t)y * w_;
            bimg::imageDecodeToR8(alloc, dst, src, w_, hs, 1, w_, mip.m_format);
        }
        uploadHeights();
    } else {
        // Couldn't find mip 0: leave heights empty, but still show the relief.
        std::fprintf(stderr, "EarthBumpMap: imageGetRawData failed %s\n", path);
        tex_ = bgfx::createTexture2D(
            (uint16_t)ic->m_width, (uint16_t)ic->m_height, ic->m_numMips > 1, ic->m_numLayers,
            (bgfx::TextureFormat::Enum)ic->m_format, kSamplerFlags, bgfx::copy(ic->m_data, ic->m_size));
    }

    bimg::imageFree(ic);
}

void EarthBumpMap::uploadHeights() {
    // The GPU gets the decoded heights (exactly what the sim samples) as R8 rather
    // than the DDS's compressed blocks: a GPU block decode can land a code value
    // off bimg's (~35 m of terrain), and the terrain shader relies on the rendered
    // ground being the sim's ground. Mips are a 2x2 box filter of that same data.
    uint32_t mips = 1;
    size_t   total = 0;
    for (uint32_t w = w_, h = h_; ; ++mips) {
        total += (size_t)w * h;
        if (w == 1 && h == 1) break;
        w = std::max(1u, w / 2);
        h = std::max(1u, h / 2);
    }

    const bgfx::Memory* mem = bgfx::alloc((uint32_t)total);
    std::memcpy(mem->data, heights_.data(), heights_.size());
    const uint8_t* src = mem->data;
    uint8_t*       dst = mem->data + heights_.size();
    uint32_t sw = w_, sh = h_;
    for (uint32_t m = 1; m < mips; ++m) {
        uint32_t dw = std::max(1u, sw / 2), dh = std::max(1u, sh / 2);
        for (uint32_t y = 0; y < dh; ++y) {
            const uint8_t* r0 = src + (size_t)std::min(2 * y,     sh - 1) * sw;
            const uint8_t* r1 = src + (size_t)std::min(2 * y + 1, sh - 1) * sw;
            for (uint32_t x = 0; x < dw; ++x) {
                uint32_t x0 = std::min(2 * x, sw - 1), x1 = std::min(2 * x + 1, sw - 1);
                dst[(size_t)y * dw + x] = (uint8_t)((r0[x0] + r0[x1] + r1[x0] + r1[x1] + 2) / 4);
            }
        }
        src = dst;
        dst += (size_t)dw * dh;
        sw = dw; sh = dh;
    }

    tex_ = bgfx::createTexture2D((uint16_t)w_, (uint16_t)h_, true, 1, bgfx::TextureFormat::R8,
                                 kSamplerFlags, mem);
}

void EarthBumpMap::Destroy() {
    if (bgfx::isValid(tex_)) bgfx::destroy(tex_);
    tex_ = BGFX_INVALID_HANDLE;
    heights_.clear();
    heights_.shrink_to_fit();
    w_ = h_ = 0;
}

double EarthBumpMap::sampleHeight01(const Vec3& r) const {
    if (heights_.empty()) return 0.0;
    Vec3 dir = r.unit();

    double u = std::atan2(dir.y, dir.x) * (0.5 / M_PI) + 0.5;
    double v = std::acos(std::clamp(dir.z, -1.0, 1.0)) / M_PI;

    double fract_u = u * (double)w_ - 0.5;
    double fract_v = v * (double)h_ - 0.5;
    double floor_u = std::floor(fract_u);
    double floor_v = std::floor(fract_v);
    double blend_weight_u = fract_u - floor_u;
    double blend_weight_v = fract_v - floor_v;

    long W = (long)w_;
    long H = (long)h_;
    long x0 = wrap_column((long)floor_u, W);
    long x1 = wrap_column((long)floor_u + 1, W);
    long y0 = clamp_row((long)floor_v, H);
    long y1 = clamp_row((long)floor_v + 1, H);

    auto at = [&](long x, long y) { return heights_[(size_t)y * w_ + x] / 255.0; };
    double top = at(x0, y0) * (1.0 - blend_weight_u) + at(x1, y0) * blend_weight_u;
    double bot = at(x0, y1) * (1.0 - blend_weight_u) + at(x1, y1) * blend_weight_u;
    return top * (1.0 - blend_weight_v) + bot * blend_weight_v;
}

double EarthBumpMap::Elevation(const Vec3& r) const {
    return sampleHeight01(r) * kMaxElevation;
}

double EarthBumpMap::SurfaceRadius3D(const Vec3& r) const {
    return EARTH_RADIUS + Elevation(r);
}

double EarthBumpMap::SurfaceRadius2D(double lat_deg, double lon_deg) const {
    double lat = lat_deg * (M_PI / 180.0);
    double lon = lon_deg * (M_PI / 180.0);
    double cl = std::cos(lat);
    Vec3 dir{ cl * std::cos(lon), cl * std::sin(lon), std::sin(lat) };
    return SurfaceRadius3D(dir);
}

double EarthBumpMap::Altitude(const Vec3& eci_m) const {
    return eci_m.mag() - SurfaceRadius3D(eci_m);
}

}
