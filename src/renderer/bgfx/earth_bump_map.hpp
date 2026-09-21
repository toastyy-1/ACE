#pragma once

#include <bgfx/bgfx.h>
#include <vector>
#include <cstdint>
#include "../../types.hpp"

namespace renderer {

class EarthBumpMap {
public:
    static constexpr double kMaxElevation = 8849.0;

    static EarthBumpMap& Get() { static EarthBumpMap s; return s; }

    EarthBumpMap() = default;
    EarthBumpMap(const EarthBumpMap&) = delete;
    EarthBumpMap& operator=(const EarthBumpMap&) = delete;

    void Load(const char* path);
    void Destroy();

    bool Valid() const { return !heights_.empty(); }
    bgfx::TextureHandle Texture() const { return tex_; }
    // Height map size in texels (equirectangular: width spans longitude).
    uint32_t Width() const  { return w_; }
    uint32_t Height() const { return h_; }

    double SurfaceRadius3D(const Vec3& r) const;

    double Altitude(const Vec3& eci_m) const;

    double Elevation(const Vec3& r) const;

    double SurfaceRadius2D(double lat_deg, double lon_deg) const;

private:
    double sampleHeight01(const Vec3& r) const;
    void   uploadHeights();   // heights_ -> tex_ (R8 + mips)

    bgfx::TextureHandle  tex_ = BGFX_INVALID_HANDLE;
    std::vector<uint8_t> heights_;
    uint32_t w_ = 0, h_ = 0;
};

}
