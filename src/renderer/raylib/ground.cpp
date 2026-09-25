#include "ground.hpp"
#include "theme.hpp"
#include "../../constants.hpp"
#include <cmath>
#include <cstdio>

namespace renderer {

namespace {

RVec3 toF(const Vec3& v) { return { (float)v.x, (float)v.y, (float)v.z }; }
Vec3  toD(const RVec3& v) { return { v.x, v.y, v.z }; }

} // namespace

void GroundMarks::Draw(RenderBackend& b, const EarthFrame& f, const RCamera& cam,
                       std::vector<WorldLabel>& labels) {
    using namespace theme;
    haveHeading_ = false;
    lines_.clear();

    // All in view km, in double: the ground is ~6400 km from the view origin.
    const double R    = EARTH_RADIUS * M_TO_KM;
    const Vec3   C    = f.center_km;
    const double cmag = C.mag();
    if (cmag < R * 0.5) return;               // no rocket yet: the scene is centred on the Earth
    const Vec3   up   = C * (-1.0 / cmag);    // local vertical at the rocket
    const Vec3   G    = C + up * R;           // the ground point under it
    const double alt  = cmag - R;

    // Local north and east. View +Y is the polar axis.
    Vec3 north = Vec3{ 0, 1, 0 } - up * up.y;
    if (north.mag() < 1e-9) north = Vec3{ 1, 0, 0 } - up * up.x;   // at a pole, any way is north
    north = north.unit();
    const Vec3 east = north.cross(up);

    // Which way the camera faces, as a compass heading.
    const Vec3 eye = toD(cam.position);
    const Vec3 fwd = (toD(cam.target) - eye).unit();
    const Vec3 fh  = fwd - up * fwd.dot(up);
    if (fh.mag() > 1e-6) {
        headingDeg_  = std::fmod(std::atan2(fh.dot(east), fh.dot(north)) * RAD_TO_DEG + 360.0, 360.0);
        haveHeading_ = true;
    }

    auto seg = [&](const Vec3& p, const Vec3& q, RColor c) {
        lines_.push_back({ toF(p), c });
        lines_.push_back({ toF(q), c });
    };

    // Dashed drop line from the rocket (the origin) to the ground, altitude beside it.
    if (alt > 0.05) {   // skip on the pad, where it would only clutter the rocket
        const int dashes = 24;
        for (int i = 0; i < dashes; ++i)
            seg(G * (i / (double)dashes), G * ((i + 0.5) / dashes), withAlpha(kSelect, 200));
        char buf[32];
        std::snprintf(buf, sizeof buf, "ALT %.2f KM", alt);
        labels.push_back({ toF(G * 0.5), buf, kSelect });
    }

    // A cross on the ground where it lands, sized to stay readable at any zoom
    // and lifted just clear of the globe's surface lines.
    const double s    = (eye - G).mag() * 0.015;
    const Vec3   base = G + up * (s * 0.02);
    seg(base - north * s, base + north * s, kSelect);
    seg(base - east * s,  base + east * s,  kSelect);

    b.DrawLines(lines_.data(), lines_.size(), 1.0f);
}

}
