// Copyright 2026 Tyler Wiggins
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include <string>
#include <vector>
#include "types.hpp"

// sun and moon positions read from a JPL DE440 SPK kernel
class Ephemeris {
    public:
    bool load(const std::string& path, const std::string& epoch_utc);

    Vec3 moon(double t) const;
    Vec3 sun(double t) const;

    private:
    struct Segment {
        int target, center;
        double init = 0, intlen = 0;
        int rsize = 0, n = 0;
        std::vector<double> data;
        Vec3 position(double et) const;
    };

    Segment emb       {3, 0};
    Segment sun_ssb   {10, 0};
    Segment moon_emb  {301, 3};
    Segment earth_emb {399, 3};

    double et0 = 0;
    double cos_era = 1;
    double sin_era = 0;

    Vec3 to_eci(const Vec3& p_km) const;
};
