// Copyright 2026 Tyler Wiggins
// SPDX-License-Identifier: Apache-2.0

#pragma once

namespace planet {

// ORBITAL BODIES
struct planetaryBody {
    double radius;          // m
    double rotation_rate;   // rad s^-1
    double gm;              // m^3 s^-2
    double j2;              // -
    double mass;            // kg

    constexpr double radius_km() const { return radius / 1000.0; }
};

// EARTH
inline constexpr planetaryBody EARTH {
    .radius = 6378137.0,
    .rotation_rate = 7.292115e-5,
    .gm = 3.986004418e14,
    .j2 = 1.08262668355e-3,
    .mass = 5.9722e24
};

// MOON
inline constexpr planetaryBody MOON {
    .radius = 1737400.0,
    .rotation_rate = 2.6617e-6,
    .gm = 4.9028e12,
    .j2 = 2.0321e-4,
    .mass = 7.342e22
};

// SUN
inline constexpr planetaryBody SUN {
    .radius = 6.957e8,
    .rotation_rate = 2.9032e-6,
    .gm = 1.32712440018e20,
    .j2 = 2.2e-7,
    .mass = 1.98841e30
};

}
