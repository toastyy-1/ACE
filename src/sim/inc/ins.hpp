// Copyright 2026 Tyler Wiggins
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <random>
#include "types.hpp"
#include "planetary_constants.hpp"

// simulated INS
class INS {
    public:
    INS() : rng(94058) {}

    // inertial specific force
    Vec3 read_acc(const Vec3& a_spec_true) { return add_noise(a_spec_true, acc_noise); }

    // angular velocity
    Vec3 read_gyr(const Vec3& w_true) { return add_noise(w_true, gyr_noise); }

    // gravitational acceleration in ECI
    static Vec3 gravity_eci(Vec3 r) {
        double rn = r.mag();
        if (rn < 1.0) return {0.0, 0.0, 0.0};

        double rn_sq = rn * rn;
        double term = planet::EARTH.gm / (rn_sq * rn);

        double zr2 = (r.z * r.z) / rn_sq;
        double j2_factor = 1.5 * planet::EARTH.j2 * (planet::EARTH.radius * planet::EARTH.radius) / rn_sq;

        return {
            -term * r.x * (1.0 + j2_factor * (1.0 - 5.0 * zr2)),
            -term * r.y * (1.0 + j2_factor * (1.0 - 5.0 * zr2)),
            -term * r.z * (1.0 + j2_factor * (3.0 - 5.0 * zr2))
        };
    }

    private:
    std::mt19937 rng;

    // nosie for sensors
    std::normal_distribution<double> acc_noise{0.0, 0.001};
    std::normal_distribution<double> gyr_noise{0.0, 0.0001};

    Vec3 add_noise(Vec3 data_in, std::normal_distribution<double>& dist) {
        return {
            data_in.x + dist(rng),
            data_in.y + dist(rng),
            data_in.z + dist(rng)
        };
    }
};
