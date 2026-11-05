// Copyright 2026 Tyler Wiggins
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <algorithm>
#include <cmath>
#include <random>
#include "types.hpp"
#include "planetary_constants.hpp"
#include "sim_constants.hpp"

// error model of one sensor
struct ImuSensorConfig {
    double noise_std = 0.0;    // white noise std sample^-1
    Vec3 bias{0, 0, 0};        // constant bias
    double bias_walk = 0.0;    // bias walk
    double scale_factor = 0.0; // scale factor error
    double range = 0.0;        // saturation limit
};

// imu settings
struct ImuConfig {
    int seed = 94058;
    ImuSensorConfig acc{0.001};  // m/s^2
    ImuSensorConfig gyr{0.0001}; // rad/s
};

// simulated INS
class INS {
    public:
    INS(const ImuConfig& cfg = ImuConfig()) : rng(cfg.seed), acc(cfg.acc), gyr(cfg.gyr), acc_bias(cfg.acc.bias), gyr_bias(cfg.gyr.bias) {}

    // inertial specific force
    Vec3 read_acc(const Vec3& a_spec_true) { return measure(a_spec_true, acc, acc_bias, acc_noise); }

    // angular velocity
    Vec3 read_gyr(const Vec3& w_true) { return measure(w_true, gyr, gyr_bias, gyr_noise); }

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

    // sensor error models
    ImuSensorConfig acc, gyr;
    Vec3 acc_bias, gyr_bias;

    // unit noise for sensors
    std::normal_distribution<double> acc_noise{0.0, 1.0};
    std::normal_distribution<double> gyr_noise{0.0, 1.0};

    /**
     * @param data_in true value
     * @param s sensor error model
     * @param bias sensor's current bias
     * @param dist normal noise for this sensor
     * @return measured value
     */
    Vec3 measure(Vec3 data_in, const ImuSensorConfig& s, Vec3& bias, std::normal_distribution<double>& dist) {
        if (s.bias_walk > 0) {
            double walk = s.bias_walk * std::sqrt(consts::TIME_STEP);
            bias = {bias.x + walk * dist(rng), bias.y + walk * dist(rng), bias.z + walk * dist(rng)};
        }

        double k = 1.0 + s.scale_factor;
        Vec3 out = {
            data_in.x * k + bias.x + s.noise_std * dist(rng),
            data_in.y * k + bias.y + s.noise_std * dist(rng),
            data_in.z * k + bias.z + s.noise_std * dist(rng)
        };

        if (s.range > 0) {
            out = {std::clamp(out.x, -s.range, s.range), std::clamp(out.y, -s.range, s.range), std::clamp(out.z, -s.range, s.range)};
        }
        return out;
    }
};
