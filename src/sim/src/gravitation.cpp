// Copyright 2026 Tyler Wiggins
// SPDX-License-Identifier: Apache-2.0

#include "sim/inc/rocket.hpp"
#include <cmath>

/**
 * applies the force of another orbital body (other than earth)
 * @param r rocket position
 * @param s body position
 * @param GM G * mass of the body
 * @return perturbing acceleration (m/s^2)
 */
static Vec3 n_body_accel(const Vec3& r, const Vec3& s, const double GM) {
    Vec3 d = s - r;
    double d_norm = d.mag();
    double s_norm = s.mag();
    return (d / (d_norm * d_norm * d_norm) - s / (s_norm * s_norm * s_norm)) * GM;
}

/**
 * gravitational acceleration in the ECI frame
 * @param r current position
 * @param GM G * mass of target planetary body
 * @param J2 J2 term coeff of target body
 * @param R planet radius
 * @param t sim time
 * @return gravitational acceleration vector (m/s^2)
 */
Vec3 Rocket::calc_gravity_accel(const Vec3& r, const double GM, const double J2, const double R, const double t) {
    double r2       = r.dot(r);
    double r_norm   = std::sqrt(r2);
    double pm       = -GM / (r2 * r_norm);

    double zr2      = (r.z * r.z) / r2;
    double k        = 1.5 * J2 * (R * R) / r2;

    Vec3 g = {
        .x = pm * r.x * (1.0 - k * (5.0 * zr2 - 1.0)),
        .y = pm * r.y * (1.0 - k * (5.0 * zr2 - 1.0)),
        .z = pm * r.z * (1.0 - k * (5.0 * zr2 - 3.0))
    };

    if (moon_gravity) g += n_body_accel(r, ephem->moon(t), planet::MOON.gm);
    if (sun_gravity)  g += n_body_accel(r, ephem->sun(t), planet::SUN.gm);

    grav_accel = g;
    return g;
}