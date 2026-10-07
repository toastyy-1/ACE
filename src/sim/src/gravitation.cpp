#include "sim/inc/rocket.hpp"
#include <cmath>

/**
 * gravitational acceleration in the ECI frame
 * @param r current position
 * @param GM G * mass of target planetary body
 * @param J2 J2 term coeff of target body
 * @param R planet radius
 * @return gravitational acceleration vector (m/s^2)
 */
Vec3 Rocket::calc_gravity_accel(const Vec3& r, const double GM, const double J2, const double R) {
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

    grav_accel = g;
    return g;
}