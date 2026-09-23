/**
 * handles all drag for rocket.cpp that involve control systems
 */

 #include "sim/inc/rocket.hpp"
 #include <algorithm>

/**
 * acceleration due to drag in the ECI frame
 */
Vec3 Rocket::calc_drag_accel(const Vec3& r, const Vec3& v, double mass) {
    double air_density, air_pressure;
    atmosphere(r.mag() - EARTH_RADIUS, air_density, air_pressure);

    // wind of earth spinning
    Vec3 w_earth = {0, 0, EARTH_ROTATION_RATE};
    Vec3 v_air = w_earth.cross(r);
    Vec3 v_relative = v - v_air;

    double craft_speed = v_relative.mag();
    if (craft_speed < 1e-6) return {0, 0, 0}; // no airspeed

    // calcualte the aoa entering into the atmosphere
    //Vec3 nose_direction = nose_direction_eci();
    //double AoA = std::acos(std::max(-1.0, std::min(1.0, v_relative.dot(nose_direction) / craft_speed)));
    //std::cout << AoA * RAD_TO_DEG << std::endl;

    // @todo apply drag (shid rn add a real drag model idoit) ((idfk how im going to do that simply))
    double area = M_PI * props.radius * props.radius;
    double drag_mag = 0.5 * air_density * craft_speed * craft_speed * props.Cd * area;
    return v_relative * (-drag_mag / (mass * craft_speed));
}