// Copyright 2026 Tyler Wiggins
// SPDX-License-Identifier: Apache-2.0

#include "sim/inc/rocket.hpp"
#include "constants.hpp"
#include <cmath>

/**
 * power relationship density equation, sets T to the layer temperature
 * @param altitude altitude above the surface (m)
 * @param T output, air temperature at this altitude (K)
 * @param rho_b air density at the base of the layer (kg/m^3)
 * @param T_b air temperature at the base of the layer (K)
 * @param L temperature lapse rate in the layer, change in temperature per meter of altitude (K/m)
 * @param layer_base_alt altitude of the base of the layer (m)
 * @return air density at this altitude (kg/m^3)
 */
static double pow_dens(double altitude, double& T, double rho_b, double T_b, double L, double layer_base_alt) {
    T = T_b + L * (altitude - layer_base_alt);
    return rho_b * pow(( T / T_b ), (-1.0 * g0 / (R_d * L)) - 1);
}

/**
 * exponential relationship density equation, sets T to the layer temperature
 * @param altitude altitude above the surface (m)
 * @param T output, air temperature at this altitude, constant across an isothermal layer (K)
 * @param rho_b air density at the base of the layer (kg/m^3)
 * @param T_b air temperature throughout the layer (K)
 * @param layer_base_alt altitude of the base of the layer (m)
 * @return air density at this altitude (kg/m^3)
 */
static double exp_dens(double altitude, double& T, double rho_b, double T_b, double layer_base_alt) {
    T = T_b;
    return rho_b * exp(-1.0 * (g0 * (altitude - layer_base_alt)) / (R_d * T_b));
}

/**
 * standard atmosphere layers
 * @param altitude altitude above the surface (m)
 * @param air_density output, air density (kg/m^3)
 * @param air_pressure output, static air pressure (Pa)
 * @param speed_of_sound output, speed of sound in the air (m/s)
 * @param mu output, dynamic viscosity of the air from Sutherland's law (Pa*s)
 */
void atmosphere(double altitude, double& air_density, double& air_pressure, double& speed_of_sound, double& mu) {
    double T = 288.15; // layer temperature, set by whichever branch runs

    // troposphere
    if (altitude < 11000) {
        air_density = pow_dens(altitude, T, 1.2250, 288.15, -0.0065, 0);
    }
    // lower stratosphere
    else if (altitude < 20000) {
        air_density = exp_dens(altitude, T, 0.36391, 216.65, 11000);
    }
    // middle stratosphere
    else if (altitude < 32000) {
        air_density = pow_dens(altitude, T, 0.088035, 216.65, 0.001, 20000);
    }
    // upper stratosphere
    else if (altitude < 47000) {
        air_density = pow_dens(altitude, T, 0.013225, 228.65, 0.0028, 32000);
    }
    // lower mesosphere
    else if (altitude < 51000) {
        air_density = exp_dens(altitude, T, 0.0014275, 270.65, 47000);
    }
    // middle mesosphere
    else if (altitude < 71000) {
        air_density = pow_dens(altitude, T, 0.00086160, 270.65, -0.0028, 51000);
    }
    // upper mesosphere
    else if (altitude < 86000) {
        air_density = pow_dens(altitude, T, 0.000064211, 214.65, -0.0020, 71000);
    }
    // thermosphere
    else {
        air_density = exp_dens(altitude, T, 0.000006958, 186.87, 86000);
    }

    air_pressure = air_density * R_d * T;
    speed_of_sound = sqrt(1.4 * R_d * T);
    mu = 1.458e-6 * pow(T, 1.5) / (T + 110.4);
}
