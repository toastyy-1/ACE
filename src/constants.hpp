#pragma once

#include <cmath>

// GLOBAL TIME STEP FOR SIMULATION AND SUCH THINGS (overwritten by config)
inline double STEP_DELAY = 0.001;
inline double TIME_STEP = 0.01; // seconds

// Math
constexpr double TAU        = 6.28318530717958647692;
constexpr double DEG_TO_RAD = 0.017453292519943295769;
constexpr double RAD_TO_DEG = 57.295779513082320876;

// Physical
constexpr double G    = 6.67430e-11;
constexpr double g0   = 9.80665;
constexpr double R_d  = 287.05287;
constexpr double p_b  = 1.2250;
constexpr double p_sl = 101325.0;

// WGS84
constexpr double EARTH_RADIUS         = 6378137.0;
constexpr double EARTH_RADIUS_KM      = 6378.137;
constexpr double EARTH_ROTATION_RATE  = 7.292115e-5;
constexpr double GM_EARTH             = 3.986004418e14;
constexpr double EARTH_MASS           = GM_EARTH / G;

// Gravity
constexpr double J2 = 1.08262668355e-3;

// Rendering
constexpr float SIM_TO_RENDER = 0.001f;

// Unit Conversions
constexpr double M_TO_KM   = 1.0e-3;
constexpr double KM_TO_M   = 1.0e3;
constexpr double FT_TO_M   = 0.3048;
constexpr double M_TO_FT   = 1.0 / 0.3048;
constexpr double NMI_TO_M  = 1852.0;
constexpr double M_TO_NMI  = 1.0 / 1852.0;
constexpr double LBF_TO_N  = 4.4482216152605;
constexpr double N_TO_LBF  = 1.0 / 4.4482216152605;
constexpr double KG_TO_LBM = 2.2046226218487758;
constexpr double LBM_TO_KG = 1.0 / 2.2046226218487758;
