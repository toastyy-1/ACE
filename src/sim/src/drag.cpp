/**
 * handles all drag for rocket.cpp
 */

 #include "sim/inc/rocket.hpp"
 #include <algorithm>
 #include <cmath>
#include <iostream>


 
///////////////////////////////////////////////////////////////////////////////////////////////
// helper functions                                                                          //
///////////////////////////////////////////////////////////////////////////////////////////////
static double dynamic_pressure(double rho, double V) {
    return 0.5 * rho * V * V;
}

static double normal_drag_force(double dynamic_pressure, double A_ref, double C_N) {
    return dynamic_pressure * A_ref * C_N;
}

static double axial_drag_force(double dynamic_pressure, double A_ref, double C_A) {
    return dynamic_pressure * A_ref * C_A;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// subsonic normal force                                                                     //
///////////////////////////////////////////////////////////////////////////////////////////////
// Munk/Barrowman potential flow
static double cone_C_N_subsonic(double AoA, double A_ref, double A_front, double A_back) {
    return (2.0 * sin(AoA) / A_ref) * (A_back - A_front);
}

// Jorgensen
static double body_C_N_subsonic(double AoA, double A_ref, double A_base, double M) {
    double C = A_base / A_ref;
    double C_d_norm = M * sin(AoA);
    return C * sin(2.0 * AoA) * cos(AoA / 2) + C_d_norm * C * sin(AoA) * sin(AoA);
}

// Niskanen viscous crossflow
static double C_N_lift_subsonic(double AoA, double A_planiform, double A_ref) {
    return 1.1 * (A_planiform / A_ref) * sin(AoA) * sin(AoA);
}

///////////////////////////////////////////////////////////////////////////////////////////////
// center of pressure                                                                        //
///////////////////////////////////////////////////////////////////////////////////////////////
static double X_CP(double cone_height, double len_body, double C_N_cone, double C_N_body) {
    double t1t = (2.0 / 3.0) * cone_height * C_N_cone;
    double t2t = (cone_height + 0.5 * len_body) * C_N_body;
    double t1b = C_N_cone + C_N_body;
    return (t1t + t2t) / t1b;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// axial force                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////
static double C_f(double Re) {
    if (Re < 1.0e4) Re = 1.0e4;
    double t = 1.5 * log(Re) - 5.6;
    return 1.0 / (t * t);
}

static double C_f_c_subsonic(double M, double Re) {
    return C_f(Re) * (1.0 - 0.1 * M * M);
}

static double C_f_c_supersonic(double M, double Re) {
    return C_f(Re) / pow(1.0 + 0.15 * M * M, 0.58);
}

// coeff axial drag for skin friction
static double C_d_friction(double M, double Re, double A_wet, double A_ref, double body_len, double body_diameter) {
    double C_f_c = 1.0;
    if (M > 1.0) {
        C_f_c = C_f_c_supersonic(M, Re);
    }
    else {
        C_f_c = C_f_c_subsonic(M, Re);
    }

    double f_b = body_len / body_diameter;
    return C_f_c * (1.0 + 1.0 / (2.0 * f_b)) * (A_wet / A_ref);
}

// coeff axial drag for presure at nose
static double C_d_wave_drag(double cone_half_angle, double M) {
    double C_D_p = 1.0;
    if (M > 1.3) {
        C_D_p = 2.1 * sin(cone_half_angle) * sin(cone_half_angle) + ((0.5 * sin(cone_half_angle)) / (sqrt(M * M - 1)));
    }
    else {
        C_D_p = 0.8 * sin(cone_half_angle) * sin(cone_half_angle);
    }
    
    return C_D_p;
}

static double C_d_base_drag(double M) {
    double C_d_b = 1.0;
    if (M > 1) {
        C_d_b = 0.25 / M;
    }
    else {
        C_d_b = 0.12 + 0.13 * M * M;
    }

    return C_d_b;
}


/**
 * acceleration due to drag in the ECI frame
 */
Vec3 Rocket::calc_drag_accel(const Vec3& r, const Vec3& v, double mass, const RocketProps& props) {

    // calculate the properties of the air
    double air_density, air_pressure, speed_of_sound, mu;
    atmosphere(r.mag() - EARTH_RADIUS, air_density, air_pressure, speed_of_sound, mu);
    std::cout << "speed of sound: " << speed_of_sound << std::endl;

    // calculate variables relating to the geometry of the craft
    double diameter = props.radius * props.radius;
    double A_ref = (M_PI * diameter * diameter) / 4.0;
    double A_front = A_ref;
    double A_back = A_ref;
    double A_base = A_ref;
    double body_len = rocket_body_length();
    double A_planiform = 0.5 * diameter * props.nosecone_length + diameter * body_len;
    double wetted_area_nose = M_PI * props.radius * sqrt(props.radius * props.radius + props.nosecone_length * props.nosecone_length);
    double wetted_area_body = M_PI * diameter * rocket_body_length();
    double A_wet = wetted_area_body + wetted_area_nose;
    double nosecone_half_angle = atan((diameter / 2) / props.nosecone_length);

    // calculate speed, mach, reynolds num
    double speed = v.mag();
    double Mach = speed / speed_of_sound;
    double Re = (air_density * speed * body_len) / mu;
    std::cout << "mach: " << Mach << std::endl;

    // calcualte the aoa entering into the atmosphere
    Vec3 nose_direction = nose_direction_eci();
    double AoA = std::acos(std::max(-1.0, std::min(1.0, v.dot(nose_direction) / speed)));
    std::cout << "AoA: " << AoA * RAD_TO_DEG << std::endl;

    // calculate dynamic presssure
    double dyn_pressure = dynamic_pressure(air_density, speed);

    // calculate normal drag acceleration
    double C_N = cone_C_N_subsonic(AoA, A_ref, A_front, A_back) + body_C_N_subsonic(AoA, A_ref, A_base, Mach) + C_N_lift_subsonic(AoA, A_planiform, A_ref);
    double a_N = normal_drag_force(dyn_pressure, A_ref, C_N) / mass;

    // calculat axial drag acceleration
    double C_A = C_d_friction(Mach, Re, A_wet, A_ref, body_len, diameter) + C_d_wave_drag(nosecone_half_angle, Mach) + C_d_base_drag(Mach);
    double a_A = axial_drag_force(dyn_pressure, A_ref, C_A) / mass;

    // calculate the normal and axial unit vectors in the body frame with respect to airflow
    

}