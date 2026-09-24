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
/**
 * @brief
 * @param rho air density (kg/m^3)
 * @param V airspeed relative to the air (m/s)
 * @return dynamic pressure
 */
static double dynamic_pressure(double rho, double V) {
    return 0.5 * rho * V * V;
}

/**
 * @brief
 * @param dynamic_pressure dynamic pressure (Pa)
 * @param A_ref reference area, the body's cross section (m^2)
 * @param C_N normal force coefficient
 * @return force perpendicular to the body axis (N)
 */
static double normal_drag_force(double dynamic_pressure, double A_ref, double C_N) {
    return dynamic_pressure * A_ref * C_N;
}

/**
 * @brief
 * @param dynamic_pressure dynamic pressure (Pa)
 * @param A_ref reference area, the body's cross section (m^2)
 * @param C_A axial drag coefficient
 * @return force along the body axis (N)
 */
static double axial_drag_force(double dynamic_pressure, double A_ref, double C_A) {
    return dynamic_pressure * A_ref * C_A;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// subsonic normal force                                                                     //
///////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Munk/Barrowman potential flow
 * @param AoA angle of attack
 * @param A_ref reference area, the body's cross section (m^2)
 * @param A_front cross sectional area at the front of the nosecone (m^2)
 * @param A_back cross sectional area at the base of the nosecone (m^2)
 * @return nosecone normal force coefficient
 */
static double cone_C_N_subsonic(double AoA, double A_ref, double A_front, double A_back) {
    return (2.0 * sin(AoA) / A_ref) * (A_back - A_front);
}

/**
 * @brief Niskanen viscous crossflow
 * @param AoA angle of attack (rad)
 * @param A_planiform planform area (m^2)
 * @param A_ref reference area, the body's cross section (m^2)
 * @return body normal force coefficient from viscous crossflow
 */
static double C_N_lift_subsonic(double AoA, double A_planiform, double A_ref) {
    return 1.1 * (A_planiform / A_ref) * sin(AoA) * sin(AoA);
}

///////////////////////////////////////////////////////////////////////////////////////////////
// center of pressure                                                                        //
///////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief
 * @param cone_height nosecone length (m)
 * @param len_body length of the cylindrical body behind the nosecone (m)
 * @param C_N_cone nosecone normal force coefficient 
 * @param C_N_body body normal force coefficient
 * @return center of pressure distance back from the nose tip (m)
 */
static double X_CP(double cone_height, double len_body, double C_N_cone, double C_N_body) {
    double t1t = (2.0 / 3.0) * cone_height * C_N_cone;
    double t2t = (cone_height + 0.5 * len_body) * C_N_body;
    double t1b = C_N_cone + C_N_body;
    return (t1t + t2t) / t1b;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// axial force                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief
 * @param Re Reynolds number
 * @return incompressible skin friction coefficient
 */
static double C_f(double Re) {
    if (Re < 1.0e4) Re = 1.0e4;
    double t = 1.5 * log(Re) - 5.6;
    return 1.0 / (t * t);
}

/**
 * @brief
 * @param M Mach number
 * @param Re Reynolds number
 * @return skin friction coefficient corrected for subsonic compressibilit
 */
static double C_f_c_subsonic(double M, double Re) {
    return C_f(Re) * (1.0 - 0.1 * M * M);
}

/**
 * @brief
 * @param M Mach number
 * @param Re Reynolds number
 * @return skin friction coefficient corrected for supersonic compressibility
 */
static double C_f_c_supersonic(double M, double Re) {
    return C_f(Re) / pow(1.0 + 0.15 * M * M, 0.58);
}

/**
 * @brief coeff axial drag for skin friction
 * @param M Mach number
 * @param Re Reynolds number
 * @param A_wet wetted surface area of the vehicle (m^2)
 * @param A_ref reference area, the body's cross section (m^2)
 * @param body_len overall vehicle length from nose tip to aft end (m)
 * @param body_diameter vehicle diameter (m)
 * @return skin friction drag coefficient, referenced to A_ref
 */
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

/**
 * @brief coeff axial drag for presure at nose
 * @param cone_half_angle half angle of the nosecone (rad)
 * @param M Mach number 
 * @return nosecone wave drag coefficient, referenced to A_ref
 */
static double C_d_wave_drag(double cone_half_angle, double M) {
    double s = sin(cone_half_angle);

    if (M > 1.3) {
        return 2.1 * s * s + (0.5 * s) / sqrt(M * M - 1.0);
    }
    return 0.8 * s * s;
}

/**
 * @brief
 * @param M Mach number
 * @param engine_burning true while the engine is producing thrust
 * @return base drag coefficient, 0 while the engine is burning
 */
static double C_d_base_drag(double M, bool engine_burning) {
    if (engine_burning) {
        return 0.0;
    }

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
 * @param r position in ECI (m)
 * @param v velocity in ECI (m/s)
 * @param q body orientation, rotates the body frame into ECI
 * @param mass current total mass of the rocket (kg)
 * @param props rocket geometry, radius and nosecone length are used here
 * @return drag acceleration in ECI (m/s^2), also stored in drag_accel
 */
Vec3 Rocket::calc_drag_accel(const Vec3& r, const Vec3& v, const Quat& q, double mass, const RocketProps& props) {

    // calculate the properties of the air
    double air_density, air_pressure, speed_of_sound, mu;
    atmosphere(r.mag() - EARTH_RADIUS, air_density, air_pressure, speed_of_sound, mu);

    // calculate variables relating to the geometry of the craft
    double diameter = 2.0 * props.radius;
    double A_ref = (M_PI * diameter * diameter) / 4.0;
    double A_front = 0.0; // nosecone tip
    double A_back = A_ref;
    double body_len = rocket_body_length();
    double total_len = body_len + props.nosecone_length;
    double A_planiform = 0.5 * diameter * props.nosecone_length + diameter * body_len;
    double wetted_area_nose = M_PI * props.radius * sqrt(props.radius * props.radius + props.nosecone_length * props.nosecone_length);
    double wetted_area_body = M_PI * diameter * body_len;
    double A_wet = wetted_area_body + wetted_area_nose;
    double nosecone_half_angle = atan((diameter / 2) / props.nosecone_length);

    // calculate speed, mach, reynolds num
    Vec3 wind_speed = surface_velocity_eci(r);
    Vec3 rel_airspeed = v - wind_speed;
    double speed = rel_airspeed.mag();
    double Mach = speed / speed_of_sound;
    double Re = (air_density * speed * total_len) / mu;
    mach = Mach;

    if (speed < 1e-6) {
        dyn_pressure = 0;
        aoa = 0;
        drag_accel = {0, 0, 0};
        return {0, 0, 0};
    }

    // calcualte the aoa entering into the atmosphere
    Vec3 nose_direction = nose_direction_eci(q);
    double AoA = std::acos(std::max(-1.0, std::min(1.0, rel_airspeed.dot(nose_direction) / speed)));

    // calculate dynamic presssure
    dyn_pressure = dynamic_pressure(air_density, speed);
    aoa = AoA;

    // calculate normal drag acceleration magnitude
    double C_N_cone = cone_C_N_subsonic(AoA, A_ref, A_front, A_back);
    double C_N_body = C_N_lift_subsonic(AoA, A_planiform, A_ref);
    double C_N = C_N_cone + C_N_body;
    double a_N = normal_drag_force(dyn_pressure, A_ref, C_N) / mass;

    // calculat axial drag acceleration magnitude
    bool engine_burning = active_stage().thrust > 0.0 && active_stage().m_fuel > 0.0;
    double C_A = C_d_friction(Mach, Re, A_wet, A_ref, total_len, diameter) + C_d_wave_drag(nosecone_half_angle, Mach) + C_d_base_drag(Mach, engine_burning);
    double a_A = axial_drag_force(dyn_pressure, A_ref, C_A) / mass;

    // calculate the normal and axial acceleration vectors in ECI
    double v_axial = rel_airspeed.dot(nose_direction);
    Vec3 axial_a = nose_direction * (v_axial >= 0.0 ? -a_A : a_A);
    Vec3 v_perp = rel_airspeed - nose_direction * v_axial;
    double v_perp_mag = v_perp.mag();
    Vec3 norm_a = v_perp_mag > 1e-9 ? v_perp * (-a_N / v_perp_mag) : Vec3{0, 0, 0};

    // center of pressure measured from the nose tip
    double x_cp = C_N > 1e-12 ? X_CP(props.nosecone_length, body_len, C_N_cone, C_N_body) : (2.0 / 3.0) * props.nosecone_length;
    z_cp = total_len - x_cp;

    Vec3 drag_a = norm_a + axial_a;

    drag_accel = drag_a;

    return drag_a;
}