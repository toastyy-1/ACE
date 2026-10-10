// Copyright 2026 Tyler Wiggins
// SPDX-License-Identifier: Apache-2.0

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
 * @param dA change in cross sectional area along the body (m^2)
 * @return normal force coefficient
 */
static double C_N_potential(double AoA, double A_ref, double dA) {
    return (2.0 * sin(AoA) / A_ref) * dA;
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
 * @brief sphere drag
 * @param M Mach number
 * @return drag coefficient of sphere
 */
static double C_d_sphere(double M) {
    static const double mach[] = {0.0, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 5.0};
    static const double C_d[]  = {0.47, 0.50, 0.60, 0.80, 0.95, 1.00, 0.98, 0.94, 0.92};
    const int n = sizeof(mach) / sizeof(mach[0]);

    if (M >= mach[n - 1]) {
        return C_d[n - 1];
    }
    int i = 1;
    while (M > mach[i]) {
        i++;
    }
    double f = (M - mach[i - 1]) / (mach[i] - mach[i - 1]);
    return C_d[i - 1] + f * (C_d[i] - C_d[i - 1]);
}

///////////////////////////////////////////////////////////////////////////////////////////////
// body geometry                                                                             //
///////////////////////////////////////////////////////////////////////////////////////////////
// running totals for the body from the nose tip back, moments are taken about the nose tip
struct AeroBody {
    double x = 0;       // length so far (m)
    double x_joint = 0; // where a change in radius at the next joint acts (m)
    double r_aft = 0;   // radius at the aft end so far (m)
    double R_max = 0;
    double A_lift = 0, A_lift_x = 0; // growth in cross section and its moment
    double A_plan = 0, A_plan_x = 0; // planform and its moment
    double A_wet = 0;
    double front_half_angle = 0;
    int n_seg = 0;
    Geometry front;
};

/**
 * @brief adds the next segment aft of everything alr in b
 * @param b body accumulated
 * @param g segment shape
 * @param L segment length (m)
 */
static void add_segment(AeroBody& b, const Geometry& g, double L) {
    double R = g.radius, A = M_PI * R * R;
    bool sphere = g.shape == Shape::Sphere;
    double r_front = g.shape == Shape::Cylinder ? R : 0.0; // cone tips and sphere poles are points

    // make cones pointy
    if (b.n_seg++ == 0) {
        b.front = g;
        b.front_half_angle = g.shape == Shape::Cone ? atan(R / L) : M_PI / 2;
    }

    // potential lift
    double x_joint = sphere ? b.x + 0.5 * L : b.x_joint;
    double dA_joint = M_PI * (r_front * r_front - b.r_aft * b.r_aft);
    double dA_cone = g.shape == Shape::Cone ? A : 0.0;
    b.A_lift += dA_joint + dA_cone;
    b.A_lift_x += dA_joint * x_joint + dA_cone * (b.x + (2.0 / 3.0) * L);

    // viscous crossflow lift and skin friction
    double plan = 0, x_plan = b.x + 0.5 * L;
    switch (g.shape) {
        case Shape::Cylinder:
            plan = 2.0 * R * L;
            b.A_wet += 2.0 * M_PI * R * L;
            break;
        case Shape::Cone:
            plan = R * L;
            x_plan = b.x + (2.0 / 3.0) * L;
            b.A_wet += M_PI * R * sqrt(R * R + L * L);
            break;
        case Shape::Sphere:
            plan = A;
            b.A_wet += 4.0 * A;
            break;
    }
    b.A_plan += plan;
    b.A_plan_x += plan * x_plan;

    b.R_max = std::max(b.R_max, R);
    b.r_aft = sphere ? 0.0 : R;
    b.x_joint = sphere ? x_joint : b.x + L;
    b.x += L;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// pitch/yaw damping                                                                         //
///////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief viscous crossflow damping
 * @param w_rel angular velocity relative to the air in body frame (rad/s)
 * @param rho air density
 * @param diameter body diameter (m)
 * @param len_aft distance from the CoM back to the aft end (m)
 * @param len_fwd distance from the CoM forward to the nose tip (m)
 * @return moment opposing the pitch/yaw rate in body frame (N-m)
 */
static Vec3 damping_moment(const Vec3& w_rel, double rho, double diameter, double len_aft, double len_fwd) {
    const double C_D_crossflow = 1.1;
    double w_perp = std::sqrt(w_rel.x * w_rel.x + w_rel.y * w_rel.y);
    if (w_perp < 1e-12) return {0, 0, 0};

    double arm4 = (std::pow(len_aft, 4) + std::pow(len_fwd, 4)) / 4.0;
    double M = 0.5 * rho * C_D_crossflow * diameter * w_perp * w_perp * arm4;
    return {-M * w_rel.x / w_perp, -M * w_rel.y / w_perp, 0};
}

/**
 * acceleration and torque due to drag
 * @param r position in ECI (m)
 * @param v velocity in ECI (m/s)
 * @param q body orientation, rotates the body frame into ECI
 * @param w angular velocity, body frame (rad/s)
 * @param mass current total mass of the rocket (kg)
 * @param props rocket geometry, the nosecone and each remaining stage's shape are used here
 * @return drag acceleration in ECI (m/s^2), also stored in drag_accel, and the aero moment about the CoM
 */
KinematicModifier Rocket::calc_drag_kinematics(const Vec3& r, const Vec3& v, const Quat& q, const Vec3& w, double mass, const RocketProps& props) {

    // calculate the properties of the air
    double air_density, air_pressure, speed_of_sound, mu;
    atmosphere(r.mag() - planet::EARTH.radius, air_density, air_pressure, speed_of_sound, mu);

    // move from tip to rear of the active stage
    AeroBody body;
    const Geometry nose = {Shape::Cone, props.stages.back().geometry.radius};
    if (props.nosecone_length > 0) add_segment(body, nose, props.nosecone_length);
    for (int i = num_stages() - 1; i >= active_idx; i--) add_segment(body, props.stages[i].geometry, props.stages[i].tip_to_end_length);

    double diameter = 2.0 * body.R_max;
    double A_ref = M_PI * body.R_max * body.R_max;
    double A_front = M_PI * body.front.radius * body.front.radius;
    double A_base = M_PI * body.r_aft * body.r_aft;
    double total_len = body.x;

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
        return {{0, 0, 0}, {0, 0, 0}};
    }

    // calcualte the aoa entering into the atmosphere
    Vec3 nose_direction = nose_direction_eci(q);
    double AoA = std::acos(std::max(-1.0, std::min(1.0, rel_airspeed.dot(nose_direction) / speed)));

    // calculate dynamic presssure
    dyn_pressure = dynamic_pressure(air_density, speed);
    aoa = AoA;

    // apply drag to sphere case
    if (body.n_seg == 1 && body.front.shape == Shape::Sphere) {
        double C_d = body.front.cd >= 0 ? body.front.cd : C_d_sphere(Mach);
        drag_accel = rel_airspeed * (-axial_drag_force(dyn_pressure, A_ref, C_d) / (mass * speed));
        z_cp = 0.5 * total_len;
        Vec3 r_cp = {0, 0, z_cp - z_cm};
        return {drag_accel, r_cp.cross(rotate_by_quat(q.conjugate(), drag_accel * mass))};
    }

    // calculate normal drag acceleration magnitude
    double C_N_pot = C_N_potential(AoA, A_ref, body.A_lift);
    double C_N_body = C_N_lift_subsonic(AoA, body.A_plan, A_ref);
    double C_N = C_N_pot + C_N_body;
    double a_N = normal_drag_force(dyn_pressure, A_ref, C_N) / mass;

    // calculat axial drag acceleration magnitude
    bool engine_burning = throttle > 0.0 && active_stage().m_fuel > 0.0;
    double C_A = C_d_friction(Mach, Re, body.A_wet, A_ref, total_len, diameter)
               + C_d_wave_drag(body.front_half_angle, Mach) * (A_front / A_ref)
               + C_d_base_drag(Mach, engine_burning) * (A_base / A_ref);
    double a_A = axial_drag_force(dyn_pressure, A_ref, C_A) / mass;

    // calculate the normal and axial acceleration vectors in ECI
    double v_axial = rel_airspeed.dot(nose_direction);
    Vec3 axial_a = nose_direction * (v_axial >= 0.0 ? -a_A : a_A);
    Vec3 v_perp = rel_airspeed - nose_direction * v_axial;
    double v_perp_mag = v_perp.mag();
    Vec3 norm_a = v_perp_mag > 1e-9 ? v_perp * (-a_N / v_perp_mag) : Vec3{0, 0, 0};

    // center of pressure measured from the nose tip
    double x_cp = 0.5 * total_len;
    if (std::abs(C_N) > 1e-12) {
        x_cp = (C_N_potential(AoA, A_ref, body.A_lift_x) + C_N_lift_subsonic(AoA, body.A_plan_x, A_ref)) / C_N;
    } else if (std::abs(body.A_lift) > 1e-9) {
        x_cp = body.A_lift_x / body.A_lift;
    }
    z_cp = total_len - x_cp;

    Vec3 drag_a = norm_a + axial_a;

    drag_accel = drag_a;

    // drag from cp torques body about cm
    Vec3 drag_force_body = rotate_by_quat(q.conjugate(), drag_a * mass);
    Vec3 r_cp = {0, 0, z_cp - z_cm};
    Vec3 aero_torque = r_cp.cross(drag_force_body);

    // affect pitch/yaw rate relative to the rotating atmosphere
    Vec3 w_earth_body = rotate_by_quat(q.conjugate(), Vec3{0, 0, planet::EARTH.rotation_rate});
    aero_torque += damping_moment(w - w_earth_body, air_density, diameter, z_cm, total_len - z_cm);

    return {drag_a, aero_torque};
}