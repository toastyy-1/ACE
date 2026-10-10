// Copyright 2026 Tyler Wiggins
// SPDX-License-Identifier: Apache-2.0

#include "sim/inc/rocket.hpp"
#include "sim_constants.hpp"
#include "planetary_constants.hpp"
#include "fc/inc/fc_api.h"
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <iostream>

/**
 * @brief
 * @param rocket_name name identifier of the rocket
 * @param origin_latitude latitude of which the rocket starts
 * @param origin_longitude longitude of which the rocket starts
 * @param target_latitude lat of which the rocket is planned to land
 * @param target_longitude long of which the rocket is planned to land
 * @param rocket_props property struct defining geometry/characteristics of the rocket
 * @param track_data true if data is to be logged to a csv
 * @param export_interval how often that data should be logged
 * @param start_in_orbit true if the rocket starts on the orbit instead of at the origin coordinates
 * @param orbit orbital elements the rocket would start on if the above is true
 * @param start_in_eci true if the rocket starts at eci_position/eci_velocity
 * @param eci_position starting position in ECI (m)
 * @param eci_velocity starting velocity in ECI (m s^-1)
 * @param moon_gravity include the moon's gravity
 * @param sun_gravity include the sun's gravity
 * @param drag include aerodynamic forces
 * @param ephem sun and moon positions, must be loaded if either of the above is true
 */
Rocket::Rocket(const std::string& rocket_name, double origin_latitude, double origin_longitude, double target_latitude,
               double target_longitude, const RocketProps& rocket_props, bool track_data, double export_interval,
               bool start_in_orbit, const OrbitElements& orbit, bool start_in_eci, const Vec3& eci_position, const Vec3& eci_velocity,
               bool moon_gravity, bool sun_gravity, bool drag, const Ephemeris* ephem)
    : moon_gravity(moon_gravity), sun_gravity(sun_gravity), ephem(ephem), drag(drag) {
    if (start_in_orbit)    set_start_orbit(orbit, target_latitude, target_longitude);
    else if (start_in_eci) set_start_eci(eci_position, eci_velocity, target_latitude, target_longitude);
    else                set_start(origin_latitude, origin_longitude, target_latitude, target_longitude);
    props = rocket_props;
    name = rocket_name;

    if (track_data) {
        std::filesystem::create_directories("data");
        data_export = std::make_unique<DataExport>("data/" + name + ".csv", export_interval);
    }
}

/**
 * @brief constructor
 */
Rocket::~Rocket() {
    // default destructor
}


/**
 * gets the current state of the rocket, mainly used for graphics
 * @return state struct of the rocket
 */
RocketState Rocket::get_state() const {
    double length = 0;
    for (int i = active_idx; i < num_stages(); i++) length += props.stages[i].tip_to_end_length;
    double s_engine = active_stage().tip_to_end_length - active_stage().engine_distance;

    RocketState s;
    s.r           = r;
    s.v           = v;
    s.a           = a;
    s.w           = w;
    s.q_rocket    = q_rocket;
    s.q_engine    = q_engine;
    s.mass        = m_current;
    s.fuel        = m_fuel_current;
    s.length      = length;
    s.cm_dist     = length - z_cm;
    s.engine_dist = length - s_engine;
    s.radius      = props.max_radius(active_idx);
    s.nose_length = props.nosecone_length;
    s.has_engine  = active_stage().m_fuel_full > 0;
    s.init        = start_state;
    s.detonation_active = detonated;
    return s;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MATH HELPERS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief
 * @param latitude_deg latitude input in degrees
 * @param longitude_deg longitude input in degrees
 * @return ecef coordinate vector corresponding to the input coordinates
 */
Vec3 Rocket::lat_lon_to_ecef(double latitude_deg, double longitude_deg) {
    double lat = latitude_deg * M_PI / 180.0;
    double lon = longitude_deg * M_PI / 180.0;
    double alt = topo->SurfaceRadius2D(latitude_deg, longitude_deg);
    return {
        .x = alt * cos(lat) * cos(lon),
        .y = alt * cos(lat) * sin(lon),
        .z = alt * sin(lat)
    };
}

/**
 * nose direction rotated into the ECI frame from an attitude
 * @param q nose direction quaternion
 * @return nose direction vector in ECI
 */
static Vec3 nose_from_quat(const Quat& q) {
    return {
        2.0 * (q.x * q.z + q.w * q.y),
        2.0 * (q.y * q.z - q.w * q.x),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    };
}

/**
 * @return the rocket's nose direction in ECI frame coordinates for an attitude q
 * @param q nose direction quaternion
 */
Vec3 Rocket::nose_direction_eci(const Quat& q) const {
    return nose_from_quat(q);
}

/**
 * angular acceleration in the body frame
 * @param w_i angular velocity of the body in the body frame
 * @param I moments of inertia about the body axes
 * @param net_torque net torque on the body in the body frame
 * @return angular acceleration in the body frame
 */
static Vec3 ang_accel(const Vec3& w_i, const Vec3& I, const Vec3& net_torque) {
    Vec3 Iw = {I.x * w_i.x, I.y * w_i.y, I.z * w_i.z};
    Vec3 gyro = w_i.cross(Iw);
    return Vec3{
        (net_torque.x - gyro.x) / I.x,
        (net_torque.y - gyro.y) / I.y,
        (net_torque.z - gyro.z) / I.z,
    };
}

/**
 * calculates time derivative of input quaternion given current orientation
 * @param q 
 * @param w
 * @return
 */
static Quat quat_deriv(const Quat& q, const Vec3& w) {
    Quat omega = {0.0, w.x, w.y, w.z};
    return (q * omega) * 0.5;
}

/**
 * length of the remaining stack from the front tank to the aft end of the active stage
 * @return the body lenght of the rocket (m) not including the nosecone
 */
double Rocket::rocket_body_length() const {
    double length = 0;
    for (int i = active_idx; i < num_stages(); i++) {
        length += props.stages[i].tip_to_end_length;
    }
    return length;
}

/**
 * checks if the rocket is on the ground
 * @param com_dist_from_gnd the rocket's center of mass's distance from the ground
 * @return if the rocket is touching the ground
 */
bool Rocket::is_rocket_on_ground(double com_dist_from_gnd) {
    double r_norm = r.mag();

    // find rocket length, nose tip to aft end
    double rocket_length = this->rocket_body_length() + props.nosecone_length;

    // if its greater than the length of  the rocket, its not worth checking at all lol
    if (com_dist_from_gnd > rocket_length) {
        return false;
    }
    else {
        // find how high the CoM is from the ground based on the angle between the rockets position in ecef and its orientation
        double cos_angle_to_gnd = r.unit().dot(nose_from_quat(q_rocket));

        // whichever end sits lower touches the ground, and the body radius lifts the CoM when it isnt vertical
        double sin_angle_to_gnd = std::sqrt(std::max(0.0, 1.0 - cos_angle_to_gnd * cos_angle_to_gnd));
        double rocket_height_component = std::max
                (
                z_cm * cos_angle_to_gnd, // OR
                -(rocket_length - z_cm) * cos_angle_to_gnd
                )

                + props.max_radius(active_idx) * sin_angle_to_gnd;
        
        // check if any component along the rocket is touching the ground, 
        if (com_dist_from_gnd < rocket_height_component) {
            Vec3 r_hat = r.unit();
            r = r_hat * (r_norm - com_dist_from_gnd + rocket_height_component);
            return true;
        }
        else {
            return false;
        }
    }
}

/**
 * change in contact point velocity from an impulse J applied there (body frame)
 * @param r_c position of the contact point relative to the center of mass, in the body frame
 * @param J impulse applied at the contact point, in the body frame
 * @param I principal moments of inertia about the body axes
 * @param m mass of the rocket
 * @return change in velocity of the contact point in the body frame
 */
static Vec3 contact_vel_change(const Vec3& r_c, const Vec3& J, const Vec3& I, double m) {
    Vec3 ang = r_c.cross(J);
    Vec3 dw = {ang.x / I.x, ang.y / I.y, ang.z / I.z};
    return J / m + dw.cross(r_c);
}

/**
 * specific ground dynamics are applied when the rocket is on the ground
 * @param I moment of inertia of the vehicle at the current state
 * @param m_end the current mass at the final step of rk4
 * @param dt time step
 */
void Rocket::apply_ground_dynamics(const Vec3& I, double m_end, double dt) {
    // the ground supplies whatever force keeps the rocket riding along with the surface
    Vec3 w_earth = {0, 0, planet::EARTH.rotation_rate};
    a = w_earth.cross(w_earth.cross(r));

    // find vector components in the body frame
    Quat q_inv = q_rocket.conjugate();
    Vec3 up_body = rotate_by_quat(q_inv, r.unit());
    double cos_angle_to_gnd = up_body.z; // body +z is the nose

    // contact point is the lowest end of the rocket
    Vec3 r_contact_from_cm = {0, 0, cos_angle_to_gnd >= 0.0 ? -z_cm : rocket_body_length() + props.nosecone_length - z_cm};
    double sin_angle_to_gnd = std::sqrt(std::max(0.0, 1.0 - cos_angle_to_gnd * cos_angle_to_gnd));
    if (sin_angle_to_gnd > 0) {
        double R = props.max_radius(active_idx);
        r_contact_from_cm.x = -up_body.x * R / sin_angle_to_gnd;
        r_contact_from_cm.y = -up_body.y * R / sin_angle_to_gnd;
    }

    // velocity of the contact point relative to the ground
    Vec3 w_earth_body = rotate_by_quat(q_inv, w_earth);
    Vec3 v_contact = rotate_by_quat(q_inv, v - surface_velocity_eci(r)) + (w - w_earth_body).cross(r_contact_from_cm);

    // ground pushes when the contact point moves into it
    double v_into_gnd = v_contact.dot(up_body);
    if (v_into_gnd < 0) {
        Vec3 impulse_body = up_body * (-v_into_gnd / contact_vel_change(r_contact_from_cm, up_body, I, m_end).dot(up_body));

        // friction stops the contact point sliding
        Vec3 v_slip = v_contact + contact_vel_change(r_contact_from_cm, impulse_body, I, m_end);
        v_slip -= up_body * v_slip.dot(up_body);
        if (v_slip.mag() > 1e-9) {
            Vec3 slip_dir = v_slip.unit();
            double friction = std::min(v_slip.mag() / contact_vel_change(r_contact_from_cm, slip_dir, I, m_end).dot(slip_dir),
                                       consts::GROUND_FRICTION_COEFF * impulse_body.mag());
            impulse_body -= slip_dir * friction;
        }

        // the impulse at the contact point changes both the CoM velocity and the spin
        v += rotate_by_quat(q_rocket, impulse_body / m_end);
        Vec3 ang_impulse = r_contact_from_cm.cross(impulse_body);
        w.x += ang_impulse.x / I.x;
        w.y += ang_impulse.y / I.y;
        w.z += ang_impulse.z / I.z;
    }

    // take away rotational energy modified by some energy loss factor that I lwk made up
    w = w_earth_body + (w - w_earth_body) * exp(-dt / 0.9);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MAIN DYNAMICS LOGIC
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * returns the current kinematic state
 * @param m_i mass
 * @param r_i position in ECI
 * @param v_i velocity in ECI
 * @param q_i body orientation
 * @param w_i angular velocity, body frame
 * @param props rocket geometry
 * @param t_burn time since the active stage ignition
 * @param t sim time
 * @return acceleration in ECI and torque about the CoM in body frame
 */
KinematicModifier Rocket::kinematic_state(double m_i, const Vec3& r_i, const Vec3& v_i, const Quat& q_i, const Vec3& w_i, const RocketProps& props, double t_burn, double t) {
    KinematicModifier drag = calc_drag_kinematics(r_i, v_i, q_i, w_i, m_i, props);
    KinematicModifier prop = calc_propulsion_kinematics(r_i, v_i, q_i, w_i, m_i, props, t_burn);
    Vec3 grav = calc_gravity_accel(r_i, planet::EARTH.gm, planet::EARTH.j2, planet::EARTH.radius, t);

    KinematicModifier state;
    state.accel = drag.accel + prop.accel + grav;
    state.torque = drag.torque + prop.torque + calc_rcs_torque();
    return state;
}

/**
 * the most important function for the dynamics calculations. 
 * calculates all dynamics of the rocket using RK4
 * @param current_time simulation time
 */
void Rocket::update_dynamics(double current_time) {
    // rocket mass
    double m = m_current;
    Vec3 I = I_body;

    // time step for the simulation
    double dt = consts::TIME_STEP;

    Stage& s = active_stage();

    // propellant burned over the first half and all of the step
    double t_burn = burn_time;
    double fuel_mid = fuel_burned(t_burn, t_burn + dt / 2);
    double fuel_end = fuel_burned(t_burn, t_burn + dt);

    // mass at the start, middle, and end of the step
    double m_mid = m - fuel_mid;
    double m_end = m - fuel_end;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // RK4 integration                                                                           //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////
    // k1 terms                       //
    ////////////////////////////////////
    KinematicModifier k1 = kinematic_state(m, r, v, q_rocket, w, props, t_burn, current_time);
    Vec3 k1_r = v;
    Vec3 k1_v = k1.accel;
    Vec3 k1_w = ang_accel(w, I, k1.torque);
    Quat k1_q = quat_deriv(q_rocket, w);

    ////////////////////////////////////
    // k2 terms                       //
    ////////////////////////////////////
    Vec3 r2 = r + k1_r * (dt / 2);
    Vec3 v2 = v + k1_v * (dt / 2);
    Vec3 w2 = w + k1_w * (dt / 2);
    Quat q2 = q_rocket + k1_q * (dt / 2);
    KinematicModifier k2 = kinematic_state(m_mid, r2, v2, q2, w2, props, t_burn + dt / 2, current_time + dt / 2);
    Vec3 k2_r = v2;
    Vec3 k2_v = k2.accel;
    Vec3 k2_w = ang_accel(w2, I, k2.torque);
    Quat k2_q = quat_deriv(q2, w2);

    ////////////////////////////////////
    // k3 terms                       //
    ////////////////////////////////////
    Vec3 r3 = r + k2_r * (dt / 2);
    Vec3 v3 = v + k2_v * (dt / 2);
    Vec3 w3 = w + k2_w * (dt / 2);
    Quat q3 = q_rocket + k2_q * (dt / 2);
    KinematicModifier k3 = kinematic_state(m_mid, r3, v3, q3, w3, props, t_burn + dt / 2, current_time + dt / 2);
    Vec3 k3_r = v3;
    Vec3 k3_v = k3.accel;
    Vec3 k3_w = ang_accel(w3, I, k3.torque);
    Quat k3_q = quat_deriv(q3, w3);

    ////////////////////////////////////
    // k4 terms                       //
    ////////////////////////////////////
    Vec3 r4 = r + k3_r * dt;
    Vec3 v4 = v + k3_v * dt;
    Vec3 w4 = w + k3_w * dt;
    Quat q4 = q_rocket + k3_q * dt;
    KinematicModifier k4 = kinematic_state(m_end, r4, v4, q4, w4, props, t_burn + dt, current_time + dt);
    Vec3 k4_r = v4;
    Vec3 k4_v = k4.accel;
    Vec3 k4_w = ang_accel(w4, I, k4.torque);
    Quat k4_q = quat_deriv(q4, w4);

    ////////////////////////////////////
    // Rk4 formula
    ////////////////////////////////////
    Vec3 delta_r = (k1_r + k2_r*2 + k3_r*2 + k4_r) * (dt / 6);
    Vec3 delta_v = (k1_v + k2_v*2 + k3_v*2 + k4_v) * (dt / 6);
    Vec3 delta_w = (k1_w + k2_w*2 + k3_w*2 + k4_w) * (dt / 6);
    Quat delta_q = (k1_q + k2_q*2 + k3_q*2 + k4_q) * (dt / 6);

    // apply changes to rocket
    r += delta_r;
    v += delta_v;
    w += delta_w;
    q_rocket += delta_q;

    // renormalize attitude quaternion
    double qnorm = q_rocket.norm();
    q_rocket.w /= qnorm;
    q_rocket.x /= qnorm;
    q_rocket.y /= qnorm;
    q_rocket.z /= qnorm;

    // burn off fuel
    if (throttle > 0) {
        s.m_fuel = std::max(0.0, s.m_fuel - fuel_end);
        burn_time += dt;
        if (burn_time >= s.thrust_curve.end_time() || s.m_fuel <= 0) {
            throttle = 0;
            engine_locked = true;
        }
    }

    // final time at new position
    double t_end = current_time + dt;

    // find altitude above the earth
    double surface_r = topo->SurfaceRadius3D(eci_to_ecef(r, t_end));
    altitude = r.mag() - surface_r;

    // determine if rocket is on the ground, then calculate its altitude based on that
    bool on_ground = is_rocket_on_ground(altitude);
    altitude = r.mag() - surface_r; // the ground contact may have moved the rocket

    // gravity, drag, and thrust at new position
    Vec3 g_end = calc_gravity_accel(r, planet::EARTH.gm, planet::EARTH.j2, planet::EARTH.radius, t_end);
    KinematicModifier drag_end = calc_drag_kinematics(r, v, q_rocket, w, m_end, props);
    KinematicModifier prop_end = calc_propulsion_kinematics(r, v, q_rocket, w, m_end, props, burn_time);

    // update acceleration of the body as consistent with RK4
    if (on_ground) {
        // apply dynamics to body but special because its touching the ground
        apply_ground_dynamics(I, m_end, dt);
    }
    else {
        // calculate RK4 total acceleration vector
        a = g_end + drag_end.accel + prop_end.accel;
    }

    // accelerometer measures everything except gravity, in the body frame
    a_spec = rotate_by_quat(q_rocket.conjugate(), a - g_end);

    // log the end-of-step state if this rocket is tracking data
    if (data_export) {
        ExportRow row;
        row.t            = t_end;
        row.r            = r;
        row.v            = v;
        row.a            = a;
        row.q            = q_rocket;
        row.w            = w;
        row.m            = m_end;
        row.m_fuel       = m_fuel_current - fuel_end;
        row.thrust       = thrust_accel.mag() * m_end;
        row.g            = grav_accel;
        row.drag         = drag_accel;
        row.thrust_a     = thrust_accel;
        row.a_spec       = a_spec;
        row.altitude     = altitude;
        row.mach         = mach;
        row.dyn_pressure = dyn_pressure;
        row.aoa          = aoa;
        row.z_cm         = z_cm;
        row.z_cp         = z_cp;
        row.stage        = active_idx;
        row.m_fuel_stage = s.m_fuel;
        data_export->write_row(row);
    }
}

/**
 * @brief updates the fuel mass based on the current rocket states
 */
void Rocket::update_mass() {
    // dry structure and propellant are tracked separately so the CoM migrates as the tanks drain
    double M = 0, M_f = 0, m_cm = 0, base = 0;
    for (int i = active_idx; i < num_stages(); i++) {
        const Stage& st = props.stages[i];
        M += st.m_dry + st.m_fuel;
        M_f += st.m_fuel;
        m_cm += st.m_dry * (base + st.tip_to_end_length - st.dry_CoM())
              + st.m_fuel * (base + st.tip_to_end_length - st.fuel_CoM());
        base += st.tip_to_end_length;
    }

    // the nosecone
    double m_nose = props.nosecone_mass;
    double z_nose = base + props.nosecone_length - props.nosecone_com_distance;
    M += m_nose;
    m_cm += m_nose * z_nose;

    m_current = M;
    m_fuel_current = M_f;
    z_cm = m_cm / M;

    // also adjust moment using assumption that the structure and the propellant column are each uniform solids
    double I_trans = 0, I_axial = 0;
    base = 0;
    for (int i = active_idx; i < num_stages(); i++) {
        const Stage& st = props.stages[i];
        const Geometry& g = st.geometry;
        double L = st.tip_to_end_length, L_f = st.fuel_length * st.fuel_fill();
        double d_dry = (base + L - st.dry_CoM()) - z_cm;
        double d_fuel = (base + L - st.fuel_CoM()) - z_cm;
        I_trans += st.m_dry * (g.transverse_inertia(L) + d_dry * d_dry);
        I_trans += st.m_fuel * (g.transverse_inertia(L_f) + d_fuel * d_fuel);
        I_axial += (st.m_dry + st.m_fuel) * g.axial_inertia();
        base += L;
    }

    // nosecone is treated as a thin conical shell
    double R2 = props.stages.back().geometry.radius * props.stages.back().geometry.radius;
    double h = props.nosecone_length, d_nose = z_nose - z_cm;
    I_trans += m_nose * (R2 / 4.0 + h * h / 18.0) + m_nose * d_nose * d_nose;
    I_axial += 0.5 * R2 * m_nose;

    I_body = { I_trans, I_trans, I_axial };
}

/**
 * sets the starting position of the rocket on the earth with coordinates
 * @param origin_latitude 
 * @param origin_longitude
 * @param target_latitude
 * @param target_longitude
 */
void Rocket::set_start(double origin_latitude, double origin_longitude, double target_latitude, double target_longitude) {
    Vec3 origin_pos = lat_lon_to_ecef(origin_latitude, origin_longitude);

    // set the position of the rocket to that asolute position
    start_state.origin_r_eci = origin_pos;
    set_pos(origin_pos);

    // pad rotates with the earth
    v = surface_velocity_eci(origin_pos);

    // set target position
    start_state.target_r_ecef = lat_lon_to_ecef(target_latitude, target_longitude);

    // determine the necessary orientation to achive normal "up" position from surface
    double lat = origin_latitude * M_PI / 180.0;
    double lon = origin_longitude * M_PI / 180.0;
    Vec3 unit_vec_from_center = {
        .x = cos(lat) * cos(lon),
        .y = cos(lat) * sin(lon),
        .z = sin(lat)
    };
    Vec3 up = {0, 0, 1}; // since +z is up
    Vec3 rotation_axis = up.cross(unit_vec_from_center);
    double axis_norm = rotation_axis.mag();
    Vec3 rot_axis_u = axis_norm > 1e-12 ? rotation_axis / axis_norm : Vec3{1, 0, 0}; 

    double half_theta = acos(sin(lat)) / 2;
    Quat q = {
        .w = cos(half_theta),
        .x = sin(half_theta) * rot_axis_u.x,
        .y = sin(half_theta) * rot_axis_u.y,
        .z = sin(half_theta) * rot_axis_u.z
    };
    
    // set the oritnetaion of the rocket to normal the surface
    set_orientation(q);
    start_state.origin_q_eci = q;

    // sitting on the pad
    Vec3 w_earth = {0, 0, planet::EARTH.rotation_rate};
    w = rotate_by_quat(q.conjugate(), w_earth);
    a = w_earth.cross(w_earth.cross(origin_pos));
    a_spec = rotate_by_quat(q.conjugate(), a - calc_gravity_accel(origin_pos, planet::EARTH.gm, planet::EARTH.j2, planet::EARTH.radius, 0.0));
}

/**
 * sets the starting position, velocity, and attitude of the rocket from orbital elements
 * @param orbit classical orbital elements the rocket starts on
 * @param target_latitude
 * @param target_longitude
 */
void Rocket::set_start_orbit(const OrbitElements& orbit, double target_latitude, double target_longitude) {
    double sma = orbit.semi_major_axis * consts::KM_TO_M;
    double e   = orbit.eccentricity;
    double nu  = orbit.true_anomaly * consts::DEG_TO_RAD;

    // in perifocal frame
    double p = sma * (1.0 - e * e);
    double r_pf = p / (1.0 + e * cos(nu));
    double v_pf = sqrt(planet::EARTH.gm / p);
    Vec3 r_perifocal = {r_pf * cos(nu), r_pf * sin(nu), 0};
    Vec3 v_perifocal = {-v_pf * sin(nu), v_pf * (e + cos(nu)), 0};

    // perifocal to ECI
    auto axis_quat = [](double angle_deg, const Vec3& axis) {
        double half = angle_deg * consts::DEG_TO_RAD / 2;
        return Quat{cos(half), sin(half) * axis.x, sin(half) * axis.y, sin(half) * axis.z};
    };
    Quat q_pf_to_eci = axis_quat(orbit.raan, {0, 0, 1}) * axis_quat(orbit.inclination, {1, 0, 0}) * axis_quat(orbit.arg_periapsis, {0, 0, 1});

    set_start_eci(rotate_by_quat(q_pf_to_eci, r_perifocal), rotate_by_quat(q_pf_to_eci, v_perifocal), target_latitude, target_longitude);
}

/**
 * sets the starting position, velocity, and attitude of the rocket from an ECI state
 * @param r_eci starting position in ECI (m)
 * @param v_eci starting inertial velocity in ECI (m s^-1)
 * @param target_latitude
 * @param target_longitude
 */
void Rocket::set_start_eci(const Vec3& r_eci, const Vec3& v_eci, double target_latitude, double target_longitude) {
    set_pos(r_eci);
    v = v_eci;
    start_state.origin_r_eci = r;

    // set target position
    start_state.target_r_ecef = lat_lon_to_ecef(target_latitude, target_longitude);

    // point the nose prograde
    Vec3 u = v.mag() > 1e-9 ? v.unit() : r.unit();
    Quat q = u.z > -1.0 + 1e-12 ? Quat{1.0 + u.z, -u.y, u.x, 0.0}.normalize() : Quat{0, 1, 0, 0};
    set_orientation(q);
    start_state.origin_q_eci = q;

    w = {0, 0, 0};
    a = calc_gravity_accel(r, planet::EARTH.gm, planet::EARTH.j2, planet::EARTH.radius, 0.0);
    a_spec = {0, 0, 0};
}
