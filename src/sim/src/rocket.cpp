#include "sim/inc/rocket.hpp"
#include "constants.hpp"
#include "fc/inc/fc_api.h"
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <iostream>

Rocket::Rocket(const std::string& rocket_name, double origin_latitude, double origin_longitude, double target_latitude,
               double target_longitude, const RocketProps& rocket_props, bool track_data, double export_interval) {
    set_start(origin_latitude, origin_longitude, target_latitude, target_longitude);
    props = rocket_props;
    name = rocket_name;

    if (track_data) {
        std::filesystem::create_directories("data");
        data_export = std::make_unique<DataExport>("data/" + name + ".csv", export_interval);
    }
}

Rocket::~Rocket() {
    // default destructor
}


/**
 * gets the current state of the rocket, mainly used for graphics
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
    s.radius      = props.radius;
    s.init        = start_state;
    s.detonation_active = detonated;
    return s;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MATH HELPERS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * updates the fuel mass based on the current rocket states
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
    m_current = M;
    m_fuel_current = M_f;
    z_cm = m_cm / M;

    // also adjust moment using assumption that the structure and the propellant column are each uniform cylinders
    double R2 = props.radius * props.radius, I_trans = 0;
    base = 0;
    for (int i = active_idx; i < num_stages(); i++) {
        const Stage& st = props.stages[i];
        double L = st.tip_to_end_length, L_f = st.fuel_length * st.fuel_fill();
        double d_dry = (base + L - st.dry_CoM()) - z_cm;
        double d_fuel = (base + L - st.fuel_CoM()) - z_cm;
        I_trans += (1.0 / 12.0) * st.m_dry * (3.0 * R2 + L * L) + st.m_dry * d_dry * d_dry;
        I_trans += (1.0 / 12.0) * st.m_fuel * (3.0 * R2 + L_f * L_f) + st.m_fuel * d_fuel * d_fuel;
        base += L;
    }
    I_body = { I_trans, I_trans, 0.5 * R2 * M };
}

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
 * rotate a vector by a quaternion
*/
static Vec3 rotate_by_quat(const Quat& q, const Vec3& u) {
    Vec3 q_vec = {q.x, q.y, q.z};
    Vec3 t = q_vec.cross(u);
    return u + t * (2.0 * q.w) + q_vec.cross(t) * 2.0;
}
/**
 * nose direction rotated into the ECI frame from an attitude
 */
static Vec3 nose_from_quat(const Quat& q) {
    return {
        2.0 * (q.x * q.z + q.w * q.y),
        2.0 * (q.y * q.z - q.w * q.x),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    };
}

/**
 * @return the rocket's nose direction in ECI frame coordinates
 */
Vec3 Rocket::nose_direction_eci() {
    return nose_from_quat(q_rocket);
}

/**
 * gimbaled thrust vector in the body frame
 * @param thrust_scale isp change as pressure changes
 */
Vec3 Rocket::engine_thrust_body(double thrust_scale) const {
    Vec3 nose_body = {0, 0, 1};
    return rotate_by_quat(q_engine, nose_body) * (active_stage().thrust * thrust_scale);
}

/**
 * net torque about the combined CopM in body frame
 * @param thrust_scale isp change as pressure changes
 */
Vec3 Rocket::net_body_torque(double thrust_scale) const {
    Vec3 net_torque = {0, 0, 0};

    // lever arm from the combined CoM to the engine along the body axis
    double s_engine = active_stage().tip_to_end_length - active_stage().engine_distance;
    Vec3 r_engine = {0, 0, s_engine - z_cm};

    net_torque += r_engine.cross(engine_thrust_body(thrust_scale));
    if (rcs_active) net_torque += applied_rcs_moment;

    return net_torque;
}

/**
 * gravitational acceleration in the ECI frame
 */
static Vec3 calc_gravity_accel(const Vec3& r) {
    double r2       = r.dot(r);
    double r_norm   = std::sqrt(r2);
    double pm       = -GM_EARTH / (r2 * r_norm);

    double zr2      = (r.z * r.z) / r2;
    double k        = 1.5 * J2 * (EARTH_RADIUS * EARTH_RADIUS) / r2;

    return {
        .x = pm * r.x * (1.0 - k * (5.0 * zr2 - 1.0)),
        .y = pm * r.y * (1.0 - k * (5.0 * zr2 - 1.0)),
        .z = pm * r.z * (1.0 - k * (5.0 * zr2 - 3.0)),
    };
}

/**
 * power relationship density equation, sets T to the layer temperature
 */
static double pow_dens(double altitude, double& T, double rho_b, double T_b, double L, double layer_base_alt) {
    T = T_b + L * (altitude - layer_base_alt);
    return rho_b * pow(( T / T_b ), (-1.0 * g0 / (R_d * L)) - 1);
}

/**
 * exponential relationship density equation, sets T to the layer temperature
 */
static double exp_dens(double altitude, double& T, double rho_b, double T_b, double layer_base_alt) {
    T = T_b;
    return rho_b * exp(-1.0 * (g0 * (altitude - layer_base_alt)) / (R_d * T_b));
}

/**
 * standard atmosphere layers
 */
void atmosphere(double altitude, double& air_density, double& air_pressure) {
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
}

/**
 * angular acceleration in the body frame
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
 */
static Quat quat_deriv(const Quat& q, const Vec3& w) {
    Quat omega = {0.0, w.x, w.y, w.z};
    return (q * omega) * 0.5;
}

/**
 * length of the remaining stack from the nose to the aft end of the active stage
 */
double Rocket::rocket_length() const {
    double length = 0;
    for (int i = active_idx; i < num_stages(); i++) {
        length += props.stages[i].tip_to_end_length;
    }
    return length;
}

/**
 * checks if the rocket is on the ground
 */
bool Rocket::is_rocket_on_ground(double com_dist_from_gnd) {
    double r_norm = r.mag();

    // find rocket length
    double rocket_length = this->rocket_length();

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

                + props.radius * sin_angle_to_gnd;
        
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
 */
static Vec3 contact_vel_change(const Vec3& r_c, const Vec3& J, const Vec3& I, double m) {
    Vec3 ang = r_c.cross(J);
    Vec3 dw = {ang.x / I.x, ang.y / I.y, ang.z / I.z};
    return J / m + dw.cross(r_c);
}

/**
 * specific ground dynamics are applied when the rocket is on the ground
 */
void Rocket::apply_ground_dynamics(const Vec3& I, double m_end, double dt) {
    // the ground supplies whatever force keeps the rocket riding along with the surface
    Vec3 w_earth = {0, 0, EARTH_ROTATION_RATE};
    a = w_earth.cross(w_earth.cross(r));

    // find vector components in the body frame
    Quat q_inv = q_rocket.conjugate();
    Vec3 up_body = rotate_by_quat(q_inv, r.unit());
    double cos_angle_to_gnd = up_body.z; // body +z is the nose

    // contact point is the lowest end of the rocket
    Vec3 r_contact_from_cm = {0, 0, cos_angle_to_gnd >= 0.0 ? -z_cm : rocket_length() - z_cm};
    double sin_angle_to_gnd = std::sqrt(std::max(0.0, 1.0 - cos_angle_to_gnd * cos_angle_to_gnd));
    if (sin_angle_to_gnd > 0) {
        r_contact_from_cm.x = -up_body.x * props.radius / sin_angle_to_gnd;
        r_contact_from_cm.y = -up_body.y * props.radius / sin_angle_to_gnd;
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
                                       GROUND_FRICTION_COEFF * impulse_body.mag());
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
 * translational acceleration in the ECI frame
 */
Vec3 Rocket::translational_accel(double m_i, const Vec3& r_i, const Vec3& v_i, const Quat& q_i, const Vec3& thrust_body) {
    return calc_gravity_accel(r_i) + calc_drag_accel(r_i, v_i, m_i) + rotate_by_quat(q_i, thrust_body) / m_i;
}

/**
 * the most important function for the dynamics calculations. 
 * calculates all dynamics of the rocket using RK4
 */
void Rocket::update_dynamics(double current_time) {
    // rocket mass
    double m = m_current;
    Vec3 I = I_body;

    // time step for the simulation
    double dt = TIME_STEP;

    // propellant drain
    Stage& s = active_stage();
    double mdot = s.mass_flow_rate();
    double burn_frac = 1.0; // fraction of the step the remaining propellant lasts
    if (mdot * dt > s.m_fuel) {
        burn_frac = s.m_fuel / (mdot * dt);
        mdot = s.m_fuel / dt;
    }

    // adjust thrust for isp change
    double air_density, air_pressure;
    atmosphere(r.mag() - EARTH_RADIUS, air_density, air_pressure);
    double thrust_scale;
    if (s.isp > 0) {
        thrust_scale = s.isp_at(air_pressure) / s.isp;
    } else {
        thrust_scale = 1.0;
    }
    thrust_scale *= burn_frac;

    // quantities the FC commands
    Vec3 thrust_body = engine_thrust_body(thrust_scale);
    Vec3 net_torque = net_body_torque(thrust_scale);

    // mass at the start, middle, and end of the step
    double m_mid = m - mdot * (dt / 2);
    double m_end = m - mdot * dt;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // RK4 integration                                                                           //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////
    // k1 terms                       //
    ////////////////////////////////////
    Vec3 k1_r = v;
    Vec3 k1_v = translational_accel(m, r, v, q_rocket, thrust_body);
    Vec3 k1_w = ang_accel(w, I, net_torque);
    Quat k1_q = quat_deriv(q_rocket, w);

    ////////////////////////////////////
    // k2 terms                       //
    ////////////////////////////////////
    Vec3 r2 = r + k1_r * (dt / 2);
    Vec3 v2 = v + k1_v * (dt / 2);
    Vec3 w2 = w + k1_w * (dt / 2);
    Quat q2 = q_rocket + k1_q * (dt / 2);
    Vec3 k2_r = v2;
    Vec3 k2_v = translational_accel(m_mid, r2, v2, q2, thrust_body);
    Vec3 k2_w = ang_accel(w2, I, net_torque);
    Quat k2_q = quat_deriv(q2, w2);

    ////////////////////////////////////
    // k3 terms                       //
    ////////////////////////////////////
    Vec3 r3 = r + k2_r * (dt / 2);
    Vec3 v3 = v + k2_v * (dt / 2);
    Vec3 w3 = w + k2_w * (dt / 2);
    Quat q3 = q_rocket + k2_q * (dt / 2);
    Vec3 k3_r = v3;
    Vec3 k3_v = translational_accel(m_mid, r3, v3, q3, thrust_body);
    Vec3 k3_w = ang_accel(w3, I, net_torque);
    Quat k3_q = quat_deriv(q3, w3);

    ////////////////////////////////////
    // k4 terms                       //
    ////////////////////////////////////
    Vec3 r4 = r + k3_r * dt;
    Vec3 v4 = v + k3_v * dt;
    Vec3 w4 = w + k3_w * dt;
    Quat q4 = q_rocket + k3_q * dt;
    Vec3 k4_r = v4;
    Vec3 k4_v = translational_accel(m_end, r4, v4, q4, thrust_body);
    Vec3 k4_w = ang_accel(w4, I, net_torque);
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
    s.m_fuel -= mdot * dt;
    if (s.m_fuel <= 0) { s.m_fuel = 0; s.thrust = 0; }

    // final time at new position
    double t_end = current_time + dt;

    // find altitude above the earth
    double surface_r = topo->SurfaceRadius3D(eci_to_ecef(r, t_end));
    altitude = r.mag() - surface_r;

    // determine if rocket is on the ground, then calculate its altitude based on that
    bool on_ground = is_rocket_on_ground(altitude);
    altitude = r.mag() - surface_r; // the ground contact may have moved the rocket

    // gravity measurement at new position
    Vec3 g_end = calc_gravity_accel(r);

    // update acceleration of the body as consistent with RK4
    if (on_ground) {
        // apply dynamics to body but special because its touching the ground
        apply_ground_dynamics(I, m_end, dt);
    }
    else {
        // calculate RK4 total acceleration vector
        a = g_end + rotate_by_quat(q_rocket, thrust_body) / m_end + calc_drag_accel(r, v, m_end);
    }

    // accelerometer measures everything except gravity, in the body frame
    a_spec = rotate_by_quat(q_rocket.conjugate(), a - g_end);

    // log the end-of-step state if this rocket is tracking data
    if (data_export) data_export->write_row(t_end, r, v, a, q_rocket, w, m_end, m_fuel_current - mdot * dt, thrust_body.mag());
}

/**
 * sets the starting position of the rocket on the earth with coordinates
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
    Vec3 w_earth = {0, 0, EARTH_ROTATION_RATE};
    w = rotate_by_quat(q.conjugate(), w_earth);
    a = w_earth.cross(w_earth.cross(origin_pos));
    a_spec = rotate_by_quat(q.conjugate(), a - calc_gravity_accel(origin_pos));
}
