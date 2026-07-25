#include "rocket.hpp"
#include "constants.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

Rocket::Rocket(double origin_latitude, double origin_longitude, double target_latitude, double target_longitude,
               const RocketProps& rocket_props) {
    set_start(origin_latitude, origin_longitude, target_latitude, target_longitude);
    props = rocket_props;
}

Rocket::~Rocket() {
    // default destructor
}

/**
 * advances the stage of the rocket to the next one
 */
RocketState Rocket::get_state() const {
    double length = 0;
    for (int i = active_idx; i < num_stages(); i++) length += props.stages[i].tip_to_end_length;
    double s_engine = active().tip_to_end_length - active().engine_distance;

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

bool Rocket::advance_stage() {
    if (active_idx + 1 < num_stages()) {
        active_idx++;
        engine_locked = false; // fresh stage
        return true;
    }
    return false;
}

/**
 * control lighting the engine on the current active stage
 */
void Rocket::light_engine() {
    if (engine_locked) return; // motor was cut off and cannot be relit on this stage
    if (active().m_fuel > 0) {
        active().thrust = active().max_thrust;
    }
}

/**
 * permanently terminates thrust on the active stage, kills motor real dead
 */
void Rocket::cutoff_engine() {
    active().thrust = 0;
    engine_locked = true;
}

/**
 * burns a fraction of a full steps worth of thrust this step, then cuts off next step
 * TREAT THIS LIKE A DEV FEATURE -- this type of thing isnt real irl so kind of ignore it
 * when analysing the program to learn about guidance shit. this is only to make the rocket
 * fc think that time is infinitely coarse instead of whatever TIME_STEP is (I hope ts makes sense)
 */
void Rocket::command_final_burn_fraction(double fraction) {
    if (engine_locked) return;
    fraction = std::clamp(fraction, 0.0, 1.0);
    active().thrust = fraction * active().max_thrust;
    engine_locked = true;  // no relight
    pending_cutoff = true; // thrust zeroed at the start of the next step
}

/**
 * tells the RCS system that it should apply a moment to the center of mass of the rocket body according to the input
 * if the applied moment is greater than possible by the RCS system it will just max out the moments
 */
void Rocket::rcs_apply_const_moment(Vec3 m) {
    Vec3 applied_moment = m;
    // cap moments
    applied_moment.x = std::clamp(m.x, -active().rcs_max_capable_moment.x, active().rcs_max_capable_moment.x);
    applied_moment.y = std::clamp(m.y, -active().rcs_max_capable_moment.y, active().rcs_max_capable_moment.y);
    applied_moment.z = std::clamp(m.z, -active().rcs_max_capable_moment.z, active().rcs_max_capable_moment.z);
    applied_rcs_moment = applied_moment; // apply moment to apply_rcs_moment
}

// rotate a vector by a quaternion
static Vec3 rotate_by_quat(const Quat& q, const Vec3& u) {
    Vec3 q_vec = {q.x, q.y, q.z};
    Vec3 t = q_vec.cross(u);
    return u + t * (2.0 * q.w) + q_vec.cross(t) * 2.0;
}

// nose direction rotated into the ECI frame from an attitude
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
    return rotate_by_quat(q_engine, nose_body) * (active().thrust * thrust_scale);
}

/**
 * net torque about the combined CopM in body frame
 * @param thrust_scale isp change as pressure changes
 */
Vec3 Rocket::net_body_torque(double thrust_scale) const {
    Vec3 net_torque = {0, 0, 0};

    // lever arm from the combined CoM to the engine along the body axis
    double s_engine = active().tip_to_end_length - active().engine_distance;
    Vec3 r_engine = {0, 0, s_engine - z_cm};

    net_torque += r_engine.cross(engine_thrust_body(thrust_scale));
    if (rcs_active) net_torque += applied_rcs_moment;

    return net_torque;
}

// gravitational acceleration in the ECI frame
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

// standard atmosphere layers
static void atmosphere(double altitude, double& air_density, double& air_pressure) {
    double T = 288.15; // layer temperature, set by whichever branch runs

    // power relationship density equation
    auto pow_dens = [&](double rho_b, double T_b, double L, double layer_base_alt) {
        T = T_b + L * (altitude - layer_base_alt);
        return rho_b * pow(( T / T_b ), (-1.0 * g0 / (R_d * L)) - 1);
    };

    // exponential relationship density equation
    auto exp_dens = [&](double rho_b, double T_b, double layer_base_alt) {
        T = T_b;
        return rho_b * exp(-1.0 * (g0 * (altitude - layer_base_alt)) / (R_d * T_b));
    };

    // troposphere
    if (altitude < 11000) {
        air_density = pow_dens(1.2250, 288.15, -0.0065, 0);
    }
    // lower stratosphere
    else if (altitude < 20000) {
        air_density = exp_dens(0.36391, 216.65, 11000);
    }
    // middle stratosphere
    else if (altitude < 32000) {
        air_density = pow_dens(0.088035, 216.65, 0.001, 20000);
    }
    // upper stratosphere
    else if (altitude < 47000) {
        air_density = pow_dens(0.013225, 228.65, 0.0028, 32000);
    }
    // lower mesosphere
    else if (altitude < 51000) {
        air_density = exp_dens(0.0014275, 270.65, 47000);
    }
    // middle mesosphere
    else if (altitude < 71000) {
        air_density = pow_dens(0.00086160, 270.65, -0.0028, 51000);
    }
    // upper mesosphere
    else if (altitude < 86000) {
        air_density = pow_dens(0.000064211, 214.65, -0.0020, 71000);
    }
    // thermosphere
    else {
        air_density = exp_dens(0.000006958, 186.87, 86000);
    }

    air_pressure = air_density * R_d * T;
}

// acceleration due to drag in the ECI frame
Vec3 Rocket::calc_drag_accel(const Vec3& r, const Vec3& v, double mass) {
    double air_density, air_pressure;
    atmosphere(r.norm() - EARTH_RADIUS, air_density, air_pressure);

    // wind of earth spinning
    Vec3 w_earth = {0, 0, EARTH_ROTATION_RATE};
    Vec3 v_air = w_earth.cross(r);
    Vec3 v_relative = v - v_air;

    double craft_speed = v_relative.norm();
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

static Quat quat_deriv(const Quat& q, const Vec3& w) {
    Quat omega = {0.0, w.x, w.y, w.z};
    return (q * omega) * 0.5;
}

void Rocket::update_dynamics(double current_time) {
    // rocket mass
    double m = m_current;
    Vec3 I = I_body;

    // time step for the simulation
    double dt = TIME_STEP;

    // propellant drain
    Stage& s = active();
    double mdot = s.mass_flow_rate();
    if (mdot * dt > s.m_fuel) mdot = s.m_fuel / dt;

    // adjust thrust for isp change
    double air_density, air_pressure;
    atmosphere(r.norm() - EARTH_RADIUS, air_density, air_pressure);
    double thrust_scale;
    if (s.isp > 0) {
        thrust_scale = s.isp_at(air_pressure) / s.isp;
    } else {
        thrust_scale = 1.0;
    }

    // quantities the FC commands
    Vec3 thrust_body = engine_thrust_body(thrust_scale);
    Vec3 net_torque = net_body_torque(thrust_scale);

    // translational acceleration
    auto accel = [&](double t_i, const Vec3& r_i, const Vec3& v_i, const Quat& q_i) {
        double m_i = m - mdot * t_i;
        return calc_gravity_accel(r_i) + calc_drag_accel(r_i, v_i, m_i) + rotate_by_quat(q_i, thrust_body) / m_i;
    };

    // angular acceleration
    auto ang_accel = [&](const Vec3& w_i) {
        Vec3 Iw = {I.x * w_i.x, I.y * w_i.y, I.z * w_i.z};
        Vec3 gyro = w_i.cross(Iw);
        return Vec3{
            (net_torque.x - gyro.x) / I.x,
            (net_torque.y - gyro.y) / I.y,
            (net_torque.z - gyro.z) / I.z,
        };
    };

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // RK4 integration                                                                           //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////
    // k1 terms                       //
    ////////////////////////////////////
    Vec3 k1_r = v;
    Vec3 k1_v = accel(0.0, r, v, q_rocket);
    Vec3 k1_w = ang_accel(w);
    Quat k1_q = quat_deriv(q_rocket, w);

    ////////////////////////////////////
    // k2 terms                       //
    ////////////////////////////////////
    Vec3 r2 = r + k1_r * (dt / 2);
    Vec3 v2 = v + k1_v * (dt / 2);
    Vec3 w2 = w + k1_w * (dt / 2);
    Quat q2 = q_rocket + k1_q * (dt / 2);
    Vec3 k2_r = v2;
    Vec3 k2_v = accel(dt / 2, r2, v2, q2);
    Vec3 k2_w = ang_accel(w2);
    Quat k2_q = quat_deriv(q2, w2);

    ////////////////////////////////////
    // k3 terms                       //
    ////////////////////////////////////
    Vec3 r3 = r + k2_r * (dt / 2);
    Vec3 v3 = v + k2_v * (dt / 2);
    Vec3 w3 = w + k2_w * (dt / 2);
    Quat q3 = q_rocket + k2_q * (dt / 2);
    Vec3 k3_r = v3;
    Vec3 k3_v = accel(dt / 2, r3, v3, q3);
    Vec3 k3_w = ang_accel(w3);
    Quat k3_q = quat_deriv(q3, w3);

    ////////////////////////////////////
    // k4 terms                       //
    ////////////////////////////////////
    Vec3 r4 = r + k3_r * dt;
    Vec3 v4 = v + k3_v * dt;
    Vec3 w4 = w + k3_w * dt;
    Quat q4 = q_rocket + k3_q * dt;
    Vec3 k4_r = v4;
    Vec3 k4_v = accel(dt, r4, v4, q4);
    Vec3 k4_w = ang_accel(w4);
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

    // calculate final accelerations
    double m_end = m - mdot * dt;
    a_spec = rotate_by_quat(q_rocket, thrust_body) / m_end + calc_drag_accel(r, v, m_end);
    a = calc_gravity_accel(r) + a_spec;
    altitude = r.norm() - EARTH_RADIUS;

    // keep rocket from falling through the earth
    double alt_eci = r.norm();
    if (alt_eci <= topo->kMaxElevation + EARTH_RADIUS) {
        double surface_alt_eci = topo->SurfaceRadius3D(eci_to_ecef(r, current_time));
        if (alt_eci < surface_alt_eci) {
            Vec3 r_hat = r.normalized();
            r = r_hat * surface_alt_eci;
            v = surface_velocity_eci(r) + r_hat * std::max(v.dot(r_hat), 0.0); // if touching ground move iwth earth spin
        }
    }

}

// updates the fuel mass based on the current rocket states
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

void Rocket::set_engine_orientation(Quat orientation) {
    // normalize input
    double norm = std::sqrt(orientation.w*orientation.w + orientation.x*orientation.x +
                            orientation.y*orientation.y + orientation.z*orientation.z);
    orientation.w /= norm;
    orientation.x /= norm;
    orientation.y /= norm;
    orientation.z /= norm;

    double angle = 2.0 * std::acos(std::max(-1.0, std::min(1.0, orientation.w)));
    double max_angle = active().engine_gimball_range * M_PI / 180.0;

    if (angle <= max_angle) {
        q_engine = orientation;
        return;
    }

    // clamp to max gimbal angle
    double sin_half = std::sin(angle / 2.0);
    if (sin_half < 1e-9) {
        q_engine = {1, 0, 0, 0};
        return;
    }
    double half_max = max_angle / 2.0;
    double s = std::sin(half_max) / sin_half;
    q_engine = {std::cos(half_max), orientation.x * s, orientation.y * s, orientation.z * s};
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
    Vec3 rot_axis_u = rotation_axis / rotation_axis.norm();

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
}