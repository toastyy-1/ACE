#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <vector>
#include "fc.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////
// entry points                                                                 //
///////////////////////////////////////////////////////////////////////////////////////////////

void* fc_init(const fc_vehicle* vehicle, double t) {
    return new FlightController(*vehicle, t);
}

void fc_update(void* state, const fc_sensors* sensors) {
    static_cast<FlightController*>(state)->flight_controller_process(*sensors);
}

void fc_free(void* state) {
    delete static_cast<FlightController*>(state);
}

///////////////////////////////////////////////////////////////////////////////////////////////
// startup                                                                                   //
///////////////////////////////////////////////////////////////////////////////////////////////

FlightController::FlightController(const fc_vehicle& vehicle, double current_time) : veh(vehicle) {
    cs.stage = STANDBY;
}

FCInitState FlightController::create_target_trajectory(double lat_target, double long_target) {
    FCInitState out;

    // target's terrain height
    double target_radius = veh.r_target_ecef.norm();

    // convert to radians
    lat_target = lat_target * M_PI / 180.0;
    long_target = long_target * M_PI / 180.0;

    // derive lat and longitude from starting position of rocket on planet
    Vec3 p = veh.r_origin_eci;
    double radius = p.norm();
    double lat_origin  = asin(p.z / radius);
    double long_origin = atan2(p.y, p.x);

    // get ECI coordinates from lat and long
    out.r_origin = {
        radius * cos(lat_origin) * cos(long_origin),
        radius * cos(lat_origin) * sin(long_origin),
        radius * sin(lat_origin)
    };
    out.r_target_ecef = {
        target_radius * cos(lat_target) * cos(long_target),
        target_radius * cos(lat_target) * sin(long_target),
        target_radius * sin(lat_target)
    };

    Vec3 unit_vec_from_center = {
        .x = cos(lat_origin) * cos(long_origin),
        .y = cos(lat_origin) * sin(long_origin),
        .z = sin(lat_origin)
    };
    Vec3 up = {0, 0, 1};
    Vec3 rotation_axis = up.cross(unit_vec_from_center);
    Vec3 rot_axis_u = rotation_axis / rotation_axis.norm();
    double half_theta = acos(sin(lat_origin)) / 2;
    out.q_origin = {
        .w = cos(half_theta),
        .x = sin(half_theta) * rot_axis_u.x,
        .y = sin(half_theta) * rot_axis_u.y,
        .z = sin(half_theta) * rot_axis_u.z
    };

    // launch azimuth
    double delta_long = long_target - long_origin;
    double launch_azimuth = atan2(
        sin(delta_long) * cos(lat_target),
        cos(lat_origin) * sin(lat_target) - sin(lat_origin) * cos(lat_target) * cos(delta_long)
    );
    if (launch_azimuth < 0) launch_azimuth += 2.0 * M_PI;
    out.launch_asimuth = launch_azimuth;

    // burn time estimate for each stage
    out.stage_burn_time.reserve(num_stages());
    for (int i = 0; i < num_stages(); i++) {
        out.stage_burn_time.push_back(fc_stage_burn_time(&stage(i)));
    }

    // return all our calculated stuff yay!
    return out;
}

// init
void FlightController::init(double current_time) {
    cs = {};
    cs.stage = ARMED;
    cs.time = current_time;

    double tgt_lat = asin(veh.r_target_ecef.z / veh.r_target_ecef.norm()) * RAD_TO_DEG;
    double tgt_long = atan2(veh.r_target_ecef.y, veh.r_target_ecef.x) * RAD_TO_DEG;
    cs.is = create_target_trajectory(tgt_lat, tgt_long);

    cs.r = cs.is.r_origin; // set initial r to starting r
    cs.v = surface_velocity_eci(cs.r); // pad rotates with earth
    cs.att = cs.is.q_origin; // set initial orientation
    cs.target_att = cs.is.q_origin;
    countdown_start = current_time;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// helper functions                                                                          //
///////////////////////////////////////////////////////////////////////////////////////////////

// aquires new data from the sim
void FlightController::pull_new_data(const fc_sensors& sensors) {
    cs.g = sensors.g;
    cs.a_inertial = sensors.a_spec;
    cs.a = cs.a_inertial + cs.g;
    cs.w = sensors.w;
    cs.dt = sensors.dt;
    cs.time = sensors.t;
}

// estimate state of the rocket in flight at the current moment
void FlightController::estimate_state() {
    // position and velocity 

    // acceleration is assumed to have been determined before this by reading from the INS

    // add delta v based on a to current v
    cs.v = cs.v + cs.a * cs.dt;

    // add delta r based on v to current r
    cs.r = cs.r + cs.v * cs.dt;

    // now for attitude estimation

    // integrate quaternion rate
    Quat q_dot = cs.att * Quat{0.0, cs.w.x, cs.w.y, cs.w.z};
    cs.att.w += 0.5 * q_dot.w * cs.dt;
    cs.att.x += 0.5 * q_dot.x * cs.dt;
    cs.att.y += 0.5 * q_dot.y * cs.dt;
    cs.att.z += 0.5 * q_dot.z * cs.dt;

    // normalize quaternion bc computer shit
    cs.att = cs.att.normalize();
}

// returns quaternion that points in direction of
Quat FlightController::quat_from_vec(Vec3 u) {
    u = u.normalized();

    // quaternion of the shortest arc from nose to u
    Quat q = {
        .w = 1.0 + u.z,
        .x = -u.y,
        .y =  u.x,
        .z =  0.0,
    };
    return q.normalize();
}

// sets the engine gimbal based on target orientation
Quat FlightController::set_new_engine_gimbal_quat() {
    if (cs.stage < STAGE_1 || cs.stage == FREE_FLIGHT) return {1, 0, 0, 0};

    Quat target = cs.target_att;
    Quat current = cs.att;

    // calculate error between the two quaternions
    Quat q_err = current.inverse() * target;
    if (q_err.w < 0) q_err = {-q_err.w, -q_err.x, -q_err.y, -q_err.z};

    // for small angles the vector part of the error quaternion is proportional 
    // to the rotational error about body axis. n is the roll, pitch, yaw error in radians
    Vec3 n = { .x = 2 * q_err.x, .y = 2 * q_err.y, .z = 2 * q_err.z };

    // calculate required torque using PD
    double wn = 0.15;
    double zeta = 0.7;
    Vec3 K_p = cs.I * (wn * wn);
    Vec3 K_d = cs.I * (2.0 * zeta * wn);
    Vec3 tau_req = n.vector_individual_multiply(K_p) - cs.w.vector_individual_multiply(K_d);

    // map torque to gimball command angles

    // active stage index for the current mission stage
    const fc_stage& s = stage(cs.stage);

    // CoM of the rocket
    double z_cm = cs.z_cm;

    // engine gimbal point
    double s_engine = s.tip_to_end_length - s.engine_distance;

    // moment arm from the engine gimbal point to the rocket CoM
    double moment_arm = z_cm - s_engine;

    // required pitch and yaw
    double pitch = -tau_req.x / (s.max_thrust * moment_arm);
    double yaw = -tau_req.y / (s.max_thrust * moment_arm);

    // determine nozzzle deflection from body in quaternion orientation from pitch and yaw commands
    Quat q_pitch = { cos(pitch / 2), sin(pitch / 2), 0.0, 0.0 };
    Quat q_yaw = { cos(yaw / 2), 0.0, sin(yaw / 2), 0.0 };
    return (q_pitch * q_yaw).normalize();
}

// figures out what moment needs to be applied by the RCS system to achieve the target orientation 
Vec3 FlightController::calculate_rcs_moments_to_achieve_target_orientation() {
    Quat target = cs.target_att;
    Quat current = cs.att;

    // calculate error between the two quaternions
    Quat q_err = current.inverse() * target;
    if (q_err.w < 0) q_err = {-q_err.w, -q_err.x, -q_err.y, -q_err.z};

    // for small angles the vector part of the error quaternion is proportional 
    // to the rotational error about body axis. n is the roll, pitch, yaw error in radians
    Vec3 n = { .x = 2 * q_err.x, .y = 2 * q_err.y, .z = 2 * q_err.z };

    // do PD for required orientation
    Vec3 I = cs.I;
    double wn = 0.15; // random placeholder
    double zeta = 0.7;
    Vec3 K_p = I * (wn * wn);
    Vec3 K_d = I * zeta * wn;
    Vec3 tau_req = n.vector_individual_multiply(K_p) - cs.w.vector_individual_multiply(K_d);
    //std::cout << "PD applying moment of " << tau_req.x << ", " << tau_req.y << ", " << tau_req.z << " n-m\n";
    return { tau_req.x, tau_req.y, tau_req.z };
}

// integrates things to give a decent estimate of what the current moment of inertia of the rocket is
void FlightController::calculate_I() {
    Vec3 I = {0};
    double R2 = veh.radius * veh.radius;

    const int first_stage = cs.stage < STAGE_1 ? STAGE_1 : cs.stage;
    const int stage_count = num_stages();

    // mass-weighted center of mass of the remaining stages
    double M_total = 0.0, m_CoM = 0.0, base = 0.0;
    for (int i = first_stage; i < stage_count; i++) {
        const fc_stage& st = stage(i);
        double m = st.m_dry + st.m_fuel;
        M_total += m;
        m_CoM += m * (base + st.tip_to_end_length - st.CoM_dist);
        base += st.tip_to_end_length;
    }
    double z_cm = m_CoM / M_total;
    cs.z_cm = z_cm;

    base = 0.0;
    for (int i = first_stage; i < stage_count; i++) {
        const fc_stage& s = stage(i); // selected stage
        double M = s.m_dry + s.m_fuel;
        double L = s.tip_to_end_length;

        // about z axis
        I.z += 0.5 * M * R2;

        // about x and y axes parallel axis theorem from each stage centroid to z_cm
        double d = (base + L - s.CoM_dist) - z_cm;
        double Ixy = M * (L * L / 12.0 + R2 / 4) + M * d * d;
        I.x += Ixy;
        I.y += Ixy;

        base += L;
    }
    cs.I = I;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// flight controller loop                                                                    //
///////////////////////////////////////////////////////////////////////////////////////////////

// called once per sim time step
void FlightController::flight_controller_process(const fc_sensors& sensors) {

    ////////////////////////////
    // control inside         //
    ////////////////////////////

    // NOTE!! the staging functions inside the switch statement only ever set flags. the fc
    // commands all get issued in one block at the bottom so the behavior isnt hidden and we
    // dont accidentally do shit we dont want to

    if (cs.stage == STANDBY) init(sensors.t);

    // get latest data from the INS and advance the clock
    pull_new_data(sensors);

    // estimate state based on pulled data
    estimate_state();

    // manages setting a target attitude for the engine gimballing stuff depending on what the stage is
    switch (cs.stage) {
    case STANDBY:
        break;
    case ARMED:
        if (cs.time - countdown_start >= HOLD_DURATION) {
            cs.light_engine_flag = true;
            cs.stage_burn_time_start = cs.time;
            cs.stage = STAGE_1;
        }
        
        cs.v = surface_velocity_eci(cs.r); // set to pad velocity because the integrator doesnt account for the normal force so it thinks the rocket moves when sitting on pad
        break;
    case STAGE_1: {
        s1_powered();
        break;
    }
    case STAGE_2:
        s2_powered();
        break;
    case PAYLOAD_DEPLOY:
        payload_deploy();
        break;
    case FREE_FLIGHT:
        free_flight();
        break;
    }

    // estimate current moment of inertia
    calculate_I();

    // check if the engine was supposed to be cut off
    if (cs.final_burn_flag) {
        cs.final_burn_flag = false;
        // fractional burn (burn that is sub step to time step)
        fc_burn_fraction(cs.final_burn_fraction);
    }
    else if (cs.cutoff_engine_flag) {
        cs.cutoff_engine_flag = false;
        fc_cutoff_engine();
    }

    // check if the stage was supposed to be separated (clears the engine lock for the fresh stage)
    if (cs.separate_stage_flag) {
        cs.separate_stage_flag = false;
        fc_separate_stage();
    }

    // check if engine was supposed to be lit
    if (cs.light_engine_flag) {
        cs.light_engine_flag = false;
        fc_light_engine();
    }

    // check if rocket was supposed to be detonated
    if (cs.detonate_flag) {
        fc_detonate();
    }

    // send targeting commands to the engine gimbal system based on target attitude in cs
    fc_set_gimbal(set_new_engine_gimbal_quat());

    // send orientation change commands to the rcs thruster system if it is active
    if (cs.rcs_activated_flag) {
        fc_rcs_enable(1);
        fc_rcs_set_moment(calculate_rcs_moments_to_achieve_target_orientation());
    }
    else {
        fc_rcs_enable(0);
    }
}
