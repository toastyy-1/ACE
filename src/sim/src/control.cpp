/**
 * handles all dynamic systems for rocket.cpp that involve control systems
 */

 #include "sim/inc/rocket.hpp"
 #include <algorithm>



/**
 * hands the sensors to whatever thing implements fc_api.h and applies what it asked for
 */
void Rocket::update_flight_controller(double current_time) {
    if (pending_cutoff) { active_stage().thrust = 0.0; pending_cutoff = false; } // process engine sub step cutoff

    fc_sim_connector::begin(this);

    // hand over the rockets spec and let the controller set itself up
    if (!fc_started) {
        fc_stages.clear();
        fc_stages.reserve(props.stages.size());
        for (const Stage& s : props.stages) {
            fc_stage fs{};
            fs.id                      = static_cast<int>(s.id);
            fs.m_dry                   = s.m_dry;
            fs.m_fuel                  = s.m_fuel_full;
            fs.isp                     = s.isp;
            fs.isp_sea_level           = s.isp_sea_level;
            fs.tip_to_end_length       = s.tip_to_end_length;
            fs.CoM_dist                = s.CoM_dist;
            fs.fuel_CoM_dist           = s.fuel_CoM_dist;
            fs.fuel_length             = s.fuel_length;
            fs.max_thrust              = s.max_thrust;
            fs.engine_distance         = s.engine_distance;
            fs.engine_gimbal_range_deg = s.engine_gimball_range;
            fs.rcs_max_moment          = s.rcs_max_capable_moment;
            fc_stages.push_back(fs);
        }

        fc_veh = std::make_unique<fc_vehicle>();
        fc_veh->radius        = props.radius;
        fc_veh->num_stages    = static_cast<int>(fc_stages.size());
        fc_veh->stages        = fc_stages.data();
        fc_veh->r_origin_eci  = start_state.origin_r_eci;
        fc_veh->q_origin_eci  = start_state.origin_q_eci;
        fc_veh->r_target_ecef = start_state.target_r_ecef;
        fc_veh->time_step     = TIME_STEP;

        fc.reset(fc_init(fc_veh.get()));
        fc_cmd = {};
        fc_cmd.gimbal = {1, 0, 0, 0};
        fc_started = true;
        fc_last_time = current_time;
    }

    // sample the sensors
    fc_sensors sensors{};
    sensors.t      = current_time;
    sensors.dt     = current_time - fc_last_time;
    sensors.a_spec = ins.read_acc(a_spec);
    sensors.w      = ins.read_gyr(w);
    sensors.g      = INS::gravity_eci(r);
    fc_last_time   = current_time;

    // one shot commands are cleared every step, persistent ones keep their state
    fc_cmd.light    = 0;
    fc_cmd.cutoff   = 0;
    fc_cmd.separate = 0;
    fc_cmd.detonate = 0;
    fc_cmd.cutoff_fraction = 0.0;

    fc_update(fc.get(), &sensors, &fc_cmd);

    fc_sim_connector::end();
    apply_fc_commands();
}

/**
 * apply a step's worth of buffered commands
 */
void Rocket::apply_fc_commands() {
    // a nonzero cutoff_fraction burns that much of this step before the cutoff
    if (fc_cmd.cutoff) {
        if (fc_cmd.cutoff_fraction > 0.0) command_final_burn_fraction(fc_cmd.cutoff_fraction);
        else                              cutoff_engine();
    }

    if (fc_cmd.separate) advance_stage(); // clears the engine lock for the fresh stage
    if (fc_cmd.light)    light_engine();
    if (fc_cmd.detonate) activate_detonation();

    set_engine_orientation(fc_cmd.gimbal);

    if (fc_cmd.rcs_on) {
        rcs_on();
        rcs_apply_const_moment(fc_cmd.rcs_moment);
    }
    else {
        rcs_off();
    }
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
    if (active_stage().m_fuel > 0) {
        active_stage().thrust = active_stage().max_thrust;
    }
}

/**
 * permanently terminates thrust on the active stage, kills motor real dead
 */
void Rocket::cutoff_engine() {
    active_stage().thrust = 0;
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
    active_stage().thrust = fraction * active_stage().max_thrust;
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
    applied_moment.x = std::clamp(m.x, -active_stage().rcs_max_capable_moment.x, active_stage().rcs_max_capable_moment.x);
    applied_moment.y = std::clamp(m.y, -active_stage().rcs_max_capable_moment.y, active_stage().rcs_max_capable_moment.y);
    applied_moment.z = std::clamp(m.z, -active_stage().rcs_max_capable_moment.z, active_stage().rcs_max_capable_moment.z);
    applied_rcs_moment = applied_moment; // apply moment to apply_rcs_moment
}

/**
 * sets the orientation of the rocket nozzle for gimbaling purposes
 */
void Rocket::set_engine_orientation(Quat orientation) {
    // normalize input
    double norm = std::sqrt(orientation.w*orientation.w + orientation.x*orientation.x +
                            orientation.y*orientation.y + orientation.z*orientation.z);
    orientation.w /= norm;
    orientation.x /= norm;
    orientation.y /= norm;
    orientation.z /= norm;

    double angle = 2.0 * std::acos(std::max(-1.0, std::min(1.0, orientation.w)));
    double max_angle = active_stage().engine_gimball_range * M_PI / 180.0;

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