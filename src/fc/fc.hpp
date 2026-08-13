#pragma once

#include "fc/fc_api.h"
#include <vector>

// the intitial states that the rocket starts at
struct FCInitState {
    Vec3 r_origin;
    Vec3 r_target_ecef;
    Quat q_origin;

    double launch_asimuth;

    Vec3 v_s1_bo_T;
    Vec3 r_s1_bo_T;

    std::vector<double> stage_burn_time;

    double target_tof;
};

// holds the rockets staging patterns
enum MissionStage {
    STANDBY = -2,
    ARMED = -1,
    STAGE_1 = 0,
    STAGE_2 = 1,
    PAYLOAD_DEPLOY = 2,
    FREE_FLIGHT = 3
};

// holds the states of controls for the rocket
struct ControlStates {
    MissionStage stage;

    // states
    Vec3 g; // gravity vector
    Vec3 a_inertial; // inertial acceleration
    Vec3 a; // g + a_inertial
    Vec3 v;
    Vec3 r;
    Vec3 w; // angualr velocity
    Quat att; // attitude
    Quat target_att; // the current iterations target attitude
    Vec3 I; // current moment of inertia
    double z_cm; // CoM estimate

    double dt; // time from last time measurement
    double time;
    double stage_burn_time_start; // mission time the current stage's engine was lit

    int s2_lambert_counter = 0; // makes v_req be computed every n steps
    int s2_lambert_counter_reset_num = 3; // resets counter every n times
    Vec3 s2_v_req{}; // optimal required velocity

    int s3_lambert_counter = 0; // makes v_req be computed every n steps
    int s3_lambert_counter_reset_num = 3;
    Vec3 s3_v_req{}; // optimal required velocity

    bool rcs_activated_flag = false; // set to true if you want the RCS system to try and point the rocket to target_att
    bool light_engine_flag = false; // set to true if you want to light the engine on that step
    bool separate_stage_flag = false; // set to true if you want to separate stage on that step
    bool cutoff_engine_flag = false; // set to true if you want to permanently cut off the engine on that step
    bool final_burn_flag = false; // set to true to burn only final_burn_fraction of this step, then cut off
    bool detonate_flag = false; // set to true if you want to blow the rocket up
    double final_burn_fraction = 1.0; // fraction of a full step to burn before the sub step cutoff
    bool payload_deploy_cutoff_done = false; // set once payload_deploy has cut the engine

    // initial states
    FCInitState is;
};

class FlightController {
    public:
    // load a fully set up rocket config/geometry into the FC and set up initial state
    FlightController(const fc_vehicle& vehicle, double current_time);

    // perform flight controller operations (should be called every time we want to update the FC)
    void flight_controller_process(const fc_sensors& sensors);

    private:
    static constexpr double HOLD_DURATION = 10.0;

    // rocket geometry handed over by the sim once at startup
    fc_vehicle veh;

    ControlStates cs;
    double countdown_start = 0;

    // helpers
    const fc_stage& stage(int i) const { return veh.stages[i]; }
    int num_stages() const { return veh.num_stages; }
    Vec3 target_eci_at_time_of_arrival(double t_arrival) const { return ecef_to_eci(cs.is.r_target_ecef, t_arrival); }
    void init(double current_time);
    FCInitState create_target_trajectory(double lat_target, double long_target);
    void estimate_state();
    void pull_new_data(const fc_sensors& sensors);
    Quat quat_from_vec(Vec3 u);
    Quat set_new_engine_gimbal_quat();
    Vec3 calculate_rcs_moments_to_achieve_target_orientation(); // longest function name ever lets go
    void calculate_I();
    void command_engine_cutoff() { cs.cutoff_engine_flag = true; }

    // stages
    void s1_powered();
    void s2_powered();
    void payload_deploy();
    void free_flight();

};