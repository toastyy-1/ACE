// Copyright 2026 Tyler Wiggins
// SPDX-License-Identifier: Apache-2.0

// fires the "time command [args]" lines of config/fc_schedule.txt. commands:
//   light, separate, cutoff, launch_abort
//   rcs x y z        rcs on with this body frame torque (N-m), clamped to the stage's rcs_max_moment
//   rcs_off
//   force x y z      ECI force on the CoM (N), force 0 0 0 to stop it
//   gimbal w x y z   nozzle quaternion relative to the body, clamped to the stage's gimbal_range_deg

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "fc/inc/fc_api.h"

struct event {
    double t;
    std::string name;
    std::vector<double> args;
};

struct fc_state {
    std::vector<event> events;
};
static fc_state state;

fc_state* fc_init(const fc_vehicle*) {
    std::ifstream f("config/fc_schedule.txt");
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream in(line);
        event e;
        if (!(in >> e.t >> e.name)) continue;
        for (double a; in >> a;) e.args.push_back(a);

        size_t n = e.name == "rcs" || e.name == "force" ? 3 : e.name == "gimbal" ? 4 : 0;
        if (e.args.size() != n) {
            std::cerr << "config error: fc_schedule.txt '" << line << "' needs " << n << " numbers after the command, ignoring\n";
            continue;
        }
        state.events.push_back(e);
    }
    return &state;
}

void fc_update(fc_state* s, const fc_sensors* sensors, fc_commands* cmd) {
    while (!s->events.empty() && sensors->t >= s->events.front().t) {
        const event& e = s->events.front();
        cmd->light |= e.name == "light";
        cmd->separate |= e.name == "separate";
        cmd->launch_abort |= e.name == "launch_abort";
        cmd->cutoff |= e.name == "cutoff";
        if (e.name == "rcs") {
            cmd->debug_rcs_on = 1;
            cmd->debug_rcs_moment = {e.args[0], e.args[1], e.args[2]};
        }
        if (e.name == "rcs_off") cmd->debug_rcs_on = 0;
        if (e.name == "force") cmd->debug_apply_force = {e.args[0], e.args[1], e.args[2]};
        if (e.name == "gimbal") cmd->gimbal = {e.args[0], e.args[1], e.args[2], e.args[3]};
        s->events.erase(s->events.begin());
    }
}

void fc_free(fc_state*) {}
