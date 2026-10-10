// Copyright 2026 Tyler Wiggins
// SPDX-License-Identifier: Apache-2.0

// fires the "time command" lines of config/fc_schedule.txt. commands: light, separate, cutoff, detonate

#include <fstream>
#include <string>
#include <vector>
#include "fc/inc/fc_api.h"

struct fc_state {
    std::vector<std::pair<double, std::string>> events;
};
static fc_state state;

fc_state* fc_init(const fc_vehicle*) {
    std::ifstream f("config/fc_schedule.txt");
    double t;
    std::string name;
    while (f >> t >> name) state.events.emplace_back(t, name);
    return &state;
}

void fc_update(fc_state* s, const fc_sensors* sensors, fc_commands* cmd) {
    while (!s->events.empty() && sensors->t >= s->events.front().first) {
        const std::string& name = s->events.front().second;
        cmd->light |= name == "light";
        cmd->separate |= name == "separate";
        cmd->detonate |= name == "detonate";
        cmd->cutoff |= name == "cutoff";
        s->events.erase(s->events.begin());
    }
}

void fc_free(fc_state*) {}
