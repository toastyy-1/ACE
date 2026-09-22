#pragma once
#include <string>
#include <vector>
#include "sim/inc/properties.hpp"

struct RocketEntry {
    double origin_lat = 0.0, origin_lon = 0.0;
    double target_lat = 0.0, target_lon = 0.0;
    RocketProps props;
};

struct SimConfig {
    std::vector<RocketEntry> rockets;
    double time_step = 0.01; // seconds
    double step_delay = 0.001;
};

// reads the sims config file
SimConfig load_sim_config(const std::string& path);
