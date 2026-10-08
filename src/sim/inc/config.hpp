// Copyright 2026 Tyler Wiggins
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include <string>
#include <vector>
#include "sim/inc/rocket.hpp"

struct RocketEntry {
    std::string name;
    bool track_data = false; // export this rocket's flight data to data/<name>.csv
    double export_interval = 0.0; // sim seconds between exported rows (0 = every step)
    double origin_lat = 0.0, origin_lon = 0.0;
    bool start_in_orbit = false; // start from orbit instead of origin_lat/origin_lon
    OrbitElements orbit;
    double target_lat = 0.0, target_lon = 0.0;
    RocketProps props;
};

struct SimConfig {
    double time_step = 0.01; // seconds
    double step_delay = 0.001; // seconds
    double max_time = 0.0; // seconds
};

// reads the sims config file
SimConfig load_sim_config(const std::string& path);

// reads the rocket config file
std::vector<RocketEntry> load_rocket_config(const std::string& path);

// reads a thrust curv
ThrustCurve load_thrust_curve(const std::string& path, const double total_initial_prop_mass_for_stage);
