#include "sim/config.hpp"
#include "fkYAML/node.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>

template <typename T>
static T value_or(const fkyaml::node& n, const char* key, T fallback) {
    if (n.is_mapping() && n.contains(key)) {
        try {
            return n.at(key).get_value<T>();
        } catch (...) {
        }
    }
    return fallback;
}

SimConfig load_sim_config(const std::string& path) {
    // parse the yaml file
    fkyaml::node root;
    try {
        std::ifstream file(path);
        root = fkyaml::node::deserialize(file);
    } catch (const fkyaml::exception& err) {
        std::cerr << "config error: could not parse '" << path << "': " << err.what() << "\n";
    }

    SimConfig cfg;

    cfg.time_step = value_or(root, "time_step", cfg.time_step);
    cfg.step_delay = value_or(root, "step_delay", cfg.step_delay);

    // one entry per rocket
    if (root.is_mapping() && root.contains("rockets") && root["rockets"].is_sequence()) {
        size_t ri = 0;
        for (const auto& rn : root["rockets"]) {
            RocketEntry rocket;

            rocket.origin_lat = value_or(rn, "origin_lat", 0.0);
            rocket.origin_lon = value_or(rn, "origin_lon", 0.0);
            rocket.target_lat = value_or(rn, "target_lat", 0.0);
            rocket.target_lon = value_or(rn, "target_lon", 0.0);

            rocket.props.radius = value_or(rn, "radius", rocket.props.radius);
            rocket.props.Cd     = value_or(rn, "drag_coefficient", rocket.props.Cd);

            // stage count comes from however many stage entries this rocket defines
            bool has_stages = rn.is_mapping() && rn.contains("stage") && rn["stage"].is_sequence();
            if (!has_stages || rn["stage"].empty()) {
                std::cerr << "config error: '" << path << "' rocket " << ri
                          << " must define at least one stage entry\n";
            }

            if (has_stages) {
                size_t si = 0;
                for (const auto& st : rn["stage"]) {
                    Stage s{};

                    s.id                    = value_or(st, "id", 0.0);
                    s.m_dry                 = value_or(st, "dry_mass", 0.0);
                    s.m_fuel                = value_or(st, "fuel_mass", 0.0);
                    s.m_fuel_full           = s.m_fuel;
                    s.isp                   = value_or(st, "isp", 0.0);
                    s.isp_sea_level         = value_or(st, "isp_sea_level", 0.0);
                    s.tip_to_end_length     = value_or(st, "length", 0.0);
                    s.CoM_dist              = value_or(st, "com_distance", 0.0);
                    s.fuel_CoM_dist         = value_or(st, "fuel_com_distance", s.CoM_dist);
                    s.fuel_length           = value_or(st, "fuel_length", 0.0);
                    s.max_thrust            = value_or(st, "max_thrust", 0.0);
                    s.engine_distance       = value_or(st, "engine_distance", 0.0);
                    s.engine_gimball_range  = value_or(st, "gimbal_range_deg", 0.0);

                    if (st.is_mapping() && st.contains("rcs_max_moment")) {
                        const fkyaml::node& rcs = st["rcs_max_moment"];
                        if (rcs.is_sequence() && rcs.size() == 3) {
                            s.rcs_max_capable_moment = {
                                rcs[0].get_value<double>(),
                                rcs[1].get_value<double>(),
                                rcs[2].get_value<double>(),
                            };
                        } else {
                            std::cerr << "config error: rocket " << ri << " stage " << si
                                      << " rcs_max_moment must be a 3-element array, ignoring\n";
                        }
                    }

                    rocket.props.stages.push_back(s);
                    si++;
                }
            }

            cfg.rockets.push_back(rocket);
            ri++;
        }
    }

    if (cfg.rockets.empty()) {
        std::cerr << "config error: '" << path << "' must define at least one rocket entry\n";
    }

    return cfg;
}
