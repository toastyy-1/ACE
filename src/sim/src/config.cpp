#include "sim/inc/config.hpp"
#include "fkYAML/node.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

// @todo add doc
/**
 * @brief
 * @tparam T
 * @param n
 * @param key
 * @param fallback
 * @return
 */
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

/**
 * @brief parses a yaml file, printing the error if it can't
 * @param path yaml file to read
 * @return the root node, null if the file couldn't be parsed
 */
static fkyaml::node parse_yaml(const std::string& path) {
    try {
        std::ifstream file(path);
        return fkyaml::node::deserialize(file);
    } catch (const fkyaml::exception& err) {
        std::cerr << "config error: could not parse '" << path << "': " << err.what() << "\n";
    }
    return fkyaml::node();
}

/**
 * @brief reads the sim settings (time step and such), rockets live in their own file
 * @param path sim yaml file
 * @return sim settings, missing keys keep their defaults
 */
SimConfig load_sim_config(const std::string& path) {
    fkyaml::node root = parse_yaml(path);

    SimConfig cfg;

    cfg.time_step = value_or(root, "time_step", cfg.time_step);
    cfg.step_delay = value_or(root, "step_delay", cfg.step_delay);
    cfg.max_time = value_or(root, "max_time", cfg.max_time);

    return cfg;
}

/**
 * @brief reads every rocket entry and its stages
 * @param path rocket yaml file, stage thrust_curve paths are relative to its folder
 * @return one entry per rocket
 */
std::vector<RocketEntry> load_rocket_config(const std::string& path) {
    fkyaml::node root = parse_yaml(path);

    std::vector<RocketEntry> rockets;

    // thrust curve csvs are looked up next to the rocket file
    std::filesystem::path config_dir = std::filesystem::path(path).parent_path();

    // one entry per rocket
    if (root.is_mapping() && root.contains("rockets") && root["rockets"].is_sequence()) {
        size_t ri = 0;
        for (const fkyaml::node& rn : root["rockets"]) {
            RocketEntry rocket;

            rocket.name       = value_or(rn, "name", "rocket_" + std::to_string(ri));
            rocket.track_data = value_or(rn, "track_data", false);
            rocket.export_interval = value_or(rn, "export_interval", rocket.export_interval);

            rocket.origin_lat = value_or(rn, "origin_lat", 0.0);
            rocket.origin_lon = value_or(rn, "origin_lon", 0.0);
            rocket.target_lat = value_or(rn, "target_lat", 0.0);
            rocket.target_lon = value_or(rn, "target_lon", 0.0);

            rocket.start_in_orbit = value_or(rn, "start_in_orbit", false);
            rocket.orbit.semi_major_axis = value_or(rn, "semi_major_axis_km", 0.0);
            rocket.orbit.eccentricity    = value_or(rn, "eccentricity", 0.0);
            rocket.orbit.inclination     = value_or(rn, "inclination_deg", 0.0);
            rocket.orbit.raan            = value_or(rn, "raan_deg", 0.0);
            rocket.orbit.arg_periapsis   = value_or(rn, "arg_periapsis_deg", 0.0);
            rocket.orbit.true_anomaly    = value_or(rn, "true_anomaly_deg", 0.0);
            if (rocket.start_in_orbit && (rocket.orbit.semi_major_axis <= 0 || rocket.orbit.eccentricity < 0 || rocket.orbit.eccentricity >= 1)) {
                std::cerr << "config error: '" << path << "' rocket " << ri
                          << " starting in orbit needs semi_major_axis_km > 0 and 0 <= eccentricity < 1\n";
            }

            rocket.props.radius = value_or(rn, "radius", rocket.props.radius);
            rocket.props.nosecone_length = value_or(rn, "nosecone_length", rocket.props.nosecone_length);
            rocket.props.nosecone_mass = value_or(rn, "nosecone_mass", rocket.props.nosecone_mass);
            // a thin walled cone's CoM sits 2/3 of the way back from the tip
            rocket.props.nosecone_com_distance = value_or(rn, "nosecone_com_distance", (2.0 / 3.0) * rocket.props.nosecone_length);

            // stage count comes from however many stage entries this rocket defines
            bool has_stages = rn.is_mapping() && rn.contains("stage") && rn["stage"].is_sequence();
            if (!has_stages || rn["stage"].empty()) {
                std::cerr << "config error: '" << path << "' rocket " << ri
                          << " must define at least one stage entry\n";
            }

            if (has_stages) {
                size_t si = 0;
                for (const fkyaml::node& st : rn["stage"]) {
                    Stage s;

                    s.id                    = value_or(st, "id", 0.0);
                    s.m_dry                 = value_or(st, "dry_mass", 0.0);
                    s.m_fuel                = value_or(st, "fuel_mass", 0.0);
                    s.m_fuel_full           = s.m_fuel;
                    s.tip_to_end_length     = value_or(st, "length", 0.0);
                    s.CoM_dist              = value_or(st, "com_distance", 0.0);
                    s.fuel_CoM_dist         = value_or(st, "fuel_com_distance", s.CoM_dist);
                    s.fuel_length           = value_or(st, "fuel_length", 0.0);
                    s.engine_distance       = value_or(st, "engine_distance", 0.0);
                    s.engine_gimball_range  = value_or(st, "gimbal_range_deg", 0.0);

                    std::string curve_file = value_or(st, "thrust_curve", std::string());
                    if (curve_file.empty()) {
                        std::cerr << "config error: '" << path << "' rocket " << ri << " stage " << si
                                  << " must set a thrust_curve csv\n";
                    } else {
                        s.thrust_curve_file = (config_dir / curve_file).string();
                        s.thrust_curve = load_thrust_curve(s.thrust_curve_file, s.m_fuel);
                    }

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

            rockets.push_back(rocket);
            ri++;
        }
    }

    if (rockets.empty()) {
        std::cerr << "config error: '" << path << "' must define at least one rocket entry\n";
    }

    return rockets;
}

/**
 * @brief reads a thrust curve csv. every row is `time (s), thrust (N)`, the first line may be a
 * header and blank lines are skipped
 * @param path csv file to read
 * @return the curve's time and thrust samples, empty if the file is missing or malformed
 */
ThrustCurve load_thrust_curve(const std::string& path, const double total_initial_prop_mass_for_stage) {
    std::ifstream file(path);
    if (!file) {
        std::cerr << "config error: could not open thrust curve '" << path << "'\n";
        return {};
    }

    ThrustCurve tc(total_initial_prop_mass_for_stage);
    std::string line;
    int line_num = 0;
    bool header_allowed = true; // only the first non blank line can be a header

    while (std::getline(file, line)) {
        line_num++;
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;

        // exactly two numbers separated by a comma
        std::istringstream row(line);
        double t = 0, thrust = 0;
        char comma = 0;
        bool parsed = (row >> t >> comma >> thrust) && comma == ',' && (row >> std::ws).eof();

        bool is_header = !parsed && header_allowed;
        header_allowed = false;
        if (is_header) continue;

        if (!parsed) {
            std::cerr << "config error: '" << path << "' line " << line_num << " must be `time, thrust`\n";
            return {};
        }
        if (t < 0 || thrust < 0) {
            std::cerr << "config error: '" << path << "' line " << line_num << " time and thrust can't be negative\n";
            return {};
        }
        if (!tc.time().empty() && t <= tc.time().back()) {
            std::cerr << "config error: '" << path << "' line " << line_num << " time must be increasing\n";
            return {};
        }

        tc.add_point(t, thrust);
    }

    if (tc.time().empty()) {
        std::cerr << "config error: '" << path << "' has no thrust curve data\n";
    }

    return tc;
}
