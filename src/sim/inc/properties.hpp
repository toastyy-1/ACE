#pragma once

#include "constants.hpp"
#include "types.hpp"
#include <vector>

// number of stages on the rocket
inline constexpr int ROCKET_NUM_STAGES = 3;

struct Stage {
    double id;
    double m_dry;                   // dry mass
    double m_fuel;                  // fuel mass
    double m_fuel_full;             // fuel mass at ignition
    double isp;                     // vacuum specific impulse (s)
    double isp_sea_level;           // specific impulse at sea level
    double tip_to_end_length;       // m
    double CoM_dist;                // dist of center of mass from front edge of the stage (full tank)
    double fuel_CoM_dist;           // dist of the full propellant column CoM from the tip
    double fuel_length;             // length of the full propellant column
    double max_thrust;              // rated (max) motor thrust
    double thrust;                  // current commanded thrust
    double engine_distance;         // distance of engine from leading edge
    double engine_gimball_range;    // deg
    Vec3 rcs_max_capable_moment;    // n-m torque that RCS system for that stage can apply about axes along CoM (set 0 if no rcs)

    // velcoty of exhaust of the engine
    double exhaust_velocity() const { return isp * g0; }

    // current flow of mass out of the engine while its burning
    double mass_flow_rate() const { return isp > 0 ? thrust / exhaust_velocity() : 0.0; }

    // maximum possible mass flow rate from engine
    double max_mass_flow_rate() const { return isp > 0 ? max_thrust / exhaust_velocity() : 0.0; }

    // fraction of the propellant load still in the tank
    double fuel_fill() const { return m_fuel_full > 0 ? m_fuel / m_fuel_full : 0.0; }

    // isp at different atmospheric pressures
    double isp_at(double pressure) const {
        if (isp_sea_level <= 0) return isp;
        double f = pressure / p_sl;
        return isp - (isp - isp_sea_level) * (f < 1.0 ? f : 1.0);
    }

    // propellant CoM from the tip
    double fuel_CoM() const { return fuel_CoM_dist + 0.5 * fuel_length * (1.0 - fuel_fill()); }

    // dry structure CoM from the tip
    double dry_CoM() const {
        if (m_dry > 0) {
            return ((m_dry + m_fuel_full) * CoM_dist - m_fuel_full * fuel_CoM_dist) / m_dry;
        } else {
            return CoM_dist;
        }
    }
};

// config and geometry of rocket whao
struct RocketProps {
    double radius = 0; // hull radius for the solid-cylinder inertia model (m)
    double nosecone_length = 0;
    double nosecone_mass = 0; // mass of the nosecone (kg)
    double nosecone_com_distance = 0; // nosecone CoM measured back from the nose tip (m)
    std::vector<Stage> stages;
};
