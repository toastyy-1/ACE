// Copyright 2026 Tyler Wiggins
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include "types.hpp"
#include "sim_constants.hpp"
#include "sim/inc/ins.hpp"
#include "sim/inc/data_export.hpp"
#include "sim/inc/ephemeris.hpp"
#include "fc/inc/fc_sim_connector.hpp"
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>
#include "renderer/earth_surface.hpp"

struct RocketStartState {
    Vec3 origin_r_eci;
    Quat origin_q_eci; // origin attitude
    Vec3 target_r_ecef;
};

// orbital elements for a rocket that starts in orbit
struct OrbitElements {
    double semi_major_axis = 0; // km
    double eccentricity = 0;
    double inclination = 0;     // deg
    double raan = 0;            // deg, measured from the ECI +x axis (the greenwich meridian at t = 0)
    double arg_periapsis = 0;   // deg
    double true_anomaly = 0;    // deg
};

// holds both an delta acceleration and delta torque vector so a single funciton can return
// acts on both accel and torque at the same time, to save computation
struct KinematicModifier {
    Vec3 accel;
    Vec3 torque;
};

// snapshot of rocket state at any given moment
struct RocketState {
    bool detonation_active = false;

    double t = 0;
    double mass = 0, fuel = 0;
    double length = 0, cm_dist = 0, engine_dist = 0, radius = 0;   // dims from nose
    double nose_length = 0;
    bool has_engine = true;

    Vec3 r{}, v{}, a{}, w{};
    Quat q_rocket{1, 0, 0, 0};
    Quat q_engine{1, 0, 0, 0};
    RocketStartState init{};
};

// number of stages on the rocket
inline constexpr int ROCKET_NUM_STAGES = 3;

// thrust curve
class ThrustCurve {
    public:
    ThrustCurve() = default;
    explicit ThrustCurve(double total_initial_prop_mass) : prop_mass(total_initial_prop_mass) {}

    // appends a sample
    void add_point(double time, double thrust) { t.push_back(time); F.push_back(thrust); }

    // getters
    double thrust(double time) const;                              // thrust at a time since ignition (N)
    double impulse(double t0, double t1) const;                    // total impulse between two times since ignition (N*s)
    double isp() const;                                            // avg specific impulse over the whole curve (s)
    double peak_thrust() const { return F.empty() ? 0.0 : *std::max_element(F.begin(), F.end()); };  // highest thrust on the curve (N)
    double end_time() const { return t.empty() ? 0.0 : t.back(); } // burnout time (s)
    const std::vector<double>& time() const { return t; }
    const std::vector<double>& thrust() const { return F; }

    private:
    std::vector<double> t;   // time since ignition (s)
    std::vector<double> F; // thrust (N)
    double prop_mass = 0;    // propellant the curve burns (kg)
};

enum class Shape { 
    Cylinder, 
    Cone, 
    Sphere 
};

// outer shape of a stage
struct Geometry {
    Shape shape = Shape::Cylinder;
    double radius = 0; // cylinder/sphere/cone case radius (m)
    double cd = -1;    // this is for sphere becaues it uses sphere mach tale for cd

    // moment of inertia per unit mass along the long axis of a uniform solid (m^2)
    double axial_inertia() const {
        switch (shape) {
            case Shape::Cone:   return 0.3 * radius * radius;
            case Shape::Sphere: return 0.4 * radius * radius;
            default:            return 0.5 * radius * radius;
        }
    }

    // moment of inertia per unit mass aklong a transverse axis through the centroid of a uniform solid (m^2)
    double transverse_inertia(double length) const {
        switch (shape) {
            case Shape::Cone:   return 0.15 * radius * radius + 0.0375 * length * length;
            case Shape::Sphere: return 0.4 * radius * radius;
            default:            return (3.0 * radius * radius + length * length) / 12.0;
        }
    }
};

struct Stage {
    double id;
    Geometry geometry;
    double m_dry;                   // dry mass
    double m_fuel;                  // fuel mass
    double m_fuel_full;             // fuel mass at ignition
    double tip_to_end_length;       // m
    double CoM_dist;                // dist of center of mass from front edge of the stage (full tank)
    double fuel_CoM_dist;           // dist of the full propellant column CoM from the tip
    double fuel_length;             // length of the full propellant column
    double engine_distance;         // distance of engine from leading edge
    double engine_gimball_range;    // deg
    Vec3 rcs_max_capable_moment;    // n-m torque that RCS system for that stage can apply about axes along CoM (set 0 if no rcs)
    std::string thrust_curve_file;  // path to the stage's thrust curve csv
    ThrustCurve thrust_curve;       // thrust curve, thrust vs time

    // effective exhaust velocity
    double exhaust_velocity() const { return m_fuel_full > 0 ? thrust_curve.impulse(0.0, thrust_curve.end_time()) / m_fuel_full : 0.0; }

    // fraction of the propellant load still in the tank
    double fuel_fill() const { return m_fuel_full > 0 ? m_fuel / m_fuel_full : 0.0; }

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
    double nosecone_length = 0; // the nosecone sits on the last stage and shares its radius
    double nosecone_mass = 0; // mass of the nosecone (kg)
    double nosecone_com_distance = 0; // nosecone CoM measured back from the nose tip (m)
    std::vector<Stage> stages;

    // widest stage radius from stage `first` up to the nose (m)
    double max_radius(int first = 0) const {
        double R = 0;
        for (size_t i = first; i < stages.size(); i++) R = std::max(R, stages[i].geometry.radius);
        return R;
    }
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MAIN ROCKET CLASS                                                                                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class Rocket {
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // public                                                                                    //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    public:

    // counter to track once the rocket is dead how long it should stay existing before deleting itself
    double life_countdown = 30.0; // stays alive for n (sim) seconds before disappearing

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // setup                                                                                     //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    Rocket(const std::string& name, double origin_latitude, double origin_longitude, double target_latitude,
           double target_longitude, const RocketProps& props, bool track_data, double export_interval,
           bool start_in_orbit, const OrbitElements& orbit, bool start_in_eci, const Vec3& eci_position, const Vec3& eci_velocity,
           bool moon_gravity, bool sun_gravity, bool drag, const Ephemeris* ephem);
    ~Rocket();

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // getters                                                                                   //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    RocketState get_state() const;
    const std::string& get_name() const { return name; }
    bool is_detonated() { return detonated; }
    int active_stage_idx() const { return active_idx; } // index the fc's stage array with this

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // setters                                                                                   //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    void set_pos(const Vec3& pos) { r = pos; } // set absolute position
    void set_orientation(const Quat& orient) { q_rocket = orient; } // set absolute orientation

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // functions used by sim                                                                     //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    void update_dynamics(double current_time);
    void update_mass();
    void update_flight_controller(double current_time);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // functions used by flight controller                                                       //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    void light_engine(); // should be used once per stage
    void cutoff_engine();
    void command_final_burn_fraction(double fraction); 
    bool advance_stage();
    void set_engine_orientation(Quat orientation);
    void rcs_on() { rcs_active = true; } // enable or disable rcs orientation correction
    void rcs_off() { rcs_active = false; }
    void activate_detonation() { detonated = true; }

    // rocket owns the flight controller state
    Rocket(Rocket&&) = default;
    Rocket& operator=(Rocket&&) = default;
    Rocket(const Rocket&) = delete;
    Rocket& operator=(const Rocket&) = delete;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // private                                                                                   //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    private:
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // rocket static configuration                                                               //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // the dropped in flight controller
    INS ins;
    fc_sim_connector::State fc; // state fc_init
    fc_commands fc_cmd{}; // what it asked for on the current step

    std::unique_ptr<fc_vehicle> fc_veh;
    std::vector<fc_stage> fc_stages;

    bool fc_started = false;
    double fc_last_time = 0.0;

    // applies the commands sent by the FC through the API
    void apply_fc_commands();

    // topography
    renderer::EarthSurface* topo = &renderer::EarthSurface::Get();

    // sun and moon gravity
    bool moon_gravity = false;
    bool sun_gravity = false;
    const Ephemeris* ephem = nullptr;

    // aerodynamic forces and torques
    bool drag = true;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // rocket static configuration                                                               //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    RocketProps props; // geometric properties of the rocket, set by the config
    std::string name;

    // flight data csv writer
    std::unique_ptr<DataExport> data_export;

    // initial launch geometry (origin, target, launch attitude, such things)
    RocketStartState start_state;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // dynamic state                                                                             //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    int active_idx = 0;          // index of the currently active stage
    bool engine_locked = false;  // once cut off, the active stage's motor cannot be relit until staged away
    bool pending_cutoff = false; // a sub-step burn is finishing; thrust is zeroed at the start of the next step
    double throttle = 0;         // fraction of the thrust curve the active stage gives
    double burn_time = 0;        // time since the active stage ignition (s)
    bool rcs_active = false;     // if active, RCS will start to apply a correcting moment if commanded by FC

    // mass properties
    double m_current = 0;        // current total mass (kg)
    double m_fuel_current = 0;   // current total fuel mass (kg)
    Vec3 I_body = {0, 0, 0};     // moments of inertia about the combined CoM, body frame
    double z_cm = 0;             // combined CoM along body +z, from the active stage's aft edge (m)
    double z_cp = 0;             // center of pressure along body +z, from the active stage's aft edge (m)


    // kinematic state
    Vec3 r = {0, 0, 0};             // position (m)
    Vec3 v = {0, 0, 0};             // velocity (m/s)
    Vec3 a = {0, 0, 0};             // acceleration (m/s^2)
    Vec3 a_spec = {0, 0, 0};        // specific force in the body frame (m/s^2) (INS reads this)
    Vec3 w = {0, 0, 0};             // angular velocity (rad/s)
    Quat q_rocket = {1, 0, 0, 0};   // orientation of rocket nose relative to ECI (+z is nose)
    Quat q_engine = {1, 0, 0, 0};   // orientation of engine relative to rocket body
    double altitude = 0;

    // tracked meta properties for data analytics
    Vec3 drag_accel = {0, 0, 0};
    Vec3 grav_accel = {0, 0, 0};
    Vec3 thrust_accel = {0, 0, 0};
    double mach = 0;
    double dyn_pressure = 0;
    double aoa = 0;


    // accessors for the currently active stage
    Stage& active_stage() { return props.stages[active_idx]; }
    const Stage& active_stage() const { return props.stages[active_idx]; }
    int num_stages() const { return static_cast<int>(props.stages.size()); }

    // rocket explode button
    bool detonated = false;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // helper functions                                                                          //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // starting
    void set_start(double origin_latitude, double origin_longitude, double target_latitude, double target_longitude); // sets the starting and target position/attitude (only called from the constructor
    void set_start_orbit(const OrbitElements& orbit, double target_latitude, double target_longitude); // same as set_start, but starting in orbit
    void set_start_eci(const Vec3& r_eci, const Vec3& v_eci, double target_latitude, double target_longitude); // sets the starting and target for ECI position and velocity
    
    // kinematic helpers
    void apply_ground_dynamics(const Vec3& I, double m_end, double dt);

    // every acceleration (ECI) and every torque about the CoM (body) acting on the rocket
    KinematicModifier kinematic_state(double m_i, const Vec3& r_i, const Vec3& v_i, const Quat& q_i, const Vec3& w_i, const RocketProps& props, double t_burn, double t); // gravity + drag + thrust + rcs
        KinematicModifier calc_drag_kinematics(const Vec3& r, const Vec3& v, const Quat& q, const Vec3& w, double mass, const RocketProps& props);
        Vec3 calc_gravity_accel(const Vec3& r, const double GM, const double J2, const double R, const double t);
        KinematicModifier calc_propulsion_kinematics(const Vec3& r, const Vec3& v, const Quat& q, const Vec3& w, double mass, const RocketProps& props, double t_burn);
        Vec3 calc_rcs_torque() const;
    double fuel_burned(double t0, double t1) const;


    // coordinate system conversion helpers
    Vec3 nose_direction_eci(const Quat& q) const;
    Vec3 lat_lon_to_ecef(double latitude_deg, double longitude_deg);

    // rocket state helpers
    double rocket_body_length() const; // nose to aft end of the remaining stack (m)
    bool is_rocket_on_ground(double com_dist_from_gnd); // snaps the rocket onto the surface if it is touching the ground
};

// standard atmosphere layers (air density/pressure, speed of sound, and dynamic viscosity mu at a given altitude above sea level)
void atmosphere(double altitude, double& air_density, double& air_pressure, double& speed_of_sound, double& mu);
