#ifndef FC_API_H
#define FC_API_H

/*
 * BALLISTICMD FLIGHT CONTROLLER API
 *
 * this is the only header a flight controller needs to include.
 * 
 * To build with specific flight controller code in mind please run:
 *     make FC_SRC=src/fc/src/fc.c
 *   or if your FC has multiple files (probably)
 *     make FC_SRC="src/fc/src/fc.c src/fc/src/file2.c"
 *
 * the sim calls fc_update once per time step. 
 * 
 * the commands are buffered and applied by the sim after fc_update returns, in a fixed order 
 * (see below), so the order you call them in should not matter.
 *
 */

#ifdef __cplusplus
#include "types.hpp"

typedef Vec3 fc_vec3;
typedef Quat fc_quat;

static_assert(sizeof(fc_vec3) == 3 * sizeof(double), "fc_vec3 must stay layout compatible with C");
static_assert(sizeof(fc_quat) == 4 * sizeof(double), "fc_quat must stay layout compatible with C");

extern "C" {
#else
#include <math.h>

typedef struct fc_vec3 { double x, y, z; } fc_vec3;
typedef struct fc_quat { double w, x, y, z; } fc_quat;
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CONSTANTS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define FC_G0                  9.80665
#define FC_GM_EARTH            3.986004418e14
#define FC_EARTH_RADIUS        6378137.0
#define FC_EARTH_ROTATION_RATE 7.292115e-5
#define FC_J2                  1.08262668355e-3
#define FC_DEG_TO_RAD          0.017453292519943295769
#define FC_RAD_TO_DEG          57.295779513082320876

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// VEHICLE DESCRIPTION
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * stage spec set by the configuration file (sim.yaml). this is the starting state of the rocket and
 * will not be updated constantly. This is simulating you writing your launch vehicle's data 
 * into the flight controller so you dont have to hardcode it. You can use this if you want
 * but you dont have to
 */
typedef struct fc_stage {
    double id;
    double m_dry;                   /* dry mass (kg) */
    double m_fuel;                  /* propellant load at ignition (kg) */
    double isp;                     /* vacuum specific impulse (s) */
    double isp_sea_level;           /* sea level specific impulse (s), 0 if unmodelled */
    double tip_to_end_length;       /* stage length (m) */
    double CoM_dist;                /* CoM from the stage's leading edge, tanks full (m) */
    double fuel_CoM_dist;           /* full propellant column CoM from the leading edge (m) */
    double fuel_length;             /* full propellant column length (m) */
    double max_thrust;              /* rated motor thrust (N) */
    double engine_distance;         /* engine gimbal point from the leading edge (m) */
    double engine_gimbal_range_deg; /* max nozzle deflection off the body axis (degrees) */
    fc_vec3 rcs_max_moment;         /* per axis rcs torque authority (N-m), zero if no rcs */
} fc_stage;

/**
 * rocket spec data access struct
 */
typedef struct fc_vehicle {
    double radius;                  /* hull radius (m) */
    double Cd;                      /* drag coefficient */

    int num_stages;
    const fc_stage* stages;         /* num_stages entries */

    fc_vec3 r_origin_eci;           /* surveyed launch point at t = 0 (m, ECI) */
    fc_quat q_origin_eci;           /* launch attitude, body +z is the nose */
    fc_vec3 r_target_ecef;          /* aim point, earth fixed. its norm is the terrain radius there */

    double time_step;               /* sim step, also the fc_update period (s) */
} fc_vehicle;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SENSOR
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct fc_sensors {
    double t;        // mission time
    double dt;       // since the previous fc_update (s). zero on the first call

    // in body frame
    fc_vec3 a_spec;  // accelerometer
    fc_vec3 w;       // gyro

    // this exists to evaluate the gravity at a current position in the sim
    // you should ignore this and apply your own gravity model based on current position
    // if you want your fc to be real, but you can use this for debug if that's too hard at the moment
    fc_vec3 g;
} fc_sensors;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// COMMANDS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * every call below just records an intent. after fc_update returns the sim applies them
 * in this order, which is what lets you stage and relight in the same step:
 *
 *   1 - fc_burn_fraction, else fc_cutoff_engine
 *   2 - fc_separate_stage
 *   3 - fc_light_engine
 *   4 - fc_detonate
 *   5 - fc_set_gimbal
 *   6 - fc_rcs_enable / fc_rcs_set_moment
 *
 * gimbal and rcs settings persist until you change them. the rest are one shot
 */

/**
 * light the active stage's solid motor
 */
void fc_light_engine(void);

/**
 * kill the active stage's motor (permenant)
 */
void fc_cutoff_engine(void);

/* burn `fraction` of a full step's worth of thrust this step, then cut off permanently.
   this exists so cutoff does not have to land on a step boundary -- it is a sim
   convenience, not something a real vehicle does. fraction is clamped to [0, 1]. */
/**
 * if the fc should burn a partial amount through the full step's worth of thrust
 * for a given time step. use this if the dt is so big that you want sub-step
 * accruracy for burn cutoff (useful for larger dt, but generally nice to use)
 * this is just for sim convenience so dont become too reliant on it as it wouldnt exist
 * for a real flight vehicle (for obvious reasons)
 */
void fc_burn_fraction(double fraction);

/**
 * separate stage and ready new stage for instructions
 */
void fc_separate_stage(void);

/* nozzle orientation relative to the body frame. identity points the nozzle straight aft.
   deflection past the stage's engine_gimbal_range_deg is clamped by the sim. */
/**
 * sets the nozzle orientation relative to the body frame. zeroed quaternion points the
 * nozzle straight down. any defleciton past the stage's max gimbal range is stopped by the sim
 */
void fc_set_gimbal(fc_quat q_engine_in_body);

/**
 * set RCS to be enabled or disabled
 */
void fc_rcs_enable(int on);

/**
 * set amount of torque RCS should apply to the stage (RCS is an experimental feature so it is not in-depth)
 */
void fc_rcs_set_moment(fc_vec3 moment_body);

/**
 * blows the vehicle up on command (aura purposes)
 */
void fc_detonate(void);

/**
 * index of the current active stage
 * use it to index fc_vehicle.stages, or set a fc var to keep track of it every update loop
 */
int fc_active_stage(void);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ENTRY POINTS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * should be called once on the first step of this vehicle's life. 
 */
void* fc_init(const fc_vehicle* vehicle, double t);

/**
 * called once per sim time step, reads the sensors and issues commands to the actual rocket
 */
void fc_update(void* state, const fc_sensors* sensors);

/**
 * called when the vehicle is destroyed to free whatever fc stuff is allocated at the moment
 */
void fc_free(void* state);

#ifdef __cplusplus
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MATH HELPERS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * C++ fc files already have these operators. But if your FC is in C (likely) they exist so you dont
 * have to write all of the helper components by hand to interface with this api (you're welcome)
 */

static inline fc_vec3 fc_v3(double x, double y, double z) {
    fc_vec3 r; r.x = x; r.y = y; r.z = z; return r;
}
static inline fc_vec3 fc_v3_add(fc_vec3 a, fc_vec3 b) {
    return fc_v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline fc_vec3 fc_v3_sub(fc_vec3 a, fc_vec3 b) {
    return fc_v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline fc_vec3 fc_v3_scale(fc_vec3 a, double s) {
    return fc_v3(a.x * s, a.y * s, a.z * s);
}
static inline double fc_v3_dot(fc_vec3 a, fc_vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline fc_vec3 fc_v3_cross(fc_vec3 a, fc_vec3 b) {
    return fc_v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
static inline double fc_v3_norm(fc_vec3 a) {
    return sqrt(fc_v3_dot(a, a));
} // norm vec
static inline fc_vec3 fc_v3_unit(fc_vec3 a) {
    double n = fc_v3_norm(a);
    return n > 0.0 ? fc_v3_scale(a, 1.0 / n) : fc_v3(0.0, 0.0, 0.0);
} // build unit vector from regular vector

static inline fc_quat fc_q(double w, double x, double y, double z) {
    fc_quat q; q.w = w; q.x = x; q.y = y; q.z = z; return q;
}
static inline fc_quat fc_q_mul(fc_quat a, fc_quat b) {
    return fc_q(
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w);
} // quat multiplication
static inline fc_quat fc_q_conj(fc_quat a) { return fc_q(a.w, -a.x, -a.y, -a.z); } // quaternion conjugate
static inline fc_quat fc_q_normalize(fc_quat a) {
    double n = sqrt(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);
    return n > 0.0 ? fc_q(a.w / n, a.x / n, a.y / n, a.z / n) : fc_q(1.0, 0.0, 0.0, 0.0);
} // normalize quaternion

/**
 * rotate vector by quaternion
 */
static inline fc_vec3 fc_q_rotate(fc_quat q, fc_vec3 u) {
    fc_vec3 qv = fc_v3(q.x, q.y, q.z);
    fc_vec3 t = fc_v3_cross(qv, u);
    return fc_v3_add(fc_v3_add(u, fc_v3_scale(t, 2.0 * q.w)), fc_v3_scale(fc_v3_cross(qv, t), 2.0));
}

/**
 * transforms from ECEF to ECI
 */
static inline fc_vec3 fc_ecef_to_eci(fc_vec3 p, double t) {
    double th = FC_EARTH_ROTATION_RATE * t;
    double c = cos(th), s = sin(th);
    return fc_v3(c * p.x - s * p.y, s * p.x + c * p.y, p.z);
}
static inline fc_vec3 fc_eci_to_ecef(fc_vec3 p, double t) { return fc_ecef_to_eci(p, -t); }

/**
 * inertaial v of a point fixed to the spinning earth
 */
static inline fc_vec3 fc_surface_velocity_eci(fc_vec3 r_eci) {
    return fc_v3(-FC_EARTH_ROTATION_RATE * r_eci.y, FC_EARTH_ROTATION_RATE * r_eci.x, 0.0);
}

/**
 * j2 gravity in ECI coords
 */
static inline fc_vec3 fc_gravity_j2(fc_vec3 r) {
    double rn = fc_v3_norm(r);
    if (rn < 1.0) return fc_v3(0.0, 0.0, 0.0);

    double rn_sq = rn * rn;
    double term = FC_GM_EARTH / (rn_sq * rn);
    double zr2 = (r.z * r.z) / rn_sq;
    double k = 1.5 * FC_J2 * (FC_EARTH_RADIUS * FC_EARTH_RADIUS) / rn_sq;

    return fc_v3(
        -term * r.x * (1.0 + k * (1.0 - 5.0 * zr2)),
        -term * r.y * (1.0 + k * (1.0 - 5.0 * zr2)),
        -term * r.z * (1.0 + k * (3.0 - 5.0 * zr2)));
}

// stage helpers
static inline double fc_stage_exhaust_velocity(const fc_stage* s) { return s->isp * FC_G0; }
static inline double fc_stage_max_mass_flow(const fc_stage* s) {
    return s->isp > 0.0 ? s->max_thrust / fc_stage_exhaust_velocity(s) : 0.0;
}
// burn time at full thrust with a full tank
static inline double fc_stage_burn_time(const fc_stage* s) {
    double mdot = fc_stage_max_mass_flow(s);
    return mdot > 0.0 ? s->m_fuel / mdot : 0.0;
}

// a stage's ideal delta v
static inline double fc_stage_delta_v(const fc_stage* s) {
    return s->m_dry > 0.0 ? fc_stage_exhaust_velocity(s) * log((s->m_dry + s->m_fuel) / s->m_dry) : 0.0;
}

#endif
// hey if you're reading this :)