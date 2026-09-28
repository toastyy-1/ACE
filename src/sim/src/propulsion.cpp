/**
 * handles all propulsion for rocket.cpp
 */

#include "sim/inc/rocket.hpp"
#include <algorithm>

///////////////////////////////////////////////////////////////////////////////////////////////
// thrust curve                                                                              //
///////////////////////////////////////////////////////////////////////////////////////////////
/**
 * thrust at a time since ignition, linear-ly inerpolate between values
 * @param time time since ignition (s)
 * @return thrust (N), 0 outside the curv
 */
double ThrustCurve::thrust_at(double time) const {
    if (t.empty() || time < t.front() || time > t.back()) return 0.0;

    // first sample after time
    size_t i = std::upper_bound(t.begin(), t.end(), time) - t.begin();
    if (i == t.size()) return F.back();

    double frac = (time - t[i - 1]) / (t[i] - t[i - 1]);
    return F[i - 1] + frac * (F[i] - F[i - 1]);
}

/**
 * impulse the curve delivers between two times since ignition
 * @param t0 start time since ignition (s)
 * @param t1 end time since ignition (s)
 * @return impulse (N-s)
 */
double ThrustCurve::impulse(double t0, double t1) const {

}

/**
 * @return highest thrust on the curve (N)
 */
double ThrustCurve::peak() const {

}

///////////////////////////////////////////////////////////////////////////////////////////////
// helper functions                                                                          //
///////////////////////////////////////////////////////////////////////////////////////////////
/**
 * gimbaled thrust vector in the body frame
 * @param thrust_force thrust magnitude (N)
 * @param q_engine nozzle orientation relative to the body
 * @return thrust force vector in the body frame, pointing along the nozzle axis after gimbal
 */
static Vec3 prop_thrust_vector(double thrust_force, const Quat& q_engine) {
    Vec3 nose_body = {0, 0, 1};
    return rotate_by_quat(q_engine, nose_body) * thrust_force;
}

/**
 * torque the nozzle thrust applies about the combined CoM in body frame
 * @param thrust_body thrust force vector in the body frame
 * @param stage the burning stage
 * @param z_cm combined CoM along body +z, from the active stage aft edge 
 * @return engine torque about the CoM 
 */
static Vec3 propulsion_torque(const Vec3& thrust_body, const Stage& stage, double z_cm) {
    double s_engine = stage.tip_to_end_length - stage.engine_distance;
    Vec3 r_engine = {0, 0, s_engine - z_cm};

    return r_engine.cross(thrust_body);
}

/**
 * thrust acceleration and engine torque from the active stage's thrust curve
 * @param r position in ECI
 * @param v velocity in ECI
 * @param q body orientation
 * @param w angular velocity, body frame
 * @param mass current total mass of the rocket
 * @param props rocket geometry
 * @param t_burn time since the active stage ignition to sample the thrust curve at
 * @return thrust acceleration in ECI
 */
KinematicModifier Rocket::calc_propulsion_kinematics(const Vec3& r, const Vec3& v, const Quat& q, const Vec3& w, double mass, const RocketProps& props, double t_burn) {
    const Stage& stage = props.stages[active_idx];

    // find where we are in the thrust curve so we know how much thrust to apply
    double thrust_force = throttle * stage.thrust_curve.thrust_at(t_burn);

    // calculate the accel vector the engine applies to the body
    Vec3 thrust_body = prop_thrust_vector(thrust_force, q_engine);
    thrust_accel = rotate_by_quat(q, thrust_body) / mass;

    // apply propulsive torque
    return {thrust_accel, propulsion_torque(thrust_body, stage, z_cm)};
}

/**
 * propellant the active stage burns between two times since ignition
 * @param t0 start time since ignition (s)
 * @param t1 end time since ignition (s)
 * @return propellant mass burned (kg), never more than what's left in the tank
 */
double Rocket::fuel_burned(double t0, double t1) const {
    const Stage& stage = active_stage();
    double exhaust_velocity = stage.exhaust_velocity();
    if (throttle <= 0 || exhaust_velocity <= 0) return 0.0;

    return std::min(stage.m_fuel, throttle * stage.thrust_curve.impulse(t0, t1) / exhaust_velocity);
}
