#include "sim/inc/sim.hpp"
#include "sim/inc/rocket.hpp"
#include "sim/inc/config.hpp"
#include "types.hpp"
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>
#include "constants.hpp"


namespace sim {

    /**
     * @brief initial constsructor for the sim, starts time at 0
     */
    Sim::Sim() {
        t = 0.0; // start at time 0
    }
    /**
     * @brief destructor for the sim
     */
    Sim::~Sim() {}

    /**
     * @brief runs the simulation. everything. literally the entire simulation is run in this one function! it drives the time step
     * @param renderer_ready true if the renderer is ready and loaded and can let the sim start!
     */
    void Sim::Run(std::function<bool()> renderer_ready) {
        // wait for the renderer to load terrain before placing rockets
        while (running.load() && renderer_ready && !renderer_ready()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        // array that holds all the rockets
        std::vector<Rocket> rocket_list = {};

        ///////////////////////////////////////////////////////////////////////////////////////////////
        // rocket placement process                                                                  //
        ///////////////////////////////////////////////////////////////////////////////////////////////
        SimConfig config = load_sim_config("config/sim.yaml");
        TIME_STEP = config.time_step;
        STEP_DELAY = config.step_delay;

        for (const RocketEntry& rocket : config.rockets) {
            rocket_list.emplace_back(rocket.name, rocket.origin_lat, rocket.origin_lon, rocket.target_lat, rocket.target_lon,
                                     rocket.props, rocket.track_data, rocket.export_interval);
        }

        // configure the rocket for starting settings
        publish_sim_states(rocket_list);

        // snapshots for renderer when its ready
        using wall_clock = std::chrono::steady_clock;
        wall_clock::time_point next_publish = wall_clock::now();

        while (running.load()) {

            ///////////////////////////////////////////////////////////////////////////////////////////////
            // sim                                                                                       //
            ///////////////////////////////////////////////////////////////////////////////////////////////

            /*
                !!! NOTE !!! THE ROCKET SHOULD NOT BE CONTROLLED FROM HERE AT ALL ASSUMING THE FC IS ACTIVE
            */

            // for each rocket, perform sim calculations individually for each one through the list until every one is updated independently of one another
            for (size_t i = 0; i < rocket_list.size(); i++) {
                Rocket& r = rocket_list[i];
                // lets the flight controller process data to send commands to the rocket
                r.update_flight_controller(t);

                // update the position, orientation, and mass of the rocket
                r.update_mass();
                r.update_dynamics(t);

                // delete rocket if it exploded
                if (r.is_detonated()) {
                    r.life_countdown -= TIME_STEP;
                    if (r.life_countdown <= 0) {
                        rocket_list.erase(rocket_list.begin() + i); // r dangles after this
                        i--;
                    }
                }
            }

            // increment time step
            t += TIME_STEP;

            // make snapshot for other threads of sim states
            if (wall_clock::now() >= next_publish) {
                publish_sim_states(rocket_list);
                next_publish = wall_clock::now() + std::chrono::milliseconds(10);
            }

            // delay sim a bit so the renderer has something to show
            std::this_thread::sleep_for(std::chrono::duration<double>(STEP_DELAY));
        }

        publish_sim_states(rocket_list); // final states
    }

    /**
     * @brief publishes the current states of all objects in the sim to the renderer and other such things that might use it
     * @param rocket_list
     */
    void Sim::publish_sim_states(const std::vector<Rocket>& rocket_list) {
        scratch_states.clear();
        scratch_states.reserve(rocket_list.size());

        for (size_t i = 0; i < rocket_list.size(); i++) {
            RocketState s = rocket_list[i].get_state();
            s.t = t;
            scratch_states.push_back(s);
        }

        std::lock_guard<std::mutex> lk(rocket_states_mutex);
        std::swap(rocket_states, scratch_states);
    }



}
