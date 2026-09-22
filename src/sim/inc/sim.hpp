#pragma once
#include <atomic>
#include <functional>
#include <mutex>
#include <vector>
#include "types.hpp"
#include "sim/inc/rocket.hpp"

namespace sim {

    class Sim {
    public:

        Sim();
        ~Sim();
        void Run(std::function<bool()> renderer_ready = {});
        void Stop() { running.store(false); }
        bool is_running() const { return running.load(); }

        std::vector<RocketState> get_state() const {
            std::lock_guard<std::mutex> lk(rocket_states_mutex);
            return rocket_states;
        }

    private:
        void publish_sim_states(const std::vector<Rocket>& rocket_list);

        mutable std::mutex rocket_states_mutex;
        std::vector<RocketState> rocket_states;
        std::vector<RocketState> scratch_states; // sim thread only

        double t = 0;
        std::atomic<bool> running{true};
    };

}
