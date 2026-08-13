#pragma once

// connects FC with rocket class

#include "fc/fc_api.h"

class Rocket;

namespace fc_bind {

/**
 * what the controller asked for. one shot flags are cleared every step, persistent ones are not!
 */
struct Commands {
    bool light = false;
    bool cutoff = false;
    bool separate = false;
    bool detonate = false;
    bool burn_fraction_set = false;
    double burn_fraction = 1.0;

    Quat gimbal{1, 0, 0, 0};
    bool rcs_on = false;
    Vec3 rcs_moment{0, 0, 0};
};

/**
 * @param r target rocket to point commands at
 * @param c target command functions
 */
void begin(Rocket* r, Commands* c);

// kill
void end();

struct State {
    void* p = nullptr;

    State() = default;
    ~State() { reset(); }

    State(State&& o) noexcept : p(o.p) { o.p = nullptr; }
    State& operator=(State&& o) noexcept {
        if (this != &o) { reset(); p = o.p; o.p = nullptr; }
        return *this;
    }
    State(const State&) = delete;
    State& operator=(const State&) = delete;

    void reset() {
        if (p) { fc_free(p); p = nullptr; }
    }
};

}
