#include "fc/inc/fc_sim_connector.hpp"
#include "sim/inc/rocket.hpp"

namespace {
    Rocket* g_rocket = nullptr;
}

namespace fc_sim_connector {

void begin(Rocket* r) {
    g_rocket = r;
}

void end() {
    g_rocket = nullptr;
}

}

extern "C" {

int fc_active_stage(void) {
    return g_rocket ? g_rocket->active_stage_idx() : 0;
}

} // extern "C"
