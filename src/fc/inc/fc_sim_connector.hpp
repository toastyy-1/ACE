#pragma once

// connects FC with rocket class

#include "fc/inc/fc_api.h"
#include <memory>

class Rocket;

namespace fc_sim_connector {

/**
 * points fc_active_stage at a rocket for the duration of an fc_update call
 * @param r target rocket to answer queries from
 */
void begin(Rocket* r);

// kill
void end();

struct StateDeleter {
    void operator()(fc_state* s) const { fc_free(s); }
};

// owns whatever fc_init handed back
using State = std::unique_ptr<fc_state, StateDeleter>;

}
