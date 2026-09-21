#include "fc/fc_bind.hpp"
#include "sim/rocket.hpp"

namespace {
    Rocket* g_rocket = nullptr;
    fc_bind::Commands* g_cmd = nullptr;
}

namespace fc_bind {

void begin(Rocket* r, Commands* c) {
    g_rocket = r;
    g_cmd = c;

    // one shot commands are set false after every step, persistent ones keep their state
    c->light = false;
    c->cutoff = false;
    c->separate = false;
    c->detonate = false;
    c->burn_fraction_set = false;
    c->burn_fraction = 1.0;
}

void end() {
    g_rocket = nullptr;
    g_cmd = nullptr;
}

}

extern "C" {

void fc_light_engine(void) {
    if (g_cmd) g_cmd->light = true;
}

void fc_cutoff_engine(void) {
    if (g_cmd) g_cmd->cutoff = true;
}

void fc_burn_fraction(double fraction) {
    if (!g_cmd) return;
    g_cmd->burn_fraction_set = true;
    g_cmd->burn_fraction = fraction;
}

void fc_separate_stage(void) {
    if (g_cmd) g_cmd->separate = true;
}

void fc_set_gimbal(fc_quat q_engine_in_body) {
    if (g_cmd) g_cmd->gimbal = q_engine_in_body;
}

void fc_rcs_enable(int on) {
    if (g_cmd) g_cmd->rcs_on = on != 0;
}

void fc_rcs_set_moment(fc_vec3 moment_body) {
    if (g_cmd) g_cmd->rcs_moment = moment_body;
}

void fc_detonate(void) {
    if (g_cmd) g_cmd->detonate = true;
}

int fc_active_stage(void) {
    return g_rocket ? g_rocket->active_stage_idx() : 0;
}

} // extern "C"
