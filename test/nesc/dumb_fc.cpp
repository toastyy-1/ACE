// Copyright 2026 Tyler Wiggins
// SPDX-License-Identifier: Apache-2.0

// stupid flight controller.  does nothign

#include "fc/inc/fc_api.h"

struct fc_state {};
static fc_state state;

fc_state* fc_init(const fc_vehicle*) { return &state; }
void fc_update(fc_state*, const fc_sensors*, fc_commands*) {}
void fc_free(fc_state*) {}
