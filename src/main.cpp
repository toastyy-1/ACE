#include "sim/inc/sim.hpp"

#ifdef HEADLESS // make headless

/**
 * @brief called after every sim step
 * @param t sim time (s)
 * @param rockets every rocket in the sim
 */
static void headless_output(double t, const std::vector<Rocket>& rockets) {
}

int main() {
    sim::Sim s;
    s.Run({}, headless_output);
    return 0;
}

#else

#include "renderer/renderer.hpp"
#ifdef USE_BGFX
#include "renderer/bgfx/bgfx_backend.hpp"
#else
#include "renderer/raylib/raylib_backend.hpp"
#endif
#include <thread>

int main() {
    sim::Sim s;

#ifdef USE_BGFX
    renderer::BgfxBackend backend;   // make bgfx
#else
    renderer::RaylibBackend backend; // make
#endif
    renderer::Renderer r(backend, s);

    std::thread sim_thread([&]{ s.Run([&]{ return r.IsInitialized(); }); });
    r.Run();

    s.Stop();
    sim_thread.join();
    return 0;
}

#endif
