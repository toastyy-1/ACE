#include "renderer/renderer.hpp"
#ifdef USE_BGFX
#include "renderer/bgfx/bgfx_backend.hpp"
#else
#include "renderer/raylib/raylib_backend.hpp"
#endif
#include "sim/sim.hpp"
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