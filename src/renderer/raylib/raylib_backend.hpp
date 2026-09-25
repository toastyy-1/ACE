#pragma once
#include <raylib.h>
#include <vector>
#include "../render_backend.hpp"
#include "earth.hpp"
#include "models.hpp"
#include "wire.hpp"

namespace renderer {

// Lightweight raylib backend (`make`): no bgfx, no textures, no shaders beyond
// one tiny wireframe program. Every solid is drawn as its hidden-line outline
// (wire.hpp), the Earth as an LOD grid over a smooth sphere (earth.hpp), and
// the overlays the renderer submits as lines are streamed straight to the GPU.
class RaylibBackend : public RenderBackend {
public:
    void Init(int width, int height, const char* title) override;
    void Shutdown() override;
    bool ShouldClose() const override;

    FrameInput PollInput() override;
    float  FrameTime() const override;
    double Time() const override;

    TextureHandle LoadTexture(const char* path) override;
    MeshHandle    CreateMesh(const Mesh& m) override;
    void          DestroyMesh(MeshHandle h) override;

    void BeginFrame(RColor clear) override;
    void EndFrame() override;
    void SetClipPlanes(float near_plane, float far_plane) override;
    void Begin3D(const RCamera& cam) override;
    void End3D() override;

    ScreenPoint WorldToScreen(const RVec3& viewPos) const override;
    int ScreenWidth() const override;
    int ScreenHeight() const override;

    void DrawModel(MeshHandle h, const RMat4& model, const Material& mat) override;
    void DrawLines(const LineVertex* v, size_t count, float width) override;
    void DrawRocket(const RocketFrame& f) override;
    void DrawEarth(const EarthFrame& f) override;

    void DrawRect(int x, int y, int w, int h, RColor c) override;
    void DrawRectLines(int x, int y, int w, int h, RColor c) override;
    void DrawText(const char* text, int x, int y, int font_size, RColor c) override;
    void DrawFPS(int x, int y) override;

private:
    wire::Pipeline             wire_;
    std::vector<wire::GpuMesh> meshes_;     // handle = index + 1
    std::vector<::Texture2D>   textures_;   // handle = index + 1 (unused by this backend's scene)

    RColor clear_ = kBlack;   // background, which is also the hidden-line fill colour

    // Aerodynamic heating for the rocket being drawn, applied to lit meshes
    // (set by DrawRocket around its hull draws).
    RVec3 heatDir_ { 0, 0, 1 };
    float heat_ = 0.0f;

    // Greek-capable font for overlay text (Font ID labels + telemetry). Falls
    // back to raylib's built-in ASCII font if the TTF fails to load.
    ::Font font_{};
    bool   haveFont_ = false;

    // Last camera handed to Begin3D, kept for the Earth's LOD and WorldToScreen.
    RCamera  cam_{};
    Camera3D cam3d_{};

    // Backend-owned scene objects.
    WireEarth   earth_;
    RocketModel rocket_;
};

}
