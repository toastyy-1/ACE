#pragma once

#include <bgfx/bgfx.h>
#include <vector>
#include "../render_backend.hpp"
#include "models.hpp"
#include "earth_bump_map.hpp"
#include "terrain.hpp"

struct GLFWwindow;

namespace renderer {

// bgfx backend, forced to the OpenGL renderer. Owns a GLFW window (bgfx draws
// to its native handle), the shader programs, the global Earth maps, and the
// scene objects built on them (RocketModel, Terrain). Geometry/transforms arrive in view space from the
// renderer (it owns the precision-critical ECI->view shift).
class BgfxBackend : public RenderBackend {
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
    void ensureEarth();
    void submit2DTris(const Vertex* v, uint32_t count, RColor tint);  // view 1, no depth
    // 2D text triangles textured by the glyph atlas (view 1, no depth).
    void submitTextTris(const Vertex* v, uint32_t count, RColor tint, bgfx::TextureHandle atlas);

    struct GpuMesh { bgfx::VertexBufferHandle vbh; bgfx::IndexBufferHandle ibh; };

    GLFWwindow* window_ = nullptr;
    int   width_ = 0, height_ = 0;
    uint32_t reset_ = 0;

    // input / timing
    double prevX_ = 0, prevY_ = 0;
    bool   havePrev_ = false;
    bool   prevTab_ = false;           // edge-detect Tab for primary-rocket cycling
    bool   prevDigit_[10] = {};        // edge-detect number keys 1-9 for overlay toggles
    float  wheel_ = 0;                 // accumulated by the scroll callback
    double startTime_ = 0, lastTime_ = 0, frameDt_ = 0;

    bgfx::VertexLayout  layout_;
    bgfx::ProgramHandle generic_ = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle cloudProg_ = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle atmosProg_ = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle flareProg_ = BGFX_INVALID_HANDLE;
    bgfx::ProgramHandle rocketProg_ = BGFX_INVALID_HANDLE;   // PBR + grime (lit DrawModel)
    bgfx::ProgramHandle heatProg_   = BGFX_INVALID_HANDLE;   // additive ablation shell (emissive)
    bgfx::UniformHandle s_tex_ = BGFX_INVALID_HANDLE, u_tint_ = BGFX_INVALID_HANDLE,
                        u_depth_ = BGFX_INVALID_HANDLE, u_light_ = BGFX_INVALID_HANDLE;
    RVec3 sunDirView_ { 0, 0, 1 };   // cached each frame from DrawEarth, for lit DrawModel
    RVec3 camPosView_ { 0, 0, 0 };   // cached camera position, for the PBR view vector
    RVec3 heatDir_    { 0, 0, 1 };   // cached from DrawRocket: travel dir for the heating glow
    float heatAmt_    = 0.0f;        // cached aerodynamic-heating intensity
    RVec3 earthCenterView_ { 0, 0, 0 };   // cached Earth centre, for rocket reflections
    bgfx::UniformHandle s_color_ = BGFX_INVALID_HANDLE, s_night_ = BGFX_INVALID_HANDLE,
                        s_emiss_ = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_sunDir_ = BGFX_INVALID_HANDLE, u_earthCenter_ = BGFX_INVALID_HANDLE,
                        u_camPos_ = BGFX_INVALID_HANDLE,
                        u_heat_ = BGFX_INVALID_HANDLE, u_earth_ = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle s_cloud_ = BGFX_INVALID_HANDLE, u_cloudAlpha_ = BGFX_INVALID_HANDLE,
                        u_cloudDisp_ = BGFX_INVALID_HANDLE, u_atmos_ = BGFX_INVALID_HANDLE;
    bgfx::UniformHandle u_rayFwd_ = BGFX_INVALID_HANDLE, u_rayRight_ = BGFX_INVALID_HANDLE,
                        u_rayUp_ = BGFX_INVALID_HANDLE;
    bgfx::TextureHandle white_ = BGFX_INVALID_HANDLE;

    std::vector<GpuMesh>             meshes_;    // handle = index + 1
    std::vector<bgfx::TextureHandle> textures_;  // handle = index + 1

    // Backend-owned scene objects.
    RocketModel         rocket_;
    Terrain             terrain_;        // the Earth's surface (quadtree LOD)
    MeshHandle          cloudMesh_ = 0;
    bgfx::TextureHandle earthColor_ = BGFX_INVALID_HANDLE,
                        earthNight_ = BGFX_INVALID_HANDLE,
                        earthCloud_ = BGFX_INVALID_HANDLE,
                        earthRough_ = BGFX_INVALID_HANDLE,
                        earthEmiss_ = BGFX_INVALID_HANDLE;

    // 3D view (view 0) state.
    float near_ = 0.01f, far_ = 1000.0f;
    RCamera cam_{};

    double fps_ = 0.0;   // smoothed
};

}
