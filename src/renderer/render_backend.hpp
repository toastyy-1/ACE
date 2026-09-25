#pragma once

#include "render_types.hpp"
#include "mesh.hpp"
#include "rmath.hpp"
#include "scene.hpp"

// Universal rendering backend interface.
//
// The rendering logic (camera orbit, rocket geometry, predicted trajectory,
// telemetry layout, all of renderer::Renderer) is written once against this
// interface using backend-neutral types. Concrete backends (raylib or BGFX)
// implement the primitives below and can be swapped in without touching the
// Renderer. Backend-neutral value types live in render_types.hpp
// (RVec3/RColor/...), mesh.hpp (Vertex/Mesh/Material/handles), rmath.hpp
// (RMat4), and the per-frame domain frames in scene.hpp
// (RocketFrame/EarthFrame). The renderer owns the precision-critical ECI->view
// shift, so all transforms reaching a backend are already in view space.

namespace renderer {

class RenderBackend {
public:
    virtual ~RenderBackend() = default;

    // Window / lifecycle.
    virtual void Init(int width, int height, const char* title) = 0;
    virtual void Shutdown() = 0;
    virtual bool ShouldClose() const = 0;

    // Input / timing.
    virtual FrameInput PollInput() = 0;
    virtual float  FrameTime() const = 0;   // seconds since last frame
    virtual double Time() const = 0;         // seconds since Init

    // Resources. Meshes/textures are uploaded once and referenced by handle
    // (0 = invalid / none). Textures get mipmaps + trilinear filtering.
    virtual TextureHandle LoadTexture(const char* path) = 0;
    virtual MeshHandle    CreateMesh(const Mesh& m) = 0;
    virtual void          DestroyMesh(MeshHandle h) = 0;

    // Frame structure.
    virtual void BeginFrame(RColor clear) = 0;
    virtual void EndFrame() = 0;
    // Near/far clip planes. Must be set before Begin3D (they are baked into the
    // projection).
    virtual void SetClipPlanes(float near_plane, float far_plane) = 0;
    virtual void Begin3D(const RCamera& cam) = 0;
    virtual void End3D() = 0;

    // Project a view-space point (as passed to Begin3D/DrawLines) to 2D screen
    // pixels, for placing overlay labels over 3D geometry. Valid after the most
    // recent Begin3D (it uses that camera/viewport). `visible` is false when the
    // point is behind the camera.
    virtual ScreenPoint WorldToScreen(const RVec3& viewPos) const = 0;
    // Current framebuffer size in pixels (for clamping/culling overlay labels).
    virtual int ScreenWidth() const = 0;
    virtual int ScreenHeight() const = 0;

    // A static mesh placed by a model matrix, shaded/blended per Material.
    virtual void DrawModel(MeshHandle h, const RMat4& model, const Material& mat) = 0;
    // A transient line list in view space (pairs of vertices), per-vertex colour.
    virtual void DrawLines(const LineVertex* v, size_t count, float width) = 0;

    // Semantic scene objects (the backend owns the geometry, textures, and
    // shaders and renders the supplied domain frame however it likes).
    virtual void DrawRocket(const RocketFrame& f) = 0;
    virtual void DrawEarth(const EarthFrame& f) = 0;

    // 2D overlay (screen-space, drawn after End3D).
    virtual void DrawRect(int x, int y, int w, int h, RColor c) = 0;
    virtual void DrawRectLines(int x, int y, int w, int h, RColor c) = 0;
    virtual void DrawText(const char* text, int x, int y, int font_size, RColor c) = 0;
    virtual void DrawFPS(int x, int y) = 0;

    // Optional: draw the whole HUD (telemetry, rocket labels, overlay legend)
    // from `hud`, after End3D. Return false to have the renderer draw its
    // default panels through the 2D primitives above instead.
    virtual bool DrawHud(const HudFrame& /*hud*/) { return false; }
};

}
