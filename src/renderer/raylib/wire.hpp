#pragma once

#include "../mesh.hpp"
#include "../rmath.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

// The raylib backend's 3D pipeline: everything is drawn as lines. A solid mesh
// is reduced to its outline (FeatureEdges) and drawn "hidden-line": its
// triangles are first filled in the background colour, pushed slightly back in
// depth, so whatever is behind the surface is hidden and only its visible edges
// show. Transparent meshes (plume, explosion) draw just their edges.
//
// Resources and state go through rlgl; the only raw GL calls are the few GL 1.1
// entry points rlgl doesn't wrap (line draws, 32-bit index draws, polygon
// offset), which every platform's GL library exports. wire.cpp must not pull in
// raylib.h: on Windows the GL header needs windows.h, which clashes with it.

namespace renderer::wire {

// A mesh on the GPU. One vertex buffer (the Vertex layout from mesh.hpp) and one
// index buffer holding the fill triangles followed by the edge lines.
struct GpuMesh {
    unsigned vao = 0, vbo = 0, ebo = 0;
    int  tris  = 0;           // triangle index count, at the start of ebo
    int  lines = 0;           // line index count, right after the triangles
    bool ownsEbo = true;      // false when ebo is shared (the Earth chunks)
};

GpuMesh Upload(const std::vector<Vertex>& verts, const std::vector<uint32_t>& tris,
               const std::vector<uint32_t>& lines);
// Upload vertices that index into an existing buffer from UploadIndices().
GpuMesh UploadShared(const std::vector<Vertex>& verts, unsigned ebo, int tris, int lines);
unsigned UploadIndices(const std::vector<uint32_t>& tris, const std::vector<uint32_t>& lines);
void     Destroy(GpuMesh& m);
void     DestroyIndices(unsigned ebo);

// The outline of a triangle mesh, as pairs of vertex indices: open borders and
// creases, plus edges between flat triangles unless they are the diagonal that
// splits a quad (so a flat cap shows its rim, not its fan).
std::vector<uint32_t> FeatureEdges(const Mesh& m);

// Per-draw colouring for Edges().
struct Shading {
    RColor tint = kWhite;       // multiplies the vertex colour
    // Aerodynamic heating: edges whose normal faces `heatDir` (the direction of
    // travel, view space) glow by `heat` in [0, 1]. 0 = off.
    RVec3  heatDir { 0, 0, 1 };
    float  heat = 0.0f;
};

class Pipeline {
public:
    void Init();              // after the window (GL context) exists
    void Shutdown();

    // Per frame, inside BeginMode3D: the camera's projection * view.
    void Begin(const RMat4& viewProj, RColor background);
    // Unbind everything so raylib's own drawing starts clean. Call before EndMode3D.
    void End();
    // Rotate the streamed line buffers. Call once per frame after the swap.
    void NextFrame();

    // Hidden-line occluder: the mesh's triangles in the background colour.
    void Fill(const GpuMesh& m, const RMat4& model);
    // The mesh's outline. Blend mode and depth writes are left to the caller.
    void Edges(const GpuMesh& m, const RMat4& model, const Shading& s);
    // Transient view-space line list (pairs), streamed to the GPU.
    void Lines(const LineVertex* v, size_t count);

private:
    enum class Mode { None, Fill, Edges, Lines };
    void use(Mode m);
    void setMatrices(const RMat4& model);
    void setShading(const Shading& s, bool fill);

    // Streamed lines go to a different buffer each frame, so a frame never
    // writes into one the GPU may still be reading.
    static constexpr int kStreamSlots = 3;
    struct Stream { unsigned vao = 0, vbo = 0; size_t capacity = 0, used = 0; };
    Stream streams_[kStreamSlots];
    int    slot_ = 0;

    unsigned prog_ = 0;
    int locMvp_ = -1, locModel_ = -1, locTint_ = -1, locFill_ = -1,
        locHeat_ = -1;

    Mode  mode_ = Mode::None;
    RMat4 viewProj_ = rmath::identity();
    float bg_[4] = { 0, 0, 0, 1 };
    // Last uniform values sent, so unchanged ones aren't re-sent every draw.
    float tint_[4] = {}, fill_[4] = {}, heat_[4] = {};
};

}
