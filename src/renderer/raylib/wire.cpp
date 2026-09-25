#include "wire.hpp"

#include <rlgl.h>

#if defined(_WIN32)
#  define WIN32_LEAN_AND_MEAN
#  define NOMINMAX
#  include <windows.h>
#  include <GL/gl.h>
#elif defined(__APPLE__)
#  define GL_SILENCE_DEPRECATION
#  include <OpenGL/gl3.h>
#else
#  include <GL/gl.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <unordered_map>

namespace renderer::wire {

namespace {

constexpr int kPos    = RL_DEFAULT_SHADER_ATTRIB_LOCATION_POSITION;
constexpr int kNormal = RL_DEFAULT_SHADER_ATTRIB_LOCATION_NORMAL;
constexpr int kColor  = RL_DEFAULT_SHADER_ATTRIB_LOCATION_COLOR;

static_assert(sizeof(LineVertex) == 16, "LineVertex is streamed to the GPU as is");

const char* kVS = R"(#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec4 vertexColor;
uniform mat4 mvp;
uniform mat4 matModel;
uniform vec4 tint;
uniform vec4 fillColor;   // a > 0: hidden-line fill, drawn in exactly this colour
uniform vec4 heat;        // xyz: unit direction of travel (view), w: heating [0,1]
out vec4 fragColor;
void main() {
    gl_Position = mvp*vec4(vertexPosition, 1.0);
    if (fillColor.a > 0.0) { fragColor = fillColor; return; }

    vec4 c = vertexColor*tint;
    if (heat.w > 0.0) {
        // Windward edges run from orange to near white as heating builds.
        vec3  n    = normalize(mat3(matModel)*vertexNormal);
        float glow = clamp(heat.w*max(dot(n, heat.xyz), 0.0)*1.5, 0.0, 1.0);
        c.rgb = mix(c.rgb, mix(vec3(1.0, 0.45, 0.1), vec3(1.0, 0.9, 0.7), glow*glow), glow);
    }
    fragColor = c;
}
)";

const char* kFS = R"(#version 330
in vec4 fragColor;
out vec4 finalColor;
void main() { finalColor = fragColor; }
)";

// RMat4 and rlgl's Matrix are both column-major, but Matrix's fields are laid out
// row by row, so copy by name (m{k} is column-major element k), not memcpy.
Matrix toMatrix(const RMat4& s) {
    Matrix r;
    r.m0  = s.m[0];  r.m1  = s.m[1];  r.m2  = s.m[2];  r.m3  = s.m[3];
    r.m4  = s.m[4];  r.m5  = s.m[5];  r.m6  = s.m[6];  r.m7  = s.m[7];
    r.m8  = s.m[8];  r.m9  = s.m[9];  r.m10 = s.m[10]; r.m11 = s.m[11];
    r.m12 = s.m[12]; r.m13 = s.m[13]; r.m14 = s.m[14]; r.m15 = s.m[15];
    return r;
}

// Vertex attributes for the Vertex layout, on the bound VAO + vertex buffer.
void vertexLayout() {
    const int stride = (int)sizeof(Vertex);
    rlSetVertexAttribute(kPos,    3, RL_FLOAT,         false, stride, (int)offsetof(Vertex, pos));
    rlSetVertexAttribute(kNormal, 3, RL_FLOAT,         false, stride, (int)offsetof(Vertex, normal));
    rlSetVertexAttribute(kColor,  4, RL_UNSIGNED_BYTE, true,  stride, (int)offsetof(Vertex, color));
    rlEnableVertexAttribute(kPos);
    rlEnableVertexAttribute(kNormal);
    rlEnableVertexAttribute(kColor);
}

// Update a cached uniform only when it changed.
void setVec4(int loc, float (&cache)[4], float x, float y, float z, float w) {
    if (cache[0] == x && cache[1] == y && cache[2] == z && cache[3] == w) return;
    cache[0] = x; cache[1] = y; cache[2] = z; cache[3] = w;
    rlSetUniform(loc, cache, RL_SHADER_UNIFORM_VEC4, 1);
}

} // namespace

// --- meshes ------------------------------------------------------------------

unsigned UploadIndices(const std::vector<uint32_t>& tris, const std::vector<uint32_t>& lines) {
    std::vector<uint32_t> all;
    all.reserve(tris.size() + lines.size());
    all.insert(all.end(), tris.begin(), tris.end());
    all.insert(all.end(), lines.begin(), lines.end());
    rlDisableVertexArray();   // don't attach this buffer to whatever VAO is bound
    unsigned ebo = rlLoadVertexBufferElement(all.data(), (int)(all.size() * sizeof(uint32_t)), false);
    rlDisableVertexBufferElement();
    return ebo;
}

GpuMesh UploadShared(const std::vector<Vertex>& verts, unsigned ebo, int tris, int lines) {
    GpuMesh m;
    m.vao = rlLoadVertexArray();
    rlEnableVertexArray(m.vao);
    m.vbo = rlLoadVertexBuffer(verts.data(), (int)(verts.size() * sizeof(Vertex)), false);
    vertexLayout();
    rlEnableVertexBufferElement(ebo);
    rlDisableVertexArray();
    m.ebo = ebo;
    m.tris = tris;
    m.lines = lines;
    m.ownsEbo = false;
    return m;
}

GpuMesh Upload(const std::vector<Vertex>& verts, const std::vector<uint32_t>& tris,
               const std::vector<uint32_t>& lines) {
    GpuMesh m = UploadShared(verts, UploadIndices(tris, lines), (int)tris.size(), (int)lines.size());
    m.ownsEbo = true;
    return m;
}

void Destroy(GpuMesh& m) {
    if (m.vao) rlUnloadVertexArray(m.vao);
    if (m.vbo) rlUnloadVertexBuffer(m.vbo);
    if (m.ebo && m.ownsEbo) rlUnloadVertexBuffer(m.ebo);
    m = GpuMesh{};
}

void DestroyIndices(unsigned ebo) {
    if (ebo) rlUnloadVertexBuffer(ebo);
}

std::vector<uint32_t> FeatureEdges(const Mesh& m) {
    const size_t nv = m.verts.size();
    const size_t nt = m.idx.size() / 3;

    // Weld by position: the builders duplicate vertices along seams and wherever
    // normals split, and those copies must count as one corner. Snap to a grid
    // far finer than any feature (1e-5 of the mesh's extent).
    float extent = 0.0f;
    for (const Vertex& v : m.verts)
        extent = std::max({ extent, std::fabs(v.pos.x), std::fabs(v.pos.y), std::fabs(v.pos.z) });
    const double snap = extent > 0.0f ? extent * 1e-5 : 1.0;
    std::unordered_map<uint64_t, uint32_t> byPos;
    std::vector<uint32_t> weld(nv);
    for (size_t i = 0; i < nv; ++i) {
        const RVec3& p = m.verts[i].pos;
        auto q = [&](float c) { return (uint64_t)(std::llround(c / snap) + (1 << 20)) & 0x1FFFFF; };
        uint64_t key = q(p.x) | q(p.y) << 21 | q(p.z) << 42;
        weld[i] = byPos.emplace(key, (uint32_t)byPos.size()).first->second;
    }

    // Face normals; zero-area triangles (sphere poles, cone tips) are skipped.
    std::vector<RVec3> normal(nt);
    std::vector<bool>  valid(nt);
    for (size_t t = 0; t < nt; ++t) {
        const RVec3& a = m.verts[m.idx[3*t]].pos;
        const RVec3& b = m.verts[m.idx[3*t + 1]].pos;
        const RVec3& c = m.verts[m.idx[3*t + 2]].pos;
        RVec3 n = rmath::cross(rmath::sub(b, a), rmath::sub(c, a));
        float len = rmath::length(n);
        valid[t]  = len > extent * extent * 1e-10f;
        normal[t] = valid[t] ? RVec3{ n.x/len, n.y/len, n.z/len } : RVec3{ 0, 0, 0 };
    }

    struct Edge { uint32_t a, b; int faces; uint32_t f0, f1; };
    std::unordered_map<uint64_t, Edge> edges;
    edges.reserve(nt * 2);
    for (size_t t = 0; t < nt; ++t) {
        if (!valid[t]) continue;
        for (int k = 0; k < 3; ++k) {
            uint32_t a = m.idx[3*t + k], b = m.idx[3*t + (k + 1) % 3];
            uint32_t wa = weld[a], wb = weld[b];
            if (wa == wb) continue;
            uint64_t key = (uint64_t)std::min(wa, wb) << 32 | std::max(wa, wb);
            auto [it, fresh] = edges.try_emplace(key, Edge{ a, b, 1, (uint32_t)t, 0 });
            if (!fresh && ++it->second.faces == 2) it->second.f1 = (uint32_t)t;
        }
    }

    // Is the edge (a, b) the longest side of triangle t (ties included)?
    auto longest = [&](uint32_t t, uint32_t a, uint32_t b) {
        auto len2 = [&](uint32_t i, uint32_t j) {
            RVec3 d = rmath::sub(m.verts[i].pos, m.verts[j].pos);
            return rmath::dot(d, d);
        };
        float e = len2(a, b);
        uint32_t i0 = m.idx[3*t], i1 = m.idx[3*t + 1], i2 = m.idx[3*t + 2];
        return e >= std::max({ len2(i0, i1), len2(i1, i2), len2(i2, i0) }) * (1.0f - 1e-4f);
    };

    const float kFlat = 0.99995f;   // cos(~0.6 deg): anything sharper is a crease
    std::vector<uint32_t> out;
    for (const auto& [key, e] : edges) {
        bool keep = true;
        if (e.faces == 2 && rmath::dot(normal[e.f0], normal[e.f1]) > kFlat)
            keep = !(longest(e.f0, e.a, e.b) && longest(e.f1, e.a, e.b));
        if (keep) { out.push_back(e.a); out.push_back(e.b); }
    }
    return out;
}

// --- pipeline ----------------------------------------------------------------

void Pipeline::Init() {
    prog_ = rlLoadShaderCode(kVS, kFS);
    if (prog_ == 0) std::fprintf(stderr, "raylib: wireframe shader failed to compile\n");
    locMvp_   = rlGetLocationUniform(prog_, "mvp");
    locModel_ = rlGetLocationUniform(prog_, "matModel");
    locTint_  = rlGetLocationUniform(prog_, "tint");
    locFill_  = rlGetLocationUniform(prog_, "fillColor");
    locHeat_  = rlGetLocationUniform(prog_, "heat");

    // Start the uniform caches off at "unset" so the first draw sends them.
    for (float* c : { tint_, fill_, heat_ }) std::fill(c, c + 4, NAN);
}

void Pipeline::Shutdown() {
    for (Stream& s : streams_) {
        if (s.vao) rlUnloadVertexArray(s.vao);
        if (s.vbo) rlUnloadVertexBuffer(s.vbo);
        s = Stream{};
    }
    if (prog_) rlUnloadShaderProgram(prog_);
    prog_ = 0;
}

void Pipeline::Begin(const RMat4& viewProj, RColor background) {
    viewProj_ = viewProj;
    bg_[0] = background.r / 255.0f;
    bg_[1] = background.g / 255.0f;
    bg_[2] = background.b / 255.0f;
    bg_[3] = 1.0f;   // alpha doubles as the shader's "this is a fill" flag
}

void Pipeline::End() {
    use(Mode::None);
    rlDisableVertexArray();
    rlDisableShader();
}

void Pipeline::NextFrame() {
    slot_ = (slot_ + 1) % kStreamSlots;
    streams_[slot_].used = 0;
}

void Pipeline::use(Mode m) {
    if (mode_ == m) return;
    // The fill draws two-sided (open shapes like the nozzle still hide what's
    // behind them), pushed back so the edges lying on it pass the depth test.
    if (mode_ == Mode::Fill) {
        glDisable(GL_POLYGON_OFFSET_FILL);
        rlEnableBackfaceCulling();
    }
    if (m == Mode::Fill) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0f, 1.0f);
        rlDisableBackfaceCulling();
    }
    mode_ = m;
}

void Pipeline::setMatrices(const RMat4& model) {
    rlSetUniformMatrix(locMvp_, toMatrix(rmath::mul(viewProj_, model)));
    rlSetUniformMatrix(locModel_, toMatrix(model));
}

void Pipeline::setShading(const Shading& s, bool fill) {
    if (fill) {
        setVec4(locFill_, fill_, bg_[0], bg_[1], bg_[2], bg_[3]);
        return;
    }
    setVec4(locFill_, fill_, 0, 0, 0, 0);
    setVec4(locTint_, tint_, s.tint.r / 255.0f, s.tint.g / 255.0f, s.tint.b / 255.0f, s.tint.a / 255.0f);
    setVec4(locHeat_, heat_, s.heatDir.x, s.heatDir.y, s.heatDir.z, s.heat);
}

void Pipeline::Fill(const GpuMesh& m, const RMat4& model) {
    if (!m.vao || m.tris == 0) return;
    use(Mode::Fill);
    rlEnableShader(prog_);
    setMatrices(model);
    setShading(Shading{}, true);
    rlEnableVertexArray(m.vao);
    glDrawElements(GL_TRIANGLES, m.tris, GL_UNSIGNED_INT, nullptr);
}

void Pipeline::Edges(const GpuMesh& m, const RMat4& model, const Shading& s) {
    if (!m.vao || m.lines == 0) return;
    use(Mode::Edges);
    rlEnableShader(prog_);
    setMatrices(model);
    setShading(s, false);
    rlEnableVertexArray(m.vao);
    glDrawElements(GL_LINES, m.lines, GL_UNSIGNED_INT, (const void*)(m.tris * sizeof(uint32_t)));
}

void Pipeline::Lines(const LineVertex* v, size_t count) {
    count &= ~(size_t)1;   // whole segments only
    if (count == 0) return;

    Stream& s = streams_[slot_];
    if (s.used + count > s.capacity) {
        // Grow into a fresh buffer. The old one may still be read by draws already
        // issued this frame; GL keeps it alive until they finish.
        size_t cap = std::max({ s.capacity * 2, s.used + count, (size_t)65536 });
        if (!s.vao) s.vao = rlLoadVertexArray();
        if (s.vbo) rlUnloadVertexBuffer(s.vbo);
        rlEnableVertexArray(s.vao);
        s.vbo = rlLoadVertexBuffer(nullptr, (int)(cap * sizeof(LineVertex)), true);
        const int stride = (int)sizeof(LineVertex);
        rlSetVertexAttribute(kPos,   3, RL_FLOAT,         false, stride, (int)offsetof(LineVertex, pos));
        rlSetVertexAttribute(kColor, 4, RL_UNSIGNED_BYTE, true,  stride, (int)offsetof(LineVertex, color));
        rlEnableVertexAttribute(kPos);
        rlEnableVertexAttribute(kColor);
        rlDisableVertexArray();
        s.capacity = cap;
        s.used = 0;
    }
    rlUpdateVertexBuffer(s.vbo, v, (int)(count * sizeof(LineVertex)), (int)(s.used * sizeof(LineVertex)));

    use(Mode::Lines);
    rlEnableShader(prog_);
    setMatrices(rmath::identity());
    setShading(Shading{}, false);
    rlEnableVertexArray(s.vao);
    glDrawArrays(GL_LINES, (GLint)s.used, (GLsizei)count);
    s.used += count;
}

}
