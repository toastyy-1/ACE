#include "geometry.hpp"
#include "rmath.hpp"
#include "../constants.hpp"
#include <cmath>

namespace renderer::geom {

namespace {

// Triangle-fan cap on a circle of radius r in the plane z=zc, facing ±Z per
// `nz`. Wound so the visible face matches the outward normal.
void appendCap(Mesh& m, float zc, float r, int sides, float nz, RColor col) {
    uint32_t center = (uint32_t)m.verts.size();
    m.verts.push_back({ {0, 0, zc}, {0, 0, nz}, 0, 0, col });
    uint32_t ring = (uint32_t)m.verts.size();
    for (int j = 0; j <= sides; ++j) {
        float a = (float)(TAU * j / sides);
        m.verts.push_back({ {r*cosf(a), r*sinf(a), zc}, {0, 0, nz}, 0, 0, col });
    }
    for (int j = 0; j < sides; ++j) {
        if (nz < 0) { // faces -Z: reverse winding
            m.idx.push_back(center); m.idx.push_back(ring + j + 1); m.idx.push_back(ring + j);
        } else {       // faces +Z
            m.idx.push_back(center); m.idx.push_back(ring + j); m.idx.push_back(ring + j + 1);
        }
    }
}

} // namespace

void appendFrustum(Mesh& m, float z0, float z1, float r0, float r1,
                   int sides, RColor col, bool cap0, bool cap1) {
    uint32_t base = (uint32_t)m.verts.size();
    float dr = r1 - r0, dz = z1 - z0;

    // Side: two stacked rings, smooth radial normals (with the profile slope).
    for (int j = 0; j <= sides; ++j) {
        float a = (float)(TAU * j / sides), c = cosf(a), s = sinf(a);
        RVec3 nrm = rmath::normalize({ dz*c, dz*s, -dr });
        m.verts.push_back({ {r0*c, r0*s, z0}, nrm, 0, 0, col });   // base + 2j
        m.verts.push_back({ {r1*c, r1*s, z1}, nrm, 0, 0, col });   // base + 2j + 1
    }
    for (int j = 0; j < sides; ++j) {
        uint32_t lo0 = base + 2*j,     hi0 = base + 2*j + 1;
        uint32_t lo1 = base + 2*(j+1), hi1 = base + 2*(j+1) + 1;
        m.idx.push_back(lo0); m.idx.push_back(lo1); m.idx.push_back(hi1);
        m.idx.push_back(lo0); m.idx.push_back(hi1); m.idx.push_back(hi0);
    }

    if (cap0 && r0 > 1e-6f) appendCap(m, z0, r0, sides, dz > 0 ? -1.0f : 1.0f, col);
    if (cap1 && r1 > 1e-6f) appendCap(m, z1, r1, sides, dz > 0 ? 1.0f : -1.0f, col);
}

void appendTriangle(Mesh& m, const RVec3& a, const RVec3& b, const RVec3& c,
                    RColor col, bool doubleSided) {
    RVec3 n = rmath::normalize(rmath::cross(rmath::sub(b, a), rmath::sub(c, a)));
    uint32_t base = (uint32_t)m.verts.size();
    m.verts.push_back({ a, n, 0, 0, col });
    m.verts.push_back({ b, n, 0, 0, col });
    m.verts.push_back({ c, n, 0, 0, col });
    m.idx.push_back(base); m.idx.push_back(base + 1); m.idx.push_back(base + 2);
    if (doubleSided) {
        RVec3 nb = { -n.x, -n.y, -n.z };
        uint32_t b2 = (uint32_t)m.verts.size();
        m.verts.push_back({ a, nb, 0, 0, col });
        m.verts.push_back({ c, nb, 0, 0, col });
        m.verts.push_back({ b, nb, 0, 0, col });
        m.idx.push_back(b2); m.idx.push_back(b2 + 1); m.idx.push_back(b2 + 2);
    }
}

void appendRevolve(Mesh& m, const std::vector<RVec3>& profile,
                   int sides, RColor col, bool capBase) {
    const int rings = (int)profile.size();
    if (rings < 2) return;
    uint32_t base = (uint32_t)m.verts.size();
    const int stride = sides + 1;

    for (int i = 0; i < rings; ++i) {
        float z = profile[i].x, r = profile[i].y;
        // Profile slope for the (radial) normal, central-differenced.
        float dz, dr;
        if (i == 0)            { dz = profile[1].x - profile[0].x;       dr = profile[1].y - profile[0].y; }
        else if (i == rings-1) { dz = profile[i].x - profile[i-1].x;     dr = profile[i].y - profile[i-1].y; }
        else                   { dz = profile[i+1].x - profile[i-1].x;   dr = profile[i+1].y - profile[i-1].y; }
        for (int j = 0; j <= sides; ++j) {
            float a = (float)(TAU * j / sides), c = cosf(a), s = sinf(a);
            RVec3 nrm = rmath::normalize({ dz*c, dz*s, -dr });
            m.verts.push_back({ {r*c, r*s, z}, nrm, 0, 0, col });
        }
    }
    for (int i = 0; i < rings - 1; ++i) {
        for (int j = 0; j < sides; ++j) {
            uint32_t a = base + i*stride + j,     b = a + 1;
            uint32_t c = base + (i+1)*stride + j, d = c + 1;
            m.idx.push_back(a); m.idx.push_back(b); m.idx.push_back(d);
            m.idx.push_back(a); m.idx.push_back(d); m.idx.push_back(c);
        }
    }
    if (capBase && profile[0].y > 1e-6f)
        appendCap(m, profile[0].x, profile[0].y, sides, -1.0f, col);
}

Mesh buildCone(int sides) {
    Mesh m;
    appendFrustum(m, 0.0f, 1.0f, 1.0f, 0.0f, sides, kWhite, /*cap0=*/true, /*cap1=*/false);
    return m;
}

Mesh buildSphere(float radius, int rings, int sectors, float lonOffset) {
    Mesh m;
    for (int i = 0; i <= rings; ++i) {
        float phi = (float)(M_PI/2.0 - M_PI * i / rings);   // +90 (north) -> -90 (south)
        float cp = cosf(phi), sp = sinf(phi);
        for (int j = 0; j <= sectors; ++j) {
            float lam = (float)(TAU * j / sectors);
            RVec3 n = { cp*cosf(lam), cp*sinf(lam), sp };   // ECI, +Z = north pole
            m.verts.push_back({ {n.x*radius, n.y*radius, n.z*radius}, n,
                                (float)j/sectors + lonOffset, (float)i/rings, kWhite });
        }
    }
    int stride = sectors + 1;
    for (int i = 0; i < rings; ++i) {
        for (int j = 0; j < sectors; ++j) {
            uint32_t a = i*stride + j,     b = a + 1;
            uint32_t c = (i+1)*stride + j, d = c + 1;
            // Outward-facing winding (verified against the +X equator).
            m.idx.push_back(a); m.idx.push_back(c); m.idx.push_back(b);
            m.idx.push_back(b); m.idx.push_back(c); m.idx.push_back(d);
        }
    }
    return m;
}

Mesh buildGrid(int n, bool skirt) {
    Mesh m;
    for (int i = 0; i <= n; ++i) {
        float gy = (float)i / n * 2.0f - 1.0f;
        for (int j = 0; j <= n; ++j) {
            float gx = (float)j / n * 2.0f - 1.0f;
            m.verts.push_back({ {gx, gy, 0.0f}, {0,0,1}, 0.0f, 0.0f, kWhite });
        }
    }
    int stride = n + 1;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            uint32_t a = i*stride + j,     b = a + 1;
            uint32_t c = (i+1)*stride + j, d = c + 1;
            m.idx.push_back(a); m.idx.push_back(c); m.idx.push_back(b);
            m.idx.push_back(b); m.idx.push_back(c); m.idx.push_back(d);
        }
    }
    if (!skirt) return m;

    // Border indices in one loop around the grid, then a copy of each at z = 1
    // and a quad strip joining the two rings.
    std::vector<uint32_t> border;
    for (int j = 0; j < n; ++j) border.push_back(j);                          // bottom, left -> right
    for (int i = 0; i < n; ++i) border.push_back(i*stride + n);               // right, bottom -> top
    for (int j = n; j > 0; --j) border.push_back(n*stride + j);               // top, right -> left
    for (int i = n; i > 0; --i) border.push_back(i*stride);                   // left, top -> bottom
    uint32_t ring = (uint32_t)m.verts.size();
    for (uint32_t b : border) {
        Vertex v = m.verts[b];
        v.pos.z = 1.0f;
        m.verts.push_back(v);
    }
    const uint32_t k = (uint32_t)border.size();
    for (uint32_t i = 0; i < k; ++i) {
        uint32_t a = border[i], b = border[(i + 1) % k];
        uint32_t sa = ring + i, sb = ring + (i + 1) % k;
        m.idx.push_back(a); m.idx.push_back(sa); m.idx.push_back(b);
        m.idx.push_back(b); m.idx.push_back(sa); m.idx.push_back(sb);
    }
    return m;
}

} // namespace renderer::geom
