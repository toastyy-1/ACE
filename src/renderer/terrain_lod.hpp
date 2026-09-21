#pragma once

#include "../types.hpp"
#include "../constants.hpp"
#include <vector>

// Backend-neutral chunked LOD for a planet surface (KSP "PQS" style). The sphere
// is a cube whose six faces are recursively quartered near the camera and left
// coarse far away, so vertex density follows view distance all the way from orbit
// down to a few metres off the ground. Pure CPU, double precision; a backend
// draws each selected TerrainChunk as one instance of a shared grid mesh (see
// bgfx/terrain.cpp + vs_terrain.sc).
//
// Faces use the equal-angle (tangent-warped) cube map: face angle a in
// [-pi/4, pi/4] maps to the cube plane at tan(a), which keeps chunks within
// ~1.4x of each other in size across a face (a plain cube varies ~5x). Chunks
// are described by their centre angles and half-size so a shader can build
// vertex offsets from the centre without precision loss.
//
// Everything here is in the planet's body frame (ECEF: metres, +Z = north).

namespace renderer {

struct TerrainChunk {
    int    face;        // cube face 0..5 (see TerrainLod::FaceBasis)
    int    level;       // quadtree depth (0 = a whole face)
    double a, b;        // centre angles along the face's A / B axes (rad)
    double half;        // half-size of the chunk, in face angle (rad)
    Vec3   cube;        // unnormalised cube point at the centre: F + A tan(a) + B tan(b)
    Vec3   dir;         // unit direction to the centre
    double edge_m;      // edge length at sea level (m)
    double distance_m;  // camera to the chunk's bounding sphere (m); 0 if inside it
};

struct TerrainLodCamera {
    Vec3   pos;         // camera position relative to the planet centre (m)
    Vec3   fwd;         // unit view direction
    double cone_half;   // half-angle (rad) of a cone that encloses the view frustum
};

struct TerrainLodParams {
    double radius         = EARTH_RADIUS;   // sea-level radius (m)
    double max_elevation  = 0.0;            // highest terrain above sea level (m), for bounds
    int    min_level      = 2;              // always split at least this far (keeps the far globe round)
    int    max_level      = 15;             // deepest split (Earth: ~300 m chunks)
    double split_distance = 3.0;            // split while camera distance < this x chunk edge
};

class TerrainLod {
public:
    // Cube face frame: F = outward normal, A and B span the face (A x B = F).
    static void FaceBasis(int face, Vec3& f, Vec3& a, Vec3& b);

    void Configure(const TerrainLodParams& p) { params_ = p; }
    const TerrainLodParams& Params() const { return params_; }

    // Replace `out` with the leaf chunks to draw this frame: horizon- and
    // frustum-culled, each at the level its distance calls for.
    void Select(const TerrainLodCamera& cam, std::vector<TerrainChunk>& out) const;

private:
    void visit(const TerrainLodCamera& cam, int face, int level,
               double a, double b, double half, std::vector<TerrainChunk>& out) const;

    TerrainLodParams params_;
};

}
