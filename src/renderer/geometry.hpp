#pragma once

#include "mesh.hpp"

// Lightweight parametric mesh builders. All coordinates are local to the part;
// the renderer places them with a model matrix (see rmath.hpp).
// UVs are only meaningful for buildSphere (the Earth).

namespace renderer::geom {

// A frustum along +Z: radius r0 at z0, radius r1 at z1 (a cylinder when
// r0==r1, a cone when one radius is 0). Caps are added at each non-degenerate
// end unless suppressed. Assumes z1 > z0. Vertices carry `col`.
void appendFrustum(Mesh& m, float z0, float z1, float r0, float r1,
                   int sides, RColor col, bool cap0 = true, bool cap1 = true);

// A single triangle with a flat normal. If doubleSided, the back face is also
// emitted so it reads from any angle (used for the tail fins).
void appendTriangle(Mesh& m, const RVec3& a, const RVec3& b, const RVec3& c,
                    RColor col, bool doubleSided = false);

// Surface of revolution around +Z. `profile` is an ordered list of points with
// x = axial z (increasing) and y = radius; revolved into `sides` segments with
// smooth normals from the profile slope. Used for the rounded nose cone.
void appendRevolve(Mesh& m, const std::vector<RVec3>& profile,
                   int sides, RColor col, bool capBase = true);

// Unit cone: base circle radius 1 in the z=0 plane, apex at z=1. Recoloured per
// draw via Material; used (scaled/oriented) for the exhaust plume.
Mesh buildCone(int sides);

// UV sphere of the given radius, built in an ECI-aligned frame (+Z = north
// pole) with equirectangular texcoords (u = longitude, v = latitude from the
// north pole), so an equirectangular Earth texture maps directly. lonOffset
// shifts the texture east as a fraction of a full turn.
Mesh buildSphere(float radius, int rings, int sectors, float lonOffset = 0.0f);

// Flat (n+1)x(n+1) grid spanning [-1,1]^2 in the XY plane (z = 0). Used as the
// shared terrain chunk: vs_terrain reads pos.xy as the position within a chunk
// and wraps it onto the sphere. With `skirt`, the border is duplicated once more
// at z = 1 and stitched to it; the shader hangs that ring below the surface so
// cracks between chunks at different levels are hidden. Normals/UVs/colour are
// unused by that shader.
Mesh buildGrid(int n, bool skirt = false);

}
