#pragma once

#include "wire.hpp"
#include "../scene.hpp"
#include "../terrain_lod.hpp"
#include <unordered_map>
#include <vector>

// The raylib backend's Earth: a smooth sphere of EARTH_RADIUS (the same ground
// the sim uses in this build), drawn as a hidden-line wireframe grid.
//
// The grid is the cube-sphere quadtree from TerrainLod, so line density follows
// view distance: a coarse lattice from orbit that refines to metre-scale squares
// near the ground. Each chunk is built on the CPU once, relative to its own
// centre in double precision (no float jitter at planet scale), and cached on
// the GPU; all chunks share one index buffer.

namespace renderer {

class WireEarth {
public:
    void Init();       // needs the GL context
    void Shutdown();

    void Draw(wire::Pipeline& p, const EarthFrame& f, const RCamera& cam, float aspect);

private:
    struct Cached { wire::GpuMesh mesh; uint64_t lastFrame; };
    const wire::GpuMesh& chunkMesh(const TerrainChunk& c);
    void evict();

    TerrainLod                lod_;
    std::vector<TerrainChunk> chunks_;   // reused every frame
    std::vector<RMat4>        models_;   // per chunk, reused every frame
    std::unordered_map<uint64_t, Cached> cache_;
    std::vector<Vertex>       scratch_;  // chunk vertices being built

    unsigned ebo_ = 0;                   // shared grid topology
    int      tris_ = 0, lines_ = 0;
    uint64_t frame_ = 0;
};

}
