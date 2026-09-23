#pragma once

#include <bgfx/bgfx.h>
#include <bx/allocator.h>
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <thread>
#include <vector>

// Small shared bgfx helpers: file loading, shader programs, and textures. Used by
// the backend and by its self-contained scene objects (EarthBumpMap, Terrain, ...)
// so each one doesn't grow its own copy of the same loaders.

namespace renderer::bgfxutil {

// Whole file as bytes; empty if it can't be opened.
std::vector<uint8_t> readFile(const char* path);

// Whole file in one malloc'd buffer, for the multi-GB textures: unlike readFile it
// isn't zero-filled first, and createTexture can hand it to bgfx without a copy.
// Touches no bgfx state, so it's safe on a worker thread. Empty (false) if the file
// can't be opened or read, or is too big for bgfx's 32-bit sizes.
struct FileBlob {
    std::unique_ptr<uint8_t, decltype(&std::free)> data{ nullptr, &std::free };
    uint32_t size = 0;
    explicit operator bool() const { return data != nullptr; }
};
FileBlob readFileBlob(const char* path);

// Program from compiled shaders in src/renderer/bgfx/shaders/, by base name,
// e.g. loadProgram("vs_generic", "fs_generic"). Missing shaders are reported and
// yield an invalid handle.
bgfx::ProgramHandle loadProgram(const char* vsName, const char* fsName);

// Texture file (DDS/KTX/PNG/... via bimg), with its mips as stored. Invalid handle
// (and a message) if missing or unparsable.
bgfx::TextureHandle loadTexture(const char* path, uint64_t flags);

// Largest texture side to create: the GPU's limit (16384 on most integrated
// GPUs), lowered further by the BR6_MAX_TEXTURE_SIZE environment variable when
// set (e.g. 8192 or 4096 to fit a small VRAM budget). Valid once bgfx is up.
uint32_t maxTextureSize();

// How many top mips to drop so a w x h texture fits maxTextureSize().
uint32_t mipsToSkip(uint32_t w, uint32_t h);

// loadTexture for a file already in memory (path is only for messages). A plain
// 2D DDS's pixels go to bgfx in place and the buffer is freed once uploaded; other
// files are parsed into a copy as before. A DDS larger than maxTextureSize() is
// loaded from its first mip that fits, if the file has a full mip chain.
bgfx::TextureHandle createTexture(FileBlob blob, const char* path, uint64_t flags);

// Allocator for bimg parsing/decoding.
bx::AllocatorI* allocator();

// fn(begin, end) over [0, n), split across the hardware threads (this one
// included), for the CPU-side texture builds at startup. fn must not call bgfx.
template <class Fn>
void parallelFor(uint32_t n, Fn fn) {
    const uint32_t threads = std::clamp(std::thread::hardware_concurrency(), 1u, std::max(n, 1u));
    auto split = [&](uint32_t i) { return (uint32_t)((uint64_t)n * i / threads); };
    std::vector<std::thread> pool;
    for (uint32_t i = 1; i < threads; ++i) pool.emplace_back(fn, split(i), split(i + 1));
    fn(0u, split(1));
    for (std::thread& t : pool) t.join();
}

}
