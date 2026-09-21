#pragma once

#include <bgfx/bgfx.h>
#include <bx/allocator.h>
#include <cstdint>
#include <vector>

// Small shared bgfx helpers: file loading, shader programs, and textures. Used by
// the backend and by its self-contained scene objects (EarthBumpMap, Terrain, ...)
// so each one doesn't grow its own copy of the same loaders.

namespace renderer::bgfxutil {

// Whole file as bytes; empty if it can't be opened.
std::vector<uint8_t> readFile(const char* path);

// Program from compiled shaders in src/renderer/bgfx/shaders/, by base name,
// e.g. loadProgram("vs_generic", "fs_generic"). Missing shaders are reported and
// yield an invalid handle.
bgfx::ProgramHandle loadProgram(const char* vsName, const char* fsName);

// Texture file (DDS/KTX/PNG/... via bimg), with its mips as stored. Invalid handle
// (and a message) if missing or unparsable.
bgfx::TextureHandle loadTexture(const char* path, uint64_t flags);

// Allocator for bimg parsing/decoding.
bx::AllocatorI* allocator();

}
