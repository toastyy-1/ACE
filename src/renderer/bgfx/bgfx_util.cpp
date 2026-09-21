#include "bgfx_util.hpp"

#include <bimg/decode.h>

#include <cstdio>
#include <fstream>
#include <string>

namespace renderer::bgfxutil {

namespace {

bx::DefaultAllocator s_allocator;

const char* kShaderDir = "src/renderer/bgfx/shaders/";

bgfx::ShaderHandle loadShader(const char* name) {
    std::string path = std::string(kShaderDir) + name + ".bin";
    std::vector<uint8_t> data = readFile(path.c_str());
    if (data.empty()) { std::fprintf(stderr, "bgfx: missing shader %s\n", path.c_str()); return BGFX_INVALID_HANDLE; }
    return bgfx::createShader(bgfx::copy(data.data(), (uint32_t)data.size()));
}

} // namespace

std::vector<uint8_t> readFile(const char* path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return {};
    std::streamsize n = f.tellg();
    f.seekg(0);
    std::vector<uint8_t> buf((size_t)n);
    f.read((char*)buf.data(), n);
    return buf;
}

bgfx::ProgramHandle loadProgram(const char* vsName, const char* fsName) {
    return bgfx::createProgram(loadShader(vsName), loadShader(fsName), true);
}

bgfx::TextureHandle loadTexture(const char* path, uint64_t flags) {
    std::vector<uint8_t> data = readFile(path);
    if (data.empty()) { std::fprintf(stderr, "bgfx: missing %s\n", path); return BGFX_INVALID_HANDLE; }
    bimg::ImageContainer* ic = bimg::imageParse(&s_allocator, data.data(), (uint32_t)data.size());
    if (!ic) { std::fprintf(stderr, "bgfx: parse failed %s\n", path); return BGFX_INVALID_HANDLE; }
    bgfx::TextureHandle th = bgfx::createTexture2D(
        (uint16_t)ic->m_width, (uint16_t)ic->m_height, ic->m_numMips > 1, ic->m_numLayers,
        (bgfx::TextureFormat::Enum)ic->m_format, flags, bgfx::copy(ic->m_data, ic->m_size));
    bimg::imageFree(ic);
    return th;
}

bx::AllocatorI* allocator() { return &s_allocator; }

}
