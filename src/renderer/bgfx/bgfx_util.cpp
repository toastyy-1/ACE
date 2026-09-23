#include "bgfx_util.hpp"

#include <bimg/decode.h>

#include <cstdio>
#include <cstdlib>
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

FileBlob readFileBlob(const char* path) {
    FileBlob blob;
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return blob;
    std::streamsize n = f.tellg();
    if (n <= 0 || n > (std::streamsize)UINT32_MAX) return blob;
    f.seekg(0);
    blob.data.reset((uint8_t*)std::malloc((size_t)n));
    if (!blob.data || !f.read((char*)blob.data.get(), n)) { blob.data.reset(); return blob; }
    blob.size = (uint32_t)n;
    return blob;
}

bgfx::ProgramHandle loadProgram(const char* vsName, const char* fsName) {
    return bgfx::createProgram(loadShader(vsName), loadShader(fsName), true);
}

bgfx::TextureHandle loadTexture(const char* path, uint64_t flags) {
    return createTexture(readFileBlob(path), path, flags);
}

uint32_t maxTextureSize() {
    uint32_t limit = bgfx::getCaps()->limits.maxTextureSize;
    if (const char* env = std::getenv("BR6_MAX_TEXTURE_SIZE")) {
        long v = std::strtol(env, nullptr, 10);
        if (v >= 1) limit = std::min(limit, (uint32_t)v);
    }
    return limit;
}

uint32_t mipsToSkip(uint32_t w, uint32_t h) {
    const uint32_t limit = maxTextureSize();
    uint32_t skip = 0;
    while ((w >> skip) > limit || (h >> skip) > limit) ++skip;
    return skip;
}

bgfx::TextureHandle createTexture(FileBlob blob, const char* path, uint64_t flags) {
    if (!blob) { std::fprintf(stderr, "bgfx: missing %s\n", path); return BGFX_INVALID_HANDLE; }

    // A DDS stores its mips exactly as bgfx wants them, so reference the file's
    // pixels instead of letting imageParse copy them (and bgfx::copy them again),
    // which for the Earth maps was two extra passes over ~5 GB. Anything else, or a
    // DDS with a partial mip chain, takes the copying path below.
    bimg::ImageContainer hdr;
    if (bimg::imageParse(hdr, blob.data.get(), blob.size)
        && hdr.m_parser == bimg::ImageParser::Dds && !hdr.m_cubeMap && hdr.m_depth <= 1) {
        const bool     hasMips = hdr.m_numMips > 1;
        const uint64_t size    = bimg::imageGetSize(nullptr, hdr.m_width, hdr.m_height, 1, false, hasMips,
                                                    hdr.m_numLayers, hdr.m_format);
        const bool fullChain = !hasMips || hdr.m_numMips == bimg::imageGetNumMips(hdr.m_format, hdr.m_width, hdr.m_height);
        if (fullChain && hdr.m_offset <= blob.size && size <= blob.size - hdr.m_offset) {
            // Too big for this GPU (e.g. the 32768-wide Earth maps on an integrated
            // GPU): start from the first mip that fits. A single layer's mips are
            // contiguous, so the rest of the chain is a tail of the file.
            const uint32_t skip = mipsToSkip(hdr.m_width, hdr.m_height);
            const uint8_t* pixels = blob.data.get() + hdr.m_offset;
            uint32_t w = hdr.m_width, h = hdr.m_height;
            uint64_t bytes = size;
            if (skip > 0) {
                bimg::ImageMip mip;
                if (!hasMips || hdr.m_numLayers > 1 || skip >= hdr.m_numMips
                    || !bimg::imageGetRawData(hdr, 0, (uint8_t)skip, blob.data.get(), blob.size, mip)) {
                    std::fprintf(stderr, "bgfx: %s is %ux%u, above this GPU's %u limit, and has no mip to fall back to\n",
                                 path, hdr.m_width, hdr.m_height, maxTextureSize());
                    return BGFX_INVALID_HANDLE;
                }
                pixels = mip.m_data;
                w = mip.m_width;
                h = mip.m_height;
                bytes = bimg::imageGetSize(nullptr, w, h, 1, false, true, 1, hdr.m_format);
                std::fprintf(stderr, "bgfx: %s is %ux%u, above the %u texture limit; using %ux%u\n",
                             path, hdr.m_width, hdr.m_height, maxTextureSize(), w, h);
            }
            uint8_t* base = blob.data.release();
            const bgfx::Memory* mem = bgfx::makeRef(pixels, (uint32_t)bytes,
                                                    [](void*, void* userData) { std::free(userData); }, base);
            return bgfx::createTexture2D((uint16_t)w, (uint16_t)h, hasMips, hdr.m_numLayers,
                                         (bgfx::TextureFormat::Enum)hdr.m_format, flags, mem);
        }
    }

    bimg::ImageContainer* ic = bimg::imageParse(&s_allocator, blob.data.get(), blob.size);
    if (!ic) { std::fprintf(stderr, "bgfx: parse failed %s\n", path); return BGFX_INVALID_HANDLE; }
    if (mipsToSkip(ic->m_width, ic->m_height) > 0)
        std::fprintf(stderr, "bgfx: %s is %ux%u, above the %u texture limit\n",
                     path, ic->m_width, ic->m_height, maxTextureSize());
    bgfx::TextureHandle th = bgfx::createTexture2D(
        (uint16_t)ic->m_width, (uint16_t)ic->m_height, ic->m_numMips > 1, ic->m_numLayers,
        (bgfx::TextureFormat::Enum)ic->m_format, flags, bgfx::copy(ic->m_data, ic->m_size));
    bimg::imageFree(ic);
    return th;
}

bx::AllocatorI* allocator() { return &s_allocator; }

}
