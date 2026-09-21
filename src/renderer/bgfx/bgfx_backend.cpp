#if defined(_WIN32)
#  define GLFW_EXPOSE_NATIVE_WIN32
#  define WIN32_LEAN_AND_MEAN
#  define NOMINMAX
#elif defined(__APPLE__)
#  define GLFW_EXPOSE_NATIVE_COCOA
#elif defined(__linux__)
#  define GLFW_EXPOSE_NATIVE_X11
#  define GLFW_EXPOSE_NATIVE_WAYLAND
#endif

#include "bgfx_backend.hpp"
#include "bgfx_util.hpp"
#include "../geometry.hpp"
#include "../../constants.hpp"

#include <bgfx/platform.h>
#include <bx/math.h>

#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#ifdef DrawText
#undef DrawText
#endif

#if defined(_WIN32)
#include <timeapi.h>
#elif defined(__linux__)
#include <dlfcn.h>
#endif

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>

#define STB_EASY_FONT_IMPLEMENTATION
#include "stb_easy_font.h"

#define STB_TRUETYPE_IMPLEMENTATION
#define STB_TRUETYPE_STATIC   // internal linkage: avoid clashing with bgfx/bimg's stb
#include <stb/stb_truetype.h>

namespace renderer {

namespace {

// --- Greek-capable glyph atlas (stb_truetype) -------------------------------
// Baked once from the bundled TTF into a single bgfx texture. Covers ASCII
// printable + the Greek block so rocket IDs like "α3" render. If the font is
// missing, `ready` stays false and DrawText falls back to stb_easy_font (ASCII).
struct GlyphFont {
    // 1024² comfortably fits all 239 glyphs at 48px with no oversampling
    // (~50px cells -> ~400 slots). 512² only held ~25, so most glyphs (including
    // every Greek letter, packed last) overflowed and rendered blank.
    static constexpr int   kAtlasW = 1024;
    static constexpr int   kAtlasH = 1024;
    static constexpr float kBakePx = 48.0f;   // native baked height; scaled per draw
    static constexpr int   kAsciiFirst = 32,  kAsciiCount = 95;    // 32..126
    static constexpr int   kGreekFirst = 0x370, kGreekCount = 144; // 0x370..0x3FF

    bool tried = false;
    bool ready = false;
    bgfx::TextureHandle atlas = BGFX_INVALID_HANDLE;
    std::vector<stbtt_packedchar> ascii;   // kAsciiCount entries
    std::vector<stbtt_packedchar> greek;   // kGreekCount entries
};

GlyphFont g_font;

const char* kFontPath = "src/renderer/assets/DejaVuSans.ttf";

// Bake the atlas on first use. Safe to call every DrawText (guarded by `tried`).
void ensureGlyphFont() {
    if (g_font.tried) return;
    g_font.tried = true;

    std::vector<uint8_t> ttf = bgfxutil::readFile(kFontPath);
    if (ttf.empty()) { std::fprintf(stderr, "bgfx: font %s missing; ASCII fallback\n", kFontPath); return; }

    g_font.ascii.resize(GlyphFont::kAsciiCount);
    g_font.greek.resize(GlyphFont::kGreekCount);

    std::vector<unsigned char> bmp((size_t)GlyphFont::kAtlasW * GlyphFont::kAtlasH);
    stbtt_pack_context pc;
    if (!stbtt_PackBegin(&pc, bmp.data(), GlyphFont::kAtlasW, GlyphFont::kAtlasH, 0, 1, nullptr)) {
        std::fprintf(stderr, "bgfx: stbtt_PackBegin failed; ASCII fallback\n");
        return;
    }
    // No oversampling: it doubles each glyph's atlas footprint, which is what
    // overflowed the old 512² atlas. A 48px bake downscaled with bilinear is crisp
    // enough for HUD/label text.
    stbtt_PackSetOversampling(&pc, 1, 1);

    stbtt_pack_range ranges[2] = {};
    ranges[0].font_size                        = GlyphFont::kBakePx;
    ranges[0].first_unicode_codepoint_in_range = GlyphFont::kAsciiFirst;
    ranges[0].num_chars                        = GlyphFont::kAsciiCount;
    ranges[0].chardata_for_range               = g_font.ascii.data();
    ranges[1].font_size                        = GlyphFont::kBakePx;
    ranges[1].first_unicode_codepoint_in_range = GlyphFont::kGreekFirst;
    ranges[1].num_chars                        = GlyphFont::kGreekCount;
    ranges[1].chardata_for_range               = g_font.greek.data();
    // Returns 0 if any glyph didn't fit; unpacked glyphs stay zeroed (invisible).
    int packed = stbtt_PackFontRanges(&pc, ttf.data(), 0, ranges, 2);
    stbtt_PackEnd(&pc);
    if (!packed)
        std::fprintf(stderr, "bgfx: glyph atlas overflow (%dx%d) — some glyphs will be blank\n",
                     GlyphFont::kAtlasW, GlyphFont::kAtlasH);
    else
        std::fprintf(stderr, "bgfx: glyph atlas baked OK (%d ASCII + %d Greek)\n",
                     GlyphFont::kAsciiCount, GlyphFont::kGreekCount);

    // Expand 8-bit coverage -> RGBA8 white with alpha = coverage, so the generic
    // 2D shader (texture * tint * vcolor) yields tinted anti-aliased glyphs.
    const uint32_t px = (uint32_t)GlyphFont::kAtlasW * GlyphFont::kAtlasH;
    const bgfx::Memory* mem = bgfx::alloc(px * 4);
    for (uint32_t i = 0; i < px; ++i) {
        mem->data[i*4+0] = 255; mem->data[i*4+1] = 255;
        mem->data[i*4+2] = 255; mem->data[i*4+3] = bmp[i];
    }
    g_font.atlas = bgfx::createTexture2D((uint16_t)GlyphFont::kAtlasW, (uint16_t)GlyphFont::kAtlasH,
                                         false, 1, bgfx::TextureFormat::RGBA8,
                                         BGFX_SAMPLER_NONE, mem);
    g_font.ready = bgfx::isValid(g_font.atlas);
}

uint32_t packRgba(RColor c) {
    return (uint32_t(c.r) << 24) | (uint32_t(c.g) << 16) | (uint32_t(c.b) << 8) | uint32_t(c.a);
}

// A view-space direction/point as a vec4 for a bgfx uniform.
void setVec4(bgfx::UniformHandle u, const RVec3& v, float w = 0.0f) {
    float d[4] = { v.x, v.y, v.z, w };
    bgfx::setUniform(u, d);
}

#if defined(__linux__)
using GlfwGetWaylandDisplayFn = struct wl_display* (*)();
using GlfwGetWaylandWindowFn = struct wl_surface* (*)(GLFWwindow*);

bool loadGlfwWaylandSymbols(GlfwGetWaylandDisplayFn& displayFn,
                             GlfwGetWaylandWindowFn& windowFn) {
    void* lib = dlopen("libglfw.so.3", RTLD_LAZY | RTLD_NOLOAD);
    if (!lib) {
        lib = dlopen("libglfw.so", RTLD_LAZY | RTLD_NOLOAD);
    }
    if (!lib) {
        lib = dlopen("libglfw.so.3", RTLD_LAZY);
    }
    if (!lib) {
        return false;
    }

    displayFn = reinterpret_cast<GlfwGetWaylandDisplayFn>(dlsym(lib, "glfwGetWaylandDisplay"));
    windowFn  = reinterpret_cast<GlfwGetWaylandWindowFn>(dlsym(lib, "glfwGetWaylandWindow"));
    dlclose(lib);
    return displayFn && windowFn;
}
#endif

} // namespace

void BgfxBackend::Init(int width, int height, const char* title) {
    width_ = width; height_ = height;
    reset_ = BGFX_RESET_VSYNC | BGFX_RESET_MSAA_X4;

#if defined(_WIN32)
    timeBeginPeriod(1);
#endif

    if (!glfwInit()) {
        std::fprintf(stderr, "bgfx: failed to initialize GLFW\n");
        std::abort();
    }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);   // bgfx owns the context
    window_ = glfwCreateWindow(width_, height_, title, nullptr, nullptr);
    if (!window_) {
        std::fprintf(stderr, "bgfx: failed to create GLFW window\n");
        glfwTerminate();
        std::abort();
    }
    glfwSetWindowUserPointer(window_, this);
    glfwSetScrollCallback(window_, [](GLFWwindow* w, double, double yoff) {
        ((BgfxBackend*)glfwGetWindowUserPointer(w))->wheel_ += (float)yoff;
    });

    bgfx::renderFrame();   // signal single-threaded (submit on this thread)
    bgfx::Init init;
    init.type = bgfx::RendererType::OpenGL;
#if defined(_WIN32)
    init.platformData.nwh = glfwGetWin32Window(window_);
#elif defined(__APPLE__)
    init.platformData.nwh = glfwGetCocoaWindow(window_);
#elif defined(__linux__)
    const char* waylandDisplay = std::getenv("WAYLAND_DISPLAY");
    if (waylandDisplay != nullptr) {
        GlfwGetWaylandDisplayFn waylandDisplayFn = nullptr;
        GlfwGetWaylandWindowFn waylandWindowFn = nullptr;
        if (loadGlfwWaylandSymbols(waylandDisplayFn, waylandWindowFn)) {
            init.platformData.ndt  = waylandDisplayFn();
            init.platformData.nwh  = waylandWindowFn(window_);
            init.platformData.type = bgfx::NativeWindowHandleType::Wayland;
        } else {
            init.platformData.ndt = glfwGetX11Display();
            init.platformData.nwh = (void*)(uintptr_t)glfwGetX11Window(window_);
        }
    } else {
        init.platformData.ndt = glfwGetX11Display();
        init.platformData.nwh = (void*)(uintptr_t)glfwGetX11Window(window_);
    }
#endif
    init.resolution.width  = (uint32_t)width_;
    init.resolution.height = (uint32_t)height_;
    init.resolution.reset  = reset_;
    // All per-frame dynamic geometry (trails, predicted paths, debug lines, the 2D
    // HUD, and rocket labels) is drawn from bgfx's shared transient vertex buffer,
    // whose 6 MB default (~175k verts) is exhausted by even ~a dozen rockets with
    // grown trails -- and once full, later submits (the HUD, labels) are silently
    // dropped, so they flicker. 64 MB (~1.86M verts) headroom, sized to the
    // per-frame line budget the renderer decimates to (see renderer.cpp).
    init.limits.maxTransientVbSize = 256u << 20;
    if (!bgfx::init(init)) {
        std::fprintf(stderr, "bgfx: init failed\n");
        glfwDestroyWindow(window_);
        glfwTerminate();
        std::abort();
    }

    // Preserve submit order (the additive plume must draw after the opaque hull).
    bgfx::setViewMode(0, bgfx::ViewMode::Sequential);
    bgfx::setViewMode(1, bgfx::ViewMode::Sequential);

    layout_.begin()
        .add(bgfx::Attrib::Position,  3, bgfx::AttribType::Float)
        .add(bgfx::Attrib::Normal,    3, bgfx::AttribType::Float)
        .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
        .add(bgfx::Attrib::Color0,    4, bgfx::AttribType::Uint8, true)
        .end();

    generic_    = bgfxutil::loadProgram("vs_generic", "fs_generic");
    cloudProg_  = bgfxutil::loadProgram("vs_cloud",   "fs_cloud");
    atmosProg_  = bgfxutil::loadProgram("vs_atmos",   "fs_atmos");
    flareProg_  = bgfxutil::loadProgram("vs_flare",   "fs_flare");
    rocketProg_ = bgfxutil::loadProgram("vs_rocket",  "fs_rocket");
    heatProg_   = bgfxutil::loadProgram("vs_heat",    "fs_heat");

    s_tex_         = bgfx::createUniform("s_tex",        bgfx::UniformType::Sampler);
    u_tint_        = bgfx::createUniform("u_tint",       bgfx::UniformType::Vec4);
    u_depth_       = bgfx::createUniform("u_depth",      bgfx::UniformType::Vec4);
    u_light_       = bgfx::createUniform("u_light",      bgfx::UniformType::Vec4);
    u_heat_        = bgfx::createUniform("u_heat",       bgfx::UniformType::Vec4);
    u_earth_       = bgfx::createUniform("u_earth",      bgfx::UniformType::Vec4);
    s_color_       = bgfx::createUniform("s_color",      bgfx::UniformType::Sampler);
    s_night_       = bgfx::createUniform("s_night",      bgfx::UniformType::Sampler);
    s_emiss_       = bgfx::createUniform("s_emiss",      bgfx::UniformType::Sampler);
    u_sunDir_      = bgfx::createUniform("u_sunDir",     bgfx::UniformType::Vec4);
    u_earthCenter_ = bgfx::createUniform("u_earthCenter",bgfx::UniformType::Vec4);
    u_camPos_      = bgfx::createUniform("u_camPos",     bgfx::UniformType::Vec4);
    s_cloud_       = bgfx::createUniform("s_cloud",      bgfx::UniformType::Sampler);
    u_cloudAlpha_  = bgfx::createUniform("u_cloudAlpha", bgfx::UniformType::Vec4);
    u_cloudDisp_   = bgfx::createUniform("u_cloudDisp",  bgfx::UniformType::Vec4);
    u_atmos_       = bgfx::createUniform("u_atmos",      bgfx::UniformType::Vec4);
    u_rayFwd_      = bgfx::createUniform("u_rayFwd",     bgfx::UniformType::Vec4);
    u_rayRight_    = bgfx::createUniform("u_rayRight",   bgfx::UniformType::Vec4);
    u_rayUp_       = bgfx::createUniform("u_rayUp",      bgfx::UniformType::Vec4);

    const uint8_t whitePix[4] = { 255, 255, 255, 255 };
    white_ = bgfx::createTexture2D(1, 1, false, 1, bgfx::TextureFormat::RGBA8, 0,
                                   bgfx::copy(whitePix, 4));

    startTime_ = lastTime_ = glfwGetTime();
}

void BgfxBackend::Shutdown() {
    for (auto& m : meshes_) { if (bgfx::isValid(m.vbh)) bgfx::destroy(m.vbh); if (bgfx::isValid(m.ibh)) bgfx::destroy(m.ibh); }
    for (auto& t : textures_) if (bgfx::isValid(t)) bgfx::destroy(t);
    terrain_.Destroy();
    if (bgfx::isValid(earthColor_)) bgfx::destroy(earthColor_);
    EarthBumpMap::Get().Destroy();
    if (bgfx::isValid(earthNight_)) bgfx::destroy(earthNight_);
    if (bgfx::isValid(earthCloud_)) bgfx::destroy(earthCloud_);
    if (bgfx::isValid(earthRough_)) bgfx::destroy(earthRough_);
    if (bgfx::isValid(earthEmiss_)) bgfx::destroy(earthEmiss_);
    if (bgfx::isValid(white_))      bgfx::destroy(white_);
    if (bgfx::isValid(g_font.atlas)) { bgfx::destroy(g_font.atlas); g_font.atlas = BGFX_INVALID_HANDLE; }
    g_font.ready = false; g_font.tried = false;   // allow a fresh bake if re-Init'd
    if (bgfx::isValid(generic_))    bgfx::destroy(generic_);
    if (bgfx::isValid(cloudProg_))  bgfx::destroy(cloudProg_);
    if (bgfx::isValid(atmosProg_))  bgfx::destroy(atmosProg_);
    if (bgfx::isValid(flareProg_))  bgfx::destroy(flareProg_);
    if (bgfx::isValid(rocketProg_)) bgfx::destroy(rocketProg_);
    if (bgfx::isValid(heatProg_))   bgfx::destroy(heatProg_);
    for (bgfx::UniformHandle u : { s_tex_, u_tint_, u_depth_, u_light_, u_heat_, u_earth_, s_color_, s_night_, s_emiss_,
                                   u_sunDir_, u_earthCenter_, u_camPos_,
                                   s_cloud_, u_cloudAlpha_, u_cloudDisp_, u_atmos_,
                                   u_rayFwd_, u_rayRight_, u_rayUp_ })
        if (bgfx::isValid(u)) bgfx::destroy(u);
    bgfx::shutdown();
    if (window_) glfwDestroyWindow(window_);
    glfwTerminate();
#if defined(_WIN32)
    timeEndPeriod(1);
#endif
}

bool BgfxBackend::ShouldClose() const { return glfwWindowShouldClose(window_); }

FrameInput BgfxBackend::PollInput() {
    glfwPollEvents();

    double now = glfwGetTime();
    frameDt_ = now - lastTime_;
    lastTime_ = now;
    if (frameDt_ > 1e-6) fps_ = fps_ * 0.9 + (1.0 / frameDt_) * 0.1;

    double x, y;
    glfwGetCursorPos(window_, &x, &y);
    if (!havePrev_) { prevX_ = x; prevY_ = y; havePrev_ = true; }
    float dx = float(x - prevX_), dy = float(y - prevY_);
    prevX_ = x; prevY_ = y;

    bool left = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    float w = wheel_; wheel_ = 0.0f;

    auto down = [&](int k){ return glfwGetKey(window_, k) == GLFW_PRESS; };
    float mx = (down(GLFW_KEY_D) ? 1.0f : 0.0f) - (down(GLFW_KEY_A) ? 1.0f : 0.0f);
    float my = (down(GLFW_KEY_E) ? 1.0f : 0.0f) - (down(GLFW_KEY_Q) ? 1.0f : 0.0f);
    float mz = (down(GLFW_KEY_W) ? 1.0f : 0.0f) - (down(GLFW_KEY_S) ? 1.0f : 0.0f);
    bool boost = down(GLFW_KEY_LEFT_SHIFT) || down(GLFW_KEY_RIGHT_SHIFT);
    bool recenter = down(GLFW_KEY_F);
    // Tab cycles the primary rocket (edge-triggered); Shift+Tab goes backwards.
    bool tab  = down(GLFW_KEY_TAB);
    bool edge = tab && !prevTab_;
    prevTab_  = tab;
    bool next = edge && !boost;
    bool prev = edge &&  boost;
    // Number keys 1-9 toggle overlays (edge-triggered). GLFW_KEY_1..9 == 49..57.
    int toggle = 0;
    for (int d = 1; d <= 9; ++d) {
        bool now = down(GLFW_KEY_0 + d);
        if (now && !prevDigit_[d]) toggle = d;
        prevDigit_[d] = now;
    }
    return FrameInput{ dx, dy, w, left, mx, my, mz, boost, recenter, next, prev, toggle };
}

float  BgfxBackend::FrameTime() const { return (float)frameDt_; }
double BgfxBackend::Time() const      { return glfwGetTime() - startTime_; }

TextureHandle BgfxBackend::LoadTexture(const char* path) {
    bgfx::TextureHandle th = bgfxutil::loadTexture(path, BGFX_SAMPLER_NONE);
    if (!bgfx::isValid(th)) return 0;
    textures_.push_back(th);
    return (TextureHandle)textures_.size();
}

MeshHandle BgfxBackend::CreateMesh(const Mesh& m) {
    const bgfx::Memory* vmem = bgfx::copy(m.verts.data(), (uint32_t)(m.verts.size() * sizeof(Vertex)));
    // 32-bit indices throughout, so the Earth can be tessellated past 64k verts.
    const bgfx::Memory* imem = bgfx::copy(m.idx.data(), (uint32_t)(m.idx.size() * sizeof(uint32_t)));
    GpuMesh g;
    g.vbh = bgfx::createVertexBuffer(vmem, layout_);
    g.ibh = bgfx::createIndexBuffer(imem, BGFX_BUFFER_INDEX32);
    meshes_.push_back(g);
    return (MeshHandle)meshes_.size();
}

void BgfxBackend::DestroyMesh(MeshHandle h) {
    if (h == 0 || h > meshes_.size()) return;
    GpuMesh& g = meshes_[h - 1];
    if (bgfx::isValid(g.vbh)) { bgfx::destroy(g.vbh); g.vbh = BGFX_INVALID_HANDLE; }
    if (bgfx::isValid(g.ibh)) { bgfx::destroy(g.ibh); g.ibh = BGFX_INVALID_HANDLE; }
}

void BgfxBackend::BeginFrame(RColor clear) {
    // Handle window resize.
    int fw, fh;
    glfwGetFramebufferSize(window_, &fw, &fh);
    if (fw > 0 && fh > 0 && (fw != width_ || fh != height_)) {
        width_ = fw; height_ = fh;
        bgfx::reset((uint32_t)fw, (uint32_t)fh, reset_);
    }

    bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, packRgba(clear), 1.0f, 0);
    bgfx::setViewRect(0, 0, 0, (uint16_t)width_, (uint16_t)height_);
    bgfx::touch(0);

    // View 1: 2D overlay, screen-space ortho (top-left origin, y down).
    float ortho[16];
    bx::mtxOrtho(ortho, 0.0f, (float)width_, (float)height_, 0.0f, 0.0f, 100.0f, 0.0f,
                 bgfx::getCaps()->homogeneousDepth);
    bgfx::setViewClear(1, BGFX_CLEAR_NONE, 0, 1.0f, 0);
    bgfx::setViewRect(1, 0, 0, (uint16_t)width_, (uint16_t)height_);
    bgfx::setViewTransform(1, nullptr, ortho);
}

void BgfxBackend::EndFrame() { bgfx::frame(); }

void BgfxBackend::SetClipPlanes(float near_plane, float far_plane) {
    near_ = near_plane; far_ = far_plane;
}

void BgfxBackend::Begin3D(const RCamera& cam) {
    cam_ = cam;
    float view[16], proj[16];
    bx::mtxLookAt(view, bx::Vec3{ cam.position.x, cam.position.y, cam.position.z },
                        bx::Vec3{ cam.target.x,   cam.target.y,   cam.target.z },
                        bx::Vec3{ cam.up.x,       cam.up.y,       cam.up.z },
                  bx::Handedness::Right);
    float aspect = height_ > 0 ? (float)width_ / (float)height_ : 1.0f;
    // bx::mtxProj applies toRad() internally, so fovy is in DEGREES here.
    bx::mtxProj(proj, cam.fovy, aspect, near_, far_,
                bgfx::getCaps()->homogeneousDepth, bx::Handedness::Right);
    bgfx::setViewTransform(0, view, proj);
    bgfx::setViewRect(0, 0, 0, (uint16_t)width_, (uint16_t)height_);
}

void BgfxBackend::End3D() {}

ScreenPoint BgfxBackend::WorldToScreen(const RVec3& viewPos) const {
    if (width_ <= 0 || height_ <= 0) return { 0, 0, false };
    // Rebuild the same view/proj as Begin3D and project the point to NDC.
    float view[16], proj[16], vp[16];
    bx::mtxLookAt(view, bx::Vec3{ cam_.position.x, cam_.position.y, cam_.position.z },
                        bx::Vec3{ cam_.target.x,   cam_.target.y,   cam_.target.z },
                        bx::Vec3{ cam_.up.x,       cam_.up.y,       cam_.up.z },
                  bx::Handedness::Right);
    float aspect = (float)width_ / (float)height_;
    bx::mtxProj(proj, cam_.fovy, aspect, near_, far_,
                bgfx::getCaps()->homogeneousDepth, bx::Handedness::Right);
    bx::mtxMul(vp, view, proj);

    float p[4]    = { viewPos.x, viewPos.y, viewPos.z, 1.0f };
    float clip[4];
    bx::vec4MulMtx(clip, p, vp);
    if (clip[3] <= 0.0f) return { 0, 0, false };            // behind the camera
    float ndcx = clip[0] / clip[3], ndcy = clip[1] / clip[3];
    return { (ndcx * 0.5f + 0.5f) * (float)width_,
             (1.0f - (ndcy * 0.5f + 0.5f)) * (float)height_, true };
}

int BgfxBackend::ScreenWidth() const  { return width_; }
int BgfxBackend::ScreenHeight() const { return height_; }

void BgfxBackend::DrawModel(MeshHandle h, const RMat4& model, const Material& mat) {
    if (h == 0 || h > meshes_.size()) return;
    const GpuMesh& g = meshes_[h - 1];
    if (!bgfx::isValid(g.vbh)) return;

    bgfx::setTransform(model.m);
    bgfx::setVertexBuffer(0, g.vbh);
    bgfx::setIndexBuffer(g.ibh);
    float tint[4] = { mat.color.r / 255.0f, mat.color.g / 255.0f, mat.color.b / 255.0f, mat.color.a / 255.0f };
    bgfx::setUniform(u_tint_, tint);
    float depth[4] = { far_, 0, 0, 0 };
    bgfx::setUniform(u_depth_, depth);
    // Program select: emissive -> additive ablation shell; lit -> PBR+grime;
    // everything else (lines, plume, markers, 2D) -> flat generic.
    float light[4] = { sunDirView_.x, sunDirView_.y, sunDirView_.z, mat.lit ? 1.0f : 0.0f };
    bgfx::setUniform(u_light_, light);
    if (mat.lit || mat.emissive) {
        float cam[4] = { camPosView_.x, camPosView_.y, camPosView_.z, 0 };
        bgfx::setUniform(u_camPos_, cam);
        float heat[4] = { heatDir_.x, heatDir_.y, heatDir_.z, heatAmt_ };
        bgfx::setUniform(u_heat_, heat);
    }
    if (mat.lit) {
        // Earth params + textures for the rocket's analytic reflection + earthshine.
        float e[4] = { earthCenterView_.x, earthCenterView_.y, earthCenterView_.z, (float)EARTH_RADIUS_KM };
        bgfx::setUniform(u_earth_, e);
        bgfx::setTexture(0, s_color_, bgfx::isValid(earthColor_) ? earthColor_ : white_);
        bgfx::setTexture(1, s_night_, bgfx::isValid(earthNight_) ? earthNight_ : white_);
        bgfx::setTexture(2, s_cloud_, bgfx::isValid(earthCloud_) ? earthCloud_ : white_);
    } else {
        bgfx::setTexture(0, s_tex_, mat.texture ? textures_[mat.texture - 1] : white_);
    }

    bgfx::ProgramHandle prog = mat.emissive ? heatProg_ : (mat.lit ? rocketProg_ : generic_);
    uint64_t state = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_DEPTH_TEST_LESS;
    if (mat.depth_write) state |= BGFX_STATE_WRITE_Z;
    if (mat.emissive) {
        state |= BGFX_STATE_BLEND_ADD | BGFX_STATE_CULL_CW;   // glow shell, single layer
    } else if (mat.blend == BlendMode::Additive) {
        state |= BGFX_STATE_BLEND_ADD;          // plume: both sides, no cull
    } else {
        state |= BGFX_STATE_BLEND_ALPHA;
        if (mat.cull) state |= BGFX_STATE_CULL_CW;   // closed solids; off for open bell / fins
    }
    bgfx::setState(state);
    bgfx::submit(0, prog);
}

void BgfxBackend::DrawLines(const LineVertex* v, size_t count, float /*width*/) {
    if (count < 2) return;                       // bgfx GL has no portable line width (~1px)
    uint32_t n = (uint32_t)count;
    // Degrade gracefully: if the batch doesn't fit the remaining transient pool,
    // draw as much as fits (an even count -- line pairs) instead of dropping it all.
    // The renderer budgets/decimates line geometry to stay well under the pool, so
    // this is a safety net that keeps a partial trail rather than a vanished one.
    uint32_t avail = bgfx::getAvailTransientVertexBuffer(n, layout_);
    if (avail < 2) return;
    if (avail < n) n = avail & ~1u;
    bgfx::TransientVertexBuffer tvb;
    bgfx::allocTransientVertexBuffer(&tvb, n, layout_);
    Vertex* dst = (Vertex*)tvb.data;
    for (uint32_t i = 0; i < n; ++i)
        dst[i] = Vertex{ v[i].pos, {0, 0, 0}, 0.0f, 0.0f, v[i].color };

    bgfx::setVertexBuffer(0, &tvb);
    float tint[4] = { 1, 1, 1, 1 };
    bgfx::setUniform(u_tint_, tint);
    float depth[4] = { far_, 0, 0, 0 };
    bgfx::setUniform(u_depth_, depth);
    float unlit[4] = { 0, 0, 0, 0 };          // lines are flat-coloured
    bgfx::setUniform(u_light_, unlit);
    bgfx::setTexture(0, s_tex_, white_);
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_WRITE_Z
                   | BGFX_STATE_DEPTH_TEST_LESS | BGFX_STATE_PT_LINES);
    bgfx::submit(0, generic_);
}

void BgfxBackend::DrawRocket(const RocketFrame& f) {
    heatDir_ = f.vel_dir;   // cached for the lit (PBR) DrawModel heating glow
    heatAmt_ = f.heating;
    rocket_.Ensure(*this, f.dims);
    rocket_.Draw(*this, f);
}

void BgfxBackend::ensureEarth() {
    if (cloudMesh_) return;
    // Half-turn: aligns the Greenwich-centred equirectangular map (u=0.5 -> 0 deg)
    // with the ECI frame the rocket is placed in, so launch sites land correctly
    // (e.g. Knoxville at -83.94 deg, not 108 deg E / China). Keep this in sync with
    // the same offset hardcoded in fs_earth, fs_rocket and vs_terrain.
    const float kLonOffset = 0.5f;
    const uint64_t kEarthFlags = BGFX_SAMPLER_V_CLAMP | BGFX_SAMPLER_MIN_ANISOTROPIC | BGFX_SAMPLER_MAG_ANISOTROPIC;
    earthColor_ = bgfxutil::loadTexture("src/renderer/bgfx/Earth-Color-Map-32768x16384.dds", kEarthFlags);
    EarthBumpMap::Get().Load("src/renderer/bgfx/Earth-Bump-Map-32768x16384.dds");
    earthNight_ = bgfxutil::loadTexture("src/renderer/bgfx/Earth-Night-Map-32768x16384.dds", kEarthFlags);
    earthEmiss_ = bgfxutil::loadTexture("src/renderer/bgfx/Earth-Night-Emission-Map-32768x16384.dds", kEarthFlags);  // white = light source strength
    earthCloud_ = bgfxutil::loadTexture("src/renderer/bgfx/Earth-Cloud-Map-32768x16384.dds", kEarthFlags);
    earthRough_ = bgfxutil::loadTexture("src/renderer/bgfx/Earth-Roughness-Map-32768x16384.dds", kEarthFlags);

    // Sphere for the cloud shells. Denser than a plain textured sphere would need
    // so the per-vertex noise displacement resolves smoothly. 32-bit indices.
    cloudMesh_  = CreateMesh(geom::buildSphere((float)EARTH_RADIUS, 512, 512, kLonOffset));

    // The solid surface at every altitude: quadtree LOD terrain (see terrain.hpp).
    terrain_.Init(layout_);
}

void BgfxBackend::DrawEarth(const EarthFrame& f) {
    ensureEarth();
    sunDirView_ = f.sun_dir;   // cache for lit DrawModel (the rocket), drawn later
    camPosView_ = f.cam_pos;
    earthCenterView_ = f.center;

    // Atmosphere FIRST: full-screen analytic single scattering, drawn before the
    // opaque earth so the earth's real (displaced) silhouette overwrites the inner
    // part. That keeps the limb glow exactly at the visible edge and hides the
    // sphere-clip discontinuity behind the planet -- no false horizon floating in
    // the disk. For each pixel we reconstruct the view ray, intersect the
    // atmosphere/planet spheres, and integrate Rayleigh+Mie in front of the planet,
    // so it is correct from any altitude.
    if (bgfx::getAvailTransientVertexBuffer(3, layout_) >= 3) {
        RVec3 fwd   = rmath::normalize(rmath::sub(cam_.target, cam_.position));
        RVec3 right = rmath::normalize(rmath::cross(fwd, cam_.up));
        RVec3 up    = rmath::cross(right, fwd);
        float tanH  = tanf(cam_.fovy * 0.5f * (float)(M_PI / 180.0));
        float aspect = height_ > 0 ? (float)width_ / (float)height_ : 1.0f;

        setVec4(u_camPos_, f.cam_pos);
        setVec4(u_earthCenter_, f.center);
        setVec4(u_sunDir_, f.sun_dir);
        // x: planet radius, y: atmosphere radius, z: scale height, w: exposure (km).
        // y: Kármán line (~100 km). z: Rayleigh scale height (~8.5 km). w: exposure
        // (raised to compensate for the thinner, denser-falloff air column).
        float at[4] = { (float)EARTH_RADIUS_KM, (float)EARTH_RADIUS_KM + 100.0f, 8.5f, 0.05f };
        bgfx::setUniform(u_atmos_, at);
        setVec4(u_rayFwd_, fwd);
        float rr[4] = { right.x*tanH*aspect, right.y*tanH*aspect, right.z*tanH*aspect, 0 };
        bgfx::setUniform(u_rayRight_, rr);
        float ru[4] = { up.x*tanH, up.y*tanH, up.z*tanH, 0 };
        bgfx::setUniform(u_rayUp_, ru);

        bgfx::TransientVertexBuffer tvb;
        bgfx::allocTransientVertexBuffer(&tvb, 3, layout_);
        Vertex* d = (Vertex*)tvb.data;
        d[0] = Vertex{ { -1, -1, 0 }, {0,0,0}, 0, 0, kWhite };   // full-screen triangle
        d[1] = Vertex{ {  3, -1, 0 }, {0,0,0}, 0, 0, kWhite };
        d[2] = Vertex{ { -1,  3, 0 }, {0,0,0}, 0, 0, kWhite };
        bgfx::setTexture(0, s_night_, bgfx::isValid(earthNight_) ? earthNight_ : white_);  // light-pollution colour
        bgfx::setTexture(1, s_emiss_, bgfx::isValid(earthEmiss_) ? earthEmiss_ : white_);  // light-pollution strength
        bgfx::setVertexBuffer(0, &tvb);
        bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_BLEND_ADD);
        bgfx::submit(0, atmosProg_);
    }

    // Camera altitude (km), for fading the cloud shells below.
    float dcx = f.cam_pos.x - f.center.x, dcy = f.cam_pos.y - f.center.y, dcz = f.cam_pos.z - f.center.z;
    float camAlt = sqrtf(dcx*dcx + dcy*dcy + dcz*dcz) - (float)EARTH_RADIUS_KM;

    // Solid surface: quadtree LOD chunks from orbit down to the ground, with
    // distance-tiered detail texturing up close. See terrain.hpp.
    {
        const EarthTextures tex = {
            bgfx::isValid(earthColor_) ? earthColor_ : white_,
            bgfx::isValid(EarthBumpMap::Get().Texture()) ? EarthBumpMap::Get().Texture() : white_,
            bgfx::isValid(earthNight_) ? earthNight_ : white_,
            bgfx::isValid(earthRough_) ? earthRough_ : white_,
            bgfx::isValid(earthCloud_) ? earthCloud_ : white_,
        };
        float aspect = height_ > 0 ? (float)width_ / (float)height_ : 1.0f;
        terrain_.Draw(f, cam_, aspect, far_, tex, EarthBumpMap::Get().Width(), EarthBumpMap::Get().Height());
    }
    float depth[4] = { far_, 0, 0, 0 };

    // Cloud shells: concentric cloud-map layers above the terrain. Drawn
    // inner->outer (back-to-front for the visible near hemisphere) with depth
    // test on (mountains occlude clouds) but no depth write (shells blend). The
    // slight radius offsets give a parallax / volumetric feel as the camera moves.
    // Fade the shells out as the camera descends into the 2-12 km layer: seen from
    // below/inside, the translucent spheres wash out the ground and you can't tell
    // cloud from terrain. Full clouds above ~35 km, gone by ~10 km.
    float cloudFade = (camAlt - 10.0f) / 25.0f;
    cloudFade = cloudFade < 0.0f ? 0.0f : (cloudFade > 1.0f ? 1.0f : cloudFade);
    if (cloudMesh_ && cloudFade > 0.001f) {
        const GpuMesh& cm  = meshes_[cloudMesh_ - 1];
        const int   kShells    = 5;
        const float baseKm     = 2.0f;    // cloud base ~2 km (low cumulus)
        const float gapKm      = 2.5f;    // shells span 2..12 km -> tops at the tropopause
        const float shellAlpha = 0.22f * cloudFade;  // per-shell opacity (x altitude fade);
                                          // base 0.22 thin enough the ground shadow shows
                                          // through (keep in sync with fs_earth shadow curve)
        const float dispAmp    = 3000.0f; // noise amplitude (m); ~> gap so shells merge
        const float dispFreq   = 10.0f;    // noise feature scale over the sphere
        for (int i = 0; i < kShells; ++i) {
            float factor = 1.0f + ((baseKm + i * gapKm) * 1000.0f) / (float)EARTH_RADIUS;
            RMat4 m = rmath::mul(f.model, rmath::scale(factor));
            setVec4(u_sunDir_, f.sun_dir);
            setVec4(u_earthCenter_, f.center);
            float ca[4] = { shellAlpha, 0, 0, 0 };
            bgfx::setUniform(u_cloudAlpha_, ca);
            float cd[4] = { dispAmp, dispFreq, i * 17.0f, 0 };   // per-shell seed
            bgfx::setUniform(u_cloudDisp_, cd);
            bgfx::setUniform(u_depth_, depth);
            bgfx::setTexture(0, s_cloud_, earthCloud_);
            bgfx::setTexture(1, s_night_, bgfx::isValid(earthNight_) ? earthNight_ : white_);  // city-light colour
            bgfx::setTexture(2, s_emiss_, bgfx::isValid(earthEmiss_) ? earthEmiss_ : white_);  // city-light strength
            bgfx::setTransform(m.m);
            bgfx::setVertexBuffer(0, cm.vbh);
            bgfx::setIndexBuffer(cm.ibh);
            bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                           | BGFX_STATE_DEPTH_TEST_LESS | BGFX_STATE_BLEND_ALPHA | BGFX_STATE_CULL_CW);
            bgfx::submit(0, cloudProg_);
        }
    }

    // --- Sun + lens flare (screen space, additive, on top of the 3D scene) ---
    if (bgfx::isValid(flareProg_)) {
        RVec3 fwd   = rmath::normalize(rmath::sub(cam_.target, cam_.position));
        RVec3 right = rmath::normalize(rmath::cross(fwd, cam_.up));
        RVec3 up    = rmath::cross(right, fwd);
        float tanH  = tanf(cam_.fovy * 0.5f * (float)(M_PI / 180.0));
        float aspect = height_ > 0 ? (float)width_ / (float)height_ : 1.0f;

        RVec3 S  = f.sun_dir;                       // unit, view space
        float sf = rmath::dot(S, fwd);              // > 0: sun in front of camera
        auto smooth01 = [](float a, float b, float x) {
            float t = (x - a) / (b - a); t = t < 0 ? 0 : (t > 1 ? 1 : t); return t*t*(3.0f - 2.0f*t);
        };
        if (sf > 0.02f) {
            // Sun screen position (NDC), inverse of the ray reconstruction.
            float nx = (rmath::dot(S, right) / sf) / (tanH * aspect);
            float ny = (rmath::dot(S, up)    / sf) / tanH;

            // Soft occlusion: fade as the sun passes behind the Earth's disk.
            RVec3 toC = rmath::sub(f.center, f.cam_pos);
            float dC  = rmath::length(toC);
            float cosA = dC > 1e-3f ? rmath::dot(S, RVec3{ toC.x/dC, toC.y/dC, toC.z/dC }) : -1.0f;
            float sunAng   = acosf(fmaxf(-1.0f, fminf(1.0f, cosA)));
            float earthAng = asinf(fminf(1.0f, (float)EARTH_RADIUS_KM / fmaxf(dC, (float)EARTH_RADIUS_KM)));
            float occ      = smooth01(earthAng * 0.95f, earthAng * 1.10f, sunAng);
            float edge     = fmaxf(fabsf(nx), fabsf(ny));
            float onScreen = 1.0f - smooth01(1.3f, 2.4f, edge);
            float master   = occ * onScreen;

            if (master > 0.003f) {
                std::vector<Vertex> fv;
                fv.reserve(9 * 6);
                auto add = [&](float t, float size, RColor c, float inten, float power, float ring) {
                    float cx = nx * (1.0f - t), cy = ny * (1.0f - t);   // along sun -> screen centre
                    float a  = fminf(1.0f, inten * master);
                    RColor col = { c.r, c.g, c.b, (unsigned char)(a * 255.0f) };
                    float sx = size / aspect, sy = size;
                    Vertex v0{ {cx-sx, cy-sy, 0}, {power, ring, 0}, -1, -1, col };
                    Vertex v1{ {cx+sx, cy-sy, 0}, {power, ring, 0},  1, -1, col };
                    Vertex v2{ {cx+sx, cy+sy, 0}, {power, ring, 0},  1,  1, col };
                    Vertex v3{ {cx-sx, cy+sy, 0}, {power, ring, 0}, -1,  1, col };
                    fv.push_back(v0); fv.push_back(v1); fv.push_back(v2);
                    fv.push_back(v0); fv.push_back(v2); fv.push_back(v3);
                };
                // Sun: warm glow + white-hot core.
                add(0.00f, 0.30f, { 255, 235, 200, 255 }, 0.60f,  2.0f, 0.0f);
                add(0.00f, 0.05f, { 255, 255, 255, 255 }, 1.00f, 10.0f, 0.0f);
                // Chromatic ghosts along the sun -> centre axis.
                add(0.30f, 0.06f, { 120, 160, 255, 255 }, 0.15f,  3.0f, 0.0f);
                add(0.50f, 0.11f, { 255, 180,  90, 255 }, 0.12f,  2.0f, 0.0f);
                add(0.62f, 0.04f, { 150, 255, 150, 255 }, 0.12f,  3.0f, 0.0f);
                add(0.80f, 0.15f, { 255, 110,  90, 255 }, 0.09f,  1.5f, 0.0f);
                add(1.10f, 0.08f, { 130, 150, 255, 255 }, 0.12f,  2.0f, 0.0f);
                add(1.30f, 0.20f, { 255, 210, 140, 255 }, 0.07f,  1.2f, 0.0f);
                // Halo ring around the screen centre.
                add(1.00f, 0.55f, { 150, 180, 255, 255 }, 0.06f,  1.0f, 0.78f);

                uint32_t n = (uint32_t)fv.size();
                if (bgfx::getAvailTransientVertexBuffer(n, layout_) >= n) {
                    bgfx::TransientVertexBuffer tvb;
                    bgfx::allocTransientVertexBuffer(&tvb, n, layout_);
                    std::memcpy(tvb.data, fv.data(), n * sizeof(Vertex));
                    bgfx::setVertexBuffer(0, &tvb);
                    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_BLEND_ADD);
                    bgfx::submit(1, flareProg_);   // view 1 = over the 3D scene, under the UI
                }
            }
        }
    }
}

// --- 2D overlay (view 1) ----------------------------------------------------

void BgfxBackend::submit2DTris(const Vertex* v, uint32_t count, RColor tint) {
    if (count < 3) return;
    if (bgfx::getAvailTransientVertexBuffer(count, layout_) < count) return;
    bgfx::TransientVertexBuffer tvb;
    bgfx::allocTransientVertexBuffer(&tvb, count, layout_);
    std::memcpy(tvb.data, v, count * sizeof(Vertex));

    bgfx::setVertexBuffer(0, &tvb);
    float t[4] = { tint.r / 255.0f, tint.g / 255.0f, tint.b / 255.0f, tint.a / 255.0f };
    bgfx::setUniform(u_tint_, t);
    float depth[4] = { far_, 0, 0, 0 };   // unused by 2D (no depth test) but keeps u_depth defined
    bgfx::setUniform(u_depth_, depth);
    float unlit[4] = { 0, 0, 0, 0 };      // 2D overlay is flat-coloured
    bgfx::setUniform(u_light_, unlit);
    bgfx::setTexture(0, s_tex_, white_);
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_BLEND_ALPHA);
    bgfx::submit(1, generic_);
}

void BgfxBackend::submitTextTris(const Vertex* v, uint32_t count, RColor tint, bgfx::TextureHandle atlas) {
    if (count < 3) return;
    if (bgfx::getAvailTransientVertexBuffer(count, layout_) < count) return;
    bgfx::TransientVertexBuffer tvb;
    bgfx::allocTransientVertexBuffer(&tvb, count, layout_);
    std::memcpy(tvb.data, v, count * sizeof(Vertex));

    bgfx::setVertexBuffer(0, &tvb);
    float t[4] = { tint.r / 255.0f, tint.g / 255.0f, tint.b / 255.0f, tint.a / 255.0f };
    bgfx::setUniform(u_tint_, t);
    float depth[4] = { far_, 0, 0, 0 };
    bgfx::setUniform(u_depth_, depth);
    float unlit[4] = { 0, 0, 0, 0 };
    bgfx::setUniform(u_light_, unlit);
    bgfx::setTexture(0, s_tex_, atlas);   // glyph atlas: white rgb, alpha = coverage
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_BLEND_ALPHA);
    bgfx::submit(1, generic_);
}

void BgfxBackend::DrawRect(int x, int y, int w, int h, RColor c) {
    float x0 = (float)x, y0 = (float)y, x1 = (float)(x + w), y1 = (float)(y + h);
    auto V = [](float px, float py) { return Vertex{ {px, py, 0}, {0,0,0}, 0, 0, kWhite }; };
    Vertex quad[6] = { V(x0,y0), V(x1,y0), V(x1,y1), V(x0,y0), V(x1,y1), V(x0,y1) };
    submit2DTris(quad, 6, c);
}

void BgfxBackend::DrawRectLines(int x, int y, int w, int h, RColor c) {
    DrawRect(x, y, w, 1, c);
    DrawRect(x, y + h - 1, w, 1, c);
    DrawRect(x, y, 1, h, c);
    DrawRect(x + w - 1, y, 1, h, c);
}

void BgfxBackend::DrawText(const char* text, int x, int y, int font_size, RColor c) {
    ensureGlyphFont();

    // Fallback: ASCII-only vector font if the TTF atlas is unavailable.
    if (!g_font.ready) {
        static char quads[64000];   // stb vertex buffer: 16 bytes/vertex, 4 verts/quad
        int nq = stb_easy_font_print((float)x, (float)y, (char*)text, nullptr, quads, sizeof(quads));
        if (nq <= 0) return;
        const float scale = font_size / 8.0f;   // stb base glyphs are ~8px tall
        struct SV { float x, y, z; unsigned char col[4]; };
        const SV* sv = (const SV*)quads;
        std::vector<Vertex> tris;
        tris.reserve(nq * 6);
        auto put = [&](const SV& s) {
            tris.push_back(Vertex{ { x + (s.x - x) * scale, y + (s.y - y) * scale, 0 }, {0,0,0}, 0, 0, kWhite });
        };
        for (int q = 0; q < nq; ++q) {
            const SV& v0 = sv[q*4+0]; const SV& v1 = sv[q*4+1];
            const SV& v2 = sv[q*4+2]; const SV& v3 = sv[q*4+3];
            put(v0); put(v1); put(v2);
            put(v0); put(v2); put(v3);
        }
        submit2DTris(tris.data(), (uint32_t)tris.size(), c);
        return;
    }

    // Textured glyph path. Quads come out of stb in baked pixels; scale to the
    // requested size and place the baseline ~0.75*size below the top-left (x,y).
    const float scale    = font_size / GlyphFont::kBakePx;
    const float baseline = y + font_size * 0.75f;
    float penx = 0.0f, peny = 0.0f;   // baked-pixel cursor (baseline at peny)

    std::vector<Vertex> tris;
    auto quad = [&](float x0, float y0, float x1, float y1, float s0, float t0, float s1, float t1) {
        auto V = [](float px, float py, float u, float v) { return Vertex{ {px, py, 0}, {0,0,0}, u, v, kWhite }; };
        tris.push_back(V(x0,y0,s0,t0)); tris.push_back(V(x1,y0,s1,t0)); tris.push_back(V(x1,y1,s1,t1));
        tris.push_back(V(x0,y0,s0,t0)); tris.push_back(V(x1,y1,s1,t1)); tris.push_back(V(x0,y1,s0,t1));
    };

    // Minimal UTF-8 decode (BMP is enough for ASCII + Greek).
    const unsigned char* s = (const unsigned char*)text;
    while (*s) {
        int cp = *s;
        if      (cp < 0x80) { s += 1; }
        else if ((cp >> 5) == 0x6 && s[1]) { cp = ((cp & 0x1F) << 6) | (s[1] & 0x3F); s += 2; }
        else if ((cp >> 4) == 0xE && s[1] && s[2]) {
            cp = ((cp & 0x0F) << 12) | ((s[1] & 0x3F) << 6) | (s[2] & 0x3F); s += 3;
        } else { s += 1; }   // skip malformed / non-BMP

        // Resolve codepoint -> (packed-char table, index), falling back to '?'.
        const stbtt_packedchar* base; int idx;
        if (cp >= GlyphFont::kAsciiFirst && cp < GlyphFont::kAsciiFirst + GlyphFont::kAsciiCount) {
            base = g_font.ascii.data(); idx = cp - GlyphFont::kAsciiFirst;
        } else if (cp >= GlyphFont::kGreekFirst && cp < GlyphFont::kGreekFirst + GlyphFont::kGreekCount) {
            base = g_font.greek.data(); idx = cp - GlyphFont::kGreekFirst;
        } else {
            base = g_font.ascii.data(); idx = '?' - GlyphFont::kAsciiFirst;
        }
        stbtt_aligned_quad q;
        stbtt_GetPackedQuad(base, GlyphFont::kAtlasW, GlyphFont::kAtlasH, idx, &penx, &peny, &q, 0);
        quad(x + q.x0 * scale, baseline + q.y0 * scale,
             x + q.x1 * scale, baseline + q.y1 * scale,
             q.s0, q.t0, q.s1, q.t1);
    }
    if (!tris.empty()) submitTextTris(tris.data(), (uint32_t)tris.size(), c, g_font.atlas);
}

void BgfxBackend::DrawFPS(int x, int y) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%d FPS", (int)(fps_ + 0.5));
    DrawText(buf, x, y, 16, RColor{ 0, 228, 48, 255 });
}

}
