#include "hud.hpp"
#include "theme.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace renderer {

namespace {

// Condensed sans with Greek for the rocket IDs. Relative to the repo root.
const char* kFontPath = "src/renderer/assets/NotoSans-SemiCondensedSemiBold.ttf";
// Every size the HUD draws at, each baked at 1:1 so glyphs land on whole pixels.
// raylib sizes a font by its full line height, so these read about three
// quarters as big as the same number in points.
const int kSizes[] = { 18, 20, 24, 30 };

constexpr int kMargin = 16;   // gap to the screen edge
constexpr int kInset  = 12;   // text inset inside a box
constexpr int kRow    = 30;   // data row height
constexpr int kCap    = 20;   // caption text size
constexpr int kVal    = 24;   // value text size
constexpr int kKeysH  = 36;   // soft-key strip along the bottom

inline Color rl(RColor c) { return { c.r, c.g, c.b, c.a }; }

// printf into a rotating set of buffers, so a few can be alive at once.
const char* fmt(const char* f, ...) {
    static char bufs[8][128];
    static int  next = 0;
    char* buf = bufs[next++ & 7];
    va_list ap;
    va_start(ap, f);
    vsnprintf(buf, 128, f, ap);
    va_end(ap);
    return buf;
}

// "00:01:23"
std::string clock(double t) {
    t = std::max(0.0, t);
    int s = (int)t;
    return fmt("%02d:%02d:%02d", s / 3600, (s / 60) % 60, s % 60);
}

std::vector<int> codepointSet() {
    std::vector<int> cp;
    for (int c = 32; c <= 126; ++c) cp.push_back(c);
    for (int c = 0x0370; c <= 0x03FF; ++c) cp.push_back(c);   // Greek, for the rocket IDs
    cp.push_back(0x00B0);                                      // °
    cp.push_back(0x00B1);                                      // ±
    return cp;
}

void diamond(float x, float y, float s, RColor c) {
    DrawLineV({ x, y - s }, { x + s, y }, rl(c));
    DrawLineV({ x + s, y }, { x, y + s }, rl(c));
    DrawLineV({ x, y + s }, { x - s, y }, rl(c));
    DrawLineV({ x - s, y }, { x, y - s }, rl(c));
}

} // namespace

// --- setup -------------------------------------------------------------------

void DisplayHud::Init() {
    std::vector<int> cp = codepointSet();
    for (int size : kSizes) {
        Face f;
        f.size = size;
        f.font = LoadFontEx(kFontPath, size, cp.data(), (int)cp.size());
        if (f.font.texture.id == 0 || f.font.glyphCount == 0) f.font = GetFontDefault();
        SetTextureFilter(f.font.texture, TEXTURE_FILTER_POINT);
        faces_.push_back(f);
    }
}

void DisplayHud::Shutdown() {
    for (Face& f : faces_)
        if (f.font.texture.id != GetFontDefault().texture.id) UnloadFont(f.font);
    faces_.clear();
}

const DisplayHud::Face& DisplayHud::face(int size) const {
    const Face* best = &faces_.front();
    for (const Face& f : faces_)
        if (std::abs(f.size - size) < std::abs(best->size - size)) best = &f;
    return *best;
}

void DisplayHud::Text(const char* s, int x, int y, int size, RColor c) const {
    const Face& f = face(size);
    DrawTextEx(f.font, s, { (float)x, (float)y }, (float)f.size, 0.0f, rl(c));
}

int DisplayHud::width(const char* s, int size) const {
    const Face& f = face(size);
    return (int)std::ceil(MeasureTextEx(f.font, s, (float)f.size, 0.0f).x);
}

// --- liftoff ------------------------------------------------------------------

void DisplayHud::track(const HudFrame& hud) {
    // Whether each vehicle has left the pad, for the PAD / COAST state.
    const size_t n = hud.rockets.size();
    if (tracks_.size() != n) {
        tracks_.clear();
        for (const HudRocket& r : hud.rockets) tracks_.push_back({ r.alt_km > 0.05 });
        return;
    }
    for (size_t i = 0; i < n; ++i) {
        const HudRocket& r = hud.rockets[i];
        if (r.thrust > 0.35f && r.vspeed > 2.0) tracks_[i].lifted = true;
    }
}

// --- drawing -----------------------------------------------------------------

void DisplayHud::Draw(const HudFrame& hud, const HudView& v) {
    track(hud);

    // Sim time per wall second, refreshed twice a second.
    if (rateWall_ < 0.0 || v.now - rateWall_ >= 0.5) {
        if (rateWall_ >= 0.0) rate_ = (hud.met - rateMet_) / (v.now - rateWall_);
        rateMet_ = hud.met;
        rateWall_ = v.now;
    }

    worldLabels(v);
    if (hud.show_labels) markers(hud, v);
    if (v.haveHeading) heading(v);
    if (hud.show_telemetry) {
        flightData(hud, v);
        gimbal(hud, v);
    }
    corner(hud, v);
    softKeys(hud, v);
}

void DisplayHud::box(int x, int y, int w, int h, const char* title) const {
    DrawRectangle(x, y, w, h, BLACK);
    DrawRectangleLines(x, y, w, h, rl(theme::kBorder));
    if (title) Text(title, x + kInset, y + 8, kCap, theme::kSelect);
}

void DisplayHud::heading(const HudView& v) const {
    // Boxed heading readout over a tape of ±45 deg, like a nav display's TRK box.
    const int cx = v.width / 2, top = kMargin;
    const char* val = fmt("%03d", (int)std::lround(v.headingDeg) % 360);
    const int vw = width(val, 30), bw = vw + 16;
    Text("HDG", cx - bw / 2 - 8 - width("HDG", kCap), top + 7, kCap, theme::kSelect);
    DrawRectangleLines(cx - bw / 2, top, bw, 34, rl(theme::kPrimary));
    Text(val, cx - vw / 2, top + 2, 30, theme::kPrimary);

    const int   w = 480, line = top + 44;
    const float ppd = w / 90.0f;
    const double h = v.headingDeg;
    DrawLine(cx - w / 2, line, cx + w / 2, line, rl(theme::kBorder));
    for (int deg = (int)std::floor((h - 45.0) / 5.0) * 5; deg <= h + 45.0; deg += 5) {
        float x = cx + (float)(deg - h) * ppd;
        if (x < cx - w / 2 || x > cx + w / 2) continue;
        int d = ((deg % 360) + 360) % 360;
        DrawLine((int)x, line, (int)x, line + (d % 10 == 0 ? 10 : 5), rl(theme::kPrimary));
        if (d % 30 == 0) {
            const char* cardinal[4] = { "N", "E", "S", "W" };
            const char* s = d % 90 == 0 ? cardinal[d / 90] : fmt("%d", d / 10);
            Text(s, (int)x - width(s, 18) / 2, line + 12, 18, theme::kPrimary);
        }
    }
    // Lubber line: where the camera points.
    DrawTriangle({ (float)cx, (float)line }, { (float)cx - 6, (float)line - 8 },
                 { (float)cx + 6, (float)line - 8 }, rl(theme::kPrimary));
}

void DisplayHud::flightData(const HudFrame& hud, const HudView&) {
    const bool live = hud.primary >= 0;
    const HudRocket* p = live ? &hud.rockets[hud.primary] : nullptr;

    // Propellant reference: reset on a new selection or at staging.
    if (live && (hud.primary != fuelFor_ || p->length < fuelLength_ - 0.05)) {
        fuelFor_ = hud.primary;
        fuelMax_ = hud.fuel;
    }
    if (live) {
        fuelMax_    = std::max(fuelMax_, hud.fuel);
        fuelLength_ = p->length;
    }

    const char* state = "--";
    RColor stateC = theme::kInactive;
    if (live) {
        bool lifted = (size_t)hud.primary < tracks_.size() && tracks_[hud.primary].lifted;
        if (p->detonated)          { state = "LOST";  stateC = theme::kWarn; }
        else if (p->thrust > 0.5f) { state = "BURN";  stateC = theme::kActive; }
        else if (!lifted)          { state = "PAD";   stateC = theme::kPrimary; }
        else                       { state = "COAST"; stateC = theme::kPrimary; }
    }

    // Rows are collected first to size the box, then drawn. An empty caption
    // is a gap between groups.
    struct Row { const char* caption; std::string value; RColor color; };
    std::vector<Row> rows;
    auto row = [&](const char* c, const std::string& val, RColor col = theme::kPrimary) {
        rows.push_back({ c, val, col });
    };
    auto gap = [&]() { rows.push_back({ "", "", theme::kPrimary }); };

    row("VEH",   live ? fmt("%s  %d/%zu", p->id.c_str(), hud.primary + 1, hud.rockets.size()) : "--");
    row("MET",   clock(hud.met));
    row("STATE", state, stateC);
    gap();
    row("ALT",   fmt("%.3f KM", hud.alt_km));
    row("VEL",   fmt("%.1f M/S", hud.speed));
    row("V/S",   fmt("%+.1f M/S", hud.vspeed));
    row("ACC",   fmt("%.2f G", hud.accel / 9.80665));
    row("PITCH", fmt("%+.2f°", hud.pitch_deg));
    gap();
    row("ROLL RATE",  fmt("%+.2f °/S", hud.rates_dps.x));
    row("PITCH RATE", fmt("%+.2f °/S", hud.rates_dps.y));
    row("YAW RATE",   fmt("%+.2f °/S", hud.rates_dps.z));
    gap();
    row("LAT",   fmt("%.4f° %c", std::fabs(hud.lat_deg), hud.lat_deg >= 0 ? 'N' : 'S'));
    row("LON",   fmt("%.4f° %c", std::fabs(hud.lon_deg), hud.lon_deg >= 0 ? 'E' : 'W'));
    row("TGT",   fmt("%.1f KM", hud.target_range_km), theme::kRoute);
    gap();
    row("ECEF X", fmt("%+.3f KM", hud.pos_km.x));
    row("ECEF Y", fmt("%+.3f KM", hud.pos_km.y));
    row("ECEF Z", fmt("%+.3f KM", hud.pos_km.z));

    // Two columns: captions left-aligned, values right-aligned. The value
    // column is sized for the widest value it shows, so the box doesn't
    // change width as the numbers do.
    int capW = std::max(width("PROP", kCap), width("THR", kCap));
    for (const Row& r : rows) capW = std::max(capW, width(r.caption, kCap));
    const int valW = width("+00000.000 KM", kVal);
    const int w    = kInset + capW + 24 + valW + kInset;
    const int gauges = 2;
    int h = kInset;
    for (const Row& r : rows) h += r.caption[0] ? kRow : kRow / 2;
    h += kRow / 2 + gauges * kRow + kInset;

    const int x = kMargin, y = kMargin;
    const int left = x + kInset, right = x + w - kInset, valX = right - valW;
    box(x, y, w, h, nullptr);

    // Captions sit on the same baseline as their (larger) values.
    const int capDy = kVal - kCap;
    // A gap between groups gets a thin divider across the box.
    auto divider = [&](int ty) {
        DrawLine(left, ty + kRow / 4, right, ty + kRow / 4, rl(theme::kBorder));
    };

    int ty = y + kInset;
    for (const Row& r : rows) {
        if (!r.caption[0]) { divider(ty); ty += kRow / 2; continue; }
        Text(r.caption, left, ty + capDy, kCap, theme::kSelect);
        Text(r.value.c_str(), right - width(r.value.c_str(), kVal), ty, kVal, r.color);
        ty += kRow;
    }
    divider(ty);
    ty += kRow / 2;

    // Bar gauges in the value column: outlined, filled to the level, with the
    // percentage right-aligned like the values above.
    const int pctW = width("100%", kVal);
    auto gauge = [&](const char* caption, float frac, RColor fill) {
        frac = std::clamp(frac, 0.0f, 1.0f);
        const char* pct = fmt("%d%%", (int)std::lround(frac * 100.0f));
        Text(caption, left, ty + capDy, kCap, theme::kSelect);
        Text(pct, right - width(pct, kVal), ty, kVal, theme::kPrimary);
        const int bx = valX, bw = right - pctW - 12 - bx, by = ty + 5, bh = 16;
        DrawRectangleLines(bx, by, bw, bh, rl(theme::kPrimary));
        DrawRectangle(bx + 2, by + 2, (int)((bw - 4) * frac), bh - 4, rl(fill));
        ty += kRow;
    };
    float prop = fuelMax_ > 0.0 ? (float)(hud.fuel / fuelMax_) : 0.0f;
    gauge("PROP", prop, prop < 0.05f ? theme::kWarn : (prop < 0.15f ? theme::kCaution : theme::kActive));
    gauge("THR",  live ? p->thrust : 0.0f, theme::kActive);
}

void DisplayHud::gimbal(const HudFrame& hud, const HudView& v) const {
    // Nozzle deflection on a two-axis indicator; the ring is the scale.
    const int w = 240, h = 290;
    const int x = v.width - kMargin - w, y = v.height - kKeysH - kMargin - h;
    box(x, y, w, h, "GIMBAL");

    const float gx = (float)hud.gimbal_x_deg, gy = (float)hud.gimbal_y_deg;
    const float mag = std::sqrt(gx * gx + gy * gy);
    const float range = mag > 10.0f ? 20.0f : (mag > 5.0f ? 10.0f : 5.0f);
    const char* rs = fmt("%.0f°", range);
    Text(rs, x + w - kInset - width(rs, kCap), y + 8, kCap, theme::kInactive);

    const int   cx = x + w / 2, cy = y + 40 + 92;
    const float r = 84.0f, ppd = r / range;
    DrawLine(cx - (int)r, cy, cx + (int)r, cy, rl(theme::kBorder));
    DrawLine(cx, cy - (int)r, cx, cy + (int)r, rl(theme::kBorder));
    DrawCircleLines(cx, cy, r * 0.5f, rl(theme::kBorder));
    DrawCircleLines(cx, cy, r, rl(theme::kPrimary));

    RColor c = mag > 0.8f * range ? theme::kCaution : theme::kActive;
    diamond(cx + gx * ppd, cy - gy * ppd, 7.0f, c);

    const int ty = y + h - kInset - kVal - 2, capDy = kVal - kCap;
    const char* xs = fmt("%+.2f°", gx);
    const char* ys = fmt("%+.2f°", gy);
    Text("X", x + kInset, ty + capDy, kCap, theme::kSelect);
    Text(xs, x + kInset + 20, ty, kVal, theme::kPrimary);
    Text(ys, x + w - kInset - width(ys, kVal), ty, kVal, theme::kPrimary);
    Text("Y", x + w - kInset - width(ys, kVal) - 20, ty + capDy, kCap, theme::kSelect);
}

void DisplayHud::softKeys(const HudFrame& hud, const HudView& v) const {
    // Labels along the bottom edge, one per number key; the ones switched on
    // are boxed.
    const int y = v.height - kKeysH;
    DrawRectangle(0, y, v.width, kKeysH, BLACK);
    DrawLine(0, y, v.width, y, rl(theme::kBorder));

    // Keys start on the screen margin, flush with the data block above them.
    const int key = 160, gap = 12;
    int x = kMargin;
    for (const HudFrame::Toggle& t : hud.toggles) {
        std::string name = std::to_string(t.key) + " " + t.name;
        for (char& ch : name) ch = (char)std::toupper((unsigned char)ch);
        if (t.on) DrawRectangleLines(x, y + 4, key, kKeysH - 8, rl(theme::kPrimary));
        Text(name.c_str(), x + (key - width(name.c_str(), kCap)) / 2, y + 6, kCap,
             t.on ? theme::kPrimary : theme::kInactive);
        x += key + gap;
    }
    const char* hints = "TAB NEXT    F CENTER    WASD/QE MOVE    WHEEL ZOOM";
    Text(hints, v.width - kMargin - width(hints, kCap), y + 6, kCap, theme::kInactive);
}

void DisplayHud::corner(const HudFrame& hud, const HudView& v) {
    const char* s = fmt("SIM x%.1f     %d FPS", rate_, v.fps);
    Text(s, v.width - kMargin - width(s, kCap), kMargin, kCap, theme::kInactive);
    // A lost vehicle gets a red caution tile, like a warning annunciator.
    if (hud.primary >= 0 && hud.rockets[hud.primary].detonated) {
        const char* lost = "VEHICLE LOST";
        int w = width(lost, kVal) + 20;
        int x = v.width - kMargin - w, y = kMargin + 30;
        DrawRectangle(x, y, w, 34, rl(theme::kWarn));
        Text(lost, x + 10, y + 5, kVal, kBlack);
    }
}

void DisplayHud::markers(const HudFrame& hud, const HudView& v) const {
    // The other vehicles: a diamond and their ID. The selected one is at the
    // centre of the view and named in the data block.
    for (size_t i = 0; i < hud.rockets.size(); ++i) {
        if ((int)i == hud.primary) continue;
        const HudRocket& r = hud.rockets[i];
        ScreenPoint sp = v.project(r.view_pos);
        if (!sp.visible || sp.x < 0 || sp.y < 0 || sp.x > v.width || sp.y > v.height - kKeysH) continue;
        if (r.detonated) {
            DrawLineV({ sp.x - 5, sp.y - 5 }, { sp.x + 5, sp.y + 5 }, rl(theme::kWarn));
            DrawLineV({ sp.x - 5, sp.y + 5 }, { sp.x + 5, sp.y - 5 }, rl(theme::kWarn));
            Text(r.id.c_str(), (int)sp.x + 9, (int)sp.y - 22, kCap, theme::kWarn);
            continue;
        }
        diamond(sp.x, sp.y, 5.0f, theme::kPrimary);
        Text(r.id.c_str(), (int)sp.x + 9, (int)sp.y - 22, kCap, theme::kPrimary);
    }
}

void DisplayHud::worldLabels(const HudView& v) const {
    if (!v.labels) return;
    for (const WorldLabel& l : *v.labels) {
        ScreenPoint sp = v.project(l.pos);
        if (!sp.visible) continue;
        int w = width(l.text.c_str(), kCap);
        int x = (int)sp.x + 8, y = (int)sp.y - 10;
        if (x + w < 0 || x > v.width || y < 0 || y > v.height - kKeysH) continue;
        DrawRectangle(x - 3, y - 1, w + 6, kCap + 2, BLACK);   // keeps it legible over the grid
        Text(l.text.c_str(), x, y, kCap, l.color);
    }
}

}
