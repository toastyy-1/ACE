#pragma once

#include "../render_types.hpp"

// The raylib build's colours, after aircraft displays: fully saturated on
// black, each colour with one job.

namespace renderer::theme {

inline constexpr RColor kPrimary  { 255, 255, 255, 255 };  // symbology and values
inline constexpr RColor kSelect   {   255, 0, 0, 255 };  // labels, references, flown path
inline constexpr RColor kActive   {  40, 255,  40, 255 };  // active state, launch site
inline constexpr RColor kRoute    { 255,  60, 255, 255 };  // plan: predicted path, aim point
inline constexpr RColor kCaution  { 255, 200,   0, 255 };
inline constexpr RColor kWarn     { 255,  40,  40, 255 };
inline constexpr RColor kInactive { 130, 130, 130, 255 };  // off / secondary
inline constexpr RColor kBorder   {  90,  90,  90, 255 };  // box outlines
inline constexpr RColor kGrid     {  35,  65,  95, 255 };  // the globe's grid

inline RColor withAlpha(RColor c, unsigned char a) { c.a = a; return c; }

// The shared renderer paints with raylib's named colours (render_types.hpp).
// Map those onto the palette above by what they mean; anything else passes
// through unchanged. Alpha is kept.
inline RColor Remap(RColor c) {
    auto is = [&](RColor k) { return c.r == k.r && c.g == k.g && c.b == k.b; };
    RColor out = c;
    if      (is(renderer::kYellow))  out = kRoute;      // predicted path, velocity arrow
    else if (is(renderer::kSkyBlue)) out = kSelect;     // flown path
    else if (is(renderer::kGray))    out = kInactive;   // the other rockets' paths
    else if (is(renderer::kGreen))   out = kActive;     // launch site
    else if (is(renderer::kRed))     out = kRoute;      // aim point
    else if (is(renderer::kOrange))  out = kCaution;    // acceleration arrow
    else if (is(renderer::kWhite))   out = kPrimary;
    else return c;
    out.a = c.a;
    return out;
}

}
