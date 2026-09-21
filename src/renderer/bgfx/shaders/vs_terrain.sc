$input a_position
$output v_texcoord0, v_wpos, v_local, v_logz

#include <bgfx_shader.sh>

// One chunk of the cube-sphere terrain quadtree (terrain_lod.hpp, bgfx/terrain.cpp).
// The same flat grid is drawn once per selected chunk: a_position.xy in [-1,1]
// spans the chunk in face angle, a_position.z = 1 marks the skirt ring that hangs
// below the border to hide cracks against neighbours at another level.
//
// Precision: each chunk is its own floating origin. The CPU passes the chunk's
// sea-level centre in view space (computed in double), and everything here is an
// *offset* from that centre built only from small, cancellation-free terms, so
// the surface is exact to well under a millimetre at any zoom and never swims.
//
// Height: the same R8 map, sampled bilinearly exactly as EarthBumpMap does for the
// sim, so the rendered ground is the ground the sim collides with.

SAMPLER2D(s_bump, 1);          // height map (R8, EarthBumpMap)
uniform vec4 u_depth;          // x: far plane (logarithmic depth)
uniform vec4 u_terrain;        // x: sea-level radius (m), y: height scale (m), zw: height map size (texels)
uniform vec4 u_chunkCube;      // xyz: cube point at the chunk centre (body frame), w: its length
uniform vec4 u_chunkA;         // xyz: face A axis (body)
uniform vec4 u_chunkB;         // xyz: face B axis (body)
uniform vec4 u_chunkTrig;      // sin, cos of the centre angle along A, then along B
uniform vec4 u_chunkOrigin;    // xyz: sea-level chunk centre, view space (km), w: half-size (rad)
uniform vec4 u_chunkTex;       // xy: height-map texel under the centre (integer part), zw: its fraction
uniform vec4 u_chunkLod;       // x: height-map mip (0 = exact bilinear), y: skirt depth (m)

// sin / cos / atan of the small angles inside a chunk, by series. GPU sin, cos
// and atan carry ~1e-6 *absolute* error, which is a large relative error on the
// ~1e-4 rad offsets inside deep chunks: neighbours then disagree about their
// shared edge by metres and the skirts show through. Valid for |x| <= pi/16
// (TerrainLodParams::min_level >= 2 keeps chunk half-sizes there).
float sinSmall(float x) {
    float x2 = x*x;
    return x * (1.0 - x2/6.0 * (1.0 - x2/20.0 * (1.0 - x2/42.0 * (1.0 - x2/72.0))));
}
float cosSmall(float x) {
    float x2 = x*x;
    return 1.0 - x2/2.0 * (1.0 - x2/12.0 * (1.0 - x2/30.0 * (1.0 - x2/56.0)));
}
// atan(y / x) for x > 0; the series where |y/x| is small, the builtin otherwise.
float atanSmall(float y, float x) {
    if (x <= 0.0 || abs(y) > 0.25 * x) return atan(y, x);
    float t = y / x, t2 = t*t;
    return t * (1.0 - t2 * (1.0/3.0 - t2 * (1.0/5.0 - t2 * (1.0/7.0 - t2 * (1.0/9.0 - t2 * (1.0/11.0 - t2 * (1.0/13.0)))))));
}

// tan(a0 + d) - tan(a0) = sin d / (cos(a0 + d) cos a0), with sc0 = (sin a0, cos a0)
// exact from the CPU; nothing large is subtracted.
float tanDelta(vec2 sc0, float d) {
    float sd = sinSmall(d);
    float c1 = sc0.y * cosSmall(d) - sc0.x * sd;   // cos(a0 + d), >= cos(pi/4)
    return sd / (c1 * sc0.y);
}

// Height in [0,1] at texel coordinate base + r (base integer, r small).
float heightAt(vec2 base, vec2 r) {
    vec2 size = u_terrain.zw;
    if (u_chunkLod.x > 0.0)                      // quads span texels: filtered mip
        return texture2DLod(s_bump, (base + r) / size, u_chunkLod.x).x;

    // Four texel-centre fetches + weights computed here: hardware bilinear only
    // has ~8 bits of weight precision, which terraces slopes up close. Indexing
    // matches EarthBumpMap::sampleHeight01 (wrap in u via the sampler, clamp in v).
    vec2 f  = r - 0.5;
    vec2 i  = floor(f);
    vec2 w  = f - i;
    vec2 t0 = (base + i + 0.5) / size;
    vec2 t1 = t0 + 1.0 / size;
    float h00 = texture2DLod(s_bump, t0, 0.0).x;
    float h10 = texture2DLod(s_bump, vec2(t1.x, t0.y), 0.0).x;
    float h01 = texture2DLod(s_bump, vec2(t0.x, t1.y), 0.0).x;
    float h11 = texture2DLod(s_bump, t1, 0.0).x;
    return mix(mix(h00, h10, w.x), mix(h01, h11, w.x), w.y);
}

void main() {
    vec3  c  = u_chunkCube.xyz;
    float Lc = u_chunkCube.w;
    float hs = u_chunkOrigin.w;

    // Cube-plane offset of this vertex from the centre (equal-angle mapping).
    vec3 d = u_chunkA.xyz * tanDelta(u_chunkTrig.xy, a_position.x * hs)
           + u_chunkB.xyz * tanDelta(u_chunkTrig.zw, a_position.y * hs);

    // normalize(c + d) - normalize(c), rearranged so nothing large cancels.
    float cd = dot(c, d);
    float dd = dot(d, d);
    float Lq = sqrt(Lc*Lc + 2.0*cd + dd);
    vec3  dn = (d*Lc - c*((2.0*cd + dd) / (Lc + Lq))) / (Lq*Lc);
    vec3  n  = c / Lc + dn;                              // unit direction (body)

    // Longitude / colatitude offsets from the centre via the angle-difference
    // identities, again from small terms only. (1e-30 keeps atan defined at a pole.)
    float rc2  = dot(c.xy, c.xy);
    float rc   = sqrt(rc2);
    vec2  q    = c.xy + d.xy;
    float rq   = sqrt(dot(q, q));
    float cdxy = dot(c.xy, d.xy);
    float dlon = atanSmall(c.x*d.y - c.y*d.x, rc2 + cdxy + 1e-30);
    float drho = (2.0*cdxy + dot(d.xy, d.xy)) / max(rq + rc, 1e-30);
    float dcol = atanSmall(c.z*drho - rc*d.z, rc*rq + c.z*(c.z + d.z));

    vec2 size = u_terrain.zw;
    vec2 r    = u_chunkTex.zw + vec2(dlon * size.x * (1.0/6.28318531), dcol * size.y * (1.0/3.14159265));
    float h   = heightAt(u_chunkTex.xy, r);

    float elev = h * u_terrain.y - a_position.z * u_chunkLod.y;    // skirt ring hangs down
    vec3  off  = dn * u_terrain.x + n * elev;                        // body metres from the anchor
    vec3  wpos = u_chunkOrigin.xyz + vec3(off.x, off.z, -off.y) * 0.001;   // body -> view (km)

    v_wpos      = wpos;
    v_local     = off;
    v_texcoord0 = (u_chunkTex.xy + r) / size;
    gl_Position = mul(u_viewProj, vec4(wpos, 1.0));
    // Logarithmic depth like the other shaders, but written per fragment (fs_earth)
    // instead of into gl_Position.z: that z is non-linear across a triangle, and
    // chunks near the camera are large enough that the clipper then cuts visible
    // ground away. Clip z stays the plain projection.
    v_logz = 1.0 + gl_Position.w;
}
