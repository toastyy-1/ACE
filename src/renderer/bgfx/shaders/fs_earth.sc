$input v_texcoord0, v_wpos, v_local, v_logz

#include <bgfx_shader.sh>

SAMPLER2D(s_color, 0);   // daytime albedo
SAMPLER2D(s_bump,  1);   // height / relief
SAMPLER2D(s_night, 2);   // city lights
SAMPLER2D(s_rough, 3);   // roughness (low over water -> glossy ocean glint)
SAMPLER2D(s_cloud, 4);   // cloud cover (for cloud shadows on the ground)
SAMPLER2D(s_detail, 5);  // tileable ground detail (TerrainDetail): r soil/veg, g rock, b sand, a snow
uniform vec4 u_sunDir;       // xyz: view-space direction TO the sun
uniform vec4 u_earthCenter;  // xyz: view-space sphere centre (km)
uniform vec4 u_camPos;       // xyz: view-space camera (km)
uniform vec4 u_depth;        // x: far plane (logarithmic depth)
uniform vec4 u_detail[4];    // per detail layer, coarse -> fine: xyz chunk anchor in tiles
                             // (wrapped to [0,1)), w: 1/period (1/m). v_local is the
                             // body-frame offset (m) from that anchor (see vs_terrain).
uniform vec4 u_detailMask;   // per chunk: xyz 1 = that triplanar projection has weight
                             // here, w = number of detail layers still visible here

// Smooth (B-spline) bicubic upsample in 4 bilinear taps. Magnifying the single
// global map with hardware bilinear shows hard texel cells; this rounds them off
// into smooth gradients so the ground stops looking blocky up close.
vec4 cubicW(float v) {
    vec4 n = vec4(1.0, 2.0, 3.0, 4.0) - v;
    vec4 s = n * n * n;
    float x = s.x;
    float y = s.y - 4.0 * s.x;
    float z = s.z - 4.0 * s.y + 6.0 * s.x;
    float w = 6.0 - x - y - z;
    return vec4(x, y, z, w) * (1.0 / 6.0);
}
vec3 texBicubic(sampler2D tex, vec2 uv, vec2 texSize) {
    vec2 invSize = 1.0 / texSize;
    uv = uv * texSize - 0.5;
    vec2 f = fract(uv);
    uv -= f;
    vec4 xc = cubicW(f.x), yc = cubicW(f.y);
    vec4 c  = uv.xxyy + vec4(-0.5, 1.5, -0.5, 1.5);
    vec4 s  = vec4(xc.x + xc.y, xc.z + xc.w, yc.x + yc.y, yc.z + yc.w);
    vec4 o  = c + vec4(xc.y, xc.w, yc.y, yc.w) / s;
    o *= invSize.xxyy;
    vec3 s0 = texture2D(tex, o.xz).xyz;
    vec3 s1 = texture2D(tex, o.yz).xyz;
    vec3 s2 = texture2D(tex, o.xw).xyz;
    vec3 s3 = texture2D(tex, o.yw).xyz;
    float sx = s.x / (s.x + s.y);
    float sy = s.z / (s.z + s.w);
    return mix(mix(s3, s2, sx), mix(s1, s0, sx), sy);
}

// Triplanar sample of the detail texture at p (tiles) with weights w. Projections
// are skipped per *chunk* (u_detailMask), never per pixel: the branch is uniform
// across the draw, so the fetches keep implicit derivatives and anisotropic
// filtering. (texture2DGrad would allow per-pixel skips, but bgfx's GLSL 1.x
// path emits it as an undefined textureGradARB.)
vec4 detailTriplanar(vec3 p, vec3 w) {
    vec4  acc = vec4_splat(0.0);
    float ws  = 0.0;
    if (u_detailMask.x > 0.5) { acc += w.x * texture2D(s_detail, p.yz); ws += w.x; }
    if (u_detailMask.y > 0.5) { acc += w.y * texture2D(s_detail, p.zx); ws += w.y; }
    if (u_detailMask.z > 0.5) { acc += w.z * texture2D(s_detail, p.xy); ws += w.z; }
    return acc / max(ws, 1e-4);
}

// Weights (sum 1) of the four detail materials -- soil/vegetation, rock, sand,
// snow, matching s_detail's channels -- guessed from the global albedo and the
// local macro slope. Soil/vegetation is the default; the others need evidence.
vec4 groundMaterials(vec3 albedo, float slope) {
    float luma = dot(albedo, vec3(0.299, 0.587, 0.114));
    float mx   = max(albedo.r, max(albedo.g, albedo.b));
    float mn   = min(albedo.r, min(albedo.g, albedo.b));
    float sat  = (mx - mn) / max(mx, 1e-3);
    float snow = smoothstep(0.45, 0.65, luma) * (1.0 - smoothstep(0.12, 0.25, sat));
    float sand = smoothstep(0.3, 0.45, luma) * smoothstep(0.08, 0.16, albedo.r - albedo.b) * (1.0 - snow);
    float rock = smoothstep(0.08, 0.3, slope) * (1.0 - snow);
    float soil = max(1.0 - snow - sand - rock, 0.0) + 0.05;
    vec4  m    = vec4(soil, rock, sand, snow);
    return m / dot(m, vec4_splat(1.0));
}

void main() {
    vec3 N = normalize(v_wpos - u_earthCenter.xyz);
    vec3 L = normalize(u_sunDir.xyz);
    vec3 V = normalize(u_camPos.xyz - v_wpos);

    // Bump relief: perturb N from the height-map gradient. The renderer rotates
    // ECI +Z (north) to view +Y, so the pole axis is +Y; build an east/north
    // tangent frame around N to apply the gradient. The step spans several texels
    // (one texel is ~1km, gradients there are tiny) and the strength is large
    // because the map is global-scale elevation; both are tuned for visible relief.
    vec2  texel = vec2(1.0/32768.0, 1.0/16384.0) * 4.0;
    float hL = texture2D(s_bump, v_texcoord0 - vec2(texel.x, 0.0)).x;
    float hR = texture2D(s_bump, v_texcoord0 + vec2(texel.x, 0.0)).x;
    float hD = texture2D(s_bump, v_texcoord0 - vec2(0.0, texel.y)).x;
    float hU = texture2D(s_bump, v_texcoord0 + vec2(0.0, texel.y)).x;
    vec3  east  = normalize(cross(vec3(0.0, 1.0, 0.0), N));
    vec3  north = cross(N, east);
    vec3  Np    = normalize(N + (east*(hL - hR) + north*(hD - hU)) * 40.0);

    vec3  albedo = texture2D(s_color, v_texcoord0).xyz;

    // --- Near-surface detail: the KSP-style near/far texture swap. The global maps
    // top out at ~1.2 km/texel, so as the camera closes in (1) the magnified base
    // map is de-blocked with a smooth bicubic upsample and (2) tiling detail layers
    // fade in one after another (TerrainDetail::kLayerPeriod: 4 km .. 8 m), each
    // while it still spans more than a few pixels. What they show is chosen per
    // pixel from what the global maps say is there (vegetation / rock / sand / snow).
    // Shading only (albedo + normal): the geometry stays the sim's ground.
    float distSurf = length(u_camPos.xyz - v_wpos);          // km to this point
    float near     = smoothstep(200.0, 5.0, distSurf);       // 0 far .. 1 near
    if (near > 0.001)
        albedo = mix(albedo, texBicubic(s_color, v_texcoord0, vec2(32768.0, 16384.0)), near);

    // Screen derivatives for the bump, taken in uniform control flow.
    vec3 px = dFdx(v_wpos), py = dFdy(v_wpos);

    float hC    = texture2D(s_bump, v_texcoord0).x;
    float land  = smoothstep(0.0008, 0.02, hC);              // oceans stay smooth
    float slope = length(vec2(hL - hR, hD - hU)) * (8849.0 / (8.0 * 1223.0));  // rise/run, 8-texel span
    vec4  mat   = groundMaterials(albedo, slope);

    // Triplanar weights from the body-frame normal (view -> body is (x,-z,y)),
    // sharpened so one projection dominates almost everywhere.
    vec3 nb = vec3(N.x, -N.z, N.y);
    vec3 tw = nb * nb; tw = tw * tw; tw = tw * tw;
    tw /= dot(tw, vec3_splat(1.0));

    // Layers fade in from 40 to 10 periods away (TerrainDetail::kFadeStart).
    float distM  = distSurf * 1000.0;
    float tone   = 0.0;       // albedo modulation
    float relief = 0.0;       // synthetic relief height (km), shading only
    for (int k = 0; k < 4; k++) {
        if (float(k) < u_detailMask.w) {
            vec4  layer = u_detail[k];
            float wk    = smoothstep(40.0, 10.0, distM * layer.w) * land;
            vec3  p     = layer.xyz + v_local * layer.w;      // position in tiles
            float sig   = dot(detailTriplanar(p, tw), mat) - 0.5;
            tone   += wk * sig;
            relief += wk * sig * (0.008 / layer.w) * 0.001;   // relief ~1% of the period
        }
    }
    float contrast = dot(mat, vec4(0.9, 0.75, 0.45, 0.2));   // per-material albedo contrast
    albedo = max(albedo * (1.0 + tone * 1.2 * contrast), vec3_splat(0.0));

    // Bump from the synthetic relief (Mikkelsen, "Bump Mapping Unparametrized
    // Surfaces on the GPU"): the screen-space height gradient tilts the normal
    // with no tangent frame needed. relief is 0 where no layer is active.
    float rx  = dFdx(relief), ry = dFdy(relief);
    vec3  r1  = cross(py, N);
    vec3  r2  = cross(N, px);
    float det = dot(px, r1);
    // Only where the surface faces along N: on a steep face (a skirt wall) det
    // tends to 0 and the formula blows up.
    if (abs(det) > 0.2 * length(px) * length(py))
        Np = normalize(abs(det) * Np - sign(det) * (rx * r1 + ry * r2));

    float lit    = max(dot(Np, L), 0.0);                 // relief shading
    float term   = dot(N, L);                            // smooth terminator
    float t      = smoothstep(-0.15, 0.15, term);        // 0 = night .. 1 = day

    vec3  day = albedo * (0.15 + 0.85*lit);

    // Cloud shadows: cast a ray from this ground point toward the sun; where it
    // crosses the cloud layer and there's cover, darken the lit ground. Sampling
    // at the cloud altitude gives the correct offset (longer shadows at low sun),
    // so the shadow falls beside the cloud instead of hiding under it.
    vec3  ocC = v_wpos - u_earthCenter.xyz;
    float Rc  = length(ocC) + 7.0;                          // ~7 km: mid-troposphere cloud deck
    float bC  = dot(ocC, L);                                // (km, matching the rendered shells)
    float tc  = -bC + sqrt(max(bC*bC - (dot(ocC, ocC) - Rc*Rc), 0.0));
    vec3  cp  = normalize((v_wpos + L*tc) - u_earthCenter.xyz);
    vec3  ce  = vec3(cp.x, -cp.z, cp.y);                     // view -> ECI
    vec2  cuv = vec2(atan(ce.y, ce.x) * (0.5/3.14159265) + 0.5,
                     acos(clamp(ce.z, -1.0, 1.0)) / 3.14159265);
    // Match the cloud's apparent opacity: it is drawn as 5 shells at alpha 0.22
    // (keep this in sync with shellAlpha in bgfx_backend.cpp), so its coverage
    // follows 1-(1-0.22*c)^5, which lifts faint texels into visible cloud and makes
    // the shadow cover the cloud's full footprint (every non-black texel casts it).
    // The gate only fades the shadow across the terminator into the night side; it
    // is kept tight (0..0.05) so low-sun shadows -- which stretch out to the side
    // and are the ones actually visible from orbit -- are not cut off.
    float c       = clamp(texture2D(s_cloud, cuv).x, 0.0, 1.0);
    float cloudSh = 1.0 - pow(1.0 - 0.22 * c, 5.0);
    day *= 1.0 - 0.85 * cloudSh * smoothstep(0.0, 0.05, term);

    // PBR specular (Cook-Torrance GGX): the roughness map makes oceans glossy
    // (sharp sun glint) and land matte. F0 ~ 0.02 (dielectric water).
    float rough = clamp(1.0 - texture2D(s_rough, v_texcoord0).x, 0.2, 1.0);
    vec3  H   = normalize(L + V);
    float NdH = max(dot(Np, H), 0.0);
    float NdV = max(dot(Np, V), 1e-4);
    float NdL = max(dot(Np, L), 0.0);
    float VdH = max(dot(V, H), 0.0);
    float a = rough * rough, a2 = a * a;
    float Dg = a2 / (3.14159265 * pow(NdH*NdH*(a2 - 1.0) + 1.0, 2.0));
    float kk = a * 0.5;
    float Gg = (NdV/(NdV*(1.0 - kk) + kk)) * (NdL/(NdL*(1.0 - kk) + kk));
    float Fr = 0.02 + 0.98 * pow(1.0 - VdH, 5.0);
    float spec = Dg * Gg * Fr / (4.0 * NdV * NdL + 1e-4);
    day += vec3(1.0, 0.97, 0.90) * (spec * NdL * 2.2);   // ocean sun glint

    float rim = pow(1.0 - max(dot(N, V), 0.0), 3.0);     // atmospheric limb glow
    day += vec3(0.30, 0.50, 0.95) * rim * (0.35 + 0.65*max(term, 0.0));

    // City lights on the dark side, at lower exposure: squaring crushes the dim
    // ocean/atmosphere baked into the map (which otherwise out-shines the dark
    // daytime ocean) while keeping the bright city lights.
    vec3 night = texture2D(s_night, v_texcoord0).xyz;
    night = night * night * 1.3;
    vec3 color = mix(night, day, t);                     // blend across the twilight band

    gl_FragColor = vec4(color, 1.0);
    // Per-fragment logarithmic depth (see vs_terrain): same mapping as the
    // per-vertex log depth of the other shaders, in [0,1] window depth.
    gl_FragDepth = log2(max(1e-6, v_logz)) / log2(u_depth.x + 1.0);
}
