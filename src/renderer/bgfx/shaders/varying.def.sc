vec3 a_position  : POSITION;
vec3 a_normal    : NORMAL;
vec2 a_texcoord0 : TEXCOORD0;
vec4 a_color0    : COLOR0;

vec2 v_texcoord0 : TEXCOORD0 = vec2(0.0, 0.0);
vec4 v_color0    : COLOR0    = vec4(1.0, 1.0, 1.0, 1.0);
vec3 v_wpos      : TEXCOORD1 = vec3(0.0, 0.0, 0.0);
vec3 v_normal    : TEXCOORD2 = vec3(0.0, 0.0, 1.0);
vec3 v_objpos    : TEXCOORD3 = vec3(0.0, 0.0, 0.0);
vec3 v_local     : TEXCOORD4 = vec3(0.0, 0.0, 0.0);
float v_logz     : TEXCOORD5 = 0.0;
