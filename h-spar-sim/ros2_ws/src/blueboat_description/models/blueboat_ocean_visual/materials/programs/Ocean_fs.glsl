// Ocean lighting derived from the Apache-2.0 UUV Simulator water shader.
// Large-wave geometry remains the same three components used by buoyancy.
// Two texture scales add visual capillary ripples only; no added wave forces.
#version 330
uniform sampler2D bumpMap;
uniform vec4 deepColor;
uniform vec4 shallowColor;
in block
{
  mat3 rotMatrix;
  vec3 eyeVec;
  vec2 bumpCoord;
  vec3 surfacePosition;
  float simTime;
} inPs;
out vec4 fragColor;

void main()
{
  vec3 V = normalize(-inPs.eyeVec);
  vec3 largeN = normalize(inPs.rotMatrix[2]);
  vec2 uv = inPs.bumpCoord;
  vec3 n1 = texture(bumpMap, uv).xyz * 2.0 - 1.0;
  vec2 uv2 = mat2(.8, .6, -.6, .8) * uv * 2.73
             + vec2(-.029, .017) * inPs.simTime;
  vec3 n2 = texture(bumpMap, uv2).xyz * 2.0 - 1.0;
  // Fade subpixel ripples with distance to avoid a shimmering horizon.
  float detail = 1.0 / (1.0 + .025 * length(inPs.eyeVec));
  vec2 slope = (n1.xy * .65 + n2.xy * .28) * detail;
  vec3 N = normalize(largeN + inPs.rotMatrix[0] * slope.x
                            + inPs.rotMatrix[1] * slope.y);
  vec3 R = reflect(-V, N);
  // Analytic sky reflection avoids the low-resolution cubemap's broad streaks.
  // This is environment lighting, not a reflection of the quay or boat.
  float sky = pow(clamp(R.z,0.0,1.0),.45);
  vec3 env = mix(vec3(.69,.76,.78),vec3(.26,.44,.60),sky);
  // Restrained environment reflection and water's normal-incidence F0.
  float ndv = max(dot(N,V), .0);
  float fresnel = .0204 + .9796 * pow(1.0 - ndv, 5.0);
  vec3 body = mix(deepColor.rgb, shallowColor.rgb, .35 + .45*largeN.z);
  vec3 color = mix(body, env * .85, fresnel);
  // A broad sun lobe plus a smaller glint follows the world's sun direction.
  vec3 L = normalize(vec3(.18,.9,.38));
  vec3 H = normalize(V+L);
  float ndh = max(dot(N,H),0.0);
  float glint = pow(ndh,180.0)*.50 + pow(ndh,36.0)*.055;
  color += vec3(1.0,.94,.82) * glint;
  // Gentle aerial perspective keeps distant water from becoming a flat wall.
  float haze = 1.0-exp(-length(inPs.eyeVec)*.0025);
  color = mix(color,vec3(.55,.65,.70),haze);
  fragColor = vec4(color,1.0);
}
