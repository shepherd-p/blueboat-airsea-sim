// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.s


// Copyright (c) 2019 Rhys Mainwaring.
//
// Modified to accept vector parameters and use the form
// for Gerstner waves published in:
//
// Jerry Tessendorf, "Simulating Ocean Water", 1999-2004
//
// theta = k * dir . x - omega * t
//
// px = x - dir.x * a * k * sin(theta)
// py = y - dir.y * a * k * sin(theta)
// pz =         a * k * cos(theta)
//
// k is the wavenumber
// omega is the angular frequency
//
// The derivative terms (Tangent, Binormal, Normal) have been
// updated to be consistent with this convention.
//

#version 330

in vec4 vertex;
in vec4 uv0;
uniform mat4 worldviewproj_matrix;

/////////// Input parameters //////////
// Waves
uniform int Nwaves;
uniform vec3 camera_position_object_space;
uniform float rescale;
uniform vec2 bumpScale;
uniform vec2 bumpSpeed;
uniform float t;
uniform vec3 amplitude;
uniform vec3 wavenumber;
uniform vec3 omega;
uniform vec3 steepness;
uniform vec3 phase;
uniform vec2 dir0;
uniform vec2 dir1;
uniform vec2 dir2;
uniform float tau;

/////////// Output variables to fragment shader //////////
out block
{
  mat3 rotMatrix;
  vec3 eyeVec;
  vec2 bumpCoord;
  vec3 surfacePosition;
  float simTime;
} outVs;

// Compute linear combination of Gerstner waves as described in
// GPU Gems, chapter 01: "Effective Water Simulation from Physical Models"
// http://http.developer.nvidia.com/GPUGems/gpugems_ch01.html

// Information regarding a single wave
struct WaveParameters {
  float k;      // wavenumber
  float a;      // amplitude
  float omega;  // phase constant of speed
  vec2 d;       // horizontal direction of wave
  float q;      // steepness for Gerstner wave (q=0: rolling sine waves)
};



out gl_PerVertex
{
  vec4 gl_Position;
};

void main()
{
  WaveParameters waves[32] = WaveParameters[](
    WaveParameters(0.12760290087, 0.000107970201611, 1.11864068754, vec2(0.994247675413, -0.107105368379), 0.0),
    WaveParameters(0.155341900722, 0.00102641668634, 1.23425428933, vec2(0.999892924796, 0.014633487043), 0.0),
    WaveParameters(0.192340251615, 0.00476304513595, 1.37339489168, vec2(0.915468379437, 0.402389918179), 0.0),
    WaveParameters(0.240954863303, 0.0122556018533, 1.53719224894, vec2(0.929412989964, 0.369041317586), 0.0),
    WaveParameters(0.265115372531, 0.0158545822976, 1.61241857718, vec2(0.976382951227, 0.216047060968), 0.0),
    WaveParameters(0.306776827965, 0.0212863748558, 1.73448925623, vec2(0.852952981774, 0.521987749745), 0.0),
    WaveParameters(0.367824476822, 0.0315784026873, 1.89924350878, vec2(0.965748595071, 0.259479577459), 0.0),
    WaveParameters(0.413452430773, 0.0431963899805, 2.0135995829, vec2(0.956631577011, 0.291300576495), 0.0),
    WaveParameters(0.47374945334, 0.0457667319367, 2.15543384881, vec2(0.949056551787, 0.315105794155), 0.0),
    WaveParameters(0.533076940822, 0.0359461467895, 2.28641618734, vec2(0.93565414505, 0.352918292032), 0.0),
    WaveParameters(0.589940978068, 0.0280496123211, 2.40527434871, vec2(0.970360933511, 0.241660213348), 0.0),
    WaveParameters(0.657627853974, 0.0233800440373, 2.53951298366, vec2(0.849770670539, 0.527152546699), 0.0),
    WaveParameters(0.727716292331, 0.0210538330416, 2.67141516395, vec2(0.979190477878, 0.20294336164), 0.0),
    WaveParameters(0.812613291602, 0.0191201952182, 2.82294423184, vec2(0.943363091892, 0.331762078689), 0.0),
    WaveParameters(0.900265058987, 0.0174171196729, 2.97129337843, vec2(0.979436116952, 0.201754536034), 0.0),
    WaveParameters(0.969330257619, 0.0162200812032, 3.08316113281, vec2(0.844753675855, 0.535155329909), 0.0),
    WaveParameters(1.06210642615, 0.0147941109171, 3.22733729009, vec2(0.960571984964, 0.278031404165), 0.0),
    WaveParameters(1.11882196988, 0.0140153770848, 3.31238516344, vec2(0.98998279226, 0.141188069716), 0.0),
    WaveParameters(1.23469871332, 0.0126143813123, 3.47969224745, vec2(0.996955418768, -0.0779736685685), 0.0),
    WaveParameters(1.33562818825, 0.011571528171, 3.61912118785, vec2(0.903949057642, 0.427640153853), 0.0),
    WaveParameters(1.42495126399, 0.0107632810277, 3.73818115038, vec2(0.889662556462, 0.456618588791), 0.0),
    WaveParameters(1.53192705417, 0.00991407143473, 3.87596084161, vec2(0.910874512491, 0.412683441023), 0.0),
    WaveParameters(1.62569456685, 0.00925962589817, 3.99282076032, vec2(0.938185516217, 0.346132831672), 0.0),
    WaveParameters(1.71642055283, 0.00869415499872, 4.10272295122, vec2(0.967761405436, 0.251868739919), 0.0),
    WaveParameters(1.8587507096, 0.00791945918781, 4.2694399687, vec2(0.819005753972, 0.57378530389), 0.0),
    WaveParameters(1.95837419618, 0.00744579226897, 4.38236127116, vec2(0.751113524287, 0.660173063396), 0.0),
    WaveParameters(2.09935829076, 0.00685504502665, 4.53736399047, vec2(0.755537716266, 0.65510515133), 0.0),
    WaveParameters(2.19499267674, 0.00649946355099, 4.63956085565, vec2(0.844265193766, 0.535925631589), 0.0),
    WaveParameters(2.35046569865, 0.00598643535301, 4.80106180378, vec2(0.974173615152, 0.225800282421), 0.0),
    WaveParameters(2.44457199712, 0.00570951753393, 4.89622936305, vec2(0.942244284614, 0.334926421937), 0.0),
    WaveParameters(2.59326829789, 0.00531564439443, 5.04294304484, vec2(0.953436964377, 0.301592365551), 0.0),
    WaveParameters(2.72025487504, 0.0050157927798, 5.1649382833, vec2(0.941113182206, 0.338091671412), 0.0));
  float phases[32] = float[](0.481933880002, 3.50171925855, 0.324696929577, 2.85755152006, 2.28314506933, 6.02630665837, 1.26723342142, 5.90080655511, 4.1306250951, 2.67452968267, 3.10202438632, 5.21296470644, 4.7858781463, 3.35606916646, 3.61580822068, 1.73835454902, 6.0668332021, 0.689161284088, 4.9241958667, 3.46250251263, 0.389536205896, 6.19944614837, 2.33222229212, 3.98457608656, 3.71316937909, 2.94387831992, 2.92858556546, 6.01758387148, 6.17072606981, 4.66794391321, 4.31129845853, 2.26246266753);

  vec4 P = vertex;

  // Iteratively compute binormal, tangent, and normal vectors:
  vec3 B = vec3(1.0, 0.0, 0.0);
  vec3 T = vec3(0.0, 1.0, 0.0);
  vec3 N = vec3(0.0, 0.0, 1.0);

  // Wave synthesis using linear combination of Gerstner waves
  for(int i = 0; i < 32; ++i)
  {
    // Evaluate wave equation:
    float k = waves[i].k;
    float a = waves[i].a * (1.0 - exp(-1.0*t/tau));
    float q = waves[i].q;
    float dx = waves[i].d.x;
    float dy = waves[i].d.y;
    float theta = dot(waves[i].d, vertex.xy)*k - t*waves[i].omega + phases[i];
    float c = cos(theta);
    float s = sin(theta);

    // Displacement of point due to wave (Eq. 9)
    P.x -= q*a*dx*s;
    P.y -= q*a*dx*s;
    P.z += a*c;

    // Modify normals due to wave displacement (Eq. 10-12)
    float ka = a*k;
    float qkac = q*ka*c;
    float kas = ka*s;
    float dxy = dx*dy;

    B += vec3(-qkac*dx*dx, -qkac*dxy, -kas*dx);
    T += vec3(-qkac*dxy, -qkac*dy*dy, -kas*dy);
    N += vec3(dx*kas, dy*kas, -qkac);
  }

  // Compute (Surf2World * Rescale) matrix
  B = normalize(B)*rescale;
  T = normalize(T)*rescale;
  N = normalize(N);
  outVs.rotMatrix = mat3(B, T, N);

  gl_Position = worldviewproj_matrix * P;

  // Compute texture coordinates for bump map
  outVs.bumpCoord = uv0.xy*bumpScale + t*bumpSpeed;

  outVs.surfacePosition = P.xyz;
  outVs.simTime = t;
  outVs.eyeVec = P.xyz - camera_position_object_space; // eye position in vertex space
}

