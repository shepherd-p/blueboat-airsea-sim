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
    WaveParameters(0.287106526958, 1.79950336019e-05, 1.67796103131, vec2(0.994247675413, -0.107105368379), 0.0),
    WaveParameters(0.349519276625, 0.000171069447724, 1.85138143399, vec2(0.999892924796, 0.014633487043), 0.0),
    WaveParameters(0.432765566134, 0.000793840855992, 2.06009233753, vec2(0.915468379437, 0.402389918179), 0.0),
    WaveParameters(0.542148442431, 0.00204260030889, 2.30578837341, vec2(0.929412989964, 0.369041317586), 0.0),
    WaveParameters(0.596509588195, 0.00264243038293, 2.41862786577, vec2(0.976382951227, 0.216047060968), 0.0),
    WaveParameters(0.690247862921, 0.00354772914263, 2.60173388434, vec2(0.852952981774, 0.521987749745), 0.0),
    WaveParameters(0.82760507285, 0.00526306711455, 2.84886526316, vec2(0.965748595071, 0.259479577459), 0.0),
    WaveParameters(0.930267969239, 0.00719939833008, 3.02039937434, vec2(0.956631577011, 0.291300576495), 0.0),
    WaveParameters(1.06593627001, 0.00762778865611, 3.23315077321, vec2(0.949056551787, 0.315105794155), 0.0),
    WaveParameters(1.19942311685, 0.00599102446492, 3.429624281, vec2(0.93565414505, 0.352918292032), 0.0),
    WaveParameters(1.32736720065, 0.00467493538684, 3.60791152307, vec2(0.970360933511, 0.241660213348), 0.0),
    WaveParameters(1.47966267144, 0.00389667400621, 3.80926947549, vec2(0.849770670539, 0.527152546699), 0.0),
    WaveParameters(1.63736165774, 0.0035089721736, 4.00712274593, vec2(0.979190477878, 0.20294336164), 0.0),
    WaveParameters(1.8283799061, 0.00318669920303, 4.23441634776, vec2(0.943363091892, 0.331762078689), 0.0),
    WaveParameters(2.02559638272, 0.00290285327882, 4.45694006765, vec2(0.979436116952, 0.201754536034), 0.0),
    WaveParameters(2.18099307964, 0.0027033468672, 4.62474169922, vec2(0.844753675855, 0.535155329909), 0.0),
    WaveParameters(2.38973945884, 0.00246568515285, 4.84100593514, vec2(0.960571984964, 0.278031404165), 0.0),
    WaveParameters(2.51734943224, 0.00233589618079, 4.96857774516, vec2(0.98998279226, 0.141188069716), 0.0),
    WaveParameters(2.77807210497, 0.00210239688538, 5.21953837118, vec2(0.996955418768, -0.0779736685685), 0.0),
    WaveParameters(3.00516342357, 0.00192858802849, 5.42868178178, vec2(0.903949057642, 0.427640153853), 0.0),
    WaveParameters(3.20614034398, 0.00179388017128, 5.60727172556, vec2(0.889662556462, 0.456618588791), 0.0),
    WaveParameters(3.44683587187, 0.00165234523912, 5.81394126242, vec2(0.910874512491, 0.412683441023), 0.0),
    WaveParameters(3.65781277542, 0.00154327098303, 5.98923114048, vec2(0.938185516217, 0.346132831672), 0.0),
    WaveParameters(3.86194624387, 0.00144902583312, 6.15408442683, vec2(0.967761405436, 0.251868739919), 0.0),
    WaveParameters(4.1821890966, 0.00131990986464, 6.40415995305, vec2(0.819005753972, 0.57378530389), 0.0),
    WaveParameters(4.4063419414, 0.00124096537816, 6.57354190674, vec2(0.751113524287, 0.660173063396), 0.0),
    WaveParameters(4.7235561542, 0.00114250750444, 6.80604598571, vec2(0.755537716266, 0.65510515133), 0.0),
    WaveParameters(4.93873352266, 0.00108324392517, 6.95934128348, vec2(0.844265193766, 0.535925631589), 0.0),
    WaveParameters(5.28854782197, 0.000997739225502, 7.20159270567, vec2(0.974173615152, 0.225800282421), 0.0),
    WaveParameters(5.50028699352, 0.000951586255655, 7.34434404457, vec2(0.942244284614, 0.334926421937), 0.0),
    WaveParameters(5.83485367024, 0.000885940732405, 7.56441456726, vec2(0.953436964377, 0.301592365551), 0.0),
    WaveParameters(6.12057346884, 0.0008359654633, 7.74740742495, vec2(0.941113182206, 0.338091671412), 0.0));
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

