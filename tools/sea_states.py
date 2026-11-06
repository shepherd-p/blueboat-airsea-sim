"""Representative WMO sea-state cases; shared CPU/GLSL spectral components."""
from pathlib import Path
import argparse
import copy
import hashlib
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
from generate_environment import dense_water, wave_tags
from make_scenario import make

ROOT = Path(__file__).resolve().parents[1]
DESC = ROOT / "h-spar-sim/ros2_ws/src/blueboat_description"
G = 9.80665


def profile(state):
    # Chosen representative short-crested seas, not a forecast or measured site.
    hs, tp = {1: (0.05, 2.0), 2: (0.30, 3.0)}[state]
    rng = np.random.default_rng(20261101)
    n = 32
    fp = 1 / tp
    edges = np.linspace(.5 * fp, 2.5 * fp, n + 1)
    df = np.diff(edges)
    frequencies = edges[:-1] + df * rng.uniform(.25, .75, n)
    sigma = np.where(frequencies <= fp, .07, .09)
    peak = np.exp(-.5 * ((frequencies - fp) / (sigma * fp)) ** 2)
    spectrum = frequencies ** -5 * np.exp(-1.25 * (fp / frequencies) ** 4) * 3.3 ** peak
    amplitudes = np.sqrt(2 * spectrum * df)
    amplitudes *= hs / (4 * np.sqrt(np.sum(amplitudes ** 2) / 2))
    phases = rng.uniform(0, 2 * np.pi, n)
    directions = rng.normal(20, 12, n)
    waves = [dict(amplitude_m=float(a), wavelength_m=float(G / (2 * np.pi * f * f)),
                  direction_deg=float(d), phase_rad=float(p))
             for a, f, d, p in zip(amplitudes, frequencies, directions, phases)]
    return dict(sea_state=state, target_hm0_m=hs, target_tp_s=tp,
                classification_range_m=[0, .1] if state == 1 else [.1, .5],
                spectrum="Truncated 32-component JONSWAP, gamma=3.3, f/fp=0.5..2.5; amplitudes normalized to Hm0=4*sqrt(sum(a^2)/2)",
                random_seed=20261101, waves=waves, wave_ramp_seconds=10,
                wind_enu_mps=[2., 0., 0.], gust_fraction=.1, gust_period_s=8,
                current_enu_mps=[.10, .03, 0.],
                notes="Sea-state codes constrain wave-height range, not a unique period, spectrum, wind or current. These are selected representative cases, not all of either sea state. Deep-water linear approximation; no breaking/slamming/radiation/diffraction or real-boat calibration.")


def vertex_shader(c):
    # Embed the same serialized component list in GLSL: no fixed-three-wave
    # uniform limit, GPU packing ambiguity, or independently chosen visual waves.
    template = (DESC / "models/blueboat_ocean_visual/materials/programs/Ocean_vs.glsl").read_text()
    begin = template.index("  // Use combination of three waves.")
    end = template.index("  vec4 P = vertex;", begin)
    rows = []
    for w in c['waves']:
        k = 2 * math.pi / w['wavelength_m']
        d = math.radians(w['direction_deg'])
        rows.append(f"WaveParameters({k:.12g}, {w['amplitude_m']:.12g}, {math.sqrt(G*k):.12g}, vec2({math.cos(d):.12g}, {math.sin(d):.12g}), 0.0)")
    setup = f"  WaveParameters waves[{len(rows)}] = WaveParameters[](\n    " + ",\n    ".join(rows) + ");\n"
    setup += "  float phases[32] = float[](" + ", ".join(f"{w['phase_rad']:.12g}" for w in c['waves']) + ");\n\n"
    result = template[:begin] + setup + template[end:]
    result = result.replace("i < Nwaves", "i < 32").replace("+ phase[i]", "+ phases[i]")
    return result


def generate():
    configs = DESC / "config/sea_states"
    configs.mkdir(parents=True, exist_ok=True)
    # Same mesh for both cases, at least ~5 vertices per shortest retained wave.
    mesh = DESC / "models/blueboat_sea_state_1_visual/meshes/water.dae"
    mesh.parent.mkdir(parents=True, exist_ok=True)
    dense_water(mesh, size=120, step=.20)
    for state in [1, 2]:
        c = profile(state)
        (configs / f"sea_state_{state}.json").write_text(json.dumps(c, indent=2) + "\n")
        base = DESC / f"models/blueboat_sea_state_{state}_visual"
        program = base / "materials/programs/Ocean_vs.glsl"
        program.parent.mkdir(parents=True, exist_ok=True)
        program.write_text(vertex_shader(c))
    print("Generated reproducible 32-component Sea State 1/2 profiles and matched GLSL.")


def make_case(out, state, sitl=True, rtf=1, port=9032):
    out = Path(out).resolve()
    c = json.loads((DESC / f"config/sea_states/sea_state_{state}.json").read_text())
    path = make(out, "combined", sitl, rtf)
    tree = ET.parse(path)
    world = tree.getroot().find('world')
    for waves in list(world.findall('.//waves')):
        waves.clear()
        holder = ET.Element('holder')
        wave_tags(holder, c)
        waves.extend(copy.deepcopy(list(holder.find('waves'))))
    env = world.find("model/plugin[@name='blueboat::Environment']")
    for name, key in [('wind', 'wind_enu_mps'), ('current', 'current_enu_mps')]:
        env.find(name).text = ' '.join(map(str, c[key]))
    env.find('gust_fraction').text = str(c['gust_fraction'])
    env.find('gust_period').text = str(c['gust_period_s'])
    world.find("model/plugin[@name='gz::sim::systems::Hydrodynamics']/default_current").text = ' '.join(map(str, c['current_enu_mps']))
    if sitl:
        world.find("model/plugin[@name='blueboat::SITL']/port").text = str(port)
    water = world.find("model[@name='ocean_surface']")
    water.find('.//geometry/mesh/uri').text = 'model://blueboat_sea_state_1_visual/meshes/water.dae'
    water.find('.//shader/vertex').text = f'model://blueboat_sea_state_{state}_visual/materials/programs/Ocean_vs.glsl'
    shader_plugin = water.find('.//plugin')
    for p in list(shader_plugin.findall('param')):
        if p.findtext('name') in ['Nwaves', 'amplitude', 'wavenumber', 'omega', 'steepness', 'phase', 'dir0', 'dir1', 'dir2']:
            shader_plugin.remove(p)
        elif p.findtext('name') == 'tau':
            p.find('value').text = str(c['wave_ramp_seconds'])
    ET.indent(tree, space='  ')
    tree.write(path, encoding='utf-8', xml_declaration=True)
    (out / 'profile.json').write_text(json.dumps(c, indent=2) + '\n')
    # Archive the exact shader and its hash alongside the test world.
    shader = DESC / f'models/blueboat_sea_state_{state}_visual/materials/programs/Ocean_vs.glsl'
    (out / 'wave_shader.glsl').write_bytes(shader.read_bytes())
    (out / 'input_hashes.json').write_text(json.dumps({
        'world_sdf_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'profile_sha256': hashlib.sha256((out / 'profile.json').read_bytes()).hexdigest(),
        'shader_sha256': hashlib.sha256(shader.read_bytes()).hexdigest(),
        'physics_source_sha256': {name: hashlib.sha256((ROOT / 'h-spar-sim/ros2_ws/src/blueboat_gazebo/src' / name).read_bytes()).hexdigest()
                                  for name in ['BlueBoatEnvironment.cc', 'WaveField.hh', 'BlueBoatBuoyancy.cc', 'BlueBoatSITL.cc', 'BlueBoatThrusters.cc']},
    }, indent=2) + '\n')
    return path


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--generate', action='store_true')
    p.add_argument('--state', type=int, choices=[1, 2])
    p.add_argument('--out', type=Path)
    p.add_argument('--rtf', type=float, default=1)
    p.add_argument('--port', type=int, default=9032)
    a = p.parse_args()
    if a.generate:
        generate()
    else:
        if a.state is None or a.out is None:
            p.error('--state and --out are required')
        print(make_case(a.out, a.state, True, a.rtf, a.port))
