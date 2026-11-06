"""Parse official chart arrays as data (never execute downloaded JavaScript)."""
from pathlib import Path
import hashlib
import json
import re

ROOT=Path(__file__).resolve().parents[1]
REF=ROOT/'references'
PKG=ROOT/'h-spar-sim/ros2_ws/src/blueboat_gazebo'

def array(path,name):
    match=re.search(r'var\s+'+name+r'\s*=\s*(\[[0-9.,\s]+\])',path.read_text())
    if not match:raise ValueError('Expected numeric chart array: '+name)
    return json.loads(match[1])

def main():
    f=REF/'m200weedless_force_plot.js';r=REF/'m200weedless_rpm_plot.js'
    pwm=array(f,'pwm_value');assert pwm==array(r,'pwm_value')
    thrust=array(f,'thrust_16v');rpm=array(r,'rpm_16v')
    assert len(pwm)==len(thrust)==len(rpm)==33
    rows=[{'pwm_us':p,'thrust_N':(-1 if p<1500 else 1)*t*9.80665,
           'rpm':(-1 if p<1500 else 1)*n} for p,t,n in zip(pwm,thrust,rpm)]
    assert all(a['thrust_N']<b['thrust_N'] and a['rpm']<b['rpm'] for a,b in zip(rows,rows[1:]))
    data={'source':'Blue Robotics M200 motor + weedless propeller official charts',
          'voltage_V':16,'method':'Paired at identical ESC PWM; piecewise linear interpolation of signed static thrust vs signed RPM.',
          'scope':'Static test mapping, not an advance-ratio/loaded-propeller or motor transient model. Vehicle thrust limits remain separate.',
          'urls':['https://docs.bluerobotics.com/thrusters/m200weedless_force_plot.js',
                  'https://docs.bluerobotics.com/thrusters/m200weedless_rpm_plot.js'],
          'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [f,r]},'samples':rows}
    (PKG.parent/'blueboat_description/config/propulsion_curve.json').write_text(json.dumps(data,indent=2)+'\n')
    lines=['// Derived from the attributed official 16 V chart samples. See propulsion_curve.json.',
           '#pragma once','#include <array>','#include <algorithm>','#include <cmath>',
           'namespace blueboat::propulsion {',
           'inline constexpr std::array<double,33> Pwm{'+','.join(repr(x['pwm_us']) for x in rows)+'};',
           'inline constexpr std::array<double,33> ForceN{'+','.join(repr(x['thrust_N']) for x in rows)+'};',
           'inline constexpr std::array<double,33> Rpm{'+','.join(repr(x['rpm']) for x in rows)+'};',
           'inline double Interpolate(double v,const std::array<double,33>&x,const std::array<double,33>&y){',
           '  if(!std::isfinite(v))return 0;',
           '  if(v<=x.front())return y.front();if(v>=x.back())return y.back();',
           '  auto i=std::upper_bound(x.begin(),x.end(),v)-x.begin();',
           '  return y[i-1]+(v-x[i-1])/(x[i]-x[i-1])*(y[i]-y[i-1]);','}',
           'inline double RpmFromThrust(double f){return Interpolate(f,ForceN,Rpm);}',
           'inline double ThrustFromRpm(double n){return Interpolate(n,Rpm,ForceN);}','}']
    (PKG/'src/PropulsionCurve.hh').write_text('\n'.join(lines)+'\n')
    print('Prepared 33 paired, monotonic, signed static-thrust/RPM samples at 16 V')

if __name__=='__main__':main()
