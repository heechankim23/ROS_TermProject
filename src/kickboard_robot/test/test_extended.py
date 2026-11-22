"""Offline checks; these are not Gazebo integration success claims."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import yaml
from kickboard_robot.extended_policy import admission, footprint_free, scan_collision, ramp
from kickboard_robot.occupancy import OccupancyMap

SIM = Path(__file__).resolve().parents[2] / 'kickboard_sim'
SCENARIOS = list((SIM/'config/scenarios_extended').glob('s*.yaml'))


def test_manifest_has_nineteen():
    m = json.loads((SIM/'config/scenarios_extended/manifest.json').read_text())
    assert len(m['legacy']) == 9
    assert len(m['extended']) == len(SCENARIOS) == 10
    assert {int(n.split('_')[0][1:]) for n in m['extended']} == set(range(10,20))


@pytest.mark.parametrize('mass,cargo,expected', [(18, [], 'COLLECT'), (32,['a'],'UNLOAD_FIRST'),
                                               (51,[],'NEEDS_HUMAN'), (float('nan'),[],'NEEDS_HUMAN')])
def test_weight_prevents_overloading(mass,cargo,expected):
    assert admission({'weight_kg':mass},cargo,{'a':32},50) == expected


def test_damaged_never_loaded():
    assert admission({'damaged':True},[],{},75) == 'NEEDS_HUMAN'


def test_wet_acceleration():
    assert ramp(0,0.8,0.15,0.1) == pytest.approx(0.015)
    assert ramp(0.2,0,0.15,0.1) == pytest.approx(0.185)


def test_dynamic_obstacle_sweep():
    assert scan_collision([(1.2,0)],0.5,0)
    assert not scan_collision([(3,2)],0.5,0)
    assert scan_collision([(-1.2,0)],-0.5,0)
    assert not scan_collision([(0.8,0)],0.5,0)  # own-body return


@pytest.mark.parametrize('map_name',['street_extended','campus_extended'])
def test_map_driveable_contract(map_name):
    om = OccupancyMap(str(SIM/'maps'/f'{map_name}.yaml'))
    assert (om.w*om.res,om.h*om.res) == (40,30)
    assert footprint_free(om,(-6.5,2,0))
    assert not footprint_free(om,(-8,5,0))  # straddles raised sidewalk
    assert not footprint_free(om,(20.01,0,0))
    mc = yaml.safe_load((SIM/'config/maps'/f'{map_name}.yaml').read_text(encoding='utf-8'))
    sp=mc['robot_spawn']
    assert footprint_free(om,(sp['x'],sp['y'],math.radians(sp['yaw_deg'])))
    assert {z['name'] for z in mc['parking_zones']} == {'N1','E1','S1'}
    world=ET.parse(SIM/'worlds'/f'{map_name}.sdf')
    assert len(world.findall('.//collision[@name="block_nw_collision"]')) == 1


@pytest.mark.parametrize('path',SCENARIOS,ids=lambda p:p.stem)
def test_spawn_access_from_road(path):
    sc=yaml.safe_load(path.read_text(encoding='utf-8'))
    om=OccupancyMap(str(SIM/'maps'/(sc['map']+'.yaml')))
    ids=[s['id'] for s in sc['scooters']]
    assert len(ids)==len(set(ids))
    for s in sc['scooters']:
        assert -20 < s['x'] <20 and -15 <s['y']<15
        assert 0 <= s['battery_pct'] <= 100
        # At least one full-footprint approach within 1.7 m arm reach, static map only.
        assert any(footprint_free(om,(s['x']+1.5*math.cos(a),s['y']+1.5*math.sin(a),a+math.pi))
                   for a in [math.radians(d) for d in range(0,360,10)]), s


def test_legacy_manifest_hashes():
    manifest=SIM/'config/scenarios_extended/legacy_sha256.json'
    for rel, expected in json.loads(manifest.read_text()).items():
        assert hashlib.sha256((SIM.parent/rel).read_bytes()).hexdigest()==expected,rel
