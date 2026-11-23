#!/usr/bin/env python3
"""Generate opt-in 40 x 30 m maps without modifying legacy assets."""
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from build_env import C, street, write_world, write_map, write_config, write_svg, write_json


def generate(root):
    root = Path(root)
    C['ground'] = (0.62, 0.78, 0.48)
    C['sidewalk'] = (0.94, 0.93, 0.87)
    for name in ('street_extended', 'campus_extended'):
        env = street()
        env.name = name
        env.title = '40 x 30 m ' + ('도로' if name.startswith('street') else '캠퍼스 차도')
        if name.startswith('campus'):
            labels = {'상가': '학생회관', '카페·편의점': '카페·편의점', '오피스': '연구동'}
            env.solids = [(*s[:-1], labels.get(s[-1], s[-1])) for s in env.solids]
        for d in ('worlds', 'maps', 'config/maps'):
            (root / d).mkdir(parents=True, exist_ok=True)
        world = root / 'worlds' / (name + '.sdf')
        write_world(env, str(world))
        # Raised pavement has physical collision in this profile as well as policy occupancy.
        tree = ET.parse(world)
        for link in tree.findall('.//world/model/link'):
            for visual in list(link.findall('visual')):
                if visual.get('name') in env.raised:
                    collision = ET.SubElement(link, 'collision', name=visual.get('name') + '_collision')
                    import copy
                    for tag in ('pose', 'geometry'):
                        child = visual.find(tag)
                        if child is not None:
                            collision.append(copy.deepcopy(child))
        tree.write(world, encoding='utf-8', xml_declaration=True)
        write_map(env, str(root / 'maps' / name), with_keepout=True)
        write_map(env, str(root / 'maps' / (name + '_lidar')), with_keepout=False)
        write_config(env, str(root / 'config/maps' / (name + '.yaml')))
        write_svg(env, str(root / 'maps' / (name + '_layout.svg')))
        write_json(env, str(root / 'maps' / (name + '_layout.json')))
        print(name)


if __name__ == '__main__':
    generate(Path(__file__).resolve().parents[1])
