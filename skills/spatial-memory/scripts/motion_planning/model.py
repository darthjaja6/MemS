"""Prepare one fixed-base URDF/SRDF for both Drake and MPlib.

Robots are named scene entries with urdf, tip, joint_names, base_transform and
tool.point_in_tip. Other movable joints are passive and must be supplied in state.
Optional collision_exclusions are local link-name pairs. Mesh paths are local to
the source URDF (or absolute); package URIs must be resolved by the integration.
"""
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh


def prepare_model(robots, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    combined = ET.Element('robot', name='robot_group')
    ET.SubElement(combined, 'link', name='planning_root')
    exclusions = ET.Element('robot', name='robot_group')
    active, passive, slices = [], [], {}
    for name, robot in robots.items():
        urdf = Path(robot['urdf']).resolve()
        tree = ET.parse(urdf).getroot()
        for joint in tree.findall('joint'):
            kind = joint.get('type')
            if kind not in ('fixed', 'revolute', 'prismatic'):
                raise ValueError(f"{name}/{joint.get('name')}: unsupported {kind} joint; "
                                 'planning requires bounded revolute or prismatic joints')
            if kind != 'fixed':
                limit = joint.find('limit')
                bounds = [float(limit.get(k, 'nan')) if limit is not None else np.nan
                          for k in ('lower', 'upper')]
                if not np.isfinite(bounds).all() or bounds[0] > bounds[1]:
                    raise ValueError(f"{name}/{joint.get('name')}: supply finite ordered joint limits")
        children = {j.find('child').get('link') for j in tree.findall('joint')}
        roots = {link.get('name') for link in tree.findall('link')} - children
        if len(roots) != 1:
            raise ValueError(f'{name}: expected one URDF root link')
        movable = [j.get('name') for j in tree.findall('joint') if j.get('type') != 'fixed']
        selected = robot['joint_names']
        if len(set(selected)) != len(selected) or not set(selected) <= set(movable):
            raise ValueError(f'{name}: joint_names must select distinct movable joints')
        slices[name] = slice(len(active), len(active) + len(selected))
        active.extend(name + '_' + j for j in selected)
        passive.extend(name + '_' + j for j in movable if j not in selected)
        pairs = {tuple(sorted(pair)) for pair in robot.get('collision_exclusions', [])}
        pairs.update(tuple(sorted((j.find('parent').get('link'), j.find('child').get('link'))))
                     for j in tree.findall('joint'))
        for a, b in sorted(pairs):
            ET.SubElement(exclusions, 'disable_collisions', link1=name + '_' + a,
                          link2=name + '_' + b, reason='Configured or adjacent links')
        for link in tree.findall('link'):
            for visual in link.findall('visual'):
                link.remove(visual)
        for mesh in tree.findall('.//mesh'):
            filename = mesh.get('filename')
            if '://' in filename:
                raise ValueError(f'Resolve mesh URI before planning: {filename}')
            source = urdf.parent / filename
            target = directory / (hashlib.sha256(source.read_bytes()).hexdigest()[:20] + '.obj')
            if not target.exists():
                trimesh.load_mesh(source).export(target)
            mesh.set('filename', target.name)
        for element in tree:
            if element.tag not in ('link', 'joint'):
                continue
            for item in element.iter():
                if item.tag in ('link', 'joint'):
                    item.set('name', name + '_' + item.get('name'))
                elif item.tag in ('parent', 'child'):
                    item.set('link', name + '_' + item.get('link'))
                elif item.tag == 'mimic':
                    item.set('joint', name + '_' + item.get('joint'))
            combined.append(element)
        base = np.asarray(robot['base_transform'])
        mount = ET.SubElement(combined, 'joint', name=name + '_mount', type='fixed')
        ET.SubElement(mount, 'parent', link='planning_root')
        ET.SubElement(mount, 'child', link=name + '_' + roots.pop())
        ET.SubElement(mount, 'origin', xyz=' '.join(map(str, base[:3, 3])),
                      rpy=' '.join(map(str, Rotation.from_matrix(base[:3, :3]).as_euler('xyz'))))
    # Model changes produce a new cache entry; original assets remain untouched.
    data, rules = ET.tostring(combined), ET.tostring(exclusions)
    stem = hashlib.sha256(data + rules).hexdigest()[:20]
    urdf, srdf = directory / (stem + '.urdf'), directory / (stem + '.srdf')
    if not urdf.exists() or not srdf.exists():
        urdf.write_bytes(data)
        srdf.write_bytes(rules)
    return urdf, srdf, active + passive, slices
