#!/usr/bin/env python3
"""JSON scene operations using the shared geometry and robot tools."""
import argparse
import json
from pathlib import Path
import sys
from filelock import FileLock
import numpy as np
from geometry import (matrix_pose, pixel_plane, plane as fit_plane, pose_matrix, project, tool_pose,
                      transform_points, triangulate)
from kinematics import Robot


def merge(state, changes):
    for key, value in changes.items():
        if value is None:
            state.pop(key, None)
        elif isinstance(value, dict):
            if not isinstance(state.get(key), dict):
                state[key] = {}
            merge(state[key], value)
        else:
            state[key] = value
    return state


def save(path, state):
    import tempfile
    path = Path(path)
    encoded = json.dumps(state, indent=2, allow_nan=False) + '\n'
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(encoded)
    try:
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def observe(state, changes):
    """Merge scene corrections and refresh grasp transforms from corrected poses."""
    merge(state, changes)
    for obj in state.get('objects', {}).values():
        if obj.get('asset'):
            for key in ('shape', 'size', 'parts', 'landmarks', 'vertices', 'faces', 'file', 'operation'):
                obj.pop(key, None)
    for name, update in (changes.get('objects') or {}).items():
        if isinstance(update, dict) and any(key in update for key in ('position', 'quaternion')):
            obj = state['objects'][name]
            obj['observed_pose'] = [*obj['position'], *obj.get('quaternion', [1, 0, 0, 0])]
            obj.pop('predicted_pose', None)
            obj['source'] = update.get('source', 'current observation')
            if obj.get('attachment'):
                attach(state, name, obj['attachment']['robot'])
    return state


def robot_model(model, root='.'):
    return Robot(Path(root) / model['urdf'])


def sync(state, observation, root='.'):
    state['cameras'] = observation['cameras']
    state['control_budget'] = observation.get('control_budget')
    state['terminal'] = observation.get('terminal', False)
    measured = observation['state']
    for name, robot in state.get('robots', {}).items():
        robot['pose'] = measured[name + '_ee_pose']
        robot['joints'] = dict(zip(robot['joint_names'], measured[name + '_arm_joint_state']))
        robot['gripper'] = measured.get(name + '_ee_joint_state', [1])[0]
        if 'gripper_scale' in robot:
            low, high = robot['gripper_scale']
            robot['joints'].update({j: low + robot['gripper'] * (high - low)
                                    for j in robot['gripper_joints']})
        if 'base_transform' not in robot:
            local = robot_model(robot, root).frames(robot['joints'])[robot['tip']]
            robot['base_transform'] = (pose_matrix(robot['pose']) @ np.linalg.inv(local)).tolist()
    for obj in state.get('objects', {}).values():
        attachment = obj.get('attachment')
        if attachment:
            T = pose_matrix(state['robots'][attachment['robot']]['pose']) @ np.array(attachment['transform'])
            p = matrix_pose(T)
            obj.setdefault('observed_pose', [*obj['position'], *obj.get('quaternion', [1, 0, 0, 0])])
            obj['predicted_pose'] = p
            obj.update(position=p[:3], quaternion=p[3:], source='predicted from attachment and measured robot pose')
    return state


def target_point(state, target, root='.'):
    if isinstance(target, str):
        target = {'object': target}
    if isinstance(target, dict):
        if 'robot' in target:
            robot = state['robots'][target['robot']]
            if 'link' in target:
                T = np.array(robot['base_transform']) @ robot_model(robot, root).frames(robot['joints'])[target['link']]
            else:
                T = pose_matrix(robot['pose'])
            return transform_points(T, target.get('offset', robot['tool']['point_in_tip'])).tolist()
        obj = state['objects'][target['object']]
        T = pose_matrix([*obj['position'], *obj.get('quaternion', [1, 0, 0, 0])])
        local = np.asarray(target.get('offset', [0, 0, 0]), float)
        if 'landmark' in target or 'bounds' in target:
            from assets import geometry_info
            info = geometry_info(state, target['object'], root)
            if 'landmark' in target:
                local += info['landmarks_m'][target['landmark']]
            else:
                low, high = np.asarray(info['bounds_m'])
                local += low + np.asarray(target['bounds']) * (high-low)
        return transform_points(T, local).tolist()
    return target


def pose(state, robot, target, approach, closing, root='.'):
    tool = state['robots'][robot]['tool']
    return tool_pose(target_point(state, target, root), approach, closing, **tool)['pose']


def locate(state, views, plane=None, height_m=0, root='.'):
    expanded = []
    for view in views:
        cameras = state['cameras']
        if 'observation' in view:
            cameras = json.loads((Path(root) / view['observation']).read_text())['cameras']
        camera = cameras[view['camera']]
        expanded.append(dict(pixel=view['pixel'], K=camera['K'], T_base_camera=camera['T_base_camera'],
                             distortion=camera.get('distortion')))
    if plane is not None:
        if len(expanded) != 1:
            raise ValueError('Plane localization needs one view; omit plane for triangulation')
        surface = state['surfaces'][plane] if isinstance(plane, str) else plane
        return pixel_plane(**expanded[0], normal=surface['normal'],
                           offset_m=surface['offset_m'], height_m=height_m)
    return triangulate(expanded)


def attach(state, object, robot):
    obj = state['objects'][object]
    T = pose_matrix([*obj['position'], *obj.get('quaternion', [1, 0, 0, 0])])
    if 'predicted_pose' not in obj:
        obj['observed_pose'] = [*obj['position'], *obj.get('quaternion', [1, 0, 0, 0])]
    relative = np.linalg.inv(pose_matrix(state['robots'][robot]['pose'])) @ T
    obj['attachment'] = dict(robot=robot, transform=relative.tolist())
    return obj


def solve_ik(state, robot, target, root='.', **options):
    model = state['robots'][robot]
    base = np.array(model['base_transform'])
    local_target = transform_points(np.linalg.inv(base), target_point(state, target, root))
    for axis in options.get('orientation_axes', []):
        axis['world'] = (base[:3, :3].T @ axis['world']).tolist()
    return robot_model(model, root).ik(local_target, model['tip'], seed=model['joints'],
        point_in_tip=model['tool']['point_in_tip'], **options)


def export(state, root=None):
    import xml.etree.ElementTree as ET
    def nums(x):
        return ' '.join(f'{float(v):.7g}' for v in x)
    directory = Path(root or '.')
    root = ET.Element('mujoco', model='spatial_memory')
    assets = ET.SubElement(root, 'asset')
    ET.SubElement(root, 'compiler', angle='radian')
    world = ET.SubElement(root, 'worldbody')
    def emit(parent, name, obj):
        if obj.get('role') == 'marker':
            return
        shape = obj.get('shape', 'box')
        if 'parts' in obj:
            body = ET.SubElement(parent, 'body', name=name, pos=nums(obj['position']),
                                 quat=nums(obj.get('quaternion', [1, 0, 0, 0])))
            for index, part in enumerate(obj['parts']):
                emit(body, name + '_' + str(index), part)
            return
        size = np.asarray(obj['size'], dtype=float)
        if shape == 'box':
            size = size / 2
        elif shape in ('cylinder', 'capsule'):
            size = size * [1, .5]
        if shape not in ('box', 'sphere', 'cylinder', 'capsule', 'ellipsoid'):
            raise ValueError('Scene shapes: box, sphere, cylinder, capsule, ellipsoid')
        ET.SubElement(parent, 'geom', name=name, type=shape, pos=nums(obj['position']),
            quat=nums(obj.get('quaternion', [1, 0, 0, 0])), size=nums(size),
            rgba=nums(obj.get('rgba', [.8, .7, .3, 1])))
    for name, obj in state.get('objects', {}).items():
        if obj.get('role') == 'marker':
            continue
        def uses_mesh(part):
            return ('asset' in part or 'scale' in part or 'operation' in part or part.get('shape') in ('mesh', 'sweep', 'revolve')
                    or any(uses_mesh(child) for child in part.get('parts', [])))
        needs_mesh = uses_mesh(obj)
        if needs_mesh:
            from assets import object_mesh
            mesh = object_mesh(state, name, directory, world=False)
            ET.SubElement(assets, 'mesh', name=name + '_mesh',
                          vertex=nums(mesh.vertices.ravel()), face=' '.join(map(str, mesh.faces.ravel())))
            ET.SubElement(world, 'geom', name=name, type='mesh', mesh=name + '_mesh',
                          pos=nums(obj.get('position', [0, 0, 0])), quat=nums(obj.get('quaternion', [1, 0, 0, 0])),
                          rgba=nums(obj.get('rgba', [.8, .7, .3, 1])), contype='0', conaffinity='0')
        else:
            emit(world, name, obj)
    for name, model in state.get('robots', {}).items():
        robot = robot_model(model, directory)
        base = np.array(model['base_transform'])
        frames = {k: base @ T for k, T in robot.frames(model['joints']).items()}
        for key, joint in robot.joints.items():
            a, b = frames[joint['parent']][:3, 3], frames[joint['child']][:3, 3]
            if np.linalg.norm(a - b) > 1e-6:
                ET.SubElement(world, 'geom', name=name + '_' + key, type='capsule',
                    fromto=nums([*a, *b]), size='.012', rgba='.3 .35 .4 1')
    ET.indent(root)
    return ET.tostring(root, encoding='unicode')


def record_plane(state, name, point=None, robot=None, link=None, offset=None, fit=False, root='.'):
    """Accumulate measured world points, optionally fit and store the plane."""
    measurements = state.setdefault('plane_samples', {}).setdefault(name, [])
    if point is not None and robot is not None:
        raise ValueError('Provide a measured point or a robot contact, not both')
    if robot is not None:
        target = {'robot': robot}
        if link is not None:
            target['link'] = link
        if offset is not None:
            target['offset'] = offset
        point = target_point(state, target, root)
        source = {'robot': robot, 'joints': dict(state['robots'][robot]['joints'])}
    else:
        source = 'measured_xyz'
    if point is not None:
        xyz = np.asarray(point, float)
        if xyz.shape != (3,) or not np.isfinite(xyz).all():
            raise ValueError('A measurement must be a finite world XYZ point')
        measurements.append({'point': xyz.tolist(), 'source': source})
    if fit:
        surface = fit_plane([m['point'] for m in measurements])
        surface['source'] = measurements.copy()
        state.setdefault('surfaces', {})[name] = surface
        return surface
    return {'name': name, 'measurements': measurements}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=['show', 'geometry', 'update', 'locate', 'project', 'pose', 'ik', 'attach', 'detach', 'export', 'fit', 'plane'])
    parser.add_argument('--scene', type=Path, default=Path('scene.json'))
    parser.add_argument('--input', type=argparse.FileType('r'))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.operation == 'export' and not args.output:
        parser.error('export requires --output scene.xml')
    args.data = json.load(args.input or sys.stdin) if args.operation not in ('show', 'export') else {}
    mutations = ('update', 'locate', 'attach', 'detach', 'fit', 'plane')
    args.mutates = args.operation in mutations and (args.operation != 'locate' or bool(args.data.get('object')))
    if args.mutates:
        with FileLock(str(args.scene.resolve()) + '.lock'):
            run(args)
    else:
        run(args)


def run(args):
    data = args.data
    state = json.loads(args.scene.read_text())
    if args.operation == 'show':
        result = state
    elif args.operation == 'geometry':
        from assets import geometry_info
        result = geometry_info(state, data['object'], args.scene.parent)
    elif args.operation == 'update':
        result = observe(state, data)
    elif args.operation == 'locate':
        object_name = data.pop('object', None)
        result = locate(state, root=args.scene.parent, **data)
        if object_name:
            observe(state, {'objects': {object_name: {'position': result['point_m'], 'source': data['views']}}})
    elif args.operation == 'fit':
        from assets import fit
        result = fit(state, args.scene.parent, **data)
    elif args.operation == 'plane':
        result = record_plane(state, root=args.scene.parent, **data)
    elif args.operation == 'project':
        camera = state['cameras'][data['camera']]
        result = project(target_point(state, data['target'], args.scene.parent), camera['K'],
                         camera['T_base_camera'], camera.get('distortion'))
    elif args.operation == 'pose':
        result = {'pose': pose(state, root=args.scene.parent, **data)}
    elif args.operation == 'ik':
        result = solve_ik(state, root=args.scene.parent, **data)
    elif args.operation == 'attach':
        result = attach(state, **data)
    elif args.operation == 'detach':
        state['objects'][data['object']].pop('attachment', None)
        result = state['objects'][data['object']]
    else:
        args.output.write_text(export(state, root=args.scene.parent))
        print(args.output)
        return
    if args.mutates:
        save(args.scene, state)
    encoded = json.dumps(result, indent=2, allow_nan=False) + '\n'
    if args.output:
        args.output.write_text(encoded)
    else:
        print(encoded, end='')


if __name__ == '__main__':
    main()
