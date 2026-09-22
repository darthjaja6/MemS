"""Read-only scene and replay serialization using the computation mesh pipeline."""
from datetime import datetime
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh

from assets import object_mesh
from kinematics import Robot, origin


def geometry(mesh):
    return {"vertices": np.asarray(mesh.vertices).tolist(), "faces": np.asarray(mesh.faces).tolist()}


def file_revision(path):
    path = Path(path).resolve()
    try:
        stat = path.stat()
        return str(path), stat.st_mtime_ns, stat.st_size
    except OSError:
        return str(path), None, None


def visual_path(robot, filename):
    if filename.startswith('package://'):
        # The viewer supports URDF packages stored beside their mesh directory.
        filename = filename.split('/', 3)[-1]
    return robot.path.parent / filename


class SceneView:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.models = {}
        self.visuals = {}
        self.cached = None

    def robot(self, descriptor):
        path = (self.path.parent / descriptor['urdf']).resolve()
        stamp = file_revision(path)
        if stamp not in self.models:
            self.models[stamp] = Robot(path)
        return self.models[stamp]

    def robot_visuals(self, name, descriptor):
        robot = self.robot(descriptor)
        key = (name, file_revision(robot.path), tuple(file_revision(visual_path(robot, v['mesh']))
               for v in robot.description()['visuals']))
        if key in self.visuals:
            return self.visuals[key]
        visuals = []
        for link, element in robot.links.items():
            for visual in element.findall('visual'):
                shape = visual.find('geometry')
                if shape is None:
                    continue
                mesh = None
                source = shape.find('mesh')
                if source is not None:
                    mesh = trimesh.load(visual_path(robot, source.get('filename')), force='mesh')
                    mesh.apply_scale(np.fromstring(source.get('scale', '1 1 1'), sep=' '))
                elif shape.find('box') is not None:
                    mesh = trimesh.creation.box(np.fromstring(shape.find('box').get('size'), sep=' '))
                elif shape.find('cylinder') is not None:
                    cylinder = shape.find('cylinder')
                    mesh = trimesh.creation.cylinder(float(cylinder.get('radius')), float(cylinder.get('length')))
                elif shape.find('sphere') is not None:
                    mesh = trimesh.creation.icosphere(radius=float(shape.find('sphere').get('radius')))
                if mesh is None:
                    continue
                mesh.apply_transform(origin(visual.find('origin')))
                color = visual.find('material/color')
                rgba = np.fromstring(color.get('rgba'), sep=' ').tolist() if color is not None else [.45, .52, .50, 1]
                visuals.append({'link': f'{name}/{link}', **geometry(mesh), 'rgba': rgba})
        self.visuals[key] = visuals
        return visuals

    def frames(self, state, updates=None):
        frames, tips = {}, {}
        for name, descriptor in state.get('robots', {}).items():
            model = self.robot(descriptor)
            q = dict(descriptor.get('joints', {}))
            q.update((updates or {}).get(name, {}).get('joints', {}))
            base = np.asarray(descriptor.get('base_transform', np.eye(4)))
            local = model.frames(q)
            frames.update({f'{name}/{link}': (base @ frame).tolist() for link, frame in local.items()})
            tip = descriptor.get('tip')
            if tip in local:
                offset = np.asarray([*descriptor.get('tool', {}).get('point_in_tip', [0, 0, 0]), 1])
                tips[name] = ((base @ local[tip]) @ offset)[:3].tolist()
        return frames, tips

    def objects(self, state):
        objects = []
        for name, obj in state.get('objects', {}).items():
            entry = {'name': name, **obj, 'drawable': False}
            try:
                mesh = object_mesh(state, name, self.path.parent, world=False)
                entry.update(geometry(mesh), dimensions=mesh.extents.tolist(), drawable=obj.get('position') is not None)
            except (ValueError, KeyError, OSError) as error:
                entry['geometry_error'] = str(error)
            objects.append(entry)
        return objects

    @staticmethod
    def plane(state):
        for surface in state.get('surfaces', {}).values():
            normal = surface.get('normal')
            if normal is not None and 'offset_m' in surface and np.linalg.norm(normal) > 0:
                return [*normal, surface['offset_m']]
        return None

    def scene_state(self, state):
        return {'objects': self.objects(state), 'plane': self.plane(state),
                'plane_object': next((s.get('object') for s in state.get('surfaces', {}).values()
                                      if s.get('normal') is not None and 'offset_m' in s), None)}

    def serialize(self, state):
        frames, tips = self.frames(state)
        visuals = [v for name, descriptor in state.get('robots', {}).items()
                   for v in self.robot_visuals(name, descriptor)]
        return {'generic': True, **self.scene_state(state), 'frames': frames, 'tips': tips,
                'title': state.get('title', 'Current scene'), 'example': state.get('example', False),
                'robot_visuals': visuals, 'robot_name': ' / '.join(dict.fromkeys(self.robot(r).description()['name']
                                                       for r in state.get('robots', {}).values())),
                'pose_known': bool(frames)}

    def dependencies(self, state):
        """Files actually used by this scene, independent of directory layout."""
        files = set()
        def mesh_files(part):
            if part.get('shape') == 'mesh' and 'file' in part:
                files.add((self.path.parent / part['file']).resolve())
            for child in part.get('parts', []):
                mesh_files(child)
        for obj in state.get('objects', {}).values():
            try:
                shape = state['assets'][obj['asset']] if 'asset' in obj else obj
                if 'asset' in obj and 'file' in shape:
                    path = (self.path.parent / shape['file']).resolve()
                    files.add(path)
                    shape = json.loads(path.read_text())
                mesh_files(shape)
            except (OSError, ValueError, KeyError):
                # objects() reports invalid or temporarily unavailable geometry.
                continue
        for descriptor in state.get('robots', {}).values():
            robot = self.robot(descriptor)
            files.add(robot.path)
            files.update(visual_path(robot, v['mesh']).resolve() for v in robot.description()['visuals'])
        return files

    def state(self, since=None):
        raw = self.path.read_bytes()
        state = json.loads(raw)
        # Asset edits must refresh the view even if scene.json was not rewritten.
        digest = hashlib.sha256(raw)
        for file in sorted(self.dependencies(state)):
            digest.update(repr(file_revision(file)).encode())
        revision = digest.hexdigest()
        if since == revision:
            return {'unchanged': True, 'revision': revision}
        if self.cached and self.cached['revision'] == revision:
            return self.cached
        result = self.serialize(state)
        result.update(revision=revision, source=str(self.path),
                      updated_at=datetime.fromtimestamp(self.path.stat().st_mtime).astimezone().isoformat())
        self.cached = result
        return result


def read_replay(path, key):
    data = json.loads(path.read_text())
    if not data.get('frames') or 'scene' not in data:
        return None
    view = SceneView(data.get('scene_path', path.parent / 'scene.json'))
    state = data['scene']
    frames = []
    for sample in data['frames']:
        robot_frames, tips = view.frames(state, sample.get('robots'))
        frames.append({'time': sample['time'], 'frames': robot_frames,
                       'tips': sample.get('tips', tips), 'objects': sample.get('objects', {}),
                       'source': sample.get('source')})
    kind = data.get('kind', 'recorded')
    return {'generic': True, 'initial': view.serialize(state), 'frames': frames,
            'scenes': [{'time': s['time'], 'state': view.scene_state(s['scene'])}
                       for s in data.get('scenes', [])],
            'trajectory_source': data.get('trajectory_source', 'Recorded joints'),
            'summary': {'id': key, 'title': data.get('title', path.parent.name),
                        'started_at': data.get('started_at', path.stat().st_mtime),
                        'duration': frames[-1]['time'], 'samples': len(frames),
                        'actions': len(data.get('actions', [])),
                        'status': 'Preview' if kind == 'preview' else 'Recorded'},
            'actions': data.get('actions', []), 'checkpoints': data.get('checkpoints', [])}


def read_observations(path, key):
    """RoboDojo archive: commands are predictions, checkpoints are measurements."""
    import copy

    folder = path.parent
    metadata = json.loads(path.read_text())
    view = SceneView(path)
    period = float(metadata.get('control_period_s', .04))  # Historical RoboDojo control period.
    state = copy.deepcopy(metadata)
    state['objects'] = {}  # Never apply the final scene to the beginning of a recording.
    state['assets'] = {}
    observations = {int(p.parent.name): p for p in folder.glob('observations/*/observation.json')
                    if p.parent.name.isdigit()}
    if not observations:
        return None
    frames, snapshots, checkpoints, actions = [], [], [], []
    previous_time = 0.

    def record(raw, moment, source):
        updates = {}
        for name, robot in state.get('robots', {}).items():
            values = raw.get(name + '_arm_joint_state')
            if values is None:
                continue
            joints = dict(zip(robot['joint_names'], values))
            opening = raw.get(name + '_ee_joint_state')
            if opening is not None and 'gripper_scale' in robot:
                low, high = robot['gripper_scale']
                joints.update({joint: low + opening[0] * (high - low)
                               for joint in robot.get('gripper_joints', [])})
            updates[name] = {'joints': joints}
        robot_frames, tips = view.frames(state, updates)
        frames.append({'time': moment, 'frames': robot_frames, 'tips': tips,
                       'objects': {}, 'source': source, 'robots': updates})

    initial = view.serialize(state)
    indices = sorted(set(observations) | {int(p.stem.split('_')[-1])
                     for pattern in ('scene_[0-9]*.json', 'preview_scene_[0-9]*.json')
                     for p in folder.glob(pattern)})

    def snapshot(path, moment):
        nonlocal state
        if path.exists():
            state = json.loads(path.read_text())
            snapshots.append({'time': moment, 'state': view.scene_state(state)})

    for index in indices:
        snapshot(folder / f'preview_scene_{index:03d}.json', previous_time)
        commands = folder / 'bridge' / f'actions_{index:03d}.json'
        if commands.exists():
            rows = json.loads(commands.read_text())
            for offset, command in enumerate(rows, 1):
                record(command, previous_time + offset * period, 'command')
            actions.append({'name': f'Command sequence {index}', 'start': previous_time,
                            'end': previous_time + len(rows) * period})
        if index in observations:
            observation = json.loads(observations[index].read_text())
            moment = observation.get('control_budget', {}).get('steps_used', 0) * period
            snapshot(folder / f'scene_{index:03d}.json', moment)
            record(observation['state'], moment, 'observed')
            checkpoints.append({'time': moment, 'name': f'Observation {index}'})
            previous_time = moment
    frames.sort(key=lambda frame: frame['time'])
    return {'generic': True, 'initial': initial, 'frames': frames, 'scenes': snapshots,
            'summary': {'id': key, 'title': folder.name.replace('_', ' '),
                        'started_at': min(p.stat().st_mtime for p in observations.values()),
                        'duration': frames[-1]['time'], 'samples': len(frames),
                        'actions': len(actions), 'status': 'Commands + observations'},
            'actions': actions, 'checkpoints': checkpoints,
            'trajectory_source': 'Commanded path · measured checkpoints'}
