#!/usr/bin/env python3
"""Shared JSON shape assets, instance transforms and image-based metric fitting."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry import camera_pixels, pixel_plane, pose_matrix, triangulate


def field_ref(asset, path):
    parent = asset
    for key in path[:-1]:
        parent = parent[key]
    return parent, path[-1]


def shared_fields(asset):
    """Equal scalar fields: first path owns the value; all members use it."""
    groups = {}
    for paths in asset.get('shared_fields', []):
        if len(paths) < 2:
            raise ValueError('A shared field group needs at least two paths')
        refs = [field_ref(asset, path) for path in paths]
        first, key = refs[0]
        value = first[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
            raise ValueError('Shared fields must select finite numeric values')
        for path, (parent, key) in zip(paths, refs):
            if tuple(path) in groups:
                raise ValueError('A shared field may belong to only one group')
            parent[key] = value
            groups[tuple(path)] = (paths, refs)
    return groups


def descriptor(state, name, root):
    obj = state['objects'][name]
    if 'asset' not in obj:
        return obj
    entry = state['assets'][obj['asset']]
    if 'file' in entry:
        path = Path(root) / entry['file']
        return json.loads(path.read_text())
    return entry


def sweep(path, radius, sections=32, closed=False):
    """Sweep a circular section along a sampled 3D centerline."""
    import trimesh
    p = np.asarray(path, float)
    if closed and np.allclose(p[0], p[-1]):
        p = p[:-1]
    if len(p) < (3 if closed else 2):
        raise ValueError('A sweep needs at least two points (three when closed)')
    edges = np.diff(np.vstack([p, p[:1]]) if closed else p, axis=0)
    if np.any(np.linalg.norm(edges, axis=1) < 1e-10):
        raise ValueError('Sweep centerline has duplicate adjacent points')
    tangent = np.roll(p, -1, axis=0) - np.roll(p, 1, axis=0) if closed else np.gradient(p, axis=0)
    lengths = np.linalg.norm(tangent, axis=1)
    if np.any(lengths < 1e-10):
        raise ValueError('Sweep centerline reverses direction at a point')
    tangent /= lengths[:, None]
    axis = np.eye(3)[np.argmin(np.abs(tangent[0]))]
    frames = []
    for t in tangent:
        axis = axis - t * np.dot(axis, t)
        if np.linalg.norm(axis) < 1e-8:
            axis = np.eye(3)[np.argmin(np.abs(t))]
            axis -= t * np.dot(axis, t)
        axis /= np.linalg.norm(axis)
        frames.append((axis.copy(), np.cross(t, axis)))
    angles = np.arange(sections) * 2 * np.pi / sections
    radii = np.broadcast_to(np.asarray(radius, float), (len(p),))
    if np.any(radii <= 0):
        raise ValueError('Sweep radii must be positive')
    vertices = np.vstack([c + r * (np.cos(angles)[:, None] * a + np.sin(angles)[:, None] * b)
                          for c, r, (a, b) in zip(p, radii, frames)])
    faces = []
    for i in range(len(p) if closed else len(p) - 1):
        j = (i + 1) % len(p)
        for k in range(sections):
            l = (k + 1) % sections
            faces.extend([[i*sections+k, i*sections+l, j*sections+k],
                          [i*sections+l, j*sections+l, j*sections+k]])
    if not closed:
        start, end = len(vertices), len(vertices) + 1
        vertices = np.vstack([vertices, p[0], p[-1]])
        for k in range(sections):
            l = (k + 1) % sections
            faces.extend([[start, l, k], [end, (len(p)-1)*sections+k, (len(p)-1)*sections+l]])
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
    mesh.fix_normals()
    return mesh


def part_mesh(part, root):
    import trimesh
    shared_fields(part)
    if 'parts' in part:
        meshes = [part_mesh(p, root) for p in part['parts'] if p.get('role') != 'marker']
        if not meshes:
            raise ValueError('An asset must contain solid geometry')
        if part.get('operation') == 'difference':
            mesh = trimesh.boolean.difference(meshes, engine='manifold')
        elif part.get('operation') is None:
            mesh = trimesh.util.concatenate(meshes)
        else:
            raise ValueError(f"Unsupported solid operation: {part['operation']}")
    else:
        shape, size = part.get('shape', 'box'), np.asarray(part.get('size', []), float)
        sections = int(part.get('sections', 48))
        if shape == 'box':
            mesh = trimesh.creation.box(extents=size)
        elif shape in ('sphere', 'ellipsoid'):
            mesh = trimesh.creation.icosphere(subdivisions=3)
            mesh.apply_scale(float(size[0]) if shape == 'sphere' else size)
        elif shape == 'cylinder':
            mesh = trimesh.creation.cylinder(radius=size[0], height=size[1], sections=sections)
        elif shape == 'capsule':
            mesh = trimesh.creation.capsule(radius=size[0], height=size[1], count=[sections, sections])
            mesh.apply_translation(-mesh.bounds.mean(axis=0))
        elif shape == 'mesh':
            mesh = (trimesh.load_mesh(Path(root) / part['file']) if 'file' in part else
                    trimesh.Trimesh(vertices=part['vertices'], faces=part['faces']))
            if isinstance(mesh, trimesh.Scene):
                mesh = mesh.dump(concatenate=True)
        elif shape == 'sweep':
            mesh = sweep(part['path'], part['radius'], sections, part.get('closed', False))
        elif shape == 'revolve':
            mesh = trimesh.creation.revolve(np.asarray(part['profile'], float), sections=sections)
        else:
            raise ValueError(f'Unsupported shape: {shape}')
    mesh.fix_normals()
    mesh.apply_scale(part.get('scale', [1, 1, 1]))
    mesh.apply_transform(pose_matrix([*part.get('position', [0, 0, 0]),
                                      *part.get('quaternion', [1, 0, 0, 0])]))
    return mesh


def mesh_signature(asset, root):
    """Identify a shape, including externally stored mesh revisions."""
    files = []
    geometry_keys = {'shape', 'size', 'position', 'quaternion', 'scale', 'role',
                     'operation', 'shared_fields', 'vertices', 'faces', 'file',
                     'path', 'radius', 'sections', 'closed', 'profile'}
    def visit(part):
        if part.get('shape') == 'mesh' and 'file' in part:
            path = Path(root) / part['file']
            stat = path.stat()
            files.append((part['file'], stat.st_size, stat.st_mtime_ns))
        value = {key: part[key] for key in geometry_keys if key in part}
        if 'parts' in part:
            value['parts'] = [visit(child) for child in part['parts']]
        return value
    geometry = visit(asset)
    return hashlib.sha256(json.dumps([geometry, files], sort_keys=True).encode()).hexdigest()


def cached_mesh(asset, root):
    """Reuse local geometry; an unwritable disk cache must not prevent viewing."""
    import trimesh
    import tempfile
    cache = Path(root) / '.geometry'
    key = mesh_signature(asset, root)
    path = cache / (key + '.npz')
    try:
        if path.exists():
            with np.load(path) as data:
                return trimesh.Trimesh(vertices=data['vertices'], faces=data['faces'], process=False)
    except OSError:
        pass
    mesh = part_mesh(copy.deepcopy(asset), root)
    temporary_path = None
    try:
        cache.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=cache, suffix='.npz', delete=False) as temporary:
            temporary_path = Path(temporary.name)
            np.savez(temporary, vertices=mesh.vertices, faces=mesh.faces)
        temporary_path.replace(path)
    except OSError:
        pass
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink(missing_ok=True)
            except OSError:
                pass
    return mesh


def object_mesh(state, name, root, world=True):
    """Return an instance's solid mesh, with scale and optional world pose applied."""
    obj = state['objects'][name]
    geometry = descriptor(state, name, root).copy()
    if geometry == obj:
        for key in ('position', 'quaternion', 'scale', 'rgba', 'source', 'observed_pose', 'predicted_pose', 'attachment'):
            geometry.pop(key, None)
    mesh = cached_mesh(geometry, root)
    mesh.apply_scale(obj.get('scale', [1, 1, 1]))
    if world:
        mesh.apply_transform(pose_matrix([*obj.get('position', [0, 0, 0]),
                                         *obj.get('quaternion', [1, 0, 0, 0])]))
    mesh.visual.face_colors = np.asarray(obj.get('rgba', [.8, .7, .3, 1])) * 255
    return mesh


def landmark_points(asset, root='.'):
    """Resolve fixed points or points carried by a nested part's current geometry."""
    shared_fields(asset)
    points, bounds = {}, {}
    for name, spec in asset.get('landmarks', {}).items():
        if not isinstance(spec, dict):
            points[name] = np.asarray(spec, float)
            continue
        part, chain = asset, []
        for index in spec['part']:
            part = part['parts'][index]
            chain.append(part)
        if 'bounds' in spec:
            key = tuple(spec['part'])
            if key not in bounds:
                local = {k: v for k, v in part.items() if k not in ('position', 'quaternion', 'scale')}
                bounds[key] = part_mesh(local, root).bounds
            low, high = bounds[key]
            point = low + np.asarray(spec['bounds'], float) * (high - low)
        elif 'vertex' in spec:
            point = np.asarray(part['vertices'][spec['vertex']], float)
        else:
            point = np.asarray(spec['point'], float)
        for parent in reversed(chain):
            point = point * parent.get('scale', [1, 1, 1])
            T = pose_matrix([*parent.get('position', [0, 0, 0]), *parent.get('quaternion', [1, 0, 0, 0])])
            point = T[:3, :3] @ point + T[:3, 3]
        points[name] = point
    return points


def fit(state, root, object, views, parameters=('position', 'rotation', 'scale'),
        scale_mode='uniform', plane=None, overlays=None, metadata=None, views_needed=None,
        geometry_fields=(), share_scale_with=()):
    """Fit named local landmarks to pixels. Unidentifiable fits are not committed."""
    from scipy.optimize import least_squares
    obj = state['objects'][object]
    pending = obj.get('pending_asset')
    fitting_state = state if not pending else {**state, 'objects': {**state['objects'], object: {**obj, 'asset': pending}}}
    asset = copy.deepcopy(descriptor(fitting_state, object, root))
    if not (pending or obj.get('asset')):
        for key in ('position', 'quaternion', 'scale'):
            asset.pop(key, None)
    shared = shared_fields(asset)
    asset_id = pending or obj.get('asset')
    for name in share_scale_with:
        other = state['objects'][name]
        if name == object or not asset_id or other.get('pending_asset', other.get('asset')) != asset_id:
            raise ValueError('share_scale_with selects other instances of the same asset')
    landmarks = landmark_points(asset, root)
    if not landmarks:
        raise ValueError('The asset needs local landmarks for metric fitting')
    if not set(parameters) <= {'position', 'rotation', 'scale'} or len(parameters) != len(set(parameters)):
        raise ValueError('parameters may contain position, rotation and scale once each')
    if scale_mode not in ('fixed', 'uniform', 'axes'):
        raise ValueError('scale_mode must be fixed, uniform or axes')
    parameters = [key for key in parameters if key != 'scale' or scale_mode != 'fixed']
    fields, field_values, lower, upper, field_paths = [], [], [], [], []
    for field in geometry_fields:
        path = field['path']
        if not path or not any(key in path for key in ('size', 'position', 'scale', 'vertices', 'profile', 'radius', 'path')):
            raise ValueError('Geometry fields select numeric geometry entries within the asset')
        paths, refs = shared.get(tuple(path), ([path], [field_ref(asset, path)]))
        parent, key = refs[0]
        value = parent[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
            raise ValueError('Each geometry field must select one finite numeric value')
        if any(tuple(p) in field_paths for p in paths):
            raise ValueError('Select one geometry field per shared group')
        lo, hi = field['bounds']
        if not np.isfinite([lo, hi]).all() or not lo < hi or not lo <= value <= hi:
            raise ValueError('Geometry field bounds must be finite, increasing and contain the initial value')
        fields.append(refs)
        field_paths.extend(tuple(p) for p in paths)
        field_values.append(value); lower.append(lo); upper.append(hi)
    labels, local, pixels, cameras, expanded = [], [], [], [], []
    groups = {}
    for view in views:
        obs = json.loads((Path(root) / view['observation']).read_text()) if 'observation' in view else state
        camera = view if 'K' in view and 'T_base_camera' in view else obs['cameras'][view['camera']]
        expanded.append((view, camera))
        for label, pixel in view['pixels'].items():
            labels.append(label)
            local.append(landmarks[label]); pixels.append(pixel); cameras.append(camera)
            groups.setdefault(label, []).append(dict(pixel=pixel, K=camera['K'], T_base_camera=camera['T_base_camera'],
                                                    distortion=camera.get('distortion')))
    local, pixels = np.asarray(local, float), np.asarray(pixels, float)
    if not len(local):
        raise ValueError('Supply landmark pixel measurements')
    q = np.asarray(obj.get('quaternion', [1, 0, 0, 0]))
    initial_r = Rotation.from_quat(q[[1, 2, 3, 0]])
    scale = np.asarray(obj.get('scale', [1, 1, 1]), float)
    if np.any(scale == 0):
        raise ValueError('Scale must be nonzero')
    signs = np.sign(scale)
    initial = dict(position=np.asarray(obj.get('position', [0, 0, 0]), float),
                   rotation=initial_r.as_rotvec(), scale=np.log(np.abs(scale)))
    # Multi-view point reconstruction supplies a useful starting translation/scale.
    pairs = []
    for label, measurements in groups.items():
        if len(measurements) >= 2:
            try:
                pairs.append((landmarks[label], triangulate(measurements)['point_m']))
            except ValueError:
                pass
    if pairs:
        a, b = np.asarray([x[0] for x in pairs]), np.asarray([x[1] for x in pairs])
        if 'scale' in parameters:
            ac, bc = a - a.mean(axis=0), (b - b.mean(axis=0)) @ initial_r.as_matrix()
            if scale_mode == 'uniform':
                scaled = ac * scale
                factor = np.sum(scaled * bc) / max(np.sum(scaled**2), 1e-12)
                candidate = scale * factor
            else:
                denominator = np.sum(ac**2, axis=0)
                candidate = np.divide(np.sum(ac*bc, axis=0), denominator, out=scale.copy(), where=denominator > 1e-10)
            good = candidate * signs > 1e-8
            scale[good] = candidate[good]
            initial['scale'] = np.log(np.abs(scale))
        if 'position' in parameters:
            initial['position'] = b.mean(axis=0) - initial_r.apply(a.mean(axis=0) * scale)
    if not pairs and plane is not None and 'position' in parameters:
        surface = state['surfaces'][plane['name']]
        landmark = plane['landmark']
        measurements = groups.get(landmark, [])
        if measurements:
            point = pixel_plane(**measurements[0], normal=surface['normal'],
                                offset_m=surface['offset_m'], height_m=plane.get('height_m', 0))['point_m']
            initial['position'] = np.asarray(point) - initial_r.apply(np.asarray(landmarks[landmark]) * scale)
    selected = list(parameters)
    if not selected and not fields:
        raise ValueError('Select at least one parameter group to fit')
    # Uniform scale is one multiplier of the instance's established proportions.
    scale_reference = np.exp(initial['scale'])
    if scale_mode == 'uniform':
        initial['scale'] = np.zeros(1)
    sizes = [len(initial[key]) for key in selected]
    boundaries = np.cumsum([0, *sizes])
    pose_count = int(boundaries[-1])
    x0 = np.concatenate([*(initial[key] for key in selected), np.asarray(field_values)])
    def unpack(x):
        value = dict(initial)
        value.update({key: x[boundaries[i]:boundaries[i+1]] for i, key in enumerate(selected)})
        fitted_scale = np.exp(value['scale']) * (scale_reference if scale_mode == 'uniform' else 1)
        return value['position'], Rotation.from_rotvec(value['rotation']), signs * fitted_scale
    def update_geometry(x):
        for refs, value in zip(fields, x[pose_count:]):
            for parent, key in refs:
                parent[key] = float(value)
        return landmark_points(asset, root) if fields else landmarks
    inverses = [np.linalg.inv(np.asarray(c['T_base_camera'])) for c in cameras]
    def projected(x):
        t, r, s = unpack(x)
        current = update_geometry(x)
        world = r.apply(np.asarray([current[label] for label in labels]) * s) + t
        camera_points = np.asarray([T[:3, :3] @ p + T[:3, 3] for T, p in zip(inverses, world)])
        projected_pixels, cursor = [], 0
        for view, camera in expanded:
            count = len(view['pixels'])
            projected_pixels.append(camera_pixels(camera_points[cursor:cursor+count], camera['K'],
                                                  camera.get('distortion')))
            cursor += count
        return np.concatenate(projected_pixels), camera_points[:, 2]
    def residual(x):
        projected_pixels, depth = projected(x)
        values = [(projected_pixels - pixels).ravel(), np.minimum(depth-1e-6, 0) * 1000]
        if plane is not None:
            t, r, s = unpack(x)
            surface = state['surfaces'][plane['name']]
            point = r.apply(update_geometry(x)[plane['landmark']] * s) + t
            normal = np.asarray(surface['normal'], float)
            distance = (normal @ point + surface['offset_m']) / np.linalg.norm(normal)
            values.append([(distance - plane.get('height_m', 0)) / plane.get('sigma_m', .001)])
        return np.concatenate(values)
    solution = least_squares(residual, x0, max_nfev=500, x_scale='jac',
                             bounds=([-np.inf]*pose_count + lower, [np.inf]*pose_count + upper))
    singular = np.linalg.svd(solution.jac, compute_uv=False)
    threshold = max(singular[0] * 1e-6, 1e-8)
    rank = int(np.sum(singular > threshold))
    fitted_pixels, depths = projected(solution.x)
    t, r, s = unpack(solution.x)
    quaternion = r.as_quat()[[3, 0, 1, 2]].tolist()
    observable = rank == len(solution.x) and bool(np.all(depths > 0)) and bool(solution.success)
    result = dict(committed=observable, pixel_rms=float(np.sqrt(np.mean((fitted_pixels-pixels)**2))),
                  identifiable=observable, rank=rank, parameter_count=len(solution.x),
                  parameters=selected, scale_mode=scale_mode, singular_values=singular.tolist(),
                  candidate=dict(position=t.tolist(), quaternion=quaternion, scale=s.tolist()),
                  note='Local identifiability assumes correct shape, landmark matches and camera calibration.')
    if fields:
        result['geometry_fields'] = [dict(path=field['path'],
                                     applied_to=shared.get(tuple(field['path']), ([field['path']], []))[0],
                                     initial=float(start), value=float(value))
                                     for field, start, value in zip(geometry_fields, field_values, solution.x[pose_count:])]
    if share_scale_with:
        result['shared_scale_with'] = list(share_scale_with)
    if overlays:
        import cv2
        output = Path(overlays)
        output.mkdir(parents=True, exist_ok=True)
        result['overlays'] = []
        cursor = 0
        # Draw fitted wireframe as well as measured/predicted landmarks.
        preview_state = {**fitting_state, 'objects': {**fitting_state['objects'], object: {**fitting_state['objects'][object], **result['candidate']}}}
        preview_state = copy.deepcopy(preview_state)
        preview_state.setdefault('assets', {})['__fit_candidate'] = asset
        preview_state['objects'][object]['asset'] = '__fit_candidate'
        mesh = object_mesh(preview_state, object, root)
        for index, (view, camera) in enumerate(expanded):
            n = len(view['pixels'])
            if 'image' not in view:
                cursor += n
                continue
            frame = cv2.imread(str(Path(root) / view['image']))
            if frame is None:
                raise ValueError(f"Cannot read image: {view['image']}")
            T = np.linalg.inv(np.asarray(camera['T_base_camera']))
            xyz = mesh.vertices @ T[:3, :3].T + T[:3, 3]
            uv = camera_pixels(xyz, camera['K'], camera.get('distortion'))
            for a, b in mesh.edges_unique:
                if min(xyz[a, 2], xyz[b, 2]) > 0:
                    aa, bb = np.clip(uv[[a, b]], -100000, 100000).astype(int)
                    cv2.line(frame, tuple(aa), tuple(bb), (160, 160, 0), 1, cv2.LINE_AA)
            for label, observed, predicted in zip(view['pixels'], pixels[cursor:cursor+n], fitted_pixels[cursor:cursor+n]):
                a, b = np.round(observed).astype(int), np.round(predicted).astype(int)
                cv2.circle(frame, tuple(a), 4, (0, 255, 0), 1)
                cv2.drawMarker(frame, tuple(b), (0, 0, 255), markerSize=8)
                cv2.putText(frame, label, tuple(a + [5, -5]), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 0), 1)
            destination = output / f'{object}_{index}.png'
            cv2.imwrite(str(destination), frame)
            result['overlays'].append(str(destination))
            cursor += n
    if observable:
        from scene import observe, save
        if fields:
            asset_id = pending or obj.get('asset')
            if asset_id:
                entry = state['assets'][asset_id]
                if 'file' in entry:
                    save(Path(root) / entry['file'], asset)
                else:
                    state['assets'][asset_id] = asset
            else:
                obj.update(asset)
        source = {'method': 'landmark_fit', 'pixel_rms': result['pixel_rms'], 'views': views,
                  'metadata': metadata or {}}
        if pending:
            obj['asset'] = obj.pop('pending_asset')
        observe(state, {'objects': {object: {**result['candidate'], 'source': source}}})
        for name in share_scale_with:
            other = state['objects'][name]
            other['scale'] = np.copysign(np.abs(s), other.get('scale', [1, 1, 1])).tolist()
        obj['geometry_status'] = 'ready'
        obj['geometry_fit'] = result.copy()
    else:
        result['note'] += ' Scene unchanged. Add translated views or hold unsupported parameter groups fixed.'
    return result


def geometry_info(state, name, root):
    """Derived metric geometry; never a second stored size estimate."""
    mesh = object_mesh(state, name, root, world=False)
    obj = state['objects'][name]
    scale = np.asarray(obj.get('scale', [1, 1, 1]))
    return dict(object=name, bounds_m=mesh.bounds.tolist(), dimensions_m=mesh.extents.tolist(),
                landmarks_m={key: (np.asarray(point) * scale).tolist()
                             for key, point in landmark_points(descriptor(state, name, root), root).items()})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=['create', 'transform', 'export'])
    parser.add_argument('--scene', type=Path, default=Path('scene.json'))
    parser.add_argument('--object')
    parser.add_argument('--id', help='Asset identifier for create')
    parser.add_argument('--input', type=Path, help='JSON descriptor for create')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--translate', nargs=3, type=float, help='World translation increment in metres')
    parser.add_argument('--rotate', nargs=3, type=float, help='Local intrinsic XYZ Euler degrees, composed after current rotation')
    parser.add_argument('--scale', nargs=3, type=float, help='Local scale multipliers')
    parser.add_argument('--mirror', choices=['x', 'y', 'z'], action='append', help='Negate local axis, repeatable')
    parser.add_argument('--color', nargs=4, type=float, help='RGBA in [0,1]')
    args = parser.parse_args()
    if args.operation == 'create':
        if not args.id or not args.input:
            parser.error('create requires --id and --input')
        if Path(args.id).name != args.id or args.id in ('.', '..'):
            parser.error('Asset id must be a filename-safe name')
    elif not args.object:
        parser.error('Operation requires --object')
    if args.operation == 'export' and not args.output:
        parser.error('export requires --output')
    if args.operation == 'export':
        run(args)
    else:
        from filelock import FileLock
        with FileLock(str(args.scene.resolve()) + '.lock'):
            run(args)


def run(args):
    from scene import observe, save
    state = json.loads(args.scene.read_text())
    if args.operation == 'create':
        asset = json.loads(args.input.read_text())
        mesh = cached_mesh(asset, args.scene.parent)
        if not np.isfinite(mesh.vertices).all() or not len(mesh.faces):
            raise ValueError('Asset must contain finite solid geometry')
        file = Path('assets') / (args.id + '.json')
        (args.scene.parent / file).parent.mkdir(parents=True, exist_ok=True)
        save(args.scene.parent / file, asset)
        state.setdefault('assets', {})[args.id] = {'file': str(file)}
        result = {'asset': args.id, 'file': str(file), 'vertices': len(mesh.vertices), 'faces': len(mesh.faces)}
    else:
        obj = state['objects'][args.object]
        if args.operation == 'export':
            object_mesh(state, args.object, args.scene.parent).export(args.output)
            print(args.output)
            return
        if args.translate:
            obj['position'] = (np.asarray(obj.get('position', [0, 0, 0])) + args.translate).tolist()
        if args.rotate:
            q = np.asarray(obj.get('quaternion', [1, 0, 0, 0]))
            rotation = Rotation.from_quat(q[[1, 2, 3, 0]]) * Rotation.from_euler('XYZ', args.rotate, degrees=True)
            obj['quaternion'] = rotation.as_quat()[[3, 0, 1, 2]].tolist()
        scale = np.asarray(obj.get('scale', [1, 1, 1]), float)
        if args.scale:
            scale *= args.scale
        for axis in args.mirror or []:
            scale['xyz'.index(axis)] *= -1
        if np.any(scale == 0):
            raise ValueError('Scale must be nonzero')
        obj['scale'] = scale.tolist()
        if args.color:
            if not all(0 <= x <= 1 for x in args.color):
                raise ValueError('RGBA must be in [0,1]')
            obj['rgba'] = args.color
        if args.translate or args.rotate:
            observe(state, {'objects': {args.object: {
                'position': obj.get('position', [0, 0, 0]),
                'quaternion': obj.get('quaternion', [1, 0, 0, 0]),
                'source': 'explicit asset transform'}}})
        result = obj
    save(args.scene, state)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
