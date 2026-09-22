"""Check metric recovery and preserve uncertainty when image scale is ambiguous."""
import itertools
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'skills/spatial-memory/scripts'))
from assets import fit, object_mesh, sweep, landmark_points, geometry_info
from geometry import camera_looking_at, project
from scene import record_plane, target_point


class AssetTests(unittest.TestCase):
    def test_mesh_reuse_and_shape_change(self):
        import assets
        with tempfile.TemporaryDirectory() as directory:
            state = dict(assets={'a':dict(parts=[dict(shape='box', size=[1,2,3])])},
                         objects={'item':dict(asset='a', position=[0,0,0])})
            with patch('assets.part_mesh', wraps=assets.part_mesh) as build:
                object_mesh(state, 'item', directory)
                first = build.call_count
                state['objects']['item'].update(position=[4,5,6], scale=[-2,2,2], rgba=[1,0,0,1])
                state['assets']['a']['uncertainty'] = ['A measurement note']
                mesh = object_mesh(state, 'item', directory)
                self.assertEqual(build.call_count, first)
                np.testing.assert_allclose(mesh.extents, [2,4,6])
                np.testing.assert_allclose(mesh.centroid, [4,5,6])
                state['assets']['a']['parts'][0]['size'][0] = 2
                np.testing.assert_allclose(object_mesh(state, 'item', directory).extents, [4,4,6])
                self.assertGreater(build.call_count, first)

    def test_multiview_shared_dimensions_and_instance_scale(self):
        import copy
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            asset = dict(parts=[dict(shape='box',size=[.08,.04,.03],position=[x,0,0]) for x in [-.2,0,.2]],
                         shared_fields=[[['parts',i,'size',0] for i in range(3)]],
                         landmarks={str(i):dict(part=[1],bounds=list(p))
                                    for i,p in enumerate(itertools.product([0,1],repeat=3))})
            state = dict(assets={'shared':asset}, objects={
                'a':dict(asset='shared',position=[0,0,.8],scale=[1,1,1]),
                'b':dict(asset='shared',position=[.4,.2,.8],scale=[-2,2,2])},cameras={})
            for i,center in enumerate([[1,-1,1.2],[-1,-.6,1.3]]):
                state['cameras'][str(i)] = dict(K=[[600,0,320],[0,600,240],[0,0,1]],
                    T_base_camera=camera_looking_at(center,[0,0,.8]).tolist())
            truth = copy.deepcopy(asset)
            truth['parts'][0]['size'][0] = .12
            views = [dict(camera=n,pixels={key:project(point+[0,0,.8],c['K'],c['T_base_camera'])
                         for key,point in landmark_points(truth).items()}) for n,c in state['cameras'].items()]
            result = fit(state,root,'a',views,parameters=[],share_scale_with=['b'],
                         geometry_fields=[dict(path=['parts',1,'size',0],bounds=[.02,.2])])
            self.assertTrue(result['committed'])
            self.assertEqual(result['parameter_count'],1)
            self.assertEqual(len(result['geometry_fields'][0]['applied_to']),3)
            np.testing.assert_allclose([p['size'][0] for p in state['assets']['shared']['parts']],[.12]*3)
            self.assertEqual(state['objects']['b']['scale'],[-1,1,1])
            self.assertEqual(state['objects']['b']['position'],[.4,.2,.8])
            np.testing.assert_allclose(geometry_info(state,'a',root)['dimensions_m'],[.52,.04,.03])
            np.testing.assert_allclose(geometry_info(state,'b',root)['dimensions_m'],[.52,.04,.03])

    def test_subtracted_solid_dimension_fit_preserves_outer_shape(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            rim = [[1,.5,.75], [.5,1,.75], [0,.5,.75], [.5,0,.75]]
            asset = dict(operation='difference', parts=[
                dict(shape='box',size=[.4,.3,.1]), dict(shape='cylinder',size=[.04,.2])],
                landmarks={str(i):dict(part=[1],bounds=p) for i,p in enumerate(rim)})
            state = dict(assets={'part':asset}, objects={'item':dict(asset='part',position=[0,0,.8])},
                         cameras={'view':dict(K=[[600,0,320],[0,600,240],[0,0,1]],
                         T_base_camera=camera_looking_at([.2,-.3,1.5],[0,0,.8]).tolist())})
            camera = state['cameras']['view']
            points = [[.08,0,.85],[0,.08,.85],[-.08,0,.85],[0,-.08,.85]]
            views = [dict(camera='view',pixels={str(i):project(p,camera['K'],camera['T_base_camera'])
                                               for i,p in enumerate(points)})]
            result = fit(state, root, 'item', views, parameters=[],
                         geometry_fields=[dict(path=['parts',1,'size',0],bounds=[.01,.12])])
            self.assertTrue(result['committed'])
            self.assertAlmostEqual(state['assets']['part']['parts'][1]['size'][0], .08)
            mesh = object_mesh(state,'item',root,world=False)
            np.testing.assert_allclose(mesh.extents,[.4,.3,.1],atol=1e-7)
            self.assertTrue(mesh.is_watertight)
            self.assertAlmostEqual(mesh.volume, (.4*.3-np.pi*.08**2)*.1, delta=1e-5)

    def test_nested_geometry_fit_updates_asset_and_targets(self):
        import copy
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            asset = dict(parts=[dict(position=[.15, 0, 0], parts=[
                dict(shape='box', size=[.08, .04, .03], position=[0, 0, .05])])],
                landmarks={str(i): dict(part=[0, 0], bounds=list(p))
                           for i, p in enumerate(itertools.product([0, 1], repeat=3))})
            original = json.dumps(asset)
            (root/'asset.json').write_text(original)
            state = dict(assets={'shared':dict(file='asset.json')}, objects={
                'item':dict(asset='shared', position=[0, 0, .8], scale=[1,1,1])}, cameras={})
            truth = copy.deepcopy(asset)
            truth['parts'][0]['parts'][0].update(size=[.12,.04,.03], position=[0,0,.09])
            for i, center in enumerate([[1,-1,1.2], [-1,-.6,1.3]]):
                state['cameras'][str(i)] = dict(K=[[600,0,320],[0,600,240],[0,0,1]],
                    T_base_camera=camera_looking_at(center, [0,0,.8]).tolist())
            views = [dict(camera=name, pixels={key:project(point+[0,0,.8], camera['K'], camera['T_base_camera'])
                     for key, point in landmark_points(truth).items()}) for name,camera in state['cameras'].items()]
            fields = [dict(path=['parts',0,'parts',0,'size',0], bounds=[.02,.2]),
                      dict(path=['parts',0,'parts',0,'position',2], bounds=[0,.2])]
            result = fit(state, root, 'item', views, parameters=[], geometry_fields=fields)
            self.assertTrue(result['committed'])
            fitted = json.loads((root/'asset.json').read_text())['parts'][0]['parts'][0]
            np.testing.assert_allclose(fitted['size'], [.12,.04,.03], atol=1e-8)
            self.assertAlmostEqual(fitted['position'][2], .09)
            np.testing.assert_allclose(geometry_info(state, 'item', root)['dimensions_m'], [.12,.04,.03])
            np.testing.assert_allclose(target_point(state, dict(object='item', landmark='7'), root), [.21,.02,.905])
            # An unobserved field cannot silently change the published asset.
            (root/'asset.json').write_text(original)
            center_views = [dict(camera=v['camera'], pixels={'center':list(v['pixels']['0'])}) for v in views]
            asset['landmarks'] = {'center':dict(part=[0,0], point=[0,0,0])}
            (root/'asset.json').write_text(json.dumps(asset))
            before = (root/'asset.json').read_text()
            result = fit(state, root, 'item', center_views, parameters=[], geometry_fields=fields[:1])
            self.assertFalse(result['committed'])
            self.assertEqual(before, (root/'asset.json').read_text())

    def test_metric_fit_and_ambiguous_scale(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            landmarks = {str(i): list(p) for i, p in enumerate(itertools.product([-.5, .5], repeat=3))}
            (root / 'asset.json').write_text(json.dumps({
                'parts': [{'shape': 'box', 'size': [1, 1, 1]}], 'landmarks': landmarks}))
            state = {'assets': {'shape': {'file': 'asset.json'}}, 'objects': {
                'item': {'pending_asset': 'shape', 'shape': 'box', 'size': [.2, .2, .2],
                         'position': [0, 0, 1], 'scale': [.2, .2, .2]}}, 'cameras': {}}
            position = np.array([.12, -.03, .8])
            rotation = Rotation.from_euler('xyz', [.2, -.3, .25])
            scale = np.array([.2, .1, .15])
            for i, camera_position in enumerate([[1, -1, 1.2], [-1, -.6, 1.3], [.2, 1, 1.1]]):
                state['cameras'][str(i)] = {'K': [[600, 0, 320], [0, 600, 240], [0, 0, 1]],
                    'T_base_camera': camera_looking_at(camera_position, position).tolist(),
                    'distortion': [.15,-.02,.002,.001,0]}
            views = [{'camera': name, 'pixels': {key: project(rotation.apply(np.array(p)*scale)+position,
                camera['K'], camera['T_base_camera'],camera['distortion']) for key, p in landmarks.items()}}
                for name, camera in state['cameras'].items()]
            result = fit(state, root, 'item', views, scale_mode='axes')
            self.assertTrue(result['committed'])
            self.assertNotIn('size', state['objects']['item'])
            np.testing.assert_allclose(state['objects']['item']['scale'], scale)
            np.testing.assert_allclose(state['objects']['item']['position'], position)
            self.assertLess(result['pixel_rms'], 1e-6)
            before = json.dumps(state)
            self.assertFalse(fit(state, root, 'item', views[:1], scale_mode='axes')['committed'])
            self.assertEqual(before, json.dumps(state))
            state['objects']['item']['scale'][0] *= -1
            mesh = object_mesh(state, 'item', root, world=False)
            np.testing.assert_allclose(mesh.extents, scale)
            self.assertGreater(mesh.volume, 0)

    def test_open_geometry_and_plane_measurements(self):
        mesh = sweep([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], .05, closed=True)
        self.assertTrue(mesh.is_watertight)
        self.assertGreater(mesh.volume, 0)
        state = {}
        for point in [[0, 0, .3], [1, 0, .3], [0, 1, .3]]:
            record_plane(state, 'surface', point=point)
        result = record_plane(state, 'surface', fit=True)
        self.assertAlmostEqual(result['offset_m'], -.3)


if __name__ == '__main__':
    unittest.main()
