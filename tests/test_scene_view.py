"""Viewer geometry stays usable on read-only data and follows referenced files."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'demo'), str(ROOT / 'skills/spatial-memory/scripts')]
from scene_view import SceneView


class SceneViewTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.path = self.root / 'scene.json'

    def view(self, state):
        self.path.write_text(json.dumps(state))
        return SceneView(self.path)

    def test_unwritable_cache_still_displays_geometry(self):
        view = self.view({'objects': {'box': {'shape': 'box', 'size': [1, 2, 3], 'position': [0, 0, 0]}}})
        with patch('assets.Path.mkdir', side_effect=PermissionError('Read-only directory')):
            result = view.state()
        self.assertTrue(result['objects'][0]['drawable'])
        self.assertEqual(result['objects'][0]['dimensions'], [1, 2, 3])
        self.assertFalse((self.root / '.geometry').exists())

    def test_external_asset_and_nested_mesh_refresh_without_scene_edit(self):
        mesh_path = self.root / 'external.stl'
        trimesh.creation.box([1, 1, 1]).export(mesh_path)
        asset = self.root / 'shape.json'
        asset.write_text(json.dumps({'parts': [{'shape': 'mesh', 'file': str(mesh_path)}]}))
        view = self.view({'assets': {'shape': {'file': 'shape.json'}},
                          'objects': {'item': {'asset': 'shape', 'position': [0, 0, 0]}}})
        first = view.state()
        trimesh.creation.box([2, 1, 1]).export(mesh_path)
        second = view.state(first['revision'])
        self.assertEqual(second['objects'][0]['dimensions'], [2, 1, 1])
        asset.write_text(json.dumps({'shape': 'box', 'size': [3, 1, 1]}))
        third = view.state(second['revision'])
        self.assertEqual(third['objects'][0]['dimensions'], [3, 1, 1])
        (self.root / 'assets').mkdir()
        (self.root / 'assets/unused.json').write_text('{}')
        self.assertTrue(view.state(third['revision'])['unchanged'])

    def test_robot_mesh_and_urdf_refresh_cached_visuals(self):
        mesh = self.root / 'robot.stl'
        trimesh.creation.box([1, 1, 1]).export(mesh)
        urdf = self.root / 'robot.urdf'
        template = ('<robot name="test"><link name="base"><visual><origin xyz="{x} 0 0"/>'
                    '<geometry><mesh filename="package://test/robot.stl"/></geometry>'
                    '</visual></link></robot>')
        urdf.write_text(template.format(x=0))
        view = self.view({'robots': {'arm': {'urdf': 'robot.urdf'}}})
        first = view.state()
        trimesh.creation.box([2, 1, 1]).export(mesh)
        second = view.state(first['revision'])
        points = np.asarray(second['robot_visuals'][0]['vertices'])
        np.testing.assert_allclose(np.ptp(points, axis=0), [2, 1, 1])
        urdf.write_text(template.format(x=3))
        third = view.state(second['revision'])
        points = np.asarray(third['robot_visuals'][0]['vertices'])
        np.testing.assert_allclose((points.min(axis=0) + points.max(axis=0)) / 2, [3, 0, 0])


if __name__ == '__main__':
    unittest.main()
