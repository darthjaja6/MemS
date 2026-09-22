"""Geometry and JSON command checks without a robot, simulator or model call."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np

SCRIPTS=Path(__file__).resolve().parents[1]/'skills/spatial-memory/scripts'
sys.path.insert(0,str(SCRIPTS))
import geometry
import scene


class SceneTests(unittest.TestCase):
    def test_distorted_pixels_and_scene_relative_observations(self):
        K=[[600,0,320],[0,600,240],[0,0,1]]
        T=np.eye(4)
        camera=dict(K=K,T_base_camera=T.tolist(),distortion=[.2,0,0,0,0])
        # Analytic radial distortion of normalized camera point (.5, .2).
        point=[.5,.2,1]
        factor=1+.2*(.5**2+.2**2)
        pixel=[320+600*.5*factor,240+600*.2*factor]
        np.testing.assert_allclose(geometry.project(point,**camera),pixel)
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'observation.json').write_text(json.dumps({'cameras':{'a':camera}}))
            state=dict(cameras={},surfaces={'table':dict(normal=[0,0,1],offset_m=-1)})
            result=scene.locate(state,[dict(camera='a',pixel=pixel,observation='observation.json')],
                                plane='table',root=root)
            np.testing.assert_allclose(result['point_m'],point,atol=1e-6)

    def test_parallel_asset_registration(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'scene.json').write_text('{}')
            (root/'shape.json').write_text(json.dumps(dict(shape='box',size=[.1,.2,.3])))
            processes=[subprocess.Popen([sys.executable,str(SCRIPTS/'assets.py'),'create',
                '--id',name,'--input','shape.json'],cwd=root,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
                for name in ['first','second','third']]
            for process in processes:
                stdout,stderr=process.communicate(timeout=30)
                self.assertEqual(process.returncode,0,stderr.decode())
            state=json.loads((root/'scene.json').read_text())
            self.assertEqual(set(state['assets']),{'first','second','third'})

    def test_reconstruction_and_object_frame_targets(self):
        K=[[200,0,160],[0,200,120],[0,0,1]]
        T=np.eye(4);other=T.copy();other[0,3]=.2
        state=dict(cameras={'a':dict(K=K,T_base_camera=T.tolist()),
                            'b':dict(K=K,T_base_camera=other.tolist())},
                   surfaces={'plane':dict(normal=[0,0,1],offset_m=-2)})
        point=[.1,.2,2]
        views=[dict(camera=c,pixel=geometry.project(point,**data)) for c,data in state['cameras'].items()]
        np.testing.assert_allclose(scene.locate(state,views)['point_m'],point,atol=1e-12)
        np.testing.assert_allclose(scene.locate(state,views[:1],plane='plane')['point_m'],point)
        state['objects']={'box':dict(position=[1,2,3],quaternion=[.7071067811865476,0,0,.7071067811865476])}
        np.testing.assert_allclose(scene.target_point(state,dict(object='box',offset=[1,0,0])),[1,3,3])

    def test_json_commands_and_compound_export(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'scene.json').write_text(json.dumps(dict(robots={},cameras={},objects={})))
            request={'objects':{'container':dict(position=[1,2,3],parts=[
                dict(shape='box',position=[0,0,0],size=[.2,.2,.01]),
                dict(shape='box',position=[.1,0,.05],size=[.01,.2,.1])])}}
            (root/'update.json').write_text(json.dumps(request))
            subprocess.run([sys.executable,str(SCRIPTS/'scene.py'),'update','--input','update.json'],
                           cwd=root,check=True,capture_output=True)
            subprocess.run([sys.executable,str(SCRIPTS/'scene.py'),'export','--output','scene.xml'],
                           cwd=root,check=True,capture_output=True)
            xml=ET.parse(root/'scene.xml')
            self.assertEqual(len(xml.findall('.//body/geom')),2)
            self.assertEqual(xml.find('.//body').get('pos'),'1 2 3')
            (root/'contact.json').write_text(json.dumps(dict(target=[0,0,1],approach=[0,0,-1],closing=[1,0,0],point_in_tip=[.145,0,0])))
            result=subprocess.run([sys.executable,str(SCRIPTS/'geometry.py'),'tool-pose','--input','contact.json'],
                                  cwd=root,check=True,capture_output=True,text=True)
            pose=json.loads(result.stdout)['pose']
            np.testing.assert_allclose(geometry.transform_points(geometry.pose_matrix(pose),[.145,0,0]),[0,0,1])
            with self.assertRaises(ValueError):
                geometry.tool_pose([0,0,1],[0,0,1],[0,0,1])

    def test_update_while_geometry_is_still_being_built(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'scene.json'
            path.write_text(json.dumps({'objects':{'item':{'position':[0,0,0]}}}))
            subprocess.run([sys.executable,str(SCRIPTS/'scene.py'),'update','--scene',str(path)],
                           input='{"title":"Work in progress"}',text=True,check=True,capture_output=True)
            self.assertEqual(json.loads(path.read_text())['title'],'Work in progress')


if __name__=='__main__':unittest.main()
