"""Actual live source metadata cannot relabel a different replay/run."""
from pathlib import Path
import ast
import hashlib
import tempfile
import unittest

SOURCE=Path(__file__).resolve().parents[1]/'carla/audit_sumo_sync_live_source_replay.py'
tree=ast.parse(SOURCE.read_text(encoding='utf-8'))
function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='outer_source_binding')
space={'Path':Path,'sha':lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()}
exec(compile(ast.Module(body=[function],type_ignores=[]),str(SOURCE),'exec'),space)
bind=space['outer_source_binding']


class LiveReplayBindingTests(unittest.TestCase):
    def test_generic_v2_requires_exact_command_manifest_live_source_and_no_live_render_claim(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); manifest=root/'manifest.json';manifest.write_bytes(b'fixture')
            process={'command':['python','replay.py','--run',str(root)],'returncode':0,
                'this_render_is_live':False,'mode':'post_capture_synchronized_replay',
                'manifest_sha256':space['sha'](manifest),'CARLA_connected_or_ticked':False,
                'source_gates_automated_passed':True,'source_gate_phase':'motion_action_preparation'}
            source={'mode':'live_official_bidirectional_actor_synchronization','postcapture_replay':False}
            self.assertTrue(bind(process,manifest,root,source)[0])
            for changed in [dict(process,command=['python','--run',str(root/'other')]),
                            dict(process,manifest_sha256='0'*64),dict(process,this_render_is_live=True),
                            dict(process,returncode=1),dict(process,CARLA_connected_or_ticked=True)]:
                self.assertFalse(bind(changed,manifest,root,source)[0])
            self.assertFalse(bind(process,manifest,root,{'mode':'replay','postcapture_replay':True})[0])


if __name__=='__main__':unittest.main()
