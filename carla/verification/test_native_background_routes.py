"""Native map geometry + TrafficManager contract tests; never Client/tick."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import unittest

import carla

BASE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE));sys.path.insert(0,str(Path(__file__).resolve().parent))
sys.path.insert(0,str(BASE.parent))
from runtime_config import RuntimeConfig
from native_background_routes import CorridorGuard,build_isolated_traffic,resolve_route
from dynamic_traffic import TrafficConfig,setup_dynamic_traffic,Footprint,horizontal_body_clearance
from test_dynamic_traffic import World,Actor,ActorList,TM,MAP_FILE


class NativeBackgroundRouteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.maps={};cls.plans={};cls.configs={};cls.evidence={}
        for name,file,seed,count in [('Town03','v2-roundabout-moving-approach.json',121203,20),('Town05','v2-wide-left-turn.json',121126,24)]:
            mapfile=BASE/f'map-inventory/{name}/map.xodr'
            if not mapfile.exists():
                mapfile=RuntimeConfig.resolve(require_carla=True).carla_root/'CarlaUE4/Content/Carla/Maps/OpenDrive'/f'{name}.xodr'
            cls.maps[name]=carla.Map(name,mapfile.read_text(encoding='utf-8'))
            cls.plans[name]=json.loads((BASE/'plans'/file).read_text(encoding='utf-8'))
            cls.configs[name],cls.evidence[name]=build_isolated_traffic(cls.maps[name],cls.plans[name],count=count,seed=seed)

    def test_full_native_route_lengths_and_exact_successor_connectivity(self):
        for name,config in self.configs.items():
            for route in config['native_routes']:
                points,proof=resolve_route(self.maps[name],route)
                self.assertGreaterEqual(proof['native_length_m'],proof['required_length_m'])
                self.assertLessEqual(proof['maximum_reference_spacing_m'],4.1)
                self.assertEqual(proof['native_successors_checked'],len(points)-1)

    def test_whole_ego_reference_envelopes_are_avoided_and_density_kept(self):
        for name,config in self.configs.items():
            m=self.maps[name];p=self.plans[name]
            guard=CorridorGuard([m.get_waypoint_xodr(*q) for s in p['stages'] for q in s['xodr_path']])
            self.assertEqual(len(config['native_routes']),20 if name=='Town03' else 24)
            for route in config['native_routes']:
                for q in route['xodr_path']:
                    self.assertTrue(guard.safe(m.get_waypoint_xodr(*q)))
            # Regression: the ACTUAL failed ring adjacent lane and left inner
            # opposing lanes fail even though their road/lane IDs are distinct.
            if name=='Town03':
                self.assertFalse(guard.safe(m.get_waypoint_xodr(1786,-5,10.012958873063328)))
            else:
                self.assertFalse(guard.safe(m.get_waypoint_xodr(6,1,35)))
                self.assertFalse(guard.safe(m.get_waypoint_xodr(47,1,20)))

    def test_seed_reproducibility_and_reference_plan_not_modified(self):
        p=copy.deepcopy(self.plans['Town03']);before=json.dumps(p,sort_keys=True)
        cfg,proof=build_isolated_traffic(self.maps['Town03'],p,count=20,seed=121203)
        self.assertEqual(cfg,self.configs['Town03'])
        self.assertEqual(before,json.dumps(p,sort_keys=True))
        self.assertFalse(proof['physical_runtime_acceptance'])
        self.assertFalse(proof['server_connected_or_ticked'])

    def test_disconnected_lane_hop_backward_and_short_routes_rejected(self):
        original=self.configs['Town03']['native_routes'][0]
        wrong=copy.deepcopy(original);wrong['xodr_path'][1]=[1786,-5,10.0]
        with self.assertRaises(ValueError):resolve_route(self.maps['Town03'],wrong)
        short=copy.deepcopy(original);short['xodr_path']=short['xodr_path'][:3]
        with self.assertRaises(ValueError):resolve_route(self.maps['Town03'],short)
        backwards=copy.deepcopy(original)
        r,l,s=backwards['xodr_path'][0]
        w=self.maps['Town03'].get_waypoint_xodr(r,l,s).previous(1.)[0]
        backwards['xodr_path'][1]=[w.road_id,w.lane_id,w.s]
        with self.assertRaises(ValueError):resolve_route(self.maps['Town03'],backwards)

    def test_duplicate_or_random_fallback_configuration_rejected(self):
        c=copy.deepcopy(self.configs['Town03']);c['count']+=1
        with self.assertRaises(ValueError):TrafficConfig.from_mapping(c)
        c=copy.deepcopy(self.configs['Town03']);c['native_routes'][1]['route_id']=c['native_routes'][0]['route_id']
        with self.assertRaises(ValueError):TrafficConfig.from_mapping(c)
        c=copy.deepcopy(self.configs['Town03']);c['spawn_indices']=[0]
        with self.assertRaises(ValueError):TrafficConfig.from_mapping(c)

    def test_setup_native_path_and_no_lane_change_on_installed_tm_api(self):
        m=self.maps['Town03'];p=self.plans['Town03']
        oldmap=carla.Map('Town04',MAP_FILE.read_text(encoding='utf-8'))
        world=World(oldmap);world.road_map=m
        spawn=p['ego_spawn'];world.ego.pose=m.get_waypoint_xodr(spawn['road_id'],spawn['lane_id'],spawn['s_m']).transform
        for i,spec in enumerate(p['traffic'],2):
            q=spec['spawn_xodr'];pose=m.get_waypoint_xodr(q['road_id'],q['lane_id'],q['s_m']).transform
            world.actors.append(Actor(i,pose,spec['blueprint'],spec['name']))
        tm=TM();registered=[]
        session=setup_dynamic_traffic(world,tm,world.ego,self.configs['Town03'],carla_api=carla,register_actor=registered.append)
        self.assertEqual(len(session.actors),20)
        self.assertEqual(len(session.native_paths),20)
        paths=[args for name,args in tm.calls if name=='set_path']
        self.assertEqual(len(paths),20)
        self.assertTrue(all(isinstance(point,carla.Location) for args in paths for point in args[1]))
        session.update(world.get_snapshot())
        self.assertFalse(any(args[1] for name,args in tm.calls if name=='auto_lane_change'))
        forbidden={'ignore_vehicles_percentage','ignore_lights_percentage','ignore_signs_percentage'}
        self.assertTrue(all(args[1]==0 for name,args in tm.calls if name in forbidden))
        self.assertEqual(world.frame,10)


if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(NativeBackgroundRouteTests)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    report={'passed':result.wasSuccessful(),'tests_run':result.testsRun,
        'failures':[str(e) for _,e in result.failures],'errors':[str(e) for _,e in result.errors],
        'helper_sha256':hashlib.sha256((BASE/'native_background_routes.py').read_bytes()).hexdigest(),
        'dynamic_traffic_sha256':hashlib.sha256((BASE/'dynamic_traffic.py').read_bytes()).hexdigest(),
        'native_client_or_tick':False,'runtime_clearance_verified':False}
    (BASE/'verification/native-background-route-offline-tests.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    raise SystemExit(0 if result.wasSuccessful() else 1)
