"""Extend native Town03 entry for a moving pre-roll and physical ring opponents."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from runtime_config import RuntimeConfig
_RUNTIME = RuntimeConfig.resolve(require_carla=True)
import json,hashlib,math,sys
import numpy as np
from PIL import Image,ImageDraw
import carla
from build_v2_wide_left_offline import advance,ref,spawn,xyz,stats,road_order,draw_panel

BASE=Path(__file__).resolve().parents[1]
SOURCE=_RUNTIME.carla_root/'CarlaUE4/Content/Carla/Maps/OpenDrive/Town03.xodr'
def main():
    m=carla.Map('Town03',SOURCE.read_text(encoding='utf-8'))
    old=BASE/'plans/base/town03-roundabout.json'
    p=json.loads(old.read_text(encoding='utf-8'))
    old_stop=m.get_waypoint_xodr(1577,-2,3.2)
    predecessor=old_stop.previous(4.0)
    if len(predecessor)!=1:raise RuntimeError('Ring stop has an ambiguous native upstream predecessor')
    stop=predecessor[0]
    entry=list(reversed(advance(stop,35,True)))
    p['scenario']='native_roundabout_moving_approach_yield_physical_circulators'
    p.update(hot_start=True,preroll_min_seconds=3.0,preroll_max_seconds=12.0,capture_start_speed_mps=4.0,version='v2')
    p['finish_capture_on_plan_complete']=True
    p['ego_spawn']=spawn(entry[0]);p['source_root_env']='CARLA_ROOT';p['source_inventory']=SOURCE.relative_to(_RUNTIME.carla_root).as_posix()
    p['source_xodr_sha256']=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    p['stages'][0].update({'name':'moving_native_35m_approach_to_ring_yield','speed_kmh':26,'xodr_path':[ref(w) for w in entry],'complete_radius_m':.9})
    # Keep the new stop-to-original-entry bridge on actual native waypoints.
    # The original stop point must remain in stage1, with no skipped 4m jump.
    bridge=advance(stop,4.0)
    if math.dist(xyz(bridge[-1]),xyz(old_stop))>.15:
        raise RuntimeError('Native stop bridge does not reconnect to the original ring entry')
    p['stages'][1]['xodr_path']=[ref(w) for w in bridge[:-1]]+p['stages'][1]['xodr_path']
    roles=[t['name'] for t in p['traffic']]
    gate=p['stages'][1]['yield_gate']
    gate.update({'actor_roles':roles,'require_observed_pass_roles':roles,'prediction_s':4})
    p['stages'][1].update({'speed_kmh':14,'lookahead_base_m':3,'lookahead_speed_s':.5,'lookahead_max_m':8})
    p['stages'][2].update({'lookahead_base_m':3,'lookahead_speed_s':.5,'lookahead_max_m':8})
    final=m.get_waypoint_xodr(*p['stages'][2]['xodr_path'][-1])
    continuation=advance(final,20)
    p['stages'].append({'name':'continue_native_exit_with_no_stopped_tail','kind':'follow_path','speed_kmh':28,'xodr_path':[ref(w) for w in continuation],'complete_radius_m':1.5})
    npc_evidence=[];native_npcs=[]
    for i,t in enumerate(p['traffic']):
        t['controller']='native_rule';t['auto_lane_change']=False
        t['desired_speed_kmh']=22 if i==0 else 24
        if i==1:t['blueprint']='vehicle.audi.tt'
        t['stages']=[{'name':'physical_native_ring_following','kind':'follow_path','speed_kmh':t['desired_speed_kmh'],'path':t['path'],'lookahead_base_m':3,'lookahead_speed_s':.5,'lookahead_max_m':8,'complete_radius_m':1.5}]
        # Each reference is the original native ring waypoint sample. Projection
        # below verifies provenance/road proximity without changing any point.
        native=[m.get_waypoint(carla.Location(x=q[0],y=q[1],z=q[2]),project_to_road=True,lane_type=carla.LaneType.Driving) for q in t['path']]
        residual=[math.dist(q,xyz(w)) for q,w in zip(t['path'],native)]
        if max(residual)>.12:raise RuntimeError('Ring reference has a non-native driving-lane point')
        native_npcs.append(native)
        distance=0;entries=[];inside_before=False
        for a,b in zip(t['path'],t['path'][1:]):
            distance+=math.dist(a,b)
            inside=(-13<=b[0]<=-5.5 and -24<=b[1]<=-13)
            if inside and not inside_before:entries.append(distance)
            inside_before=inside
        npc_evidence.append({'role':t['name'],'spawn_xodr':t['spawn_xodr'],'original_native_reference_points':len(t['path']),'reference_total_length_m':distance,'max_reference_to_native_driving_centerline_m':max(residual),'reference_arc_distances_to_gate_m':entries,'actor_physics_and_clearance_not_yet_verified':True})
    p['expected_evidence'].update({'ego_approach_road_order':road_order(entry),'must_observe_both_circulating_traffic_first':True,'must_observe_minimum_body_clearance_m':1.5,'must_observe_stopped_yield':True,'runtime_actions_not_yet_verified':True})
    p['capture_suggestion'].update({'seconds':60,'warmup_seconds':3,'warmup_controller_steps_required':True,'recorded_first_frame_min_ego_speed_mps':4,'recorded_first_frame_required_stage_index':0})
    p['required_runtime_features']=['Per-NPC traffic.controller native_rule/traffic.stages applied every warmup/recorded physical tick. TM set_path alone is not accepted.','Gate observes both inner and outer circulators from recorded approach; do not release on an unseen stopped actor.','Actual native body clearance >=1.5m, collision0 and reference-tracking/route completion independently audited.','Do not move/spawn or teleport traffic during capture to repair interaction.']
    reserve=[{'x':xyz(entry[0])[0],'y':xyz(entry[0])[1],'radius_m':10},* [{'x':t['path'][0][0],'y':t['path'][0][1],'radius_m':10} for t in p['traffic']]]
    p['dynamic_traffic']={'count':20,'seed':121203,'focus_points':[[0,0,0]],'focus_radius_m':100,'speed_range_kmh':[24,42],'exclude_circles':reserve,'exclude_xodr':[{'road_id':28,'lane_ids':[2],'s_start':0,'s_end':21},{'road_id':25,'lane_ids':[-2],'s_start':0,'s_end':2.33},{'road_id':1577,'lane_ids':[-2,-3],'s_start':0,'s_end':33.24}]}
    p['offline_geometry_check']={'only_native_driving_waypoints':True,'extended_approach':stats(entry),'original_approach_first_point':[25,-2,.05],'new_approach_spawn':spawn(entry[0]),'stop_upstream_native_distance_m':4.0,'original_stop_reference':spawn(old_stop),'new_stop_reference':spawn(stop),'stop_to_ring_native_bridge':stats(bridge),'server_connected':False,'world_ticked':False,'runtime_accepted':False}
    path=BASE/'plans/v2-roundabout-moving-approach.json';path.write_text(json.dumps(p,indent=2),encoding='utf-8')
    evidence={'map':'Town03','source_root_env':'CARLA_ROOT','source_xodr':SOURCE.relative_to(_RUNTIME.carla_root).as_posix(),'source_sha256':p['source_xodr_sha256'],'plan':path.relative_to(BASE).as_posix(),'plan_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'native_map_offline_only':True,'server_connected':False,'world_ticked':False,'entry_geometry':stats(entry),'entry_spawn':spawn(entry[0]),'entry_stop_reference':spawn(stop),'original_stop_reference':spawn(old_stop),'stop_upstream_native_distance_m':4.0,'stop_to_ring_native_bridge':stats(bridge),'npc_native_reference_evidence':npc_evidence,'gate_roles':gate['actor_roles'],'minimum_runtime_body_clearance_required_m':1.5,'planned_and_not_runtime_accepted':True}
    out=BASE/'verification/v2-roundabout-entry-geometry.json';out.write_text(json.dumps(evidence,indent=2),encoding='utf-8')
    ego=entry+[m.get_waypoint_xodr(*q) for stage in p['stages'][1:3] for q in stage['xodr_path']]
    im=Image.new('RGB',(900,1200),(20,26,33));draw_panel(im,m,[0,0],0,ego,native_npcs,gate['polygon_xy'])
    draw=ImageDraw.Draw(im);draw.text((30,30),'Town03 native ring: physical rule-driven actors, 35m moving entry',fill='white')
    draw.text((30,55),'GREEN ego | ORANGE native ring references | RED observed-pass yield gate',fill='white')
    draw.text((30,80),'Offline geometry only. Runtime stop / clearance / completion still required.',fill='white')
    im.save(BASE/'verification/v2-roundabout-entry-geometry.png')
    print(json.dumps({'plan':path.relative_to(BASE).as_posix(),'evidence':out.relative_to(BASE).as_posix(),'spawn':spawn(entry[0]),'entry':stats(entry),'npc_evidence':npc_evidence}))

if __name__=='__main__':main()
