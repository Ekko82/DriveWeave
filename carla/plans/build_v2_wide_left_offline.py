"""Build a native Town05 wide-left-turn plan; no CARLA Client or world tick."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from runtime_config import RuntimeConfig
from signal_policy import town05_wide_left_signal_config
_RUNTIME = RuntimeConfig.resolve(require_carla=True)
import sys,json,math,hashlib
import numpy as np
from PIL import Image,ImageDraw
import carla

BASE=Path(__file__).resolve().parents[1]
PACKAGE=_RUNTIME.carla_root/'CarlaUE4/Content/Carla/Maps/OpenDrive'
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def angle(a,b): return (b-a+180)%360-180
def xyz(w):
    p=w.transform.location
    return [p.x,p.y,p.z]
def ref(w): return [w.road_id,w.lane_id,w.s]
def spawn(w): return dict(zip(['road_id','lane_id','s_m'],ref(w)))
def road_order(points):
    out=[]
    for w in points:
        if not out or out[-1]!=w.road_id:out.append(w.road_id)
    return out
def advance(w,meters,backwards=False):
    out=[w]
    for _ in range(round(meters/.5)):
        candidates=w.previous(.5) if backwards else w.next(.5)
        if not candidates:raise RuntimeError('Native route terminates unexpectedly')
        w=min(candidates,key=lambda q:abs(angle(out[-1].transform.rotation.yaw,q.transform.rotation.yaw)))
        out.append(w)
    return out
def segment(m,road,lane,start,end):
    n=math.ceil(abs(end-start)/.5)
    out=[m.get_waypoint_xodr(road,lane,float(s)) for s in np.linspace(start,end,n+1)]
    if any(w is None or w.lane_type!=carla.LaneType.Driving for w in out): raise RuntimeError('Invalid native driving segment')
    return out
def join(*parts):
    out=[]
    for part in parts:
        for w in part:
            if out and math.dist(xyz(w),xyz(out[-1]))<1e-5:continue
            out.append(w)
    return out
def stats(points):
    p=np.array([xyz(w) for w in points]);d=np.linalg.norm(np.diff(p,axis=0),axis=1)
    headings=np.unwrap(np.radians([w.transform.rotation.yaw for w in points]))
    curvature=np.abs(np.diff(headings))/np.maximum(d,.001)
    return {'points':len(points),'road_order':road_order(points),'length_m':float(d.sum()),'maximum_waypoint_spacing_m':float(d.max()),'maximum_heading_step_deg':float(np.max(np.abs(np.diff(headings)))*180/math.pi),'minimum_local_turn_radius_m':float(1/max(curvature.max(),1e-10)),'heading_change_deg':float((headings[-1]-headings[0])*180/math.pi),'only_native_driving_waypoints':all(w.lane_type==carla.LaneType.Driving for w in points)}
def conflict(a,b):
    a=np.array([xyz(w)[:2] for w in a]);b=np.array([xyz(w)[:2] for w in b]);d=np.linalg.norm(a[:,None]-b[None,:],axis=2)
    i,j=np.unravel_index(d.argmin(),d.shape)
    return {'ego_xy_m':a[i].tolist(),'npc_xy_m':b[j].tolist(),'centerline_sample_gap_m':float(d[i,j]),'midpoint_xy_m':((a[i]+b[j])/2).tolist()}
def lane_width(m,w):
    lanes=[]
    for n in range(-8,9):
        q=m.get_waypoint_xodr(w.road_id,n,w.s) if n else None
        if q and q.lane_type==carla.LaneType.Driving:lanes.append({'lane_id':n,'width_m':q.lane_width})
    return {'road_id':w.road_id,'s_m':w.s,'driving_lanes':lanes,'total_driving_width_m':sum(q['width_m'] for q in lanes)}
def draw_panel(im,m,center,offset,ego,npcs,gate=None):
    panel=Image.new('RGB',(900,1200),(20,26,33));draw=ImageDraw.Draw(panel);scale=7.2
    def xy(p):return (450+(p[0]-center[0])*scale,650+(p[1]-center[1])*scale)
    for w in m.generate_waypoints(.5):
        p=xyz(w)
        if abs(p[0]-center[0])>55 or abs(p[1]-center[1])>65:continue
        candidates=w.next(.5)
        if not candidates:continue
        q=candidates[0];p2=xyz(q)
        if math.dist(p,p2)>1:continue
        polys=[]
        for r,sign in [(w,1),(q,1),(q,-1),(w,-1)]:
            pt=xyz(r);a=math.radians(r.transform.rotation.yaw)
            polys.append(xy([pt[0]-math.sin(a)*r.lane_width*.5*sign,pt[1]+math.cos(a)*r.lane_width*.5*sign]))
        draw.polygon(polys,fill=(66,72,79))
    for points,color in [(ego,(38,229,138)),*[(p,(244,122+30*i,48)) for i,p in enumerate(npcs)]]:
        draw.line([xy(xyz(w)) for w in points],fill=color,width=5)
        for w in [points[0],points[-1]]:
            x,y=xy(xyz(w));draw.ellipse([x-5,y-5,x+5,y+5],fill=color)
    if gate:
        vertices=[xy(p) for p in gate];draw.line(vertices+[vertices[0]],fill=(240,74,94),width=2)
    im.paste(panel,(offset,0))

def main():
    source=PACKAGE/'Town05.xodr';m=carla.Map('Town05',source.read_text(encoding='utf-8'))
    approach=segment(m,6,-1,15.5,50.5)
    before_turn=segment(m,6,-1,50.5,54.36-.001)
    turn=segment(m,1182,-1,.001,24.002449571130818-.001)
    exit_path=segment(m,47,-1,.001,40)
    # End naturally after 40 native metres of the exit. The next native light
    # is 63.465m downstream of its entrance; a further 70m stage crosses it.
    ego_full=join(approach,before_turn,turn,exit_path)
    npcs=[];npc_plans=[];crossings=[];opposing_core_geometry=[]
    # Positive native lanes run towards decreasing road-s. Start directly on
    # the selected green approach, without passing an unrelated upstream lamp.
    # Low intersection speeds keep actual opposing passage observable when the
    # rolling ego reaches its stop point; runtime body/stop audits remain strict.
    for i,(lane,speed,bp) in enumerate([(1,14,'vehicle.audi.a2'),(2,12,'vehicle.nissan.patrol')]):
        incoming=segment(m,7,lane,.1,.001)
        connector=segment(m,1140,lane,30.18-.001,.001)
        outgoing=segment(m,6,lane,54.36-.001,.001)
        continuation=advance(outgoing[-1],80)
        path=join(incoming,connector,outgoing,continuation)
        opposing_core_geometry.append(stats(join(incoming,connector,outgoing)))
        npcs.append(path);crossings.append(conflict(turn,connector))
        npc_plans.append({'name':f'oncoming_{i}','blueprint':bp,'spawn_xodr':spawn(path[0]),'controller':'native_rule','desired_speed_kmh':speed,'auto_lane_change':False,'stages':[{'name':'actual_oncoming_straight_through_wide_junction','kind':'follow_path','speed_kmh':speed,'xodr_path':[ref(w) for w in path],'complete_radius_m':1.5,'lookahead_base_m':4,'lookahead_speed_s':.5,'lookahead_max_m':10}],'path':[xyz(w) for w in path],'note':'NativeRuleController owns physical VehicleControl. TM set_path is not accepted as a guarantee of this straight route.'})
    cp=np.array([p['midpoint_xy_m'] for p in crossings]);lo=cp.min(axis=0)-[7,3.5];hi=cp.max(axis=0)+[7,3.5]
    gate=[[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]]
    roles=[p['name'] for p in npc_plans]
    reserve=[{'x':xyz(w)[0],'y':xyz(w)[1],'radius_m':9} for w in [ego_full[0],*[p[0] for p in npcs]]]
    plan={'scenario':'wide_native_unprotected_left_turn_true_oncoming_straight','map':'Town05','controller':'native_rule','source_root_env':'CARLA_ROOT','source_inventory':source.relative_to(_RUNTIME.carla_root).as_posix(),'source_xodr_sha256':sha(source),'actions_verified':False,'ego_spawn':spawn(approach[0]),'desired_speed_kmh':26,'stages':[{'name':'moving_approach_to_wide_junction_yield','kind':'follow_path','speed_kmh':26,'xodr_path':[ref(w) for w in approach],'stop_at_end':True,'complete_radius_m':.9,'lookahead_base_m':4,'lookahead_speed_s':.5,'lookahead_max_m':10},{'name':'yield_then_smooth_native_left_turn_and_exit','kind':'follow_path','speed_kmh':20,'xodr_path':[ref(w) for w in join(before_turn,turn,exit_path)],'yield_gate':{'polygon_xy':gate,'prediction_s':5,'actor_roles':roles,'require_observed_pass_roles':roles},'lookahead_base_m':3,'lookahead_speed_s':.5,'lookahead_max_m':8,'complete_radius_m':1.5}], 'traffic':npc_plans,'dynamic_traffic':{'seed':121126,'count':24,'focus_points':[[-50.69,-92.60,0]],'exclude_circles':reserve,'exclude_xodr':[{'road_id':6,'lane_ids':[-1],'s_start':12,'s_end':54.36},{'road_id':7,'lane_ids':[1,2],'s_start':0,'s_end':45.47},{'road_id':1182,'lane_ids':[-1],'s_start':0,'s_end':24.003},{'road_id':1140,'lane_ids':[1,2],'s_start':0,'s_end':30.18},{'road_id':47,'lane_ids':[-1],'s_start':0,'s_end':42}]},'expected_evidence':{'junction_id':1126,'ego_core_road_order':[6,1182,47],'opposing_core_road_order':[7,1140,6],'opposing_expected_heading_change_deg':0,'opposing_lane_ids':[1,2],'must_observe_true_oncoming_straight':True,'must_observe_stopped_yield':True,'must_observe_both_conflict_actors_before_ego_turn':True,'must_observe_exit_motion_distance_m':15,'must_observe_minimum_body_clearance_m':1.5,'collision_count':0,'runtime_actions_not_yet_verified':True},'capture_suggestion':{'seconds':30,'fps':20,'warmup_seconds':3,'warmup_controller_steps_required':True,'recorded_first_frame_min_ego_speed_mps':4,'recorded_first_frame_required_stage_index':0,'note':'Pre-roll ego and dedicated NPC physics every official tick. Actual arrival timings, signal state, stop, clearance, route completion and smoothness must be checked from the capture.'},'required_runtime_features':['traffic.controller native_rule and traffic.stages physically controlled each warmup/recorded tick','Permissive opposite-direction green if native junction is signalized; cross traffic retains red. Do not ignore red lights or claim a protected left arrow.','All dedicated and dynamic traffic must pass collision/clearance and actual lane-connector audits.'],'offline_geometry_only':True}
    plan['finish_capture_on_plan_complete']=True
    plan.update(hot_start=True,preroll_min_seconds=3.0,preroll_max_seconds=12.0,capture_start_speed_mps=4.0,version='v2')
    plan['signal_policy']=town05_wide_left_signal_config()
    plan['native_junction_signalized']=True
    plan['signal_description']='Native three-light signalized junction. Opposite-axis permissive green, transverse red; ego must physically yield to actual straight opposing traffic.'
    plan['dynamic_traffic']['exclude_xodr'].extend([
        {'road_id':6,'lane_ids':[1],'s_start':0,'s_end':54.36},
        {'road_id':47,'lane_ids':[1],'s_start':0,'s_end':55},
    ])
    plan['dynamic_traffic']['reservation_note']='Inner opposing lanes 6/+1 and 47/+1 are reserved from dynamic spawns. Outer +2 remains available. Dedicated native-rule straight NPCs are unchanged by dynamic spawn reservations. Reservations do not guarantee later TM routes; actual full-run body-clearance audit remains mandatory.'
    plan['expected_evidence'].update(
        must_observe_stopped_yield_duration_s=.5,
        must_observe_final_plan_complete_while_moving=True,
        minimum_final_ego_speed_mps=.5,
        final_native_waypoint=spawn(exit_path[-1]),
    )
    plan['capture_suggestion']['seconds']=45
    plan['capture_suggestion']['npc_start_note']='NPCs start on road7 positive native lanes at s0.1 and physically advance on every preroll tick. Speeds14/12kmh are intersection approach targets. No time-based release or minimum artificial waiting is added. Actual stopped yield must last >=0.5s while opposing bodies traverse and clear the gate.'
    next_exit_signals=[]
    for landmark in exit_path[0].get_landmarks(100,False):
        if landmark.type!='1000001':continue
        w=landmark.waypoint
        next_exit_signals.append({'landmark_id':str(landmark.id),
            'distance_from_exit_start_m':landmark.distance,
            'native_affected_waypoint':spawn(w),'xyz_m':xyz(w),
            'lane_validities':landmark.get_lane_validities()})
    if not next_exit_signals:raise RuntimeError('Missing next native exit signal evidence')
    next_signal_distance=min(q['distance_from_exit_start_m'] for q in next_exit_signals)
    actual_exit_distance=stats(exit_path)['length_m']
    if actual_exit_distance<20 or next_signal_distance-actual_exit_distance<20:
        raise RuntimeError('Native endpoint must include 20m moving exit and precede next lamp by 20m')
    old_source=PACKAGE/'Town04.xodr';old=carla.Map('Town04',old_source.read_text(encoding='utf-8'))
    oldturn=segment(old,61,-1,.001,15.621477707692403-.001)
    oldego=join(segment(old,29,-1,43.9,57.899),oldturn,segment(old,27,-1,.001,23))
    evidence={'source_root_env':'CARLA_ROOT','source_xodr':source.relative_to(_RUNTIME.carla_root).as_posix(),'source_xodr_sha256':sha(source),'native_carla_map_only':True,'server_connected':False,'world_ticked':False,'selected_map':'Town05','selected_junction':1126,'old_actual_map':'Town04','old_actual_junction':53,'old_junction_bbox_size_xy_m':[19.33709716796875,18.804672241210938],'new_junction_bbox_size_xy_m':[30.208988189697266,36.01702117919922],'old_incoming_width':lane_width(old,old.get_waypoint_xodr(29,-1,50)),'new_incoming_width':lane_width(m,approach[-1]),'new_exit_width':lane_width(m,exit_path[20]),'old_turn_geometry':stats(oldturn),'new_turn_geometry':stats(turn),'ego_full_geometry':stats(ego_full),'opposing_geometry':[stats(p) for p in npcs],'native_conflict_crossings':crossings,'yield_gate_polygon_xy':gate,'ego_approach_length_m':stats(approach)['length_m'],'npc_initial_native_approach_road_s_m':[.1,.1],'production_scene_geometry_edited':False,'runtime_acceptance':False}
    plan_path=BASE/'plans/v2-wide-left-turn.json';plan_path.write_text(json.dumps(plan,indent=2),encoding='utf-8')
    evidence['plan_sha256']=sha(plan_path)
    evidence.update(
        generator_sha256=sha(Path(__file__).resolve()),
        ego_stage_count=len(plan['stages']),
        removed_extra_exit_tail_m=70,
        natural_finish_capture_on_plan_complete=plan['finish_capture_on_plan_complete'],
        final_native_waypoint=spawn(exit_path[-1]),
        native_exit_geometry=stats(exit_path),
        next_exit_signal_landmarks=next_exit_signals,
        native_endpoint_to_next_lamp_arc_distance_m=next_signal_distance-actual_exit_distance,
        native_landmarks_are_not_runtime_trigger_volume_evidence=True,
        opposing_initial_waypoints=[spawn(p[0]) for p in npcs],
        opposing_true_straight_core_geometry=opposing_core_geometry,
        opposing_intersection_target_speed_kmh=[p['desired_speed_kmh'] for p in npc_plans],
        dynamic_spawn_reserved_inner_opposing_lanes=[{'road_id':6,'lane_id':1},{'road_id':47,'lane_id':1}],
        dynamic_outer_opposing_lanes_remain_available=True,
        runtime_body_clearance_and_stopped_yield_still_required=True,
    )
    evidence_path=BASE/'verification/v2-wide-left-turn-geometry.json';evidence_path.write_text(json.dumps(evidence,indent=2),encoding='utf-8')
    im=Image.new('RGB',(1800,1200),(20,26,33));draw=ImageDraw.Draw(im)
    draw_panel(im,old,[256.3778,-247.6789],0,oldego,[])
    draw_panel(im,m,[-50.6907,-92.6045],900,ego_full,npcs,gate)
    draw.line([(900,0),(900,1200)],fill=(115,125,140),width=2)
    draw.text((35,30),'OLD: Town04 junction53 | 7m two-way driving width | native turn',fill='white')
    draw.text((935,30),'NEW: Town05 junction1126 | 14m / 4 driving lanes | true opposite straight',fill='white')
    draw.text((935,60),'GREEN: ego native left | ORANGE: actual native straight reference | RED: yield gate',fill='white')
    draw.text((935,90),'Offline geometry evidence only. No server, no tick, no runtime acceptance.',fill='white')
    image_path=BASE/'verification/v2-wide-left-turn-geometry.png';im.save(image_path)
    print(json.dumps({'plan':plan_path.relative_to(BASE).as_posix(),'evidence':evidence_path.relative_to(BASE).as_posix(),'png':image_path.relative_to(BASE).as_posix(),'turn':evidence['new_turn_geometry'],'approach_m':evidence['ego_approach_length_m'],'opponent_roads':[p['road_order'] for p in evidence['opposing_geometry']]}))

if __name__=='__main__':main()
