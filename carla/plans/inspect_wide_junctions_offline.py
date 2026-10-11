"""Inspect native CARLA road geometry offline, without creating a Client."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from runtime_config import RuntimeConfig
_RUNTIME = RuntimeConfig.resolve(require_carla=True)
import json, math, hashlib
import carla

BASE=Path(__file__).resolve().parents[1]
PACKAGE=_RUNTIME.carla_root/'CarlaUE4/Content/Carla/Maps/OpenDrive'

def angle(a,b): return (b-a+180)%360-180
def xy(w): return [w.transform.location.x,w.transform.location.y]
def external(w,forward):
    pending=[w]
    for _ in range(25):
        new=[]
        for q in pending:
            for n in q.next(.5) if forward else q.previous(.5):
                if not n.is_junction: return n
                new.append(n)
        pending=new
    return None
def record(w):
    return {'road_id':w.road_id,'lane_id':w.lane_id,'s_m':w.s,'lane_width_m':w.lane_width,'xy_m':xy(w),'yaw_deg':w.transform.rotation.yaw,'junction':w.is_junction}
def lanes(m,w):
    result=[]
    for i in range(-12,13):
        if not i: continue
        q=m.get_waypoint_xodr(w.road_id,i,w.s)
        if q and q.lane_type==carla.LaneType.Driving:
            result.append({'lane_id':i,'width_m':q.lane_width,'same_direction':i*w.lane_id>0})
    return result
def inspect(town):
    source=PACKAGE/(town+'.xodr')
    m=carla.Map(town,source.read_text(encoding='utf-8'))
    joints={}
    for w in m.generate_waypoints(5):
        if w.is_junction: joints[w.get_junction().id]=w.get_junction()
    data=[]
    for jid,j in joints.items():
        box=j.bounding_box
        paths=[]
        for a,b in j.get_waypoints(carla.LaneType.Driving):
            ia,ob=external(a,False),external(b,True)
            if ia is None or ob is None: continue
            ls=lanes(m,ia)
            paths.append({'connector_road_lane':[a.road_id,a.lane_id],'entry':record(a),'exit':record(b),'incoming':record(ia),'outgoing':record(ob),'heading_change_deg':angle(ia.transform.rotation.yaw,ob.transform.rotation.yaw),'incoming_driving_lanes':ls,'incoming_total_driving_width_m':sum(q['width_m'] for q in ls),'incoming_same_direction_lanes':sum(q['same_direction'] for q in ls)})
        candidates=[]
        for ego in paths:
            if not -135<ego['heading_change_deg']<-45: continue
            for npc in paths:
                oppose=abs(angle(ego['incoming']['yaw_deg'],npc['incoming']['yaw_deg']))
                if oppose>160 and abs(npc['heading_change_deg'])<15:
                    candidates.append({'ego_connector':ego['connector_road_lane'],'oncoming_connector':npc['connector_road_lane'],'ego_width_m':ego['incoming_total_driving_width_m'],'oncoming_width_m':npc['incoming_total_driving_width_m'],'ego_same_direction_lanes':ego['incoming_same_direction_lanes'],'oncoming_same_direction_lanes':npc['incoming_same_direction_lanes']})
        data.append({'junction_id':jid,'center_xy_m':[box.location.x,box.location.y],'bbox_size_xy_m':[box.extent.x*2,box.extent.y*2],'paths':paths,'left_against_true_straight_candidates':candidates})
    result={'map':town,'source_root_env':'CARLA_ROOT','source_xodr':source.relative_to(_RUNTIME.carla_root).as_posix(),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'offline_carla_map':True,'server_connected':False,'world_ticked':False,'junctions':data}
    out=BASE/'verification'/f'{town}-wide-junctions-offline.json'
    out.write_text(json.dumps(result,indent=2),encoding='utf-8')
    ranked=sorted([j for j in data if j['left_against_true_straight_candidates']],key=lambda j:max(c['ego_width_m']+c['oncoming_width_m'] for c in j['left_against_true_straight_candidates']),reverse=True)
    print(town,'total',len(data),'candidate junctions',len(ranked))
    for j in ranked[:12]:
        c=max(j['left_against_true_straight_candidates'],key=lambda c:c['ego_width_m']+c['oncoming_width_m'])
        print(json.dumps({k:v for k,v in j.items() if k not in ['paths','left_against_true_straight_candidates']}),c)
    return m,result

if __name__=='__main__':
    for town in ['Town03','Town04','Town05']: inspect(town)
