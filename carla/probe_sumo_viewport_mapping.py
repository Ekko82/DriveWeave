"""Reproduce selected immutable poses in native SUMO GUI for diagnostic evidence.

No CARLA Client/tick, source edits, lane filling or snapping. Records actual
TraCI viewport/poses/lanes and independently rendered PNG identity. This tool
does not grant an acceptance exception or modify an existing strict audit.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import json
import math
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0,str(Path(__file__).resolve().parent))
from sumo_sync_replay import _RUNTIME, traci, carla, BridgeHelper, transform, sha, free_port, wait_native_png, hidden_process_options


def probe(replay, samples, out, source_preroll=False):
    if out.exists():raise ValueError('Never overwrite prior diagnostic evidence')
    manifest=json.loads((replay/'manifest.json').read_text());run=Path(manifest['source_run'])
    source=[json.loads(l) for l in (run/'frames.jsonl').read_text().splitlines()]
    mapping=[json.loads(l) for l in (replay/'mapping.jsonl').read_text().splitlines()]
    network=Path(manifest['network']);net=ET.parse(network).getroot()
    BridgeHelper.offset=tuple(map(float,net.find('location').get('netOffset').split(',')))
    out.mkdir(parents=True);shutil.copyfile(replay/'gui-settings.xml',out/'gui-settings.xml')
    cfg=out/'probe.sumocfg'
    cfg.write_text('<configuration><input><net-file value="'+network.as_posix()+'"/><route-files value="'+(replay/'vtypes.rou.xml').as_posix()+'"/></input><time><step-length value="0.05"/></time><gui_only><gui-settings-file value="'+(out/'gui-settings.xml').as_posix()+'"/></gui_only></configuration>')
    port=free_port();command=[str(_RUNTIME.sumo_gui_binary),'-c',str(cfg),'--remote-port',str(port),'--start','--quit-on-end','--no-step-log','--duration-log.disable','--collision.action','warn','--collision.check-junctions','true','--time-to-teleport','-1','--window-size','960,640','--window-pos','-30000,-30000']
    log=(out/'sumo-gui.log').open('w');proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,**hidden_process_options())
    connection=None;observations=[];known=set()
    try:
        connection=traci.connect(port=port,numRetries=60,waitBetweenRetries=.25,proc=proc)
        view=connection.gui.getIDList()[0];connection.gui.setSchema(view,'native_replay')
        selected=set(samples)
        sequence=list(range(max(samples)+1)) if source_preroll else samples
        for sample in sequence:
            row=source[sample];original=mapping[sample];targets=[]
            for a in row['vehicles']:
                vid='carla_'+str(a['id'])
                if vid not in known:
                    connection.vehicle.add(vid,'',typeID='native_'+str(a['id']),depart='now')
                    connection.vehicle.setSpeedMode(vid,0);connection.vehicle.setLaneChangeMode(vid,0)
                    connection.vehicle.setColor(vid,(25,145,240,255) if a['ego'] else (235,137,59,255));known.add(vid)
                extent=carla.Vector3D(*a['bounding_box']['extent_m']);t=BridgeHelper.get_sumo_transform(transform(a),extent)
                connection.vehicle.setSpeed(vid,a['speed_mps']);connection.vehicle.moveToXY(vid,'',0,t.location.x,t.location.y,angle=t.rotation.yaw%360,keepRoute=2)
                targets.append((a,vid,t))
            ego=next(t for a,vid,t in targets if a['ego']);x,y=ego.location.x,ego.location.y
            requested=[x-80,y-45,x+80,y+45]
            connection.gui.setBoundary(view,*requested);before=connection.gui.getBoundary(view)
            png=out/f'{row["frame"]:08d}.png'
            if sample in selected:connection.gui.screenshot(view,str(png),1280,720)
            connection.simulationStep();after=connection.gui.getBoundary(view)
            actor_rows=[]
            for a,vid,t in targets:
                xy=connection.vehicle.getPosition(vid);angle=connection.vehicle.getAngle(vid);lane=connection.vehicle.getLaneID(vid)
                expected=next(v for v in original['actors'] if v['carla_id']==a['id'])
                actor_rows.append({'actor_id':a['id'],'role':a['role_name'],'ego':a['ego'],'actual_xy':list(xy),'actual_yaw':angle,'actual_lane':lane,
                    'source_replay_xy':expected['readback_sumo_front_bumper_xy_m'],'source_replay_yaw':expected['readback_sumo_nav_angle_deg'],'source_replay_lane':expected['readback_sumo_lane_id'],
                    'xy_error_m':math.dist(xy,expected['readback_sumo_front_bumper_xy_m']),'yaw_error_deg':abs((angle-expected['readback_sumo_nav_angle_deg']+180)%360-180)})
            if sample not in selected:continue
            png_sha=wait_native_png(png,1280,720);source_png=Path(original['native_sumo_png'])
            observations.append({'sample':sample,'source_frame':row['frame'],'requested_boundary':requested,'actual_before_step_boundary':before,'actual_after_step_boundary':after,
                'native_source_png':str(source_png),'native_source_png_sha256':sha(source_png),'probe_png':str(png),'probe_png_sha256':png_sha,
                'probe_and_original_PNG_bytes_identical':png.read_bytes()==source_png.read_bytes(),'actors':actor_rows})
        connection.close();connection=None;returncode=proc.wait(timeout=30)
    finally:
        if connection is not None:connection.close()
        if proc.poll() is None:proc.terminate();proc.wait(timeout=15)
        log.close()
    report={'scope':'Independent actual native SUMO viewport reproduction, diagnostic only; no acceptance exception',
        'created_utc':datetime.now(timezone.utc).isoformat(),'actual_returncode':returncode,'command':command,
        'source_replay_manifest_sha256':sha(replay/'manifest.json'),'source_mapping_sha256':sha(replay/'mapping.jsonl'),
        'source_raw_manifest_sha256':sha(run/'manifest.json'),'source_raw_frames_sha256':sha(run/'frames.jsonl'),
        'source_network_sha256':sha(network),'source_GUI_settings_sha256':sha(replay/'gui-settings.xml'),
        'probe_GUI_settings_sha256':sha(out/'gui-settings.xml'),'observations':observations,
        'source_preroll_performed':source_preroll,'actual_SUMO_steps':len(sequence),
        'CARLA_connected_or_ticked':False,'native_or_replay_modified':False,'acceptance_exception_granted':False}
    (out/'probe.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'actual_returncode':returncode,'samples':len(observations),'all_native_PNG_bytes_identical':all(r['probe_and_original_PNG_bytes_identical'] for r in observations)}),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--replay',type=Path,required=True);p.add_argument('--samples',required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--source-preroll',action='store_true')
    a=p.parse_args();probe(a.replay.resolve(),[int(v) for v in a.samples.split(',')],a.out.resolve(),a.source_preroll)
