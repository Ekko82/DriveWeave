"""Independent qualification of a pinned roundabout GUI representation.

Preserves the strict all-lane FAIL: two visible ego endpoint samples and one
14-sample invisible background gap. Verifies exact source pose/time/PNG binding
and actual repeated native GUI view. No lane filling, pose snapping, new live
control claims, or changes to physical/action/sensor gates are permitted.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import importlib.util
import json
import math
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0,str(Path(__file__).resolve().parent))
from audit_sumo_sync_boundary_qualified import require,sha,read,rows,completely_outside,lane_table,shape,point_line_distance

TYPE='qualified_source_pose_native_GUI_roundabout_lane_metadata_limitations'
CHECK='all_frame_source_time_mapping_and_coordinate_checks_pass'


def strict_gate(strict,scope):
    expected=[{'kind':'readback_lane_missing_from_actual_network','sample':i,
               'actor_id':scope['background_actor_id'] if i in range(41,55) else scope['ego_actor_id'],'lane':''}
              for i in [*range(41,55),157,473]]
    require(scope['background_samples']==list(range(41,55)) and scope['ego_samples']==[157,473],
            'Only the explicit 14 background and two ego samples are approved')
    require(strict.get('audit_type')=='independent_all_sample_native_SUMO_replay_mapping'
            and strict.get('passed') is False and strict.get('failures_count')==16
            and strict.get('failures_examples')==expected and strict.get('failed_checks')==[CHECK],
            'Additional or different strict failures cannot be qualified')
    require(strict['checks'].get(CHECK) is False and
            all(v is True for k,v in strict['checks'].items() if k!=CHECK),
            'A strict coordinate, time, image, process or source check failed')
    require(strict['samples_checked']==scope['expected_samples']==1001
            and strict['actors_checked']==scope['expected_actor_samples']==23023
            and strict['native_pngs_fully_decoded']==1001
            and strict['input_sha256']==scope['strict_input_sha256'], 'Pinned full source bindings differ')


def repeat_render_gate(changed_pixels,psnr_db,actual_boundary,expected_boundary,actors):
    require(actual_boundary==expected_boundary,'Native GUI viewport differs from the original source view')
    require(changed_pixels<=4,
            'Native repeated GUI pixel variation exceeds the explicit diagnostic limit')
    require(bool(actors) and all(a['xy_error_m']==0 and a['yaw_error_deg']<=1e-10 for a in actors),
            'Native repeated GUI actor pose differs from the exact recorded pose')


def image_metrics(source,probe):
    from PIL import Image
    with Image.open(source) as a,Image.open(probe) as b:
        a.load();b.load();require(a.size==b.size==(1280,720),'Missing or different native viewport dimensions')
        av,bv=a.convert('RGB').tobytes(),b.convert('RGB').tobytes()
        changed=sum(av[i:i+3]!=bv[i:i+3] for i in range(0,len(av),3))
        squared=sum((x-y)**2 for x,y in zip(av,bv))
        psnr=10*math.log10(255**2/(squared/len(av))) if squared else None
        # Native replay assigns blue only to the ego. Observe it around the
        # viewport center, allowing its front-bumper/body-center offset.
        colors=a.convert('RGB').crop((560,280,720,440)).tobytes()
        ego_blue=sum(colors[i]<70 and 80<colors[i+1]<215 and colors[i+2]>190 for i in range(0,len(colors),3))
        return {'changed_pixels':changed,'PSNR_dB':psnr,'bytes_identical':Path(source).read_bytes()==Path(probe).read_bytes(),
                'original_center_ego_blue_pixels':ego_blue}


def connected_endpoint(network,before,after,point):
    lt=lane_table(network);require(before['readback_sumo_lane_id'] in lt and after['readback_sumo_lane_id'] in lt,
                                  'Both actual adjacent ego lanes must exist')
    be,bl=lt[before['readback_sumo_lane_id']];ae,al=lt[after['readback_sumo_lane_id']]
    bf,af=be.get('function')=='internal',ae.get('function')=='internal'
    require(bf != af,'Ego exception must be one normal/internal endpoint')
    if bf:
        connected=any(c.get('from')==be.get('id') and c.get('fromLane')==bl.get('index')
                      and c.get('to')==ae.get('id') and c.get('toLane')==al.get('index')
                      for c in network.findall('connection'))
    else:
        connected=any(c.get('from')==be.get('id') and c.get('fromLane')==bl.get('index')
                      and c.get('via')==al.get('id') for c in network.findall('connection'))
    require(connected,'Actual ego lane pair has no original network connection')
    bp,ap=shape(bl),shape(al);require(math.dist(bp[-1],ap[0])<=1e-6,'Ego endpoints do not coincide')
    distance=math.dist(point,bp[-1]);require(distance<=.1,'Ego endpoint distance exceeds 0.1 m')
    for a,l in ((before,bl),(after,al)):
        params={p.get('key'):p.get('value') for p in l.findall('param')}
        require(a['readback_lane_origId']==params.get('origId') and
                point_line_distance(a['readback_sumo_front_bumper_xy_m'],shape(l))<=float(l.get('width'))/2+.1,
                'Adjacent ego original lane IDs or physical membership are incorrect')
    return distance


def verify_proofs(replay,mappings,scope,index):
    selected=[*range(41,55),157,473];entries=index['actual_GUI_proofs']
    require(sorted(e['sample'] for e in entries)==selected and len(entries)==16,
            'Every affected sample needs one actual native GUI proof')
    evidence=[];metrics={}
    for item in entries:
        path=Path(item['proof']);require(sha(path)==item['sha256'],'Actual GUI proof changed')
        proof=read(path);sample=item['sample'];row=mappings[sample]
        require(proof['actual_returncode']==item['actual_returncode']==0
                and proof['CARLA_connected_or_ticked'] is False and proof['native_or_replay_modified'] is False
                and proof['source_replay_manifest_sha256']==sha(replay/'manifest.json')
                and proof['source_mapping_sha256']==sha(replay/'mapping.jsonl')
                and proof['source_network_sha256']==scope['strict_input_sha256']['network']
                and proof['source_GUI_settings_sha256']==proof['probe_GUI_settings_sha256']==sha(replay/'gui-settings.xml'),
                'Actual native GUI process/provenance differs')
        require(len(proof['observations'])==1,'Each proof must refer to its one affected source sample')
        observation=proof['observations'][0]
        require(observation['sample']==sample and observation['source_frame']==row['source_frame'],
                'Actual GUI proof source sample/frame differs')
        source_png=Path(row['native_sumo_png']);probe_png=Path(observation['probe_png'])
        require(Path(observation['native_source_png']).resolve()==source_png.resolve()
                and sha(source_png)==row['native_sumo_png_sha256']==observation['native_source_png_sha256']
                and sha(probe_png)==observation['probe_png_sha256'],'Native original/probe screenshot missing or changed')
        actual_actors=observation['actors'];require(len(actual_actors)==len(row['actors']), 'Repeated GUI actor set is incomplete')
        byid={a['actor_id']:a for a in actual_actors};require(len(byid)==len(actual_actors),'Duplicate GUI actor evidence')
        for a in row['actors']:
            p=byid.get(a['carla_id'],{})
            require(p.get('role')==a['role'] and p.get('ego') is a['ego']
                    and p.get('actual_xy')==a['readback_sumo_front_bumper_xy_m']
                    and abs((p.get('actual_yaw',999)-a['readback_sumo_nav_angle_deg']+180)%360-180)<=1e-10
                    and p.get('source_replay_lane')==a['readback_sumo_lane_id'],
                    'Repeated actual native actor pose/identity or original source lane differs')
            if not a['readback_sumo_lane_id']:
                require(p.get('actual_lane')=='','Affected native empty lane was not independently reproduced')
        ego=next(a for a in row['actors'] if a['ego']);x,y=ego['readback_sumo_front_bumper_xy_m']
        metric=image_metrics(source_png,probe_png);boundary=observation['actual_after_step_boundary']
        metric['repeat_internal_lane_selection_differences']=[{'actor_id':a['carla_id'],
                    'original_lane':a['readback_sumo_lane_id'],'repeat_lane':byid[a['carla_id']]['actual_lane']}
                    for a in row['actors'] if byid[a['carla_id']]['actual_lane']!=a['readback_sumo_lane_id']]
        repeat_render_gate(metric['changed_pixels'],metric['PSNR_dB'],boundary,[[x-80,y-45],[x+80,y+45]],actual_actors)
        if sample in scope['ego_samples']:
            require(metric['original_center_ego_blue_pixels']>=20,'Native original screenshot does not show the ego')
        else:
            a=next(a for a in row['actors'] if a['carla_id']==scope['background_actor_id'])
            require(completely_outside(a,boundary),'Offnetwork background body is inside the actual viewport')
        metric.update(sample=sample,source_frame=row['source_frame'],actual_boundary=boundary)
        metrics[sample]=metric;evidence.append({'path':str(path),'sha256':sha(path)})
    return metrics,evidence


def qualification_facts(mappings,network,scope,metrics):
    expected={(i,scope['background_actor_id']) for i in range(41,55)} | {(i,scope['ego_actor_id']) for i in (157,473)}
    actual={(i,a['carla_id']) for i,r in enumerate(mappings) for a in r['actors'] if not a['readback_sumo_lane_id']}
    require(actual==expected,'Additional, longer, different or missing lane gaps refused')
    ego_facts=[]
    for sample in (157,473):
        actor=next(a for a in mappings[sample]['actors'] if a['carla_id']==scope['ego_actor_id'])
        require(actor['ego'] is True and actor['role']=='hero' and metrics[sample]['original_center_ego_blue_pixels']>=20,
                'The two visible endpoint exceptions must refer to the actual native ego')
        before=next(a for a in mappings[sample-1]['actors'] if a['carla_id']==scope['ego_actor_id'])
        after=next(a for a in mappings[sample+1]['actors'] if a['carla_id']==scope['ego_actor_id'])
        distance=connected_endpoint(network,before,after,actor['readback_sumo_front_bumper_xy_m'])
        ego_facts.append({'source_sample':sample,'native_frame':mappings[sample]['source_frame'],
                          'connected_endpoint_distance_m':distance,'native_GUI_ego_visible':True,
                          'previous_sumo_lane':before['readback_sumo_lane_id'],'following_sumo_lane':after['readback_sumo_lane_id']})
    background=[]
    for sample in range(41,55):
        actor=next(a for a in mappings[sample]['actors'] if a['carla_id']==scope['background_actor_id'])
        wp=actor['recorded_native_nearest_waypoint']
        require(actor['ego'] is False and actor['role']=='dynamic_bg_013' and wp['road_id']==44 and wp['lane_id']==-5
                and completely_outside(actor,metrics[sample]['actual_boundary']),
                'Background gap role/native road/lane or actual invisibility differs')
        background.append({'source_sample':sample,'native_frame':mappings[sample]['source_frame'],'native_s_m':wp['s_m']})
    require(all(b['native_s_m']>a['native_s_m'] for a,b in zip(background,background[1:])),
            'Recorded background native trajectory is not continuous forward motion')
    lt=lane_table(network)
    narrow=[l.get('id') for e,l in lt.values() if e.get('id')=='-44' and float(l.get('width'))<1]
    require(narrow,'Claimed converted narrow road44 geometry is absent')
    return {'qualification':TYPE,'all_actor_lane_membership':False,'ego_lane_membership_all_frames':False,
            'offnetwork_actor_samples':16,'ego_offnetwork_samples':2,'background_offnetwork_samples':14,
            'ego_endpoint_samples':ego_facts,'background_sample_start':41,'background_sample_end_inclusive':54,
            'background_role':'dynamic_bg_013','background_native_road_id':44,'background_native_lane_id':-5,
            'background_native_s_start_m':background[0]['native_s_m'],'background_native_s_end_m':background[-1]['native_s_m'],
            'converted_narrow_lane_ids':narrow,'all_gap_background_bodies_outside_actual_viewport':True,
            'exact_source_pose_time_and_original_PNGs_verified':True,'lane_field_filled':False,'source_modified':False,
            'native_GUI_repeat_render_max_changed_pixels':max(m['changed_pixels'] for m in metrics.values()),
            'native_GUI_repeat_render_min_nonidentical_PSNR_dB':min((m['PSNR_dB'] for m in metrics.values() if m['PSNR_dB'] is not None),default=None),
            'all_repeat_PNG_bytes_identical':all(m['bytes_identical'] for m in metrics.values())}


def revalidate_report(report):
    require(report.get('audit_type')==TYPE and report.get('passed') is True
            and report.get('all_actor_lane_membership') is False
            and report.get('ego_lane_membership_all_frames') is False
            and report.get('mode')=='post_capture_synchronized_replay','Incorrect round qualification scope')
    paths=report['qualification_evidence']
    for k in ('original_strict_report','strict_recheck','scope','viewport_proof_index','outer_replay_process'):
        require(sha(paths[k])==paths[k+'_sha256'],'Qualification evidence changed')
    require(sha(__file__)==report['qualification_auditor_sha256']
            and sha(Path(__file__).with_name('audit_sumo_sync_replay.py'))==report['input_sha256']['auditor']
            and sha(Path(__file__).with_name('audit_sumo_sync_boundary_qualified.py'))==report['qualification_helper_sha256'],
            'Qualified or original audit source changed')
    strict=read(paths['strict_recheck']);original=read(paths['original_strict_report']);scope=read(paths['scope'])
    for k in ('checks','failed_checks','failures_count','failures_examples','samples_checked','actors_checked',
              'native_pngs_fully_decoded','input_sha256','maximum_recomputed_errors'):
        require(strict[k]==original[k],'Original strict FAIL and full recheck disagree')
    strict_gate(strict,scope);replay=Path(report['replay']);manifest=read(replay/'manifest.json');run=Path(manifest['source_run']);net=Path(manifest['network'])
    inputs={'replay_manifest':replay/'manifest.json','mapping':replay/'mapping.jsonl','raw_manifest':run/'manifest.json',
        'raw_frames':run/'frames.jsonl','raw_xodr':run/'map.xodr','raw_execution':run.parent.parent/'executions'/(run.name+'.json'),
        'network':net,'network_investigation':net.parent.parent/'investigation.json','official_bridge_helper':Path(manifest['official_bridge_helper']),
        'official_converter':Path(read(net.parent.parent/'investigation.json')['official_converter']),
        'outer_replay_process_record':Path(paths['outer_replay_process'])}
    require(report['input_sha256']==strict['input_sha256'] and all(sha(p)==strict['input_sha256'][k] for k,p in inputs.items()),
            'Actual full source/network/process bindings changed')
    mappings=rows(replay/'mapping.jsonl');require(len(mappings)==1001 and sum(len(r['actors']) for r in mappings)==23023,
                                              'Full source/actor range changed')
    metrics,evidence=verify_proofs(replay,mappings,scope,read(paths['viewport_proof_index']))
    facts=qualification_facts(mappings,ET.parse(net).getroot(),scope,metrics)
    require(facts==report['sumo_replay_qualification'] and evidence==report['actual_native_GUI_proof_receipts'],
            'Qualified public facts or actual native evidence changed')
    return facts


def audit(replay,original,process,scope,index,out):
    require(not out.exists(),'Never overwrite a previous audit')
    recheck=out.with_name(out.stem+'.strict-recheck.json');require(not recheck.exists(),'Never overwrite strict evidence')
    from audit_sumo_sync_replay import audit as strict_audit
    strict_audit(replay,recheck,process);strict=read(recheck);strict_gate(strict,read(scope))
    metrics,evidence=verify_proofs(replay,rows(replay/'mapping.jsonl'),read(scope),read(index))
    facts=qualification_facts(rows(replay/'mapping.jsonl'),ET.parse(read(replay/'manifest.json')['network']).getroot(),read(scope),metrics)
    paths={}
    for k,p in (('original_strict_report',original),('strict_recheck',recheck),('scope',scope),('viewport_proof_index',index),('outer_replay_process',process)):
        paths[k]=str(p.resolve());paths[k+'_sha256']=sha(p)
    report=dict(strict);report.update(audit_type=TYPE,created_utc=datetime.now(timezone.utc).isoformat(),passed=True,
        checks={'all_unaffected_full_strict_checks_pass':True,'pinned_16_empty_lane_facts_unchanged':True,
                'actual_native_viewport_and_ego_presence_verified':True,'original_strict_FAIL_preserved':True},
        failed_checks=[],failures_count=0,failures_examples=[],all_actor_lane_membership=False,
        ego_lane_membership_all_frames=False,original_strict_all_lane_audit_passed=False,
        sumo_replay_qualification=facts,qualification_evidence=paths,actual_native_GUI_proof_receipts=evidence,
        native_GUI_repeat_render_metrics=list(metrics.values()),qualification_auditor_sha256=sha(__file__),
        qualification_helper_sha256=sha(Path(__file__).with_name('audit_sumo_sync_boundary_qualified.py')))
    report['limitations'].append('Roundabout retains 16 unchanged empty lane fields: two visible ego connected-endpoint samples and 14 consecutive samples of one invisible background road44/-5 vehicle. All-actor and all-ego lane membership are false. Native repeated GUI raster variation is measured separately; original full PNG hashes remain immutable. This is a pose-synchronized GUI representation, not a new closed-loop roundabout simulation.')
    revalidate_report(report);out.parent.mkdir(parents=True,exist_ok=True);tmp=out.with_name(out.name+'.tmp');tmp.write_text(json.dumps(report,indent=2)+'\n');tmp.replace(out)
    print(json.dumps({'passed':True,'all_actor_lane_membership':False,'ego_lane_membership_all_frames':False,'qualified_actor_samples':16}),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for k in ('replay','original-strict-report','process-file','scope','viewport-proof-index','out'):p.add_argument('--'+k,type=Path,required=True)
    a=p.parse_args();audit(a.replay.resolve(),a.original_strict_report.resolve(),a.process_file.resolve(),a.scope.resolve(),a.viewport_proof_index.resolve(),a.out.resolve())
