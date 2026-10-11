"""Convert one actual OpenDRIVE file using CARLA's installed official converter.

Only offline carla.Map/netconvert work is performed. No CARLA Client or tick.
The new output retains actual source/converter/network SHA and complete logs.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from runtime_config import RuntimeConfig, hidden_process_options
from investigate_sumo_networks import xml_summary


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xodr',type=Path,required=True)
    parser.add_argument('--map',required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+',args.map):
        parser.error('Use a literal native map name')
    runtime=RuntimeConfig.resolve(require_sumo=True,require_bridge=True)
    source=args.xodr.resolve()
    root=ET.parse(source).getroot()
    output=args.out.resolve()
    output.mkdir(parents=True,exist_ok=False)
    town=output/args.map
    town.mkdir()
    converter=runtime.carla_sumo_root/'util/netconvert_carla.py'
    network=town/'official.net.xml'
    env=dict(os.environ,SUMO_HOME=str(runtime.sumo_home))
    env['PATH']=str(runtime.sumo_home/'bin')+os.pathsep+env.get('PATH','')
    version=subprocess.run([str(runtime.netconvert_binary),'--version'],
        capture_output=True,text=True,env=env,check=True,**hidden_process_options()).stdout
    (output/'netconvert-version.txt').write_text(version,encoding='utf-8')
    started=datetime.now(timezone.utc).isoformat()
    command=[sys.executable,str(converter),str(source),'--output',str(network)]
    result=subprocess.run(command,capture_output=True,text=True,encoding='utf-8',
        errors='replace',env=env,**hidden_process_options())
    log=result.stdout+result.stderr
    (town/'official-carla.log').write_text(log,encoding='utf-8')
    attempt={'kind':'official-carla','command':command,'started_utc':started,
        'finished_utc':datetime.now(timezone.utc).isoformat(),
        'actual_returncode':result.returncode,'warning_count':sum('Warning:' in s for s in log.splitlines()),
        'error_lines':[s for s in log.splitlines() if 'Error:' in s],
        'log_sha256':sha(town/'official-carla.log')}
    if result.returncode==0 and network.is_file():
        attempt['network']=xml_summary(network,town/'original-id-mapping.json')
    report={'created_utc':datetime.now(timezone.utc).isoformat(),
        'scope':'Actual native OpenDRIVE through installed official offline CARLA converter; no Client/tick.',
        'carla_version':'0.9.16','netconvert':str(runtime.netconvert_binary),
        'netconvert_version':version.splitlines()[0],
        'official_converter':str(converter),'official_converter_sha256':sha(converter),
        'actual_vehicle_synchronization_performed':False,
        'maps':[{'town':args.map,'source_xodr':str(source),'source_xodr_sha256':sha(source),
                 'source_road_count':len(root.findall('road')),'source_junction_count':len(root.findall('junction')),
                 'attempts':[attempt]}]}
    (output/'investigation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'map':args.map,'actual_returncode':result.returncode,
        'source_sha256':sha(source),'network_sha256':attempt.get('network',{}).get('sha256'),
        'warnings':attempt['warning_count'],'errors':attempt['error_lines'],
        'network_summary':{k:attempt.get('network',{}).get(k) for k in ['normal_edges','internal_edges','lanes','traffic_light_programs']},
        'CARLA_connected_or_ticked':False}),flush=True)
    return result.returncode if result.returncode else (0 if 'network' in attempt else 2)


if __name__=='__main__':
    raise SystemExit(main())
