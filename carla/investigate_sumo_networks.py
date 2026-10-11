"""Offline SUMO OpenDRIVE conversion investigation. Never connects to CARLA."""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import collections
import hashlib
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from runtime_config import RuntimeConfig


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def xml_summary(path, mapping_path):
    tree = ET.parse(path)
    root = tree.getroot()
    lane_rows, normal_edges, internal_edges = [], [], []
    for edge in root.findall('edge'):
        internal = edge.get('function') == 'internal'
        (internal_edges if internal else normal_edges).append(edge.get('id'))
        for lane in edge.findall('lane'):
            params = {p.get('key'): p.get('value') for p in lane.findall('param')}
            lane_rows.append({'edge_id': edge.get('id'), 'lane_id': lane.get('id'),
                              'lane_index': int(lane.get('index')), 'internal': internal,
                              'original_odr_ids': params.get('origId'),
                              'params': params, 'length_m': float(lane.get('length')),
                              'width_m': float(lane.get('width', '3.2')),
                              'shape_xyz': [[float(x) for x in p.split(',')] for p in lane.get('shape').split()]})
    connectors = []
    for conn in root.findall('connection'):
        params = {p.get('key'): p.get('value') for p in conn.findall('param')}
        if params.get('origId'):
            connectors.append({'from_edge': conn.get('from'), 'to_edge': conn.get('to'),
                               'from_lane_index': conn.get('fromLane'), 'to_lane_index': conn.get('toLane'),
                               'via_internal_lane': conn.get('via'), 'original_odr_ids': params.get('origId'),
                               'params': params})
    mapping = {'net_xml': str(path.resolve()), 'net_xml_sha256': sha(path),
               'location': dict(root.find('location').attrib),
               'lanes': lane_rows, 'native_junction_connector_mapping': connectors,
               'mapping_rule': 'Use each actual lane/connection param key=origId; native road_lane tokens may map to multiple SUMO lanes and split edge sections. Do not infer native lane_id from SUMO lane_index.'}
    mapping_path.write_text(json.dumps(mapping, indent=2), encoding='utf-8')
    return {'normal_edges': len(normal_edges), 'internal_edges': len(internal_edges),
            'lanes': len(lane_rows), 'normal_lanes_with_original_ids': sum(not x['internal'] and bool(x['original_odr_ids']) for x in lane_rows),
            'original_native_junction_connector_mappings': len(connectors),
            'traffic_light_programs': len(root.findall('tlLogic')),
            'location': mapping['location'], 'bytes': path.stat().st_size,
            'sha256': mapping['net_xml_sha256'], 'mapping_file': str(mapping_path.resolve())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--include-official', action='store_true')
    args = parser.parse_args()
    base = Path(__file__).resolve().parent
    output = args.output_root.resolve()
    output.mkdir(parents=True, exist_ok=False)
    runtime = RuntimeConfig.resolve(require_sumo=True, require_bridge=True)
    sumo_home = runtime.sumo_home
    netconvert = runtime.netconvert_binary
    official = runtime.carla_sumo_root/'util/netconvert_carla.py'
    env = os.environ.copy()
    env['SUMO_HOME'] = str(sumo_home)
    env['PATH'] = str(sumo_home/'bin')+os.pathsep+env.get('PATH', '')
    version = subprocess.run([str(netconvert), '--version'], capture_output=True, text=True, env=env)
    (output/'netconvert-version.txt').write_text(version.stdout+version.stderr, encoding='utf-8')
    report = {'created_utc': datetime.now(timezone.utc).isoformat(),
              'scope': 'Offline exported-map conversion only. No CARLA Client, tick, world load, actor change, or trajectory synchronization.',
              'netconvert': str(netconvert), 'netconvert_version': version.stdout.splitlines()[0],
              'official_converter': str(official), 'official_converter_sha256': sha(official),
              'carla_version': '0.9.16', 'actual_vehicle_synchronization_performed': False,
              'sources': ['https://carla.readthedocs.io/en/0.9.16/adv_sumo/',
                          'https://eclipse.dev/sumo/docs/Networks/Import/OpenDRIVE.html',
                          'https://eclipse.dev/sumo/docs/Networks/Further_Outputs.html'],
              'maps': []}
    for town in ['Town04', 'Town06', 'Town03']:
        source = base/'map-inventory'/town/'map.xodr'
        town_output = output/town
        town_output.mkdir()
        source_xml = ET.parse(source).getroot()
        stats = {'town': town, 'source_xodr': str(source), 'source_xodr_sha256': sha(source),
                 'source_road_count': len(source_xml.findall('road')),
                 'source_junction_count': len(source_xml.findall('junction')),
                 'source_lane_types': dict(collections.Counter(l.get('type') for l in source_xml.findall('.//lane'))),
                 'attempts': []}
        commands = [('direct-original-ids', [str(netconvert), '--opendrive-files', str(source),
                    '--output-file', str(town_output/'direct.net.xml'), '--output.original-names', 'true',
                    '--opendrive.curve-resolution', '1', '--opendrive.internal-shapes', 'true'])]
        if args.include_official:
            commands.append(('official-carla', [sys.executable, str(official), str(source),
                             '--output', str(town_output/'official.net.xml')]))
        for name, command in commands:
            started = datetime.now(timezone.utc).isoformat()
            proc = subprocess.run(command, capture_output=True, text=True, encoding='utf-8', errors='replace', env=env)
            log_path = town_output/(name+'.log')
            log_path.write_text(proc.stdout+proc.stderr, encoding='utf-8')
            net_path = town_output/('direct.net.xml' if name.startswith('direct') else 'official.net.xml')
            attempt = {'kind': name, 'command': command, 'started_utc': started,
                       'finished_utc': datetime.now(timezone.utc).isoformat(), 'actual_returncode': proc.returncode,
                       'log': str(log_path), 'log_sha256': sha(log_path),
                       'warning_count': sum('Warning:' in line for line in (proc.stdout+proc.stderr).splitlines()),
                       'error_lines': [line for line in (proc.stdout+proc.stderr).splitlines() if 'Error:' in line or 'Traceback' in line or 'AssertionError' in line or 'RuntimeError' in line],
                       'output_exists': net_path.exists()}
            if proc.returncode == 0 and net_path.exists():
                attempt['network'] = xml_summary(net_path, town_output/(name+'-mapping.json'))
            stats['attempts'].append(attempt)
            print(json.dumps({'town': town, 'attempt': name, 'returncode': proc.returncode,
                              'network': attempt.get('network'), 'errors': attempt['error_lines']}, ensure_ascii=False), flush=True)
        report['maps'].append(stats)
        # Progress is durable even if a later offline conversion fails.
        (output/'investigation.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(str(output/'investigation.json'))


if __name__ == '__main__':
    main()
