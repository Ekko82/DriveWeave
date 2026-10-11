"""Render genuine SUMO-GUI viewport frames from recorded native CARLA poses.

This is explicitly post-capture synchronized replay, not a new live co-simulation.
No CARLA Client is constructed. Original recording files are never modified.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import faulthandler
import hashlib
import json
import math
import os
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from runtime_config import RuntimeConfig, hidden_process_options
_RUNTIME = RuntimeConfig.resolve(require_sumo=True, require_bridge=True, require_gui=True)
_RUNTIME.activate()
SUMO_HOME, CARLA_SUMO = _RUNTIME.sumo_home, _RUNTIME.carla_sumo_root
import carla
import traci
from sumo_integration.bridge_helper import BridgeHelper


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic_json(path, value):
    temporary = path.with_name(path.name+'.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def native_gui_settings(path):
    # Native SUMO settings only. No screenshot drawing, text or custom shapes.
    path.write_text('''<viewsettings>
  <delay value="0"/>
  <scheme name="native_replay">
    <opengl antialiase="1" dither="1"/>
    <background backgroundColor="222,230,220" showGrid="0"/>
    <edges laneEdgeMode="0" laneShowBorders="1" showBikeMarkings="1" showLinkDecals="1" showLinkRules="0" showRails="0" hideConnectors="0" widthExaggeration="1" minSize="0" showDirection="0" showSublanes="0" spreadSuperposed="0" edgeName_show="0" internalEdgeName_show="0" streetName_show="0">
      <colorScheme name="uniform">
        <entry color="68,76,82" name="road"/>
        <entry color="174,183,180" name="sidewalk"/>
        <entry color="68,76,82" name="bike lane"/>
        <entry color="0,0,0,0" name="green verge"/>
        <entry color="150,200,200" name="waterway"/>
        <entry color="68,76,82" name="railway"/>
        <entry color="68,76,82" name="rails on road"/>
        <entry color="68,76,82" name="no passenger"/>
        <entry color="68,76,82" name="closed"/>
        <entry color="68,76,82" name="connector"/>
        <entry color="68,76,82" name="forbidden"/>
      </colorScheme>
    </edges>
    <vehicles vehicleMode="0" vehicleQuality="2" vehicle_minSize="1" vehicle_exaggeration="1" vehicle_constantSize="0" showBlinker="0" vehicleName_show="0"/>
    <junctions junctionMode="0" junctionName_show="0" tlIndex_show="0" junctionIndex_show="0">
      <colorScheme name="uniform"><entry color="68,76,82" name=""/></colorScheme>
    </junctions>
    <legend showSizeLegend="0" showColorLegend="0" showVehicleColorLegend="0"/>
  </scheme>
</viewsettings>
''', encoding='utf-8')


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def transform(actor):
    t = actor['transform']
    p, r = t['location_m'], t['rotation_deg']
    return carla.Transform(carla.Location(*p), carla.Rotation(pitch=r[0], yaw=r[1], roll=r[2]))


def angle_error(a, b):
    return abs((a-b+180)%360-180)


def wait_native_png(path, width, height, timeout=15):
    # SUMO's GUI render thread finishes screenshots asynchronously. Do not
    # schedule another source pose before this genuine native file is complete.
    from PIL import Image
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        try:
            with Image.open(path) as im:
                im.load()
                if im.size != (width, height):
                    raise ValueError('Unexpected native SUMO screenshot dimensions.')
            return sha(path)
        except (OSError, FileNotFoundError):
            time.sleep(.01)
    raise TimeoutError('Native SUMO GUI did not complete screenshot: '+str(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--network', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--samples', type=int, help='Replay this many source samples from start; default all.')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--view-width-m', type=float, default=160)
    parser.add_argument('--port', type=int)
    parser.add_argument('--rpc-timeout-s', type=float, default=30,
                        help='Bound native TraCI waits; a blocked replay fails without editing its source')
    args = parser.parse_args()
    if args.rpc_timeout_s < 5 or not math.isfinite(args.rpc_timeout_s):
        raise ValueError('RPC timeout must be finite and at least five seconds.')
    run, output, network = args.run.resolve(), args.out.resolve(), args.network.resolve()
    source_manifest = json.loads((run/'manifest.json').read_text())
    execution_file = run.parent.parent/'executions'/(run.name+'.json')
    execution = json.loads(execution_file.read_text())
    if not (execution['returncode'] == 0 and source_manifest['native_cleanup_complete'] is True and
            execution['manifest_sha256'] == sha(run/'manifest.json') and
            execution['frames_jsonl_sha256'] == sha(run/'frames.jsonl')):
        raise ValueError('Replay requires original, normally exited CARLA capture with matching raw hashes.')
    rows = [json.loads(s) for s in (run/'frames.jsonl').read_text().splitlines()]
    if args.samples is not None:
        rows = rows[:args.samples]
    if not rows:
        raise ValueError('Empty replay.')
    fps = source_manifest['fps']
    output.mkdir(parents=True, exist_ok=False)
    images = output/'frames_native_sumo'
    images.mkdir()
    settings_file = output/'gui-settings.xml'
    native_gui_settings(settings_file)
    net_root = ET.parse(network).getroot()
    offset = tuple(float(v) for v in net_root.find('location').get('netOffset').split(','))
    BridgeHelper.offset = offset
    types = ET.Element('routes')
    first_actors = rows[0]['vehicles']
    for a in first_actors:
        e = a['bounding_box']['extent_m']
        kind = 'truck' if 'carlacola' in a['blueprint'] else 'passenger'
        attrs = {'id': 'native_'+str(a['id']), 'vClass': kind,
                 'guiShape': 'truck' if kind == 'truck' else 'passenger',
                 'length': str(e[0]*2), 'width': str(e[1]*2), 'height': str(e[2]*2),
                 'minGap': '0', 'accel': '10', 'decel': '10', 'maxSpeed': '60', 'sigma': '0'}
        ET.SubElement(types, 'vType', attrs)
    vtypes_file = output/'vtypes.rou.xml'
    ET.ElementTree(types).write(vtypes_file, encoding='utf-8', xml_declaration=True)
    config = output/'replay.sumocfg'
    config.write_text(f'''<configuration><input><net-file value="{network.as_posix()}"/><route-files value="{vtypes_file.as_posix()}"/></input><time><step-length value="{1/fps}"/></time><gui_only><gui-settings-file value="{settings_file.as_posix()}"/></gui_only></configuration>''', encoding='utf-8')
    port = args.port or free_port()
    command = [str(_RUNTIME.sumo_gui_binary), '-c', str(config), '--remote-port', str(port),
               '--start', '--quit-on-end', '--no-step-log', '--duration-log.disable',
               '--collision.action', 'warn', '--collision.check-junctions', 'true',
               '--time-to-teleport', '-1', '--window-size', '960,640', '--window-pos', '-30000,-30000']
    log = (output/'sumo-gui.log').open('w', encoding='utf-8')
    proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                            **hidden_process_options())
    manifest = {'status': 'running', 'mode': 'post_capture_synchronized_replay',
                'source_run': str(run), 'source_manifest_sha256': sha(run/'manifest.json'),
                'source_frames_jsonl_sha256': sha(run/'frames.jsonl'),
                'source_execution_sha256': sha(execution_file),
                'network': str(network), 'network_sha256': sha(network),
                'official_bridge_helper': str(CARLA_SUMO/'sumo_integration/bridge_helper.py'),
                'official_bridge_helper_sha256': sha(CARLA_SUMO/'sumo_integration/bridge_helper.py'),
                'network_offset_m': list(offset), 'source_fps': fps,
                'interpolated_frames': 0, 'carla_connected_or_ticked': False,
                'two_way_live_control_claimed': False, 'native_gui_only': True,
                'rpc_timeout_s': args.rpc_timeout_s,
                'viewport_px': [args.width, args.height], 'command': command,
                'created_utc': datetime.now(timezone.utc).isoformat()}
    atomic_json(output/'manifest.json', manifest)
    connection = None
    max_xy = max_yaw = max_inverse = 0.0
    mapped_samples, mapped_actors, errors = 0, 0, []
    previous_socket_timeout = socket.getdefaulttimeout()
    try:
        # TraCI has no public per-connection timeout argument. The standard
        # Python socket default applies to the socket created during connect;
        # restore the global immediately afterwards, leaving that socket bounded.
        socket.setdefaulttimeout(args.rpc_timeout_s)
        connection = traci.connect(port=port, numRetries=60, waitBetweenRetries=.25, proc=proc)
        socket.setdefaulttimeout(previous_socket_timeout)
        view = connection.gui.getIDList()[0]
        connection.gui.setSchema(view, 'native_replay')
        known = set()
        with (output/'mapping.jsonl').open('w', encoding='utf-8') as mapping:
            for index, row in enumerate(rows):
                faulthandler.dump_traceback_later(args.rpc_timeout_s, repeat=False)
                atomic_json(output/'rpc-progress.json', {'source_sample':row['sample'],
                            'source_frame':row['frame'], 'phase':'assign_actual_source_poses',
                            'previous_samples_completed':mapped_samples})
                targets = []
                for a in row['vehicles']:
                    vid = 'carla_'+str(a['id'])
                    if vid not in known:
                        connection.vehicle.add(vid, '', typeID='native_'+str(a['id']), depart='now')
                        connection.vehicle.setSpeedMode(vid, 0)
                        connection.vehicle.setLaneChangeMode(vid, 0)
                        connection.vehicle.setColor(vid, (25, 145, 240, 255) if a['ego'] else (235, 137, 59, 255))
                        known.add(vid)
                    extent = carla.Vector3D(*a['bounding_box']['extent_m'])
                    sumo_t = BridgeHelper.get_sumo_transform(transform(a), extent)
                    x, y, yaw = sumo_t.location.x, sumo_t.location.y, sumo_t.rotation.yaw % 360
                    # Source pose overwrites SUMO motion after this one simulation step.
                    # Speed is likewise external recorded state, never a new SUMO policy.
                    connection.vehicle.setSpeed(vid, a['speed_mps'])
                    connection.vehicle.moveToXY(vid, '', 0, x, y, angle=yaw, keepRoute=2)
                    targets.append((a, vid, sumo_t, extent))
                main_actor = next(t for t in targets if t[0]['ego'])
                x, y = main_actor[2].location.x, main_actor[2].location.y
                half_width = args.view_width_m/2
                half_height = half_width*args.height/args.width
                connection.gui.setBoundary(view, x-half_width, y-half_height, x+half_width, y+half_height)
                png = images/f'{row["frame"]:08d}.png'
                connection.gui.screenshot(view, str(png), args.width, args.height)
                atomic_json(output/'rpc-progress.json', {'source_sample':row['sample'],
                            'source_frame':row['frame'], 'phase':'native_SUMO_simulationStep',
                            'previous_samples_completed':mapped_samples})
                connection.simulationStep()
                actor_mapping = []
                for a, vid, target, extent in targets:
                    actual_xy = connection.vehicle.getPosition(vid)
                    actual_yaw = connection.vehicle.getAngle(vid)
                    xy_error = math.dist(actual_xy, [target.location.x, target.location.y])
                    yaw_error = angle_error(actual_yaw, target.rotation.yaw)
                    readback_t = carla.Transform(carla.Location(actual_xy[0], actual_xy[1], target.location.z),
                                                carla.Rotation(pitch=target.rotation.pitch, yaw=actual_yaw, roll=target.rotation.roll))
                    inverse = BridgeHelper.get_carla_transform(readback_t, extent)
                    inverse_error = math.dist([inverse.location.x, inverse.location.y], a['transform']['location_m'][:2])
                    max_xy, max_yaw, max_inverse = max(max_xy, xy_error), max(max_yaw, yaw_error), max(max_inverse, inverse_error)
                    if xy_error > .02 or yaw_error > .05 or inverse_error > .03:
                        errors.append({'sample': row['sample'], 'vehicle': vid, 'xy_error_m': xy_error,
                                       'yaw_error_deg': yaw_error, 'inverse_center_xy_error_m': inverse_error})
                    lane_id = connection.vehicle.getLaneID(vid)
                    actor_mapping.append({'carla_id': a['id'], 'role': a['role_name'], 'ego': a['ego'],
                        'sumo_id': vid, 'carla_actor_center_m': a['transform']['location_m'],
                        'carla_yaw_deg': a['transform']['rotation_deg'][1], 'bbox_extent_m': a['bounding_box']['extent_m'],
                        'target_sumo_front_bumper_xy_m': [target.location.x, target.location.y],
                        'target_sumo_nav_angle_deg': target.rotation.yaw % 360,
                        'readback_sumo_front_bumper_xy_m': list(actual_xy), 'readback_sumo_nav_angle_deg': actual_yaw,
                        'xy_error_m': xy_error, 'yaw_error_deg': yaw_error, 'inverse_carla_center_xy_error_m': inverse_error,
                        'readback_sumo_lane_id': lane_id, 'readback_lane_origId': connection.lane.getParameter(lane_id, 'origId') if lane_id else None,
                        'recorded_native_nearest_waypoint': a['waypoint'], 'source_speed_mps': a['speed_mps'],
                        'sumo_readback_speed_mps': connection.vehicle.getSpeed(vid)})
                    mapped_actors += 1
                png_sha = wait_native_png(png, args.width, args.height)
                record = {'source_sample': row['sample'], 'source_frame': row['frame'],
                          'source_timestamp_s': row['timestamp'], 'display_time_s': row['time_s'],
                          'sumo_replay_time_s': connection.simulation.getTime(), 'native_sumo_png': str(png),
                          'native_sumo_png_sha256': png_sha, 'gui_view_id': view, 'actors': actor_mapping}
                mapping.write(json.dumps(record)+'\n')
                mapping.flush()
                mapped_samples += 1
                faulthandler.cancel_dump_traceback_later()
                if index % 40 == 0:
                    print(json.dumps({'sample': row['sample'], 'mapped': mapped_samples, 'max_xy_error_m': max_xy,
                                      'max_yaw_error_deg': max_yaw, 'png_exists': png.exists()}), flush=True)
        connection.close(wait=True)
        connection = None
        proc.wait(timeout=30)
        log.close()
        pngs = list(images.glob('*.png'))
        from PIL import Image
        image_checks = []
        for p in pngs:
            with Image.open(p) as im:
                extrema = im.convert('RGB').getextrema()
                image_checks.append({'path': str(p), 'size': list(im.size),
                                     'nonuniform': any(high-low > 20 for low, high in extrema)})
        manifest.update({'status': 'complete' if proc.returncode == 0 and not errors and len(pngs) == len(rows)
                         and all(x['nonuniform'] for x in image_checks) else 'failed',
                         'sumo_process_exit_code': proc.returncode, 'samples_mapped': mapped_samples,
                         'actors_mapped': mapped_actors, 'native_png_count': len(pngs),
                         'maximum_front_bumper_xy_error_m': max_xy, 'maximum_nav_angle_error_deg': max_yaw,
                         'maximum_inverse_carla_center_xy_error_m': max_inverse,
                         'strict_readback_errors': errors, 'all_native_pngs_nonuniform': all(x['nonuniform'] for x in image_checks),
                         'native_png_dimensions': image_checks[0]['size'] if image_checks else None,
                         'mapping_sha256': sha(output/'mapping.jsonl'), 'sumo_log_sha256': sha(output/'sumo-gui.log'),
                         'native_frame_alignment': 'PNG source sample i is requested before exactly one SUMO simulationStep with source sample i moveToXY targets, then read back after that same step. No interpolation.',
                         'finished_utc': datetime.now(timezone.utc).isoformat()})
        atomic_json(output/'manifest.json', manifest)
        print(json.dumps(manifest, indent=2), flush=True)
        return 0 if manifest['status'] == 'complete' else 2
    except Exception as exc:
        manifest.update({'status': 'failed', 'error': repr(exc), 'samples_mapped': mapped_samples,
                         'maximum_front_bumper_xy_error_m': max_xy, 'maximum_nav_angle_error_deg': max_yaw})
        atomic_json(output/'manifest.json', manifest)
        raise
    finally:
        faulthandler.cancel_dump_traceback_later()
        socket.setdefaulttimeout(previous_socket_timeout)
        if connection is not None:
            try:
                connection.close(wait=False)
            except Exception:
                pass
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=20)
        if not log.closed:
            log.close()


if __name__ == '__main__':
    raise SystemExit(main())
