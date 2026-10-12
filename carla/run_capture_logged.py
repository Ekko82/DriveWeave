"""Run capture with an explicit native process return-code report.

Only executes a simulator client when the operator invokes this wrapper; it
does not start a CARLA server. No Python or native error status is suppressed.
"""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import os
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('capture_args', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    forwarded = args.capture_args
    if forwarded[:1] == ['--']:
        forwarded = forwarded[1:]
    command = [sys.executable, '-u', str(Path(__file__).with_name('capture_native.py')), *forwarded]
    args.log.parent.mkdir(parents=True, exist_ok=True)
    source_archive = args.log.with_name(args.log.stem + '-executed-sources')
    source_archive.mkdir(exist_ok=False)
    archived_sources = {}
    for name in ['capture_native.py', 'native_controller.py', 'dynamic_traffic.py',
                 'physical_preroll.py', 'signal_policy.py', 'v3_traffic.py', Path(__file__).name]:
        source = Path(__file__).with_name(name)
        if source.is_file():
            payload = source.read_bytes()
            (source_archive / name).write_bytes(payload)
            archived_sources[name] = hashlib.sha256(payload).hexdigest()
    start = datetime.datetime.now(datetime.timezone.utc).isoformat()
    with args.log.open('wb') as handle:
        result = subprocess.run(command, stdout=handle, stderr=subprocess.STDOUT, check=False)
    finish = datetime.datetime.now(datetime.timezone.utc).isoformat()
    capture_run = None
    for line in args.log.read_text(encoding='utf-8',errors='replace').splitlines():
        if line.startswith('CAPTURE_RUN '):
            capture_run = Path(line.removeprefix('CAPTURE_RUN ')).resolve()
    def file_info(path):
        if not path.is_file():
            return {'sha256':None,'bytes':None}
        return {'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'bytes':path.stat().st_size}
    manifest_info = file_info(capture_run/'manifest.json') if capture_run else {'sha256':None,'bytes':None}
    frames_info = file_info(capture_run/'frames.jsonl') if capture_run else {'sha256':None,'bytes':None}
    report = {'command': command, 'started_utc': start,'finished_utc':finish,
        'source_archive': str(source_archive.resolve()), 'source_archive_sha256': archived_sources,
        'process_start_utc':start,'process_end_utc':finish,
        'returncode':result.returncode,'capture_run':str(capture_run) if capture_run else None,
        'manifest_sha256':manifest_info['sha256'],'manifest_bytes':manifest_info['bytes'],
        'frames_jsonl_sha256':frames_info['sha256'],'frames_jsonl_bytes':frames_info['bytes'],
        'log_sha256':file_info(args.log)['sha256'],
        'native_process_returncode': result.returncode, 'log': str(args.log.resolve()),
        'meaning': 'Actual Python client return code; independent of PowerShell stream-redirection status.',
        'sensor_or_action_validation': False}
    def write_report(path):
        path.parent.mkdir(parents=True,exist_ok=True)
        temporary=path.with_name(path.name+'.tmp')
        with temporary.open('wb') as handle:
            handle.write(json.dumps(report,indent=2).encode('utf-8'))
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary,path)
    write_report(args.log.with_suffix('.process.json'))
    execution_name=capture_run.name if capture_run else datetime.datetime.now().strftime('failed-before-run-%Y%m%d-%H%M%S-%f')
    execution_path=Path(__file__).with_name('executions')/(execution_name+'.json')
    report['execution_report']=str(execution_path.resolve())
    write_report(execution_path)
    print(json.dumps(report), flush=True)
    return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
