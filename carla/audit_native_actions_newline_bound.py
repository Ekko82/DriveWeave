"""Native action audit with explicit, hash-proven CRLF/LF source equivalence.

Retains the original byte-strict report. No geometry, physical threshold,
snapshot, plan or raw file changes are made. Only the in-memory source-map
byte identifier is adapted after the original declared SHA exactly matches the
exported native map with CRLF converted to LF. Both identifiers are reported.
"""
from pathlib import Path
import argparse
import copy
import hashlib
import json
import sys

sys.path.insert(0,str(Path(__file__).resolve().parent))
from audit_native_actions import audit, file_sha


def binding(raw, declared):
    actual=hashlib.sha256(raw).hexdigest()
    normalized=hashlib.sha256(raw.replace(b'\r\n',b'\n')).hexdigest()
    return {'passed':bool(declared) and declared in (actual,normalized),
        'original_declared_native_map_sha256':declared,
        'actual_exported_map_byte_sha256':actual,
        'actual_exported_map_crlf_to_lf_sha256':normalized,
        'mode':'exact_bytes' if actual==declared else 'sha256_proven_CRLF_to_LF_only',
        'all_other_byte_transformations_permitted':False}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    run,out=args.run.resolve(),args.out.resolve()
    if run in out.parents:raise ValueError('Report must remain outside immutable raw capture')
    manifest=json.loads((run/'manifest.json').read_text(encoding='utf-8'))
    plan=manifest['route_specification']
    proof=binding((run/'map.xodr').read_bytes(),plan.get('source_xodr_sha256'))
    strict=out.with_name(out.stem+'-original-byte-strict.json')
    if strict.exists() or out.exists():raise ValueError('Fresh report paths required')
    audit(run,plan,strict)
    original=json.loads(strict.read_text(encoding='utf-8'))
    if not proof['passed']:
        result=original
        result['source_newline_equivalence']=proof
        result['scenario_actions_validated']=False
        if 'source_newline_equivalence' not in result['failed_checks']:
            result['failed_checks'].append('source_newline_equivalence')
    else:
        adapted=copy.deepcopy(plan)
        adapted['source_xodr_sha256']=proof['actual_exported_map_byte_sha256']
        audit(run,adapted,out)
        result=json.loads(out.read_text(encoding='utf-8'))
        result['source_newline_equivalence']=proof
        result['in_memory_map_byte_identifier_adapted']=proof['mode']!='exact_bytes'
        result['original_byte_strict_report']={'path':str(strict),'sha256':file_sha(strict),
            'scenario_actions_validated':original['scenario_actions_validated'],'failed_checks':original['failed_checks']}
        result['capture_manifest_modified']=False
        result['plan_modified_on_disk']=False
        result['independent_physical_thresholds_modified']=False
        result['wrapper_sha256']=file_sha(Path(__file__))
        result['limitations'].append('Native map source binding accepts only exact bytes or explicitly SHA-proven CRLF-to-LF equivalence; original strict-byte failures remain retained.')
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({'report':str(out),'scenario_actions_validated':result['scenario_actions_validated'],
        'failed_checks':result['failed_checks'],'map_binding_mode':proof['mode']}),flush=True)
    return 0 if result['scenario_actions_validated'] else 2


if __name__=='__main__':
    raise SystemExit(main())
