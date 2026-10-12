"""Inventory or clean superseded binary caches only after verified publication.

Default operations are read-only. Cleanup requires --execute, a reviewed exact
file plan, a real published public GitHub release/tag/commit, and streamed SHA256
verification of every listed release download. No scheduler is created.
Standard-library-only; never reads credentials or deletes a directory tree.
"""
from pathlib import Path, PurePosixPath
from datetime import datetime, timezone
import argparse
import hashlib
import json
import os
import re
import stat
import subprocess
import sys
import urllib.parse
import urllib.request

ROOT = Path(__file__).resolve().parent
SCHEMA_VERSION = 1
BINARY_EXTENSIONS = {'.png', '.jpg', '.jpeg', '.bgra', '.npy', '.npz', '.bin',
                     '.ply', '.pcd', '.las', '.mp4', '.gif', '.zip', '.blend', '.blend1'}
ALLOWED_PREFIXES = {
    'scenes': 'legacy_sumo_blender', 'output': 'legacy_sumo_blender',
    'sumo': 'old_sumo_concept_binary',
    'carla/captures': 'superseded_or_unaccepted_capture',
    'carla/live-captures': 'superseded_or_unaccepted_live_capture',
    'carla/processed': 'superseded_processed_cache',
    'carla/sumo-replay': 'superseded_sumo_replay_cache',
    'carla/captures-v2': 'superseded_or_unaccepted_capture',
    'carla/captures-v3': 'superseded_or_unaccepted_capture',
    'carla/live-captures-v2': 'superseded_or_unaccepted_live_capture',
    'carla/processed-v2': 'superseded_processed_cache',
    'carla/processed-v3': 'superseded_processed_cache',
    'carla/sumo-replay-v2': 'superseded_sumo_replay_cache',
    'carla/v2-probes': 'superseded_probe_binary',
    'carla/output': 'superseded_v1_delivery_media',
    'carla/output-sumo': 'superseded_v1_delivery_media',
    'carla/output-v2': 'superseded_version_media',
    'carla/output-sumo-v2': 'superseded_version_media',
    'carla/release-v2': 'superseded_version_release_copy',
    'carla/releases': 'superseded_release_copy'}
VERSION_SCOPED_MEDIA_ROOTS = {'carla/output-v2':'v2',
                            'carla/output-sumo-v2':'v2', 'carla/release-v2':'v2'}
EXACT_V1_MEDIA_ROOTS = {'carla/output', 'carla/output-sumo'}
V1_MEDIA_EXTENSIONS = {'.mp4', '.gif', '.zip'}
PERMANENT_PREFIXES = ['.git', 'assets', 'verification', 'research',
    'carla/verification', 'carla/executions', 'carla/plans', 'carla/research',
    'carla/map-inventory', 'carla/sumo-live-assets', 'carla/sumo-network-investigation']
AUTO_ACCEPTED = ['carla/output/delivery-manifest.json']
REFERENCE_CONTAINERS = set(ALLOWED_PREFIXES) | {'carla', 'carla/output', 'carla/output-sumo'}


class Refusal(RuntimeError):
    pass


def now():
    return datetime.now(timezone.utc).isoformat()


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4*1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def report_path(path):
    raw = path.absolute()
    if not raw.is_relative_to(ROOT) or raw.suffix.lower() not in {'.json', '.jsonl'}:
        raise Refusal('Reports must be JSON/JSONL inside project maintenance/')
    safe = inside(relative(raw), False)
    if not safe.is_relative_to(ROOT/'maintenance'):
        raise Refusal('Report output cannot overwrite source, audit or delivery files')
    return safe


def write(path, value):
    path = report_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name+'.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def relative(path):
    return path.relative_to(ROOT).as_posix()


def inside(value, must_exist=True):
    """Require a literal project-relative path; reject traversals and reparse points."""
    text = str(value).replace('\\', '/')
    parts = PurePosixPath(text).parts
    if not text or text.startswith('/') or ':' in text or any(p in ('.', '..') for p in parts):
        raise Refusal('Expected a project-relative path: '+text)
    raw = ROOT.joinpath(*parts)
    resolved = raw.resolve()
    if resolved == ROOT or not resolved.is_relative_to(ROOT):
        raise Refusal('Path leaves the project or selects its root: '+text)
    cursor = ROOT
    for part in parts:
        cursor = cursor/part
        if cursor.exists() or cursor.is_symlink():
            s = cursor.lstat()
            if stat.S_ISLNK(s.st_mode) or getattr(s, 'st_file_attributes', 0) & 0x400:
                raise Refusal('Symlink/reparse point is forbidden: '+text)
    if must_exist and not resolved.exists():
        raise Refusal('Required local path is missing: '+text)
    return resolved


def below(path, parent):
    return path == parent or path.is_relative_to(parent)


def iter_files(folder):
    for current, directories, names in os.walk(folder, followlinks=False):
        directories[:] = [d for d in directories if d not in {'.git', 'private', 'credentials'}
                          and not Path(current, d).is_symlink()
                          and not (getattr(Path(current, d).lstat(), 'st_file_attributes', 0) & 0x400)]
        for name in names:
            path = Path(current, name)
            s = path.lstat()
            if stat.S_ISREG(s.st_mode) and not (getattr(s, 'st_file_attributes', 0) & 0x400):
                yield path, s


def strings(value):
    if isinstance(value, str):
        yield value
    elif isinstance(value, dict):
        for item in value.values():
            yield from strings(item)
    elif isinstance(value, list):
        for item in value:
            yield from strings(item)


def accepted_protection(manifest=None):
    """Follow accepted manifest references, including whole source/cache directories."""
    protected = {inside(p, False) for p in PERMANENT_PREFIXES}
    seeds = [inside(p) for p in AUTO_ACCEPTED if (ROOT/p).is_file()]
    declared = (manifest or {}).get('accepted_manifests', [])
    for item in declared:
        path = inside(item['path'])
        if digest(path) != item['sha256']:
            raise Refusal('Accepted manifest has changed: '+item['path'])
        seeds.append(path)
    for p in (manifest or {}).get('keep_paths', []):
        protected.add(inside(p))
    queue, seen, inputs = list(seeds), set(), {}
    while queue:
        path = queue.pop()
        if path in seen or path.suffix.lower() != '.json' or not path.is_file():
            continue
        seen.add(path)
        if path.stat().st_size > 16*1024*1024:
            continue
        try:
            obj = read(path)
        except (ValueError, OSError):
            raise Refusal('Cannot parse an accepted evidence reference: '+relative(path))
        if path in seeds and path in [ROOT/p for p in AUTO_ACCEPTED] and not obj.get('final_delivery_accepted'):
            raise Refusal('Current delivery is not accepted; cleanup is disabled')
        protected.add(path)
        inputs[relative(path)] = digest(path)
        for text in strings(obj):
            if (not text or len(text) > 1024 or '\n' in text or '\r' in text or '://' in text
                    or any(c in text for c in '*?<>"|')):
                continue
            candidate = Path(text)
            if not candidate.is_absolute():
                # Relative references are resolved against the owning manifest,
                # then the project. A string that is not a local path is ignored.
                candidates = [path.parent/candidate, ROOT/candidate]
            else:
                candidates = [candidate]
            for local in candidates:
                # Never stat or read an outside/private credential directory.
                if not local.is_absolute() or not local.is_relative_to(ROOT):
                    continue
                resolved = local.resolve()
                if resolved == ROOT or not resolved.is_relative_to(ROOT):
                    continue
                if not resolved.exists():
                    continue
                safe = inside(relative(resolved))
                # A settings "out" parent or the categorical word "carla"
                # is not an accepted run/cache. Protect concrete versions only.
                if safe.is_dir() and relative(safe) in REFERENCE_CONTAINERS:
                    continue
                protected.add(safe)
                if safe.is_dir():
                    # Their own manifests give additional provenance, while
                    # the entire original directory remains protected.
                    for name in ['manifest.json', 'processing-manifest.json']:
                        if (safe/name).is_file():
                            queue.append(safe/name)
                elif safe.suffix.lower() == '.json':
                    queue.append(safe)
                break
    fingerprint = hashlib.sha256(json.dumps({'paths': sorted(relative(p) for p in protected),
        'accepted_reference_hashes': inputs}, sort_keys=True).encode()).hexdigest()
    return protected, inputs, fingerprint


def category(path, protected):
    if path.suffix.lower() not in BINARY_EXTENSIONS:
        return None  # All source, audits, metadata, logs, CSV, XML, licenses stay.
    if path in protected or any(parent in protected for parent in path.parents):
        return None
    rel = relative(path)
    for prefix, kind in ALLOWED_PREFIXES.items():
        if rel.startswith(prefix+'/'):
            if prefix in EXACT_V1_MEDIA_ROOTS and path.suffix.lower() not in V1_MEDIA_EXTENSIONS:
                return None
            return kind
    return None


def inventory(report):
    started = now()
    protected, inputs, fingerprint = accepted_protection()
    groups, total, count, candidates = {}, 0, 0, []
    for path, s in iter_files(ROOT):
        rel = relative(path)
        group = '/'.join(rel.split('/')[:3]) if rel.startswith('carla/') else rel.split('/')[0]
        row = groups.setdefault(group, {'bytes': 0, 'files': 0})
        row['bytes'] += s.st_size
        row['files'] += 1
        total += s.st_size
        count += 1
        kind = category(path, protected)
        if kind:
            candidates.append({'path': rel, 'bytes': s.st_size, 'mtime_ns': s.st_mtime_ns, 'category': kind})
    git = subprocess.run(['git', '-C', str(ROOT), 'rev-parse', '--is-inside-work-tree'],
                         capture_output=True, text=True)
    value = {'scan_started_utc': started, 'created_utc': now(), 'root': str(ROOT), 'total_bytes': total, 'file_count': count,
        'size_metric': 'Observed regular-file st_size sum; logical bytes, not allocated disk blocks. Concurrent project changes are not locked.',
        'git_repository_present': git.returncode == 0 and git.stdout.strip() == 'true',
        'groups': dict(sorted(groups.items(), key=lambda item: -item[1]['bytes'])),
        'suggested_candidates': candidates, 'suggested_candidate_bytes': sum(c['bytes'] for c in candidates),
        'suggested_candidate_files': len(candidates), 'eligible_for_execution': False,
        'candidate_note': 'Suggestions only. A reviewed obsolete_versions declaration, exact hashed plan and verified publication are still required. No file was deleted.',
        'protected_paths': sorted(relative(p) for p in protected),
        'accepted_reference_hashes': inputs, 'protection_sha256': fingerprint,
        'excluded_from_inventory': ['.git', 'private', 'credentials', 'symlinks/reparse points']}
    write(report, value)
    print(json.dumps({'report': str(report), 'total_bytes': total, 'candidate_bytes': value['suggested_candidate_bytes'],
                      'candidate_files': len(candidates), 'deleted': 0}, ensure_ascii=False))


def publication_manifest(path):
    m = read(path)
    if m.get('schema_version') != SCHEMA_VERSION or m.get('cleanup_trigger') != 'after_successful_publication':
        raise Refusal('Unknown schema or cleanup trigger')
    if not re.fullmatch(r'[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+', m['repository']):
        raise Refusal('Expected public GitHub owner/repository')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._/-]*', m['tag']) or '..' in m['tag']:
        raise Refusal('Unsafe release tag')
    if not re.fullmatch(r'[0-9a-f]{40}', m['commit_sha']):
        raise Refusal('A full published commit SHA is required')
    if not m.get('assets') or not m.get('accepted_manifests') or not m.get('keep_paths'):
        raise Refusal('Assets, accepted manifests and current keep_paths are required')
    if len({a['name'] for a in m['assets']}) != len(m['assets']):
        raise Refusal('Duplicate release asset names')
    for a in m['assets']:
        if Path(a['name']).name != a['name'] or '\\' in a['name'] or '/' in a['name']:
            raise Refusal('Asset name must be a basename')
        p = inside(a['path'])
        if not p.is_file() or p.stat().st_size != a['bytes'] or digest(p) != a['sha256']:
            raise Refusal('Local release resource size/hash mismatch: '+a['path'])
        expected = 'https://github.com/'+m['repository']+'/releases/download/'+urllib.parse.quote(m['tag'], safe='')+'/'+urllib.parse.quote(a['name'], safe='')
        if a['url'] != expected:
            raise Refusal('Release resource URL is not bound to this repository/tag/name')
    validate_obsolete_versions(m)
    return m


def validate_obsolete_versions(m):
    if not m.get('obsolete_versions'):
        raise Refusal('Declare the prior versions and their exact cache paths')
    for old in m['obsolete_versions']:
        if not old['version'] or old['version'] == m['version'] or not old['reason'] or not old['paths']:
            raise Refusal('Each obsolete version needs a different ID, reason and exact paths')
        for p in old['paths']:
            exact = inside(p)
            rel = relative(exact)
            if any(rel == prefix or rel.startswith(prefix+'/') for prefix in EXACT_V1_MEDIA_ROOTS):
                current_major = re.match(r'^v(\d+)(?:$|[.\-_/])', m['version'])
                if (old['version'] != 'v1' or not current_major or int(current_major[1]) < 2
                        or not exact.is_file() or exact.suffix.lower() not in V1_MEDIA_EXTENSIONS):
                    raise Refusal('Legacy delivery media require an exact MP4/GIF/ZIP file, obsolete version v1 and a newer major publication: '+p)
            if relative(exact) in REFERENCE_CONTAINERS:
                scoped_version = VERSION_SCOPED_MEDIA_ROOTS.get(relative(exact))
                current_major = re.match(r'^v(\d+)(?:$|[.\-_/])',m['version'])
                if not (scoped_version and old['version'] == scoped_version
                        and current_major and int(current_major[1])>int(scoped_version[1:])):
                    raise Refusal('Obsolete paths must select concrete prior runs/scenes/files, never a whole cache parent: '+p)
            for ancestor in [exact, *exact.parents]:
                if ancestor == ROOT:
                    break
                for name in ['manifest.json', 'processing-manifest.json']:
                    record = ancestor/name
                    if not record.is_file() or record.stat().st_size == 0:
                        continue
                    try:
                        status = read(record).get('status')
                    except (ValueError, OSError):
                        continue  # Preserve invalid failed evidence; do not repair it.
                    if status in {'running', 'capturing', 'recording', 'processing'}:
                        raise Refusal('A candidate belongs to an active capture/processing run: '+p)


def build_plan(manifest_path, out):
    m = publication_manifest(manifest_path)
    protected, inputs, fingerprint = accepted_protection(m)
    protected.update(inside(a['path']) for a in m['assets'])
    files, seen = [], set()
    for old in m['obsolete_versions']:
        for value in old['paths']:
            p = inside(value)
            entries = iter_files(p) if p.is_dir() else [(p, p.stat())]
            for path, s in entries:
                if path in seen or not category(path, protected):
                    continue
                path = inside(relative(path))
                seen.add(path)
                files.append({'path': relative(path), 'bytes': s.st_size, 'mtime_ns': s.st_mtime_ns,
                              'sha256': digest(path), 'obsolete_version': old['version'],
                              'category': category(path, protected)})
    value = {'schema_version': 1, 'created_utc': now(), 'root': str(ROOT), 'mode': 'dry_run',
        'manifest': str(manifest_path.resolve()), 'manifest_sha256': digest(manifest_path),
        'version': m['version'], 'protection_sha256': fingerprint, 'accepted_reference_hashes': inputs,
        'files': sorted(files, key=lambda f: f['path']), 'candidate_bytes': sum(f['bytes'] for f in files),
        'candidate_files': len(files), 'publication_verified': False, 'deleted_files': 0}
    write(out, value)
    print(json.dumps({'plan': str(out), 'candidate_files': len(files), 'candidate_bytes': value['candidate_bytes'], 'deleted': 0}))
    return value


def github_json(endpoint):
    url = 'https://api.github.com/'+endpoint
    request = urllib.request.Request(url, headers={'Accept': 'application/vnd.github+json',
        'User-Agent': 'autodrive-showcase-maintenance', 'X-GitHub-Api-Version': '2022-11-28'})
    with urllib.request.urlopen(request, timeout=45) as stream:
        return json.load(stream)


def verify_publication(m):
    repo = m['repository']
    if github_json('repos/'+repo).get('private') is not False:
        raise Refusal('The required public repository could not be verified')
    tag = urllib.parse.quote(m['tag'], safe='')
    release = github_json('repos/'+repo+'/releases/tags/'+tag)
    if release.get('draft') is not False or not release.get('published_at') or release.get('tag_name') != m['tag']:
        raise Refusal('Release is not actually published with the declared tag')
    obj = github_json('repos/'+repo+'/git/ref/tags/'+tag)['object']
    for _ in range(8):
        if obj['type'] == 'commit':
            break
        if obj['type'] != 'tag':
            raise Refusal('Git tag does not resolve to a commit')
        obj = github_json('repos/'+repo+'/git/tags/'+obj['sha'])['object']
    if obj['type'] != 'commit' or obj['sha'] != m['commit_sha']:
        raise Refusal('Remote release tag does not resolve to the recorded commit')
    local = subprocess.run(['git', '-C', str(ROOT), 'rev-parse', '--verify', m['tag']+'^{commit}'],
                           capture_output=True, text=True)
    if local.returncode or local.stdout.strip() != m['commit_sha']:
        raise Refusal('Local tag/commit does not match the published version')
    assets = {a['name']: a for a in release['assets']}
    verified = []
    for expected in m['assets']:
        a = assets.get(expected['name'])
        if not a or a.get('state') != 'uploaded' or a['browser_download_url'] != expected['url'] or a['size'] != expected['bytes']:
            raise Refusal('Release resource is missing or differs: '+expected['name'])
        if a.get('digest') and a['digest'] != 'sha256:'+expected['sha256']:
            raise Refusal('GitHub release resource digest mismatch: '+expected['name'])
        h, count = hashlib.sha256(), 0
        request = urllib.request.Request(expected['url'], headers={'User-Agent': 'autodrive-showcase-maintenance'})
        with urllib.request.urlopen(request, timeout=60) as stream:
            final = urllib.parse.urlparse(stream.geturl())
            if final.scheme != 'https' or not (final.hostname == 'github.com' or (final.hostname or '').endswith('.githubusercontent.com')):
                raise Refusal('Release download redirected outside GitHub')
            for block in iter(lambda: stream.read(4*1024*1024), b''):
                count += len(block)
                h.update(block)
        if count != expected['bytes'] or h.hexdigest() != expected['sha256']:
            raise Refusal('Actual downloaded resource size/hash mismatch: '+expected['name'])
        verified.append({'name': expected['name'], 'url': expected['url'], 'bytes': count, 'sha256': h.hexdigest(), 'github_asset_id': a['id']})
    return {'verified_utc': now(), 'repository': repo, 'tag': m['tag'], 'commit_sha': obj['sha'],
            'release_id': release['id'], 'release_url': release['html_url'],
            'published_at': release['published_at'], 'assets_streamed_and_hash_verified': verified}


def preflight(m, plan):
    validate_obsolete_versions(m)
    protected, inputs, fingerprint = accepted_protection(m)
    protected.update(inside(a['path']) for a in m['assets'])
    if fingerprint != plan['protection_sha256'] or inputs != plan['accepted_reference_hashes']:
        raise Refusal('Accepted assets/evidence changed since the reviewed plan')
    obsolete = [(old['version'], inside(p)) for old in m['obsolete_versions'] for p in old['paths']]
    selected, seen = [], set()
    for f in plan['files']:
        p = inside(f['path'])
        if p in seen or not p.is_file() or not category(p, protected):
            raise Refusal('Duplicate, protected or non-binary candidate: '+f['path'])
        if not any(version == f['obsolete_version'] and below(p, folder) for version, folder in obsolete):
            raise Refusal('Candidate is not declared in its obsolete version')
        s = p.stat()
        if s.st_size != f['bytes'] or s.st_mtime_ns != f['mtime_ns'] or digest(p) != f['sha256']:
            raise Refusal('Candidate changed after planning: '+f['path'])
        seen.add(p)
        selected.append((p, f))
    if len(selected) != plan['candidate_files'] or sum(f['bytes'] for _, f in selected) != plan['candidate_bytes']:
        raise Refusal('Plan totals are inconsistent')
    return selected


def cleanup(manifest_path, plan_path, receipt_path, execute=False):
    m = publication_manifest(manifest_path)
    plan = read(plan_path)
    if plan.get('mode') != 'dry_run' or plan['manifest_sha256'] != digest(manifest_path) or plan['version'] != m['version']:
        raise Refusal('Plan belongs to a different publication manifest/version')
    selected = preflight(m, plan)
    if not execute:
        print(json.dumps({'mode': 'dry_run', 'candidate_files': len(selected), 'candidate_bytes': plan['candidate_bytes'], 'deleted': 0}))
        return
    if receipt_path.exists():
        raise Refusal('Execution receipt already exists; refusing accidental repeat')
    journal_path = report_path(receipt_path.with_suffix('.deleted.jsonl'))
    if journal_path.exists():
        raise Refusal('Deletion journal already exists; refusing accidental repeat')
    evidence = verify_publication(m)  # Fresh remote verification; no cached success flag.
    selected = preflight(m, plan)  # Repeat all checks after potentially long download.
    receipt = {'started_utc': now(), 'version': m['version'], 'publication': evidence,
        'manifest_sha256': digest(manifest_path), 'plan_sha256': digest(plan_path),
        'deleted': [], 'deletion_journal': str(journal_path),
        'status': 'executing', 'directories_recursively_deleted': 0}
    write(receipt_path, receipt)
    try:
        with journal_path.open('x', encoding='utf-8') as journal:
            for p, f in selected:
                # Re-resolve every exact absolute target immediately before unlink.
                # One Python process performs all operations; no shell strings.
                current = inside(f['path'])
                s = current.stat()
                if current != p or s.st_mtime_ns != f['mtime_ns'] or s.st_size != f['bytes'] or digest(current) != f['sha256']:
                    raise Refusal('Candidate changed during cleanup: '+f['path'])
                current.unlink()  # Exact reviewed binary files only, never rmtree.
                receipt['deleted'].append(f)
                journal.write(json.dumps({'deleted_utc': now(), **f})+'\n')
                journal.flush()
                os.fsync(journal.fileno())
        receipt.update(status='complete', finished_utc=now(), deleted_bytes=sum(f['bytes'] for f in receipt['deleted']))
        write(receipt_path, receipt)
    except Exception as exc:
        receipt.update(status='partial_failure', error=str(exc), finished_utc=now())
        write(receipt_path, receipt)
        raise
    print(json.dumps({'receipt': str(receipt_path), 'deleted_files': len(receipt['deleted']), 'deleted_bytes': receipt['deleted_bytes']}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    inv = commands.add_parser('inventory', help='Read-only sizes, protection and suggestions; no hashes of big caches')
    inv.add_argument('--report', type=Path, default=ROOT/'maintenance/reports/inventory.json')
    plan = commands.add_parser('plan', help='Hash an explicitly declared obsolete-version candidate set; never deletes')
    plan.add_argument('--manifest', type=Path, required=True)
    plan.add_argument('--out', type=Path, required=True)
    verify = commands.add_parser('verify-publication', help='Actually verify public release/tag/commit and download hashes; no deletion')
    verify.add_argument('--manifest', type=Path, required=True)
    verify.add_argument('--report', type=Path, required=True)
    clean = commands.add_parser('cleanup', help='Dry-run by default. --execute requires fresh GitHub verification')
    clean.add_argument('--manifest', type=Path, required=True)
    clean.add_argument('--plan', type=Path, required=True)
    clean.add_argument('--receipt', type=Path, required=True)
    clean.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if args.command == 'inventory':
        inventory(args.report)
    elif args.command == 'plan':
        build_plan(args.manifest.resolve(), args.out.resolve())
    elif args.command == 'verify-publication':
        manifest_path = args.manifest.resolve()
        evidence = verify_publication(publication_manifest(manifest_path))
        write(args.report, {'schema_version':1, 'status':'verified',
            'verification_method':'actual_public_github_tag_commit_and_full_resource_download_sha256',
            'publication_manifest_sha256':digest(manifest_path),
            'verification_script_sha256':digest(Path(__file__)), 'publication':evidence,
            'deleted_files':0, 'docs_modified':False})
    else:
        cleanup(args.manifest.resolve(), args.plan.resolve(), args.receipt.resolve(), args.execute)


if __name__ == '__main__':
    try:
        main()
    except (Refusal, OSError, ValueError, KeyError) as exc:
        print('REFUSED: '+str(exc), file=sys.stderr)
        raise SystemExit(2)
