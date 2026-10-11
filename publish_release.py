"""Publish reviewed source/media, then run the verified post-upload cleanup.

Use explicit --execute. Git never force-pushes. Authentication uses gh's normal
login or an existing Git Credential Manager credential held in process memory;
credentials are never written into source, reports, command arguments or Git.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import urllib.error
import urllib.parse
import urllib.request

ROOT=Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from open_source_check import require_clean


def checked_source_files():
    """Fail before authentication if reviewed source or Git staging is unsafe."""
    require_clean(ROOT)
    paths=json.loads((ROOT/'OPEN_SOURCE_FILES.json').read_text(encoding='utf-8'))['files']
    allowed=set(paths)
    repo=subprocess.run(['git','rev-parse','--show-toplevel'],cwd=ROOT,
        capture_output=True,text=True,encoding='utf-8')
    if repo.returncode==0:
        if Path(repo.stdout.strip()).resolve()!=ROOT:
            raise ValueError('Project must own its repository; refusing a parent repository')
        for argv in (['git','ls-files','-z'],
                     ['git','diff','--cached','--name-only','-z']):
            result=subprocess.run(argv,cwd=ROOT,capture_output=True,check=True)
            names={x.decode('utf-8') for x in result.stdout.split(b'\0') if x}
            if names-allowed:
                # Only relative file names, never source contents or credentials.
                raise ValueError('Git tracks/stages files outside source allowlist: '+', '.join(sorted(names-allowed)))
    return paths


def run(argv, env=None, capture=False, input_text=None):
    result=subprocess.run([str(x) for x in argv],cwd=ROOT,env=env,text=True,
        input=input_text,capture_output=capture,encoding='utf-8',check=True)
    return result.stdout.strip() if capture else None


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(4*1024*1024),b''):h.update(block)
    return h.hexdigest()


def credential_environment(gh):
    env=dict(os.environ)
    if env.get('GH_TOKEN') or env.get('GITHUB_TOKEN'):
        return env
    status=subprocess.run([gh,'auth','status'],capture_output=True,env=env)
    if status.returncode==0:return env
    result=subprocess.run(['git','credential','fill'],input='protocol=https\nhost=github.com\n\n',
        text=True,capture_output=True,encoding='utf-8',timeout=30,
        env=dict(env,GIT_TERMINAL_PROMPT='0',GCM_INTERACTIVE='never'))
    if result.returncode:
        raise RuntimeError('GitHub login required: use gh auth login, then retry')
    values=dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
    if not values.get('password'):raise RuntimeError('No existing GitHub credential')
    env['GH_TOKEN']=values['password']
    return env


def local_path(value):
    p=(ROOT/value).resolve()
    if not p.is_relative_to(ROOT) or p==ROOT or not p.is_file():
        raise ValueError('Expected existing project file: '+value)
    return p


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository',required=True)
    parser.add_argument('--tag',required=True)
    parser.add_argument('--release-plan',type=Path,required=True)
    parser.add_argument('--gh',default=shutil.which('gh'))
    parser.add_argument('--execute',action='store_true')
    a=parser.parse_args()
    if not re.fullmatch(r'[\w.-]+/[\w.-]+',a.repository):raise ValueError('Invalid repository')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._-]*',a.tag):raise ValueError('Invalid tag')
    plan=json.loads(a.release_plan.read_text(encoding='utf-8'))
    if plan.get('version')!=a.tag or plan.get('accepted') is not True:
        raise ValueError('Release plan must explicitly accept this exact version')
    paths=checked_source_files()
    for path in paths:local_path(path)
    assets=[]
    for asset in plan['assets']:
        path=local_path(asset['path'])
        name=asset['name']
        if Path(name).name!=name or not re.fullmatch(r'[A-Za-z0-9_.-]+',name):
            raise ValueError('Use simple ASCII release asset names')
        if path.name != name:
            raise ValueError('Release asset name must equal the reviewed local basename')
        if path.stat().st_size!=asset['bytes'] or digest(path)!=asset['sha256']:
            raise ValueError('Release asset changed: '+name)
        assets.append(asset)
    print(json.dumps({'repository':a.repository,'tag':a.tag,'source_files':len(paths),
        'asset_names':[x['name'] for x in assets],'execute':a.execute}))
    if not a.execute:return
    if not a.gh:raise RuntimeError('Install GitHub CLI or pass --gh')
    env=credential_environment(a.gh)
    login=json.loads(run([a.gh,'api','user'],env,capture=True))['login']
    if a.repository.split('/')[0].lower()!=login.lower():
        raise ValueError('Create releases only in the authenticated user repository')
    # Existing remote repo must be public; no implicit visibility conversion.
    result=subprocess.run([a.gh,'api','repos/'+a.repository],env=env,capture_output=True,text=True,encoding='utf-8')
    if result.returncode:
        if '(HTTP 404)' not in result.stderr:raise RuntimeError('Repository lookup failed')
        payload=json.dumps({'name':a.repository.split('/')[1],
            'description':'Native CARLA driving scenarios, real sensors and SUMO co-simulation',
            'private':False,'auto_init':False})
        created=json.loads(run([a.gh,'api','--method','POST','user/repos','--input','-'],env,capture=True,input_text=payload))
        print(json.dumps({'created_public_repository':created['html_url']}))
    elif json.loads(result.stdout).get('private') is not False:
        raise ValueError('Target repository is not public')
    if not (ROOT/'.git').exists():
        run(['git','init','-b','main'])
        run(['git','config','user.name',login])
        user=json.loads(run([a.gh,'api','user'],env,capture=True))
        run(['git','config','user.email',str(user['id'])+'+'+login+'@users.noreply.github.com'])
    remote='https://github.com/'+a.repository+'.git'
    current=subprocess.run(['git','remote','get-url','origin'],cwd=ROOT,capture_output=True,text=True)
    if current.returncode:run(['git','remote','add','origin',remote])
    elif current.stdout.strip()!=remote:raise ValueError('Existing origin differs; refusing to replace it')
    # Recheck immediately before staging after remote/authentication work.
    # A concurrently edited local source must not bypass the earlier gate.
    if checked_source_files()!=paths:
        raise ValueError('Source allowlist changed during publication')
    run(['git','add','--',*paths])
    result=subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT)
    if result.returncode==1:run(['git','commit','-m','Publish '+a.tag+' native driving showcase'])
    elif result.returncode!=0:raise RuntimeError('Could not inspect staged changes')
    commit=run(['git','rev-parse','HEAD'],capture=True)
    existing=subprocess.run(['git','rev-parse','--verify','refs/tags/'+a.tag],cwd=ROOT,capture_output=True,text=True)
    if existing.returncode:run(['git','tag',a.tag,commit])
    elif existing.stdout.strip()!=commit:raise ValueError('Existing local tag differs; refusing overwrite')
    run(['git','push','-u','origin','HEAD:main'])
    run(['git','push','origin','refs/tags/'+a.tag])
    notes=local_path(plan['notes_path'])
    published=subprocess.run([a.gh,'release','view',a.tag,'--repo',a.repository],env=env,capture_output=True)
    if published.returncode:
        run([a.gh,'release','create',a.tag,'--repo',a.repository,'--verify-tag',
            '--title',plan.get('title',a.tag),'--notes-file',notes],env)
    # Exact existing assets can be reused only if their GitHub digest matches.
    release=json.loads(run([a.gh,'api','repos/'+a.repository+'/releases/tags/'+a.tag],env,capture=True))
    existing_assets={v['name']:v for v in release['assets']}
    for asset in assets:
        old=existing_assets.get(asset['name'])
        if old:
            if old.get('digest')!='sha256:'+asset['sha256'] or old['size']!=asset['bytes']:
                raise ValueError('Published asset differs; refusing replacement: '+asset['name'])
            continue
        # gh's #suffix is a display label, not an asset filename override.
        run([a.gh,'release','upload',a.tag,'--repo',a.repository,
            str(local_path(asset['path']))],env)
    manifest={'schema_version':1,'cleanup_trigger':'after_successful_publication',
        'version':a.tag,'repository':a.repository,'tag':a.tag,'commit_sha':commit,
        'assets':[dict(x,url='https://github.com/'+a.repository+'/releases/download/'+a.tag+'/'+x['name']) for x in assets],
        'accepted_manifests':plan['accepted_manifests'],'keep_paths':plan['keep_paths'],
        'obsolete_versions':plan['obsolete_versions']}
    manifest_path=ROOT/'maintenance'/'publications'/(a.tag+'.json')
    manifest_path.parent.mkdir(parents=True,exist_ok=True)
    manifest_path.write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    report_dir=ROOT/'maintenance'/'reports'
    cleanup_plan=report_dir/(a.tag+'-cleanup-plan.json')
    receipt=report_dir/(a.tag+'-cleanup-receipt.json')
    run([sys.executable,ROOT/'maintenance.py','plan','--manifest',manifest_path,'--out',cleanup_plan])
    run([sys.executable,ROOT/'maintenance.py','cleanup','--manifest',manifest_path,
        '--plan',cleanup_plan,'--receipt',receipt,'--execute'])
    print(json.dumps({'repository':remote.removesuffix('.git'),'tag':a.tag,
        'commit_sha':commit,'publication_manifest':str(manifest_path),'cleanup_receipt':str(receipt)}))


if __name__=='__main__':main()
