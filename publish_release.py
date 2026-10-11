"""Publish reviewed source/media, then run the verified post-upload cleanup.

Use explicit --execute. Git never force-pushes. Authentication uses gh's normal
login or an existing Git Credential Manager credential held in process memory;
credentials are never written into source, reports, command arguments or Git.
Asset POSTs use GitHub's release-specific upload URL, raw bytes and a bounded
socket timeout. --resume only finishes an existing published tag: it never
stages, commits, pushes, creates a repository/release, or moves a tag.
"""
from pathlib import Path
import argparse
import hashlib
import http.client
import json
import math
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


def checked_source_files(*, scan_current=True, require_repository=False):
    """Fail before authentication if reviewed source or Git staging is unsafe."""
    # Resume publishes no working-tree source. Its immutable tag tree is checked
    # separately, so later local edits cannot make us recommit or retag a release.
    if scan_current:require_clean(ROOT)
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
    elif require_repository:
        raise ValueError('Resume requires an existing project-owned repository')
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
    status=subprocess.run([gh,'auth','status'],capture_output=True,env=env,timeout=30)
    if status.returncode==0:
        # urllib needs the token, not merely gh's authenticated status. Keep the
        # captured value in memory; never pass it on the command line or print it.
        result=subprocess.run([gh,'auth','token'],capture_output=True,text=True,
            encoding='utf-8',env=env,timeout=30)
        if result.returncode or not result.stdout.strip():
            raise RuntimeError('Could not obtain the existing GitHub CLI credential')
        return dict(env,GH_TOKEN=result.stdout.strip())
    result=subprocess.run(['git','credential','fill'],input='protocol=https\nhost=github.com\n\n',
        text=True,capture_output=True,encoding='utf-8',timeout=30,
        env=dict(env,GIT_TERMINAL_PROMPT='0',GCM_INTERACTIVE='never'))
    if result.returncode:
        raise RuntimeError('GitHub login required: use gh auth login, then retry')
    values=dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
    if not values.get('password'):raise RuntimeError('No existing GitHub credential')
    return dict(env,GH_TOKEN=values['password'])


def local_path(value):
    if not isinstance(value,str) or not value or '\\' in value or ':' in value or any(
            part in {'','.','..'} for part in value.split('/')):
        raise ValueError('Expected a literal relative project file path')
    p=(ROOT/value).resolve()
    if not p.is_relative_to(ROOT) or p==ROOT or not p.is_file():
        raise ValueError('Expected existing project file: '+value)
    return p


def checked_release_assets(plan,tag):
    """Require the same reviewed local bytes and pinned acceptance on retries."""
    if plan.get('version')!=tag or plan.get('accepted') is not True:
        raise ValueError('Release plan must explicitly accept this exact version')
    assets=plan.get('assets',[])
    if not isinstance(assets,list) or not assets:
        raise ValueError('Release plan has no accepted assets')
    names,paths=set(),set()
    for asset in assets:
        name=asset.get('name','')
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*',name) or name.endswith('.'):
            raise ValueError('Use simple ASCII release asset names')
        path=local_path(asset['path'])
        if name.casefold() in names or str(path).casefold() in paths:
            raise ValueError('Release plan contains duplicate assets')
        names.add(name.casefold());paths.add(str(path).casefold())
        if path.name!=name:
            raise ValueError('Release asset name must equal the reviewed local basename')
        if (type(asset.get('bytes')) is not int or asset['bytes']<=0 or
                not re.fullmatch(r'[0-9a-f]{64}',asset.get('sha256','')) or
                path.stat().st_size!=asset['bytes'] or digest(path)!=asset['sha256']):
            raise ValueError('Release asset changed: '+name)
    accepted=plan.get('accepted_manifests',[])
    if not isinstance(accepted,list) or not accepted or not plan.get('keep_paths'):
        raise ValueError('Release plan requires pinned acceptance and protected current paths')
    for item in accepted:
        if (not re.fullmatch(r'[0-9a-f]{64}',item.get('sha256','')) or
                digest(local_path(item['path']))!=item['sha256']):
            raise ValueError('Pinned acceptance manifest changed')
    return assets


class GitHubAPIError(RuntimeError):
    """Safe transport error: never include request headers, tokens or body text."""
    def __init__(self,message,status=None):
        super().__init__(message)
        self.status=status


class NoAuthenticatedRedirects(urllib.request.HTTPRedirectHandler):
    def redirect_request(self,req,fp,code,msg,headers,newurl):
        # No authenticated requests are allowed to forward credentials elsewhere.
        raise GitHubAPIError('Authenticated GitHub redirect refused',code)


class GitHubAPI:
    """A bounded JSON client; upload data remains raw binary, never JSON-wrapped."""
    def __init__(self,token,timeout_s=60):
        if not isinstance(token,str) or not token or any(c in token for c in '\r\n'):
            raise ValueError('Missing or invalid in-memory GitHub credential')
        if not math.isfinite(timeout_s) or not 1<=timeout_s<=600:
            raise ValueError('Request timeout must be between 1 and 600 seconds')
        self._token=token
        self.timeout_s=timeout_s
        self._opener=urllib.request.build_opener(NoAuthenticatedRedirects())

    def request(self,url,*,data=None,content_type=None):
        target=urllib.parse.urlsplit(url)
        if (target.scheme!='https' or target.hostname not in {'api.github.com','uploads.github.com'}
                or target.netloc!=target.hostname or target.fragment):
            raise ValueError('Unexpected authenticated GitHub endpoint')
        headers={'Authorization':'Bearer '+self._token,'Accept':'application/vnd.github+json',
            'User-Agent':'autodrive-showcase-publisher','X-GitHub-Api-Version':'2022-11-28'}
        if data is not None:
            if not isinstance(data,bytes):raise ValueError('Upload body must be raw bytes')
            headers['Content-Type']=content_type or 'application/octet-stream'
            headers['Content-Length']=str(len(data))
        request=urllib.request.Request(url,data=data,headers=headers,
            method='POST' if data is not None else 'GET')
        try:
            with self._opener.open(request,timeout=self.timeout_s) as response:
                if response.getcode()!=(201 if data is not None else 200):
                    raise GitHubAPIError('Unexpected GitHub response status',response.getcode())
                payload=response.read(16*1024*1024+1)
                if len(payload)>16*1024*1024:raise GitHubAPIError('GitHub JSON response too large')
                return json.loads(payload.decode('utf-8'))
        except urllib.error.HTTPError as error:
            code=error.code
            error.close()
            raise GitHubAPIError('GitHub request failed with HTTP '+str(code),code) from None
        except (urllib.error.URLError,OSError,http.client.HTTPException,UnicodeError,json.JSONDecodeError):
            raise GitHubAPIError('GitHub request failed or its result is unverified') from None

    def get(self,path):
        if not isinstance(path,str) or path.startswith('/') or ':' in path or '..' in path.split('/'):
            raise ValueError('Expected a relative GitHub API endpoint')
        return self.request('https://api.github.com/'+path)


def verify_public_repository(repository,data):
    if (not isinstance(data,dict) or data.get('private') is not False or
            data.get('full_name','').lower()!=repository.lower() or
            data.get('html_url')!='https://github.com/'+repository):
        raise ValueError('Target repository identity or public visibility does not match')


def remote_tag_commit(api,repository,tag):
    reference=api.get('repos/'+repository+'/git/ref/tags/'+tag)
    if not isinstance(reference,dict) or reference.get('ref')!='refs/tags/'+tag:
        raise ValueError('Remote tag does not match the requested exact tag')
    obj=reference.get('object',{})
    seen=set()
    for _ in range(8):
        sha=obj.get('sha','')
        if not re.fullmatch(r'[0-9a-f]{40}',sha) or sha in seen:
            raise ValueError('Remote tag object is invalid or cyclic')
        seen.add(sha)
        if obj.get('type')=='commit':return sha
        if obj.get('type')!='tag':raise ValueError('Remote tag does not resolve to a commit')
        annotated=api.get('repos/'+repository+'/git/tags/'+sha)
        if annotated.get('sha')!=sha:raise ValueError('Annotated remote tag identity mismatch')
        obj=annotated.get('object',{})
    raise ValueError('Remote tag nesting limit exceeded')


def local_resume_commit(repository,tag,plan):
    """Read the existing immutable tag, never HEAD or newly modified sources."""
    try:
        origin=run(['git','remote','get-url','origin'],capture=True)
        if origin!='https://github.com/'+repository+'.git':
            raise ValueError('Existing origin differs; refusing resume')
        commit=run(['git','rev-parse','--verify','refs/tags/'+tag+'^{commit}'],capture=True)
        if not re.fullmatch(r'[0-9a-f]{40}',commit):raise ValueError('Local tag commit is invalid')
        pinned=plan.get('commit_sha')
        if pinned is not None and pinned!=commit:raise ValueError('Release plan commit differs from local tag')
        allowlist=json.loads(run(['git','show',commit+':OPEN_SOURCE_FILES.json'],capture=True))
        allowed=allowlist.get('files',[])
        if allowlist.get('schema_version')!=1 or not allowed or len(allowed)!=len(set(allowed)):
            raise ValueError('Published tag has no valid positive source allowlist')
        tagged=set(run(['git','ls-tree','-r','--name-only','-z',commit],capture=True).split('\0'))-{''}
        if tagged-set(allowed):raise ValueError('Published tag tree contains files outside its source allowlist')
        return commit
    except (subprocess.CalledProcessError,json.JSONDecodeError):
        raise ValueError('Resume requires a readable existing local tag and its source allowlist') from None


def verify_release_identity(repository,tag,release,expected_id=None):
    if (not isinstance(release,dict) or type(release.get('id')) is not int or release['id']<=0
            or release.get('tag_name')!=tag or release.get('draft') is not False
            or not release.get('published_at') or
            release.get('html_url')!='https://github.com/'+repository+'/releases/tag/'+tag or
            (expected_id is not None and release['id']!=expected_id)):
        raise ValueError('Expected the same public, published release for this exact tag')
    return release


def fetch_release(api,repository,tag,expected_id=None):
    release=verify_release_identity(repository,tag,
        api.get('repos/'+repository+'/releases/tags/'+tag),expected_id)
    # Do not rely on an embedded or truncated asset listing for idempotency.
    assets=[]
    for page in range(1,101):
        batch=api.get('repos/'+repository+'/releases/'+str(release['id'])+
            '/assets?per_page=100&page='+str(page))
        if not isinstance(batch,list):raise ValueError('Invalid GitHub release asset listing')
        assets.extend(batch)
        if len(batch)<100:break
    else:raise ValueError('Release asset pagination limit exceeded')
    return dict(release,assets=assets)


def exact_asset(repository,tag,release,asset):
    matches=[old for old in release.get('assets',[]) if old.get('name')==asset['name']]
    if len(matches)>1:raise ValueError('Duplicate published asset name; refusing overwrite')
    if not matches:return None
    old=matches[0]
    url='https://github.com/'+repository+'/releases/download/'+tag+'/'+asset['name']
    if (type(old.get('id')) is not int or old['id']<=0 or old.get('state')!='uploaded' or
            old.get('digest')!='sha256:'+asset['sha256'] or old.get('size')!=asset['bytes']
            or old.get('browser_download_url')!=url):
        raise ValueError('Published asset differs or is incomplete; refusing replacement: '+asset['name'])
    return old


def check_asset_set(repository,tag,release,assets,*,allow_missing):
    wanted={a['name'] for a in assets}
    names=[old.get('name') for old in release.get('assets',[])]
    if len(names)!=len(set(names)) or set(names)-wanted:
        raise ValueError('Published release has duplicate or unreviewed assets')
    for asset in assets:
        if exact_asset(repository,tag,release,asset) is None and not allow_missing:
            raise ValueError('Published release is missing a required accepted asset')


def upload_url(repository,release,name):
    actual=release.get('upload_url','')
    if actual.endswith('{?name,label}'):actual=actual[:-len('{?name,label}')]
    target=urllib.parse.urlsplit(actual)
    expected='/repos/'+repository+'/releases/'+str(release['id'])+'/assets'
    if (target.scheme!='https' or target.netloc!='uploads.github.com' or target.path!=expected
            or target.query or target.fragment):
        raise ValueError('Release upload_url does not match the verified GitHub release')
    return actual+'?'+urllib.parse.urlencode({'name':name})


def ensure_uploaded_asset(api,repository,tag,release_id,asset):
    """At most one POST; after any unknown result, query before a later retry."""
    release=fetch_release(api,repository,tag,release_id)
    old=exact_asset(repository,tag,release,asset)
    if old:return old
    path=local_path(asset['path'])
    payload=path.read_bytes()
    if len(payload)!=asset['bytes'] or hashlib.sha256(payload).hexdigest()!=asset['sha256']:
        raise ValueError('Release asset changed before upload: '+asset['name'])
    url=upload_url(repository,release,asset['name'])
    try:
        returned=api.request(url,data=payload,
            content_type='video/mp4' if path.suffix.lower()=='.mp4' else 'application/octet-stream')
    except GitHubAPIError:
        # A timeout does not tell us whether GitHub stored the bytes. Never issue
        # another POST here, delete a starter asset, or claim a success receipt.
        try:
            observed=fetch_release(api,repository,tag,release_id)
            confirmed=exact_asset(repository,tag,observed,asset)
        except (GitHubAPIError,ValueError):
            raise RuntimeError('Upload outcome remains unverified; inspect the release and retry with --resume') from None
        if confirmed:return confirmed
        raise RuntimeError('Upload was not confirmed; retry this exact accepted plan with --resume') from None
    if exact_asset(repository,tag,{'assets':[returned]},asset) is None:
        raise ValueError('Upload response does not identify the required exact asset')
    observed=fetch_release(api,repository,tag,release_id)
    confirmed=exact_asset(repository,tag,observed,asset)
    if not confirmed:raise RuntimeError('Uploaded asset is absent on readback; retry with --resume')
    return confirmed


def verified_resume(api,repository,tag,plan,local_commit):
    verify_public_repository(repository,api.get('repos/'+repository))
    remote_commit=remote_tag_commit(api,repository,tag)
    if remote_commit!=local_commit:raise ValueError('Local and remote tag commits differ; refusing resume')
    release=fetch_release(api,repository,tag)
    check_asset_set(repository,tag,release,plan['assets'],allow_missing=True)
    return release


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository',required=True)
    parser.add_argument('--tag',required=True)
    parser.add_argument('--release-plan',type=Path,required=True)
    parser.add_argument('--gh',default=shutil.which('gh'))
    parser.add_argument('--resume',action='store_true',
        help='Finish uploads for an existing published local/remote tag; never stage, commit, push or retag')
    parser.add_argument('--request-timeout',type=float,default=60,
        help='GitHub API/upload socket timeout in seconds (1..600; default 60)')
    parser.add_argument('--execute',action='store_true')
    a=parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+',a.repository):raise ValueError('Invalid repository')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._-]*',a.tag):raise ValueError('Invalid tag')
    if not math.isfinite(a.request_timeout) or not 1<=a.request_timeout<=600:
        raise ValueError('Request timeout must be between 1 and 600 seconds')
    plan=json.loads(a.release_plan.read_text(encoding='utf-8'))
    paths=checked_source_files(scan_current=not a.resume,require_repository=a.resume)
    if not a.resume:
        for path in paths:local_path(path)
    assets=checked_release_assets(plan,a.tag)
    commit=local_resume_commit(a.repository,a.tag,plan) if a.resume else None
    print(json.dumps({'repository':a.repository,'tag':a.tag,'source_files':len(paths),
        'asset_names':[x['name'] for x in assets],'execute':a.execute,'resume':a.resume,
        'request_timeout_s':a.request_timeout}))
    if not a.execute:return
    if not a.gh:raise RuntimeError('Install GitHub CLI or pass --gh')
    env=credential_environment(a.gh)
    api=GitHubAPI(env.get('GH_TOKEN') or env.get('GITHUB_TOKEN'),a.request_timeout)
    user=api.get('user')
    login=user['login']
    if a.repository.split('/')[0].lower()!=login.lower():
        raise ValueError('Create releases only in the authenticated user repository')
    remote='https://github.com/'+a.repository+'.git'
    if a.resume:
        release=verified_resume(api,a.repository,a.tag,plan,commit)
    else:
        # Existing remote repo must be public; no implicit visibility conversion.
        try:
            verify_public_repository(a.repository,api.get('repos/'+a.repository))
        except GitHubAPIError as error:
            if error.status!=404:raise
            payload=json.dumps({'name':a.repository.split('/')[1],
                'description':'Native CARLA driving scenarios, real sensors and SUMO co-simulation',
                'private':False,'auto_init':False})
            created=json.loads(run([a.gh,'api','--method','POST','user/repos','--input','-'],env,capture=True,input_text=payload))
            verify_public_repository(a.repository,created)
            print(json.dumps({'created_public_repository':created['html_url']}))
        if not (ROOT/'.git').exists():
            run(['git','init','-b','main'])
            run(['git','config','user.name',login])
            run(['git','config','user.email',str(user['id'])+'+'+login+'@users.noreply.github.com'])
        current=subprocess.run(['git','remote','get-url','origin'],cwd=ROOT,capture_output=True,text=True)
        if current.returncode:run(['git','remote','add','origin',remote])
        elif current.stdout.strip()!=remote:raise ValueError('Existing origin differs; refusing to replace it')
        # Recheck immediately before staging after remote/authentication work.
        if checked_source_files()!=paths:raise ValueError('Source allowlist changed during publication')
        run(['git','add','--',*paths])
        result=subprocess.run(['git','diff','--cached','--quiet'],cwd=ROOT)
        if result.returncode==1:run(['git','commit','-m','Publish '+a.tag+' native driving showcase'])
        elif result.returncode!=0:raise RuntimeError('Could not inspect staged changes')
        commit=run(['git','rev-parse','HEAD'],capture=True)
        existing=subprocess.run(['git','rev-parse','--verify','refs/tags/'+a.tag+'^{commit}'],cwd=ROOT,capture_output=True,text=True)
        if existing.returncode:run(['git','tag',a.tag,commit])
        elif existing.stdout.strip()!=commit:raise ValueError('Existing local tag differs; use --resume or a new tag')
        run(['git','push','-u','origin','HEAD:main'])
        run(['git','push','origin','refs/tags/'+a.tag])
        if remote_tag_commit(api,a.repository,a.tag)!=commit:
            raise ValueError('Published remote tag does not match the local commit')
        notes=local_path(plan['notes_path'])
        try:
            release=fetch_release(api,a.repository,a.tag)
        except GitHubAPIError as error:
            if error.status!=404:raise
            run([a.gh,'release','create',a.tag,'--repo',a.repository,'--verify-tag',
                '--title',plan.get('title',a.tag),'--notes-file',notes],env)
            release=fetch_release(api,a.repository,a.tag)
        check_asset_set(a.repository,a.tag,release,assets,allow_missing=True)
    for asset in assets:
        uploaded=ensure_uploaded_asset(api,a.repository,a.tag,release['id'],asset)
        print(json.dumps({'confirmed_asset':asset['name'],'github_asset_id':uploaded['id']}))
    # A new lookup after every POST verifies public identity, immutable tag and
    # the exact complete asset set. Full remote byte hashing still belongs to
    # maintenance's verifier; client timeout never substitutes for that proof.
    release=verified_resume(api,a.repository,a.tag,plan,commit)
    check_asset_set(a.repository,a.tag,release,assets,allow_missing=False)
    checked_release_assets(plan,a.tag)
    manifest={'schema_version':1,'cleanup_trigger':'after_successful_publication',
        'version':a.tag,'repository':a.repository,'tag':a.tag,'commit_sha':commit,
        'assets':[dict(x,url='https://github.com/'+a.repository+'/releases/download/'+a.tag+'/'+x['name']) for x in assets],
        'accepted_manifests':plan['accepted_manifests'],'keep_paths':plan['keep_paths'],
        'obsolete_versions':plan['obsolete_versions']}
    manifest_path=ROOT/'maintenance'/'publications'/(a.tag+'.json')
    manifest_path.parent.mkdir(parents=True,exist_ok=True)
    payload=json.dumps(manifest,indent=2)+'\n'
    if manifest_path.exists() and json.loads(manifest_path.read_text(encoding='utf-8'))!=manifest:
        raise ValueError('Existing publication manifest differs; refusing overwrite')
    temp=manifest_path.with_suffix('.json.tmp')
    temp.write_text(payload,encoding='utf-8')
    temp.replace(manifest_path)
    report_dir=ROOT/'maintenance'/'reports'
    cleanup_plan=report_dir/(a.tag+'-cleanup-plan.json')
    receipt=report_dir/(a.tag+'-cleanup-receipt.json')
    run([sys.executable,ROOT/'maintenance.py','plan','--manifest',manifest_path,'--out',cleanup_plan])
    run([sys.executable,ROOT/'maintenance.py','cleanup','--manifest',manifest_path,
        '--plan',cleanup_plan,'--receipt',receipt,'--execute'])
    print(json.dumps({'repository':remote.removesuffix('.git'),'tag':a.tag,
        'commit_sha':commit,'publication_manifest':str(manifest_path),'cleanup_receipt':str(receipt)}))


if __name__=='__main__':main()
