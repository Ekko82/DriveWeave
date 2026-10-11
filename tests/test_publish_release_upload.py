"""Mock transport/tag contracts: no credentials, network, Git writes or cleanup.

All bytes, commits, API responses and acceptance records here are synthetic.
"""
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import urllib.error
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import publish_release as publisher

REPO='fixture-owner/fixture-repo'
TAG='v2.0.0-fixture'
COMMIT='a'*40
CONTENT=b'synthetic media contract bytes; not a real video'


def release(assets=None):
    return {'id':17,'tag_name':TAG,'draft':False,'published_at':'2026-01-01T00:00:00Z',
        'html_url':'https://github.com/'+REPO+'/releases/tag/'+TAG,
        'upload_url':'https://uploads.github.com/repos/'+REPO+'/releases/17/assets{?name,label}',
        'assets':assets or []}


def local_asset():
    return {'name':'demo.mp4','path':'media/demo.mp4','bytes':len(CONTENT),
        'sha256':hashlib.sha256(CONTENT).hexdigest()}


def remote_asset(**changes):
    asset=local_asset()
    result={'id':33,'name':asset['name'],'state':'uploaded','size':asset['bytes'],
        'digest':'sha256:'+asset['sha256'],
        'browser_download_url':'https://github.com/'+REPO+'/releases/download/'+TAG+'/demo.mp4'}
    return dict(result,**changes)


class Response(io.BytesIO):
    def __init__(self,value,status=200):
        super().__init__(json.dumps(value).encode('utf-8'))
        self.status=status

    def getcode(self):return self.status


class TransportTests(unittest.TestCase):
    def test_post_uses_release_url_raw_bytes_and_explicit_socket_timeout(self):
        api=publisher.GitHubAPI('fixture-token',23)
        api._opener=Mock()
        api._opener.open.return_value=Response(remote_asset(),201)
        url=publisher.upload_url(REPO,release(),'demo.mp4')
        self.assertEqual(api.request(url,data=CONTENT,content_type='video/mp4'),remote_asset())
        request=api._opener.open.call_args.args[0]
        self.assertEqual(request.full_url,
            'https://uploads.github.com/repos/'+REPO+'/releases/17/assets?name=demo.mp4')
        self.assertEqual(request.get_method(),'POST')
        self.assertEqual(request.data,CONTENT)
        self.assertEqual(request.get_header('Content-length'),str(len(CONTENT)))
        self.assertEqual(request.get_header('Content-type'),'video/mp4')
        self.assertEqual(request.get_header('Authorization'),'Bearer fixture-token')
        self.assertEqual(api._opener.open.call_args.kwargs,{'timeout':23})

    def test_get_timeout_and_auth_not_in_url(self):
        api=publisher.GitHubAPI('fixture-token')
        api._opener=Mock()
        api._opener.open.return_value=Response({'login':'fixture-owner'})
        self.assertEqual(api.get('user')['login'],'fixture-owner')
        request=api._opener.open.call_args.args[0]
        self.assertEqual(request.full_url,'https://api.github.com/user')
        self.assertIsNone(request.data)
        self.assertEqual(api._opener.open.call_args.kwargs['timeout'],60)

    def test_errors_do_not_expose_response_body_headers_or_credential(self):
        api=publisher.GitHubAPI('fixture-token')
        api._opener=Mock()
        api._opener.open.side_effect=urllib.error.HTTPError('https://api.github.com/user',502,
            'fixture-token raw private error',{},io.BytesIO(b'private-response-body'))
        with self.assertRaises(publisher.GitHubAPIError) as caught:api.get('user')
        self.assertEqual(str(caught.exception),'GitHub request failed with HTTP 502')
        self.assertEqual(caught.exception.status,502)
        self.assertNotIn('fixture-token',str(caught.exception))
        self.assertNotIn('private',str(caught.exception))
        self.assertIsNone(caught.exception.__cause__)

    def test_malformed_json_is_unverified_redacted(self):
        api=publisher.GitHubAPI('fixture-token')
        api._opener=Mock()
        api._opener.open.return_value=Response({})
        api._opener.open.return_value=io.BytesIO(b'private malformed reply')
        api._opener.open.return_value.getcode=lambda:200
        with self.assertRaisesRegex(publisher.GitHubAPIError,'unverified'):api.get('user')

    def test_credentials_never_follow_redirects(self):
        handler=publisher.NoAuthenticatedRedirects()
        with self.assertRaisesRegex(publisher.GitHubAPIError,'redirect refused'):
            handler.redirect_request(None,None,302,'',{},'https://untrusted.example/upload')

    def test_foreign_endpoints_and_nonfinite_timeouts_refused_before_transport(self):
        api=publisher.GitHubAPI('fixture-token')
        api._opener=Mock()
        for url in ('http://api.github.com/user','https://api.github.com:443/user',
                'https://user@api.github.com/user','https://untrusted.example/user',
                'https://api.github.com/user#fragment'):
            with self.subTest(url=url),self.assertRaises(ValueError):api.request(url)
        api._opener.open.assert_not_called()
        for timeout in (0,float('inf'),float('nan'),601):
            with self.subTest(timeout=timeout),self.assertRaises(ValueError):
                publisher.GitHubAPI('fixture-token',timeout)

    def test_upload_endpoint_must_match_authoritative_release_id_repository_and_host(self):
        for changed in ('https://uploads.github.com/repos/'+REPO+'/releases/18/assets{?name,label}',
                'https://uploads.github.com/repos/other/repo/releases/17/assets{?name,label}',
                'https://untrusted.example/repos/'+REPO+'/releases/17/assets{?name,label}',
                'https://uploads.github.com/repos/'+REPO+'/releases/17/assets?unexpected=1',
                'https://uploads.github.com:443/repos/'+REPO+'/releases/17/assets'):
            with self.subTest(url=changed),self.assertRaises(ValueError):
                publisher.upload_url(REPO,dict(release(),upload_url=changed),'demo.mp4')

    def test_existing_gh_login_token_is_captured_only_in_memory(self):
        results=[subprocess.CompletedProcess([],0,b'',b''),
            subprocess.CompletedProcess([],0,'fixture-token\n','')]
        with patch.dict(publisher.os.environ,{},clear=True),patch.object(
                publisher.subprocess,'run',side_effect=results) as run:
            env=publisher.credential_environment('fixture-gh')
        self.assertEqual(env['GH_TOKEN'],'fixture-token')
        self.assertEqual(run.call_args.args[0],['fixture-gh','auth','token'])
        self.assertEqual(run.call_args.kwargs['timeout'],30)
        self.assertNotIn('fixture-token',repr([call.args for call in run.call_args_list]))


class UploadTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name).resolve()
        (self.root/'media').mkdir()
        (self.root/'media/demo.mp4').write_bytes(CONTENT)
        self.override=patch.object(publisher,'ROOT',self.root)
        self.override.start()
        self.asset=local_asset()

    def tearDown(self):
        self.override.stop();self.temp.cleanup()

    def test_exact_existing_asset_is_reused_without_a_post(self):
        api=Mock()
        with patch.object(publisher,'fetch_release',return_value=release([remote_asset()])):
            result=publisher.ensure_uploaded_asset(api,REPO,TAG,17,self.asset)
        self.assertEqual(result['id'],33)
        api.request.assert_not_called()

    def test_one_raw_post_then_authoritative_readback(self):
        api=Mock()
        api.request.return_value=remote_asset()
        with patch.object(publisher,'fetch_release',side_effect=[release(),release([remote_asset()])]) as fetch:
            self.assertEqual(publisher.ensure_uploaded_asset(api,REPO,TAG,17,self.asset),remote_asset())
        self.assertEqual(fetch.call_count,2)
        self.assertEqual(api.request.call_count,1)
        self.assertEqual(api.request.call_args.kwargs,{'data':CONTENT,'content_type':'video/mp4'})

    def test_timeout_checks_remote_actual_digest_before_accepting_success(self):
        api=Mock()
        api.request.side_effect=publisher.GitHubAPIError('Uncertain timeout')
        with patch.object(publisher,'fetch_release',side_effect=[release(),release([remote_asset()])]) as fetch:
            result=publisher.ensure_uploaded_asset(api,REPO,TAG,17,self.asset)
        self.assertEqual(result,remote_asset())
        self.assertEqual(fetch.call_count,2)
        self.assertEqual(api.request.call_count,1)

    def test_timeout_absence_stops_without_second_post_or_receipt(self):
        api=Mock()
        api.request.side_effect=publisher.GitHubAPIError('Uncertain timeout')
        with patch.object(publisher,'fetch_release',side_effect=[release(),release()]):
            with self.assertRaisesRegex(RuntimeError,'retry.*--resume'):
                publisher.ensure_uploaded_asset(api,REPO,TAG,17,self.asset)
        self.assertEqual(api.request.call_count,1)
        self.assertFalse((self.root/'maintenance').exists())

    def test_timeout_readback_failure_is_unverified_not_success(self):
        api=Mock()
        api.request.side_effect=publisher.GitHubAPIError('Uncertain timeout')
        with patch.object(publisher,'fetch_release',side_effect=[release(),publisher.GitHubAPIError('Failed query')]):
            with self.assertRaisesRegex(RuntimeError,'unverified'):
                publisher.ensure_uploaded_asset(api,REPO,TAG,17,self.asset)
        self.assertEqual(api.request.call_count,1)

    def test_starter_wrong_size_digest_url_and_duplicates_never_clobber(self):
        for changed in ({'state':'starter'},{'digest':None},{'size':0},
                {'digest':'sha256:'+'b'*64},{'browser_download_url':'https://untrusted.example/file'}):
            api=Mock()
            with self.subTest(changed=changed),patch.object(publisher,'fetch_release',
                    return_value=release([remote_asset(**changed)])):
                with self.assertRaisesRegex(ValueError,'refusing replacement'):
                    publisher.ensure_uploaded_asset(api,REPO,TAG,17,self.asset)
            api.request.assert_not_called()
        api=Mock()
        with patch.object(publisher,'fetch_release',return_value=release([remote_asset(),remote_asset(id=34)])):
            with self.assertRaisesRegex(ValueError,'Duplicate'):
                publisher.ensure_uploaded_asset(api,REPO,TAG,17,self.asset)
        api.request.assert_not_called()

    def test_changed_local_bytes_stop_before_post(self):
        (self.root/'media/demo.mp4').write_bytes(b'changed')
        api=Mock()
        with patch.object(publisher,'fetch_release',return_value=release()):
            with self.assertRaisesRegex(ValueError,'changed before upload'):
                publisher.ensure_uploaded_asset(api,REPO,TAG,17,self.asset)
        api.request.assert_not_called()

    def test_missing_success_readback_cannot_become_a_receipt(self):
        api=Mock()
        api.request.return_value=remote_asset()
        with patch.object(publisher,'fetch_release',side_effect=[release(),release()]):
            with self.assertRaisesRegex(RuntimeError,'absent on readback'):
                publisher.ensure_uploaded_asset(api,REPO,TAG,17,self.asset)
        self.assertFalse((self.root/'maintenance').exists())

    def test_pinned_acceptance_and_unique_assets_required_before_resume(self):
        (self.root/'acceptance.json').write_text('{"accepted": true}')
        plan={'version':TAG,'accepted':True,'assets':[self.asset],
            'accepted_manifests':[{'path':'acceptance.json',
                'sha256':publisher.digest(self.root/'acceptance.json')}],
            'keep_paths':['media']}
        self.assertEqual(publisher.checked_release_assets(plan,TAG),[self.asset])
        (self.root/'acceptance.json').write_text('{"accepted": false}')
        with self.assertRaisesRegex(ValueError,'acceptance manifest changed'):
            publisher.checked_release_assets(plan,TAG)
        duplicate=dict(plan,assets=[self.asset,self.asset])
        with self.assertRaisesRegex(ValueError,'duplicate'):
            publisher.checked_release_assets(duplicate,TAG)


class ResumeTests(unittest.TestCase):
    def test_annotated_remote_tag_is_peeled_to_actual_commit(self):
        api=Mock()
        api.get.side_effect=[{'ref':'refs/tags/'+TAG,'object':{'type':'tag','sha':'b'*40}},
            {'sha':'b'*40,'object':{'type':'commit','sha':COMMIT}}]
        self.assertEqual(publisher.remote_tag_commit(api,REPO,TAG),COMMIT)
        self.assertEqual(api.get.call_args_list[1].args[0],
            'repos/'+REPO+'/git/tags/'+'b'*40)

    def test_wrong_exact_ref_or_cyclic_annotation_is_rejected(self):
        api=Mock()
        api.get.return_value={'ref':'refs/tags/'+TAG+'-other','object':{'type':'commit','sha':COMMIT}}
        with self.assertRaisesRegex(ValueError,'exact tag'):publisher.remote_tag_commit(api,REPO,TAG)
        api.get.side_effect=[{'ref':'refs/tags/'+TAG,'object':{'type':'tag','sha':'b'*40}},
            {'sha':'b'*40,'object':{'type':'tag','sha':'b'*40}}]
        with self.assertRaisesRegex(ValueError,'cyclic'):publisher.remote_tag_commit(api,REPO,TAG)

    def test_local_resume_reads_tag_not_new_head_and_only_readonly_commands(self):
        outputs=['https://github.com/'+REPO+'.git',COMMIT,
            json.dumps({'schema_version':1,'files':['OPEN_SOURCE_FILES.json','source.py']}),
            'OPEN_SOURCE_FILES.json\0source.py\0']
        with patch.object(publisher,'run',side_effect=outputs) as run:
            self.assertEqual(publisher.local_resume_commit(REPO,TAG,{}),COMMIT)
        commands=[call.args[0] for call in run.call_args_list]
        self.assertEqual(commands[1][-1],'refs/tags/'+TAG+'^{commit}')
        self.assertTrue(all(cmd[1] in {'remote','rev-parse','show','ls-tree'} for cmd in commands))
        self.assertFalse(any('HEAD' in cmd for cmd in commands))

    def test_local_resume_wrong_origin_plan_pin_or_unlisted_tag_tree_refused(self):
        for outputs,plan,pattern in ([['https://github.com/other/repo.git'],{},'origin differs'],
                [['https://github.com/'+REPO+'.git',COMMIT],{'commit_sha':'b'*40},'plan commit'],
                [['https://github.com/'+REPO+'.git',COMMIT,
                    json.dumps({'schema_version':1,'files':['OPEN_SOURCE_FILES.json']}),
                    'OPEN_SOURCE_FILES.json\0unreviewed.txt\0'],{},'tag tree']):
            with self.subTest(pattern=pattern),patch.object(publisher,'run',side_effect=outputs):
                with self.assertRaisesRegex(ValueError,pattern):publisher.local_resume_commit(REPO,TAG,plan)

    def test_resume_requires_same_tag_commit_public_repo_and_published_release(self):
        api=Mock()
        repo={'private':False,'full_name':REPO,'html_url':'https://github.com/'+REPO}
        api.get.return_value=repo
        plan={'assets':[local_asset()]}
        with patch.object(publisher,'remote_tag_commit',return_value=COMMIT),patch.object(
                publisher,'fetch_release',return_value=release()):
            self.assertEqual(publisher.verified_resume(api,REPO,TAG,plan,COMMIT)['id'],17)
        with patch.object(publisher,'remote_tag_commit',return_value='b'*40):
            with self.assertRaisesRegex(ValueError,'commits differ'):
                publisher.verified_resume(api,REPO,TAG,plan,COMMIT)
        api.get.return_value=dict(repo,private=True)
        with self.assertRaisesRegex(ValueError,'public visibility'):
            publisher.verified_resume(api,REPO,TAG,plan,COMMIT)
        for bad in (dict(release(),draft=True),dict(release(),published_at=None),
                dict(release(),tag_name=TAG+'-other'),dict(release(),id=18)):
            with self.subTest(release=bad),self.assertRaises(ValueError):
                publisher.verify_release_identity(REPO,TAG,bad,17)

    def test_extra_or_missing_assets_cannot_form_complete_publication(self):
        with self.assertRaisesRegex(ValueError,'unreviewed'):
            publisher.check_asset_set(REPO,TAG,release([remote_asset(name='unreviewed.mp4')]),
                [local_asset()],allow_missing=True)
        with self.assertRaisesRegex(ValueError,'missing'):
            publisher.check_asset_set(REPO,TAG,release(),[local_asset()],allow_missing=False)

    def test_asset_listing_uses_pagination_and_actual_release_identity(self):
        api=Mock()
        first=[dict(remote_asset(),id=i+1,name=str(i)+'.mp4') for i in range(100)]
        api.get.side_effect=[release(),first,[remote_asset()]]
        result=publisher.fetch_release(api,REPO,TAG,17)
        self.assertEqual(len(result['assets']),101)
        self.assertTrue(api.get.call_args.args[0].endswith('per_page=100&page=2'))


class ResumeFlowTests(unittest.TestCase):
    """Exercise argparse/main, including stopping before receipts on uncertainty."""
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name).resolve()
        (self.root/'media').mkdir()
        (self.root/'media/demo.mp4').write_bytes(CONTENT)
        (self.root/'acceptance.json').write_text('{"accepted": true}')
        self.plan={'version':TAG,'accepted':True,'assets':[local_asset()],
            'accepted_manifests':[{'path':'acceptance.json',
                'sha256':publisher.digest(self.root/'acceptance.json')}],
            'keep_paths':['media'],'obsolete_versions':[]}
        (self.root/'plan.json').write_text(json.dumps(self.plan))
        self.argv=['publish_release.py','--repository',REPO,'--tag',TAG,
            '--release-plan',str(self.root/'plan.json'),'--resume','--execute','--gh','fixture-gh']
        self.patches=[patch.object(publisher,'ROOT',self.root),patch.object(sys,'argv',self.argv),
            patch.object(publisher,'checked_source_files',return_value=['source.py']),
            patch.object(publisher,'local_resume_commit',return_value=COMMIT),
            patch.object(publisher,'credential_environment',return_value={'GH_TOKEN':'fixture-token'}),
            patch.object(publisher,'GitHubAPI'),patch.object(publisher,'verified_resume',return_value=release()),
            patch.object(publisher,'ensure_uploaded_asset',return_value=remote_asset()),
            patch.object(publisher,'run'),patch('builtins.print')]
        self.mocks=[p.start() for p in self.patches]
        self.mocks[5].return_value.get.return_value={'login':'fixture-owner','id':99}
        self.mocks[6].side_effect=[release(),release([remote_asset()])]

    def tearDown(self):
        for p in reversed(self.patches):p.stop()
        self.temp.cleanup()

    def test_resume_execute_does_not_stage_commit_push_create_tag_repo_or_release(self):
        publisher.main()
        self.mocks[2].assert_called_once_with(scan_current=False,require_repository=True)
        self.assertEqual(self.mocks[6].call_count,2)
        # Only the existing maintenance verifier/cleanup is dispatched after all
        # remote metadata checks. run is mocked: nothing executes or is deleted.
        self.assertEqual(self.mocks[8].call_count,2)
        for call in self.mocks[8].call_args_list:
            self.assertNotIn('git',call.args[0])
            self.assertIn(self.root/'maintenance.py',call.args[0])
        api=self.mocks[5].return_value
        api.request.assert_not_called()
        manifest=json.loads((self.root/'maintenance/publications'/(TAG+'.json')).read_text())
        self.assertEqual(manifest['commit_sha'],COMMIT)

    def test_unconfirmed_upload_writes_no_manifest_and_never_calls_cleanup(self):
        self.mocks[7].side_effect=RuntimeError('Upload outcome remains unverified')
        with self.assertRaisesRegex(RuntimeError,'unverified'):publisher.main()
        self.mocks[8].assert_not_called()
        self.assertFalse((self.root/'maintenance').exists())

    def test_resume_preflight_only_never_authenticates(self):
        self.argv.remove('--execute')
        publisher.main()
        self.mocks[4].assert_not_called()
        self.mocks[5].assert_not_called()
        self.mocks[8].assert_not_called()


if __name__=='__main__':unittest.main()
