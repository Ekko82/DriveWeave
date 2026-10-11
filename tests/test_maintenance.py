"""Safety checks use temporary fixtures/mocked network; never delete project assets."""
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import TestCase, main
from unittest.mock import patch
import hashlib
import importlib.util
import io
import json
import types

SOURCE = Path(__file__).resolve().parents[1]/'maintenance.py'
spec = importlib.util.spec_from_file_location('showcase_maintenance', SOURCE)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class Download(io.BytesIO):
    def geturl(self):
        return 'https://release-assets.githubusercontent.com/test/fixture.mp4'


class Safety(TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory()
        self.root = Path(self.temp.name).resolve()
        self.change_root = patch.object(m, 'ROOT', self.root)
        self.change_root.start()
        self.accepted = self.put('carla/output/delivery-manifest.json', json.dumps({
            'final_delivery_accepted': True,
            'source_run': str(self.root/'carla/captures/current'),
            'settings': {'out': str(self.root/'carla/captures')},
            'tls_manager': 'carla'}).encode())
        self.current = self.put('carla/captures/current/rgb/00000001.png', b'current')
        self.old = self.put('carla/captures/old/rgb/00000001.png', b'old')
        self.code = self.put('carla/captures/old/source.py', b'print(1)')
        self.audit = self.put('carla/captures/old/manifest.json', b'{}')
        self.asset = self.put('carla/output/new.mp4', b'verified release bytes')

    def tearDown(self):
        self.change_root.stop()
        self.temp.cleanup()

    def put(self, relative, payload):
        path = self.root/relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
        return path

    def manifest(self):
        return {'schema_version': 1, 'cleanup_trigger': 'after_successful_publication',
            'version': 'v2', 'repository': 'example/showcase', 'tag': 'v2', 'commit_sha': 'a'*40,
            'assets': [{'name': 'new.mp4', 'path': 'carla/output/new.mp4',
                        'url': 'https://github.com/example/showcase/releases/download/v2/new.mp4',
                        'bytes': self.asset.stat().st_size, 'sha256': m.digest(self.asset)}],
            'accepted_manifests': [{'path': 'carla/output/delivery-manifest.json', 'sha256': m.digest(self.accepted)}],
            'keep_paths': ['carla/captures/current'],
            'obsolete_versions': [{'version': 'v1', 'reason': 'superseded capture', 'paths': ['carla/captures/old']}]}

    def test_path_escape_absolute_root_and_reparse_refused(self):
        for path in ['../outside', '.', '/etc/passwd', 'C:/private/value']:
            with self.assertRaises(m.Refusal):
                m.inside(path, False)
        original = Path.lstat
        def fake(path):
            if path == self.root/'carla/captures/old':
                return types.SimpleNamespace(st_mode=0o40755, st_file_attributes=0x400)
            return original(path)
        with patch.object(Path, 'lstat', fake), self.assertRaises(m.Refusal):
            m.inside('carla/captures/old/rgb/00000001.png')

    def test_current_run_protected_but_parent_setting_is_not_an_asset(self):
        protected, _, _ = m.accepted_protection()
        self.assertIsNone(m.category(self.current, protected))
        self.assertIsNotNone(m.category(self.old, protected))
        self.assertNotIn(self.root/'carla', protected)
        self.assertNotIn(self.root/'carla/captures', protected)

    def test_v2_namespaces_protect_accepted_binary_and_preserve_old_audits(self):
        namespaces=['captures-v2','live-captures-v2','processed-v2','sumo-replay-v2','v2-probes']
        references=[]
        for name in namespaces:
            references.append(str(self.root/f'carla/{name}/current'))
            self.put(f'carla/{name}/current/frame.png',b'current v2')
            self.put(f'carla/{name}/old/frame.png',b'old v2')
            for filename in ['source.py','audit.json','frames.jsonl','data.csv','map.xodr','route.xml','LICENSE.txt']:
                self.put(f'carla/{name}/old/{filename}',b'permanent evidence')
        self.accepted.write_text(json.dumps({'final_delivery_accepted':True,
            'references':references,'settings':{'out':str(self.root/'carla/captures-v2')}}))
        protected,_,_=m.accepted_protection()
        for name in namespaces:
            self.assertIsNone(m.category(self.root/f'carla/{name}/current/frame.png',protected))
            self.assertIsNotNone(m.category(self.root/f'carla/{name}/old/frame.png',protected))
            self.assertNotIn(self.root/f'carla/{name}',protected)
            for filename in ['source.py','audit.json','frames.jsonl','data.csv','map.xodr','route.xml','LICENSE.txt']:
                self.assertIsNone(m.category(self.root/f'carla/{name}/old/{filename}',protected))

    def test_v2_container_and_active_runs_cannot_be_declared_obsolete(self):
        for name in ['captures-v2','live-captures-v2','processed-v2','sumo-replay-v2','v2-probes']:
            self.put(f'carla/{name}/old/frame.png',b'old v2')
            self.put(f'carla/{name}/old/manifest.json',b'{"status":"capturing"}')
            value=self.manifest()
            value['obsolete_versions'][0]['paths']=[f'carla/{name}']
            path=self.put('publication-v2.json',json.dumps(value).encode())
            with self.assertRaises(m.Refusal):m.publication_manifest(path)
            value['obsolete_versions'][0]['paths']=[f'carla/{name}/old']
            path.write_text(json.dumps(value))
            with self.assertRaises(m.Refusal):m.publication_manifest(path)

    def test_sources_audits_and_licenses_never_candidates(self):
        protected, _, _ = m.accepted_protection()
        for path in [self.code, self.audit, self.put('output/backup/README.md', b'keep'),
                     self.put('verification/old.png', b'evidence')]:
            self.assertIsNone(m.category(path, protected))

    def test_report_output_cannot_replace_delivery_or_source(self):
        for path in [self.accepted, self.root/'README.md', self.root.parent/'outside.json']:
            with self.assertRaises(m.Refusal):
                m.write(path, {'not_written': True})
        self.assertTrue(json.loads(self.accepted.read_text())['final_delivery_accepted'])

    def test_broad_parent_and_active_capture_cannot_be_declared_obsolete(self):
        value = self.manifest()
        value['obsolete_versions'][0]['paths'] = ['carla/captures']
        path = self.put('publication.json', json.dumps(value).encode())
        with self.assertRaises(m.Refusal):
            m.publication_manifest(path)
        self.audit.write_text(json.dumps({'status': 'running'}))
        path.write_text(json.dumps(self.manifest()))
        with self.assertRaises(m.Refusal):
            m.publication_manifest(path)

    def test_plan_exact_old_binary_and_changed_file_refused(self):
        manifest = self.put('publication.json', json.dumps(self.manifest()).encode())
        with patch('sys.stdout', io.StringIO()):
            plan = m.build_plan(manifest, self.root/'maintenance/reports/plan.json')
        self.assertEqual([f['path'] for f in plan['files']], ['carla/captures/old/rgb/00000001.png'])
        self.old.write_bytes(b'changed')
        with self.assertRaises(m.Refusal):
            m.preflight(self.manifest(), plan)

    def test_dry_run_does_not_verify_network_or_unlink(self):
        manifest = self.put('publication.json', json.dumps(self.manifest()).encode())
        with patch('sys.stdout', io.StringIO()):
            m.build_plan(manifest, self.root/'maintenance/reports/plan.json')
            with patch.object(m, 'verify_publication') as network, patch.object(Path, 'unlink') as unlink:
                m.cleanup(manifest, self.root/'maintenance/reports/plan.json', self.root/'maintenance/reports/receipt.json', False)
                network.assert_not_called()
                unlink.assert_not_called()

    def test_failed_remote_verification_cannot_unlink(self):
        manifest = self.put('publication.json', json.dumps(self.manifest()).encode())
        with patch('sys.stdout', io.StringIO()):
            m.build_plan(manifest, self.root/'maintenance/reports/plan.json')
        with (patch.object(m, 'verify_publication', side_effect=m.Refusal('release not published')),
              patch.object(Path, 'unlink') as unlink):
            with self.assertRaises(m.Refusal):
                m.cleanup(manifest, self.root/'maintenance/reports/plan.json', self.root/'maintenance/reports/receipt.json', True)
            unlink.assert_not_called()
        self.assertFalse((self.root/'maintenance/reports/receipt.json').exists())

    def api(self, endpoint):
        if endpoint == 'repos/example/showcase':
            return {'private': False}
        if '/releases/tags/' in endpoint:
            a = self.manifest()['assets'][0]
            return {'id': 2, 'html_url': 'https://github.com/example/showcase/releases/tag/v2',
                'draft': False, 'published_at': '2026-10-12T00:00:00Z', 'tag_name': 'v2',
                'assets': [{'id': 3, 'name': a['name'], 'state': 'uploaded', 'size': a['bytes'],
                            'browser_download_url': a['url'], 'digest': None}]}
        return {'object': {'type': 'commit', 'sha': 'a'*40}}

    def test_actual_download_hash_required_even_with_valid_release_metadata(self):
        command = types.SimpleNamespace(returncode=0, stdout='a'*40+'\n')
        with patch.object(m, 'github_json', side_effect=self.api), patch.object(m.subprocess, 'run', return_value=command):
            with patch.object(m.urllib.request, 'urlopen', return_value=Download(self.asset.read_bytes())):
                result = m.verify_publication(self.manifest())
                self.assertEqual(result['assets_streamed_and_hash_verified'][0]['sha256'], m.digest(self.asset))
            with patch.object(m.urllib.request, 'urlopen', return_value=Download(b'corrupt')):
                with self.assertRaises(m.Refusal):
                    m.verify_publication(self.manifest())

    def test_wrong_remote_commit_refused_before_download(self):
        def api(endpoint):
            value = self.api(endpoint)
            if '/git/ref/' in endpoint:
                value['object']['sha'] = 'b'*40
            return value
        with patch.object(m, 'github_json', side_effect=api), patch.object(m.urllib.request, 'urlopen') as download:
            with self.assertRaises(m.Refusal):
                m.verify_publication(self.manifest())
            download.assert_not_called()


if __name__ == '__main__':
    main()
