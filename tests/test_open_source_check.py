"""Positive publication-list checks without reading real private directories."""
from pathlib import Path
import json
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from open_source_check import allowed_path, verify_allowlist
import hashlib


class PublicationListTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def manifest(self, files):
        (self.root / "OPEN_SOURCE_FILES.json").write_text(
            json.dumps({"schema_version": 1, "files": files}), encoding="utf-8")

    def test_only_named_file_read_and_clean_sha_recorded(self):
        (self.root / "source.py").write_text("print('test')\n", encoding="utf-8")
        self.manifest(["source.py"])
        result = verify_allowlist(self.root)
        self.assertEqual(result["status"], "pass")
        self.assertEqual(len(result["source_sha256"]["source.py"]), 64)

    def test_private_path_and_traversal_refused_before_read(self):
        for value in ("../outside.py", "private/account.json", "captures/run/manifest.json",
                      "carla/captures-v3/run/manifest.json", "carla/v3-runtime/owned-process.json",
                      "carla/processed-v3/scene/processing-manifest.json", "carla/output-v3/scene/media-manifest.json",
                      "./script.py", "dir\\script.py", "C:/outside.py"):
            with self.assertRaises(ValueError):
                allowed_path(self.root, value)

    def test_missing_file_and_duplicate_entries_fail(self):
        self.manifest(["missing.py"])
        self.assertEqual(verify_allowlist(self.root)["findings"][0]["rule"], "missing_file")
        self.manifest(["missing.py", "missing.py"])
        with self.assertRaises(ValueError):
            verify_allowlist(self.root)

    def test_path_finding_does_not_disclose_value(self):
        path = "D" + ":/" + "Business" + "/unpublished/example"
        (self.root / "source.py").write_text(repr(path), encoding="utf-8")
        self.manifest(["source.py"])
        result = verify_allowlist(self.root)
        self.assertEqual(result["status"], "fail")
        self.assertEqual(result["findings"][0]["line"], 1)
        self.assertNotIn(path, json.dumps(result))

    def test_credential_pattern_is_redacted(self):
        fake = "ghp_" + "a" * 40
        (self.root / "source.py").write_text(repr(fake), encoding="utf-8")
        self.manifest(["source.py"])
        result = verify_allowlist(self.root)
        self.assertEqual(result["findings"][0]["rule"], "github_access_token_literal")
        self.assertNotIn(fake, json.dumps(result))

    def test_readme_gif_requires_explicit_sanitized_acceptance_and_matching_sha(self):
        media=self.root/'docs/media';media.mkdir(parents=True)
        payload=b'GIF89a'+b'unit test header fixture'
        (media/'highway.gif').write_bytes(payload)
        summary_name='docs/media/accepted-v2-summary.json'
        ids=['highway','ramp-entry','ramp-exit','left-turn','roundabout','live-highway']
        scenes=[]
        for scene_id in ids:
            scenes.append({'id':scene_id,'requested_action_verified':True,'full_internal_plan_completed':True,
                'manual_review':{'accepted':True},'media':[{'role':'carla-sumo-four-pane','sha256':'0'*64}],
                'preview':{'path':'docs/media/'+scene_id+'.gif','sha256':hashlib.sha256(payload).hexdigest(),
                'bytes':len(payload),'duration_s':6,'size':[480,270],'fps':10,'interpolated_frames':False,
                'source_video_sha256':'0'*64}})
        summary={'schema_version':1,'version':'v2','status':'accepted_local_media_publication_pending','scenes':scenes}
        (media/'accepted-v2-summary.json').write_text(json.dumps(summary),encoding='utf-8')
        self.manifest(['docs/media/highway.gif'])
        self.assertEqual(verify_allowlist(self.root)['status'],'fail')
        self.manifest([summary_name,'docs/media/highway.gif'])
        self.assertEqual(verify_allowlist(self.root)['status'],'pass')
        (media/'highway.gif').write_bytes(payload+b'changed')
        self.assertEqual(verify_allowlist(self.root)['status'],'fail')

    def test_v3_preview_requires_its_own_complete_summary_and_accepted_source(self):
        media=self.root/'docs/media';media.mkdir(parents=True)
        payload=b'GIF89a'+b'unit test v3 header fixture'
        (media/'highway-v3.gif').write_bytes(payload)
        summary_name='docs/media/accepted-v3-summary.json'
        scenes=[{'id':x} for x in ['highway','ramp-entry','ramp-exit','left-turn','roundabout']]
        highway=scenes[0]
        highway.update(requested_action_verified=True,full_internal_plan_completed=True,
            manual_review={'accepted':True},media=[{'role':'chase','sha256':'1'*64}],
            preview={'path':'docs/media/highway-v3.gif','sha256':hashlib.sha256(payload).hexdigest(),
                     'bytes':len(payload),'duration_s':6,'size':[480,270],'fps':10,
                     'interpolated_frames':False,'source_video_sha256':'1'*64})
        summary={'schema_version':1,'version':'v3','status':'accepted_local_media_publication_pending','scenes':scenes}
        def write_summary():
            (media/'accepted-v3-summary.json').write_text(json.dumps(summary),encoding='utf-8')
        write_summary();self.manifest([summary_name,'docs/media/highway-v3.gif'])
        self.assertEqual(verify_allowlist(self.root)['status'],'pass')
        highway['manual_review']['accepted']=False;write_summary()
        self.assertEqual(verify_allowlist(self.root)['status'],'fail')
        highway['manual_review']['accepted']=True
        highway['preview']['source_video_sha256']='2'*64;write_summary()
        self.assertEqual(verify_allowlist(self.root)['status'],'fail')
        highway['preview']['source_video_sha256']='1'*64
        summary['scenes']=scenes[:-1];write_summary()
        self.assertEqual(verify_allowlist(self.root)['status'],'fail')


if __name__ == "__main__":
    unittest.main()
