"""Publication preflight contract tests; no real authentication or Git mutation."""
from pathlib import Path
import json
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import publish_release as publisher


class PublisherPreflightTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name).resolve()
        (self.root / "source.py").write_text("print('ok')\n", encoding="utf-8")
        (self.root / "OPEN_SOURCE_FILES.json").write_text(json.dumps({
            "schema_version": 1, "files": ["source.py", "OPEN_SOURCE_FILES.json"]}), encoding="utf-8")
        self.override = patch.object(publisher, "ROOT", self.root)
        self.override.start()

    def tearDown(self):
        self.override.stop()
        self.temp.cleanup()

    def git_responses(self, tracked=b"source.py\0", staged=b"", root=None):
        return [subprocess.CompletedProcess([], 0, str(root or self.root)+"\n", ""),
                subprocess.CompletedProcess([], 0, tracked, b""),
                subprocess.CompletedProcess([], 0, staged, b"")]

    def test_clean_new_directory_does_not_authenticate_or_initialize_git(self):
        with patch.object(publisher.subprocess, "run", return_value=
                subprocess.CompletedProcess([], 128, "", "not a repository")) as run:
            self.assertEqual(publisher.checked_source_files(), ["source.py", "OPEN_SOURCE_FILES.json"])
            self.assertEqual(run.call_count, 1)

    def test_existing_clean_index_passes_readonly(self):
        with patch.object(publisher.subprocess, "run", side_effect=self.git_responses()) as run:
            self.assertEqual(len(publisher.checked_source_files()), 2)
            self.assertEqual(run.call_count, 3)

    def test_tracked_outside_list_refused(self):
        with patch.object(publisher.subprocess, "run", side_effect=
                self.git_responses(tracked=b"source.py\0unreviewed.txt\0")):
            with self.assertRaisesRegex(ValueError, "outside source allowlist"):
                publisher.checked_source_files()

    def test_staged_outside_list_refused(self):
        with patch.object(publisher.subprocess, "run", side_effect=
                self.git_responses(staged=b"unreviewed.txt\0")):
            with self.assertRaisesRegex(ValueError, "outside source allowlist"):
                publisher.checked_source_files()

    def test_parent_git_repository_refused(self):
        with patch.object(publisher.subprocess, "run", side_effect=
                self.git_responses(root=self.root.parent)):
            with self.assertRaisesRegex(ValueError, "parent repository"):
                publisher.checked_source_files()

    def test_unclean_source_fails_before_git_or_credentials(self):
        local = "D" + ":/" + "Business" + "/local-only"
        (self.root / "source.py").write_text(repr(local), encoding="utf-8")
        with patch.object(publisher.subprocess, "run") as run:
            with self.assertRaisesRegex(ValueError, "Public-source check failed"):
                publisher.checked_source_files()
            run.assert_not_called()


if __name__ == "__main__":
    unittest.main()
