"""A native map may differ only by the exactly hashed CRLF/LF transform."""
from pathlib import Path
import hashlib
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from audit_native_actions_newline_bound import binding


def digest(data):
    return hashlib.sha256(data).hexdigest()


class NativeSourceBinding(unittest.TestCase):
    def test_exact_bytes_pass_without_adaptation(self):
        data=b'<OpenDRIVE>\nroad</OpenDRIVE>\n'
        result=binding(data,digest(data))
        self.assertTrue(result['passed']);self.assertEqual(result['mode'],'exact_bytes')
    def test_actual_windows_newline_only_has_two_explicit_hashes(self):
        source=b'<OpenDRIVE>\nroad</OpenDRIVE>\n';exported=source.replace(b'\n',b'\r\n')
        result=binding(exported,digest(source))
        self.assertTrue(result['passed']);self.assertNotEqual(result['actual_exported_map_byte_sha256'],result['original_declared_native_map_sha256'])
        self.assertEqual(result['actual_exported_map_crlf_to_lf_sha256'],digest(source))
    def test_changed_native_road_rejects_even_with_windows_newlines(self):
        self.assertFalse(binding(b'<road id="2">\r\n',digest(b'<road id="1">\n'))['passed'])
    def test_whitespace_reformatting_is_not_allowed(self):
        self.assertFalse(binding(b'<road> </road>\r\n',digest(b'<road></road>\n'))['passed'])
    def test_standalone_carriage_return_is_not_normalized(self):
        self.assertFalse(binding(b'<road>\r',digest(b'<road>\n'))['passed'])
    def test_missing_declared_source_sha_cannot_pass(self):
        self.assertFalse(binding(b'<road>\r\n',None)['passed'])


if __name__=='__main__':
    unittest.main()
