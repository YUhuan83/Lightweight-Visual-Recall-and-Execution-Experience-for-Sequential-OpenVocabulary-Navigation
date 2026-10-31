import unittest
from unittest.mock import patch
import kit


def item(version='1', deps=()):
    return dict(version=version, requires_dist=list(deps))


class ClosureTests(unittest.TestCase):
    def test_linux_not_windows(self):
        items = {'a': item(deps=['nvidia; sys_platform == "linux"', 'win; sys_platform == "win32"'])}
        self.assertEqual(kit.closure(items, ['a'])[0], ['nvidia'])

    def test_python310_not_host(self):
        items = {'a': item(deps=['old; python_version < "3.11"', 'new; python_version >= "3.11"'])}
        self.assertEqual(kit.closure(items, ['a'])[0], ['old'])

    def test_recursive_extras(self):
        items = {'a': item(deps=['b[fast]']), 'b': item(deps=['c; extra == "fast"']),
                 'c': item(deps=['uvloop; sys_platform != "win32"'])}
        self.assertEqual(kit.closure(items, ['a'])[0], ['uvloop'])

    def test_extra_not_requested(self):
        self.assertEqual(kit.closure({'a': item(deps=['b; extra == "test"'])}, ['a'])[0], [])

    def test_cycle_and_extra_union(self):
        items = {'a': item(deps=['b', 'b[fast]']), 'b': item(deps=['a', 'c; extra == "fast"'])}
        self.assertEqual(kit.closure(items, ['a'])[0], ['c'])

    def test_version_conflict(self):
        self.assertTrue(kit.closure({'a': item('2')}, ['a<2'])[1])

    def test_local_cuda_version(self):
        self.assertEqual(kit.closure({'torch': item('2.7.0+cu128')}, ['torch==2.7.0'])[1], [])

    def test_tags(self):
        from pip._vendor.packaging.tags import Tag
        self.assertIn(Tag('cp310', 'cp310', 'manylinux_2_28_x86_64'), kit.TAGS)
        self.assertIn(Tag('cp37', 'abi3', 'manylinux2014_x86_64'), kit.TAGS)
        self.assertNotIn(Tag('cp310', 'cp310', 'win_amd64'), kit.TAGS)
        self.assertNotIn(Tag('cp311', 'cp311', 'manylinux_2_28_x86_64'), kit.TAGS)

    def test_sanitize(self):
        text = kit.sanitize('https://user:<REDACTED_CREDENTIAL>@example.org/file?token=secret')
        self.assertNotIn('secret', text)

    def test_missing_and_hash_rejected(self):
        manifest = {'wheels': [dict(name='a', version='1', filename='a.whl', sha256='correct')]}
        with patch.object(kit.json, 'loads', return_value=manifest), patch.object(kit.Path, 'read_text', return_value='a==1 --hash=sha256:correct\n'):
            with patch.object(kit, 'inventory', return_value={}):
                with self.assertRaisesRegex(ValueError, 'inventory'):
                    kit.verify()
            with patch.object(kit, 'inventory', return_value={'a': dict(filename='a.whl')}), patch.object(kit, 'digest', return_value='corrupt'):
                with self.assertRaisesRegex(ValueError, 'Hash mismatch'):
                    kit.verify()


if __name__ == '__main__':
    unittest.main()
