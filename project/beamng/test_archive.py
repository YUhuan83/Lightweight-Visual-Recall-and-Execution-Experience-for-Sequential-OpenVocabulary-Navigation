import tempfile
from pathlib import Path
import unittest
import zipfile
from inspect_archive import inspect, unsafe_name


class ArchiveTests(unittest.TestCase):
    def test_paths(self):
        for name in ('../escape','root/../../bad','/absolute','C:/drive','..\\escape'):
            self.assertTrue(unsafe_name(name))
        self.assertFalse(unsafe_name('BeamNG/BinLinux/BeamNG.tech.x64'))

    def test_inventory_crc_no_extract(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'sample.zip'
            with zipfile.ZipFile(path,'w') as archive:
                archive.writestr('BeamNG/BinLinux/BeamNG.tech.x64',b'fixture-not-executable')
                archive.writestr('BeamNG/tech/docker/README.md',b'fixture')
            result = inspect(path,True)
            self.assertTrue(result['linux_binary_present'])
            self.assertFalse(result['runtime_verified'])
            self.assertEqual(result['crc_mode'],'passed_non_secret_entries')
            self.assertEqual(len(result['sha256']),64)
            self.assertEqual(list(Path(temp).iterdir()),[path])

    def test_skip_secret(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'sample.zip'
            with zipfile.ZipFile(path,'w') as archive:
                archive.writestr('tech.key',b'synthetic-not-a-key')
            result=inspect(path,True)
            self.assertEqual(result['crc_skipped_secret_files'],1)
            self.assertNotIn('sha256',result)

    def test_truncated_zip(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'sample.zip'
            path.write_bytes(b'PK truncated fixture')
            with self.assertRaises(zipfile.BadZipFile):
                inspect(path)


if __name__ == '__main__':
    unittest.main()
