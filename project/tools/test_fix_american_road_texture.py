import contextlib
import hashlib
import io
from pathlib import Path
import tempfile
import unittest
import zipfile

from fix_american_road_texture import NEW, OLD, patch_archive


class TexturePatchTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.source = Path(self.temp.name) / "original.zip"
        self.output = Path(self.temp.name) / "patched.zip"
        self.json = b'{"eca_bld_woodcladding_01_bare_mat":{"detailMap":"' + OLD + b'"}}\n'
        self.cs = b'// retain comment\nsingleton Material(test) { detailMap = "' + OLD + b'"; };\n'
        self.stdout = io.StringIO()

    def create(self, target=True, stale=True):
        with zipfile.ZipFile(self.source, "w", zipfile.ZIP_DEFLATED) as archive:
            archive.comment = b"archive comment"
            info = zipfile.ZipInfo("levels/american_road/main.materials.json", (2020, 1, 2, 3, 4, 6))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o100644 << 16
            info.comment = b"member comment"
            info.extra = b"\xfe\xca\x02\x00ab"
            archive.writestr(info, self.json if stale else self.json.replace(OLD, NEW))
            archive.writestr("materials.cs", self.cs if stale else self.cs.replace(OLD, NEW))
            archive.writestr("unrelated.bin", OLD + bytes(range(256)) * 100)
            archive.writestr("grid.materials.json", b'{"Grid512_Red_Mat":{"mapTo":"512_red"}}')
            archive.writestr("empty/", b"")
            if target:
                archive.writestr(NEW.decode(), b"synthetic DDS")

    def run_patch(self, apply=False, output=None):
        with contextlib.redirect_stdout(self.stdout):
            return patch_archive(self.source, output or self.output, apply=apply)

    def test_dry_run_default(self):
        self.create()
        before = self.source.read_bytes()
        self.assertEqual(self.run_patch(), 2)
        self.assertFalse(self.output.exists())
        self.assertEqual(self.source.read_bytes(), before)
        self.assertIn("eca_bld_woodcladding_01_bare_mat", self.stdout.getvalue())
        self.assertIn("REPORT ONLY Grid512", self.stdout.getvalue())

    def test_apply_contents_metadata_crc_and_original_hash(self):
        self.create()
        before = hashlib.sha256(self.source.read_bytes()).digest()
        self.assertEqual(self.run_patch(apply=True), 2)
        self.assertEqual(hashlib.sha256(self.source.read_bytes()).digest(), before)
        with zipfile.ZipFile(self.source) as original, zipfile.ZipFile(self.output) as patched:
            self.assertIsNone(patched.testzip())
            self.assertEqual(original.comment, patched.comment)
            self.assertEqual(original.namelist(), patched.namelist())
            for info in original.infolist():
                actual = patched.getinfo(info.filename)
                for field in ("date_time", "compress_type", "external_attr", "internal_attr",
                              "comment", "extra", "create_system", "create_version"):
                    self.assertEqual(getattr(info, field), getattr(actual, field), field)
                data = original.read(info)
                if info.filename.endswith((".json", ".cs")):
                    self.assertEqual(patched.read(info.filename), data.replace(OLD, NEW))
                    if OLD in data:
                        self.assertNotEqual(actual.CRC, info.CRC)
                else:
                    self.assertEqual(patched.read(info.filename), data)

    def test_source_cannot_be_output(self):
        self.create()
        before = self.source.read_bytes()
        with self.assertRaises(ValueError):
            self.run_patch(apply=True, output=self.source)
        self.assertEqual(self.source.read_bytes(), before)

    def test_existing_output_rejected_in_both_modes(self):
        self.create()
        self.output.write_bytes(b"do not overwrite")
        for apply in (False, True):
            with self.assertRaises(ValueError):
                self.run_patch(apply=apply)
            self.assertEqual(self.output.read_bytes(), b"do not overwrite")

    def test_missing_target_rejected(self):
        self.create(target=False)
        with self.assertRaisesRegex(ValueError, "target is missing"):
            self.run_patch(apply=True)
        self.assertFalse(self.output.exists())

    def test_no_matches_no_output(self):
        self.create(stale=False)
        self.assertEqual(self.run_patch(apply=True), 0)
        self.assertFalse(self.output.exists())


if __name__ == "__main__":
    unittest.main()
