"""Tests execute only stdlib and extracted AST, never model/ROS imports."""
import ast
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import prepare

SRC = prepare.DEFAULT_DEST
V = SRC / "visual_navigation/visual_navigation"
spec = importlib.util.spec_from_file_location("local_models", V / "utils/local_models.py")
models = importlib.util.module_from_spec(spec)
spec.loader.exec_module(models)


class PortTests(unittest.TestCase):
    def test_syntax_all_exported_python(self):
        paths = list(SRC.rglob("*.py")) + list(prepare.ROOT.glob("*.py"))
        for path in paths:
            compile(path.read_bytes(), str(path), "exec")
        self.assertGreater(len(paths), 100)

    def test_default_model_paths(self):
        root, model, adaptor = models.model_paths({"model_version": "c-radio_v3-b", "adaptor_version": None})
        self.assertEqual(root, Path.home() / "ckpts")
        self.assertEqual(model, "c-radio_v3-b")
        self.assertEqual(adaptor, root)
        root, model, adaptor = models.model_paths({"model_version": "radio", "adaptor_version": "siglip2"}, True)
        self.assertEqual(model, root / "radio")
        self.assertEqual(adaptor, root / "siglip2")

    def test_local_paths_and_failure(self):
        with tempfile.TemporaryDirectory(dir=prepare.ROOT) as folder:
            root = Path(folder)
            config = {"ckpt_root": folder, "model_version": "radio", "adaptor_version": "siglip2", "local_files_only": True}
            with self.assertRaises(FileNotFoundError):
                models.model_paths(config)
            (root / "radio").write_bytes(b"test, not weights")
            with self.assertRaises(FileNotFoundError):
                models.model_paths(config)
            (root / "siglip2_ms").mkdir()
            (root / "siglip2_ms/config.json").write_text("{}")
            self.assertEqual(models.model_paths(config), (root, root / "radio", root / "siglip2_ms"))
            config["adaptor_ckpt_path"] = str(root / "siglip2_ms")
            self.assertEqual(models.model_paths(config)[2], root / "siglip2_ms")
            config["adaptor_ckpt_path"] = str(root / "siglip2")
            self.assertEqual(models.model_paths(config)[2], root / "siglip2_ms")

    def test_siglip_explicit_local_branch(self):
        tree = ast.parse((SRC / "nvidia_radio/radio/siglip2_adaptor.py").read_bytes())
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and
                 any(k.arg == "local_files_only" and isinstance(k.value, ast.Constant) and k.value.value is True for k in n.keywords)]
        self.assertEqual(len(calls), 2)

    def test_frame_strip_actual_expression(self):
        tree = ast.parse((V / "explorfm_triangulation/obj_mask_triangulation.py").read_bytes())
        node = next(n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                    and n.func.attr == "lstrip")
        node.func.value = ast.Name(id="frame", ctx=ast.Load())
        expression = compile(ast.fix_missing_locations(ast.Expression(node)), "frame", "eval")
        for frame in ("camera_front", "/camera_front", "///camera_front", ""):
            self.assertEqual(eval(expression, {"frame": frame}), frame.lstrip("/"))

    def test_launch_defaults_and_contract(self):
        launch = (SRC / "visual_navigation/launch/wildos_launch.py").read_text()
        self.assertIn("default_value='wildos_nav_conf.yaml'", launch)
        self.assertIn("LaunchConfiguration('triangulation_config')", launch)
        tri = (V / "explorfm_triangulation/obj_mask_triangulation.py").read_text()
        self.assertIn('default="triangulation3d_objsearch_conf.yaml"', tri)
        conf = (SRC / "visual_navigation/configs/isolated_wildos.yaml").read_text()
        self.assertIn("cams_inverted: false", conf)
        self.assertIn("local_files_only: true", conf)
        self.assertNotIn("/compressed", conf)
        cmake = (SRC / "graphnav_mapper/CMakeLists.txt").read_text()
        self.assertIn("set(CMAKE_CXX_STANDARD 17)", cmake)

    def test_snapshot_excludes_and_includes_working_source(self):
        with tempfile.TemporaryDirectory(dir=prepare.ROOT) as folder:
            root = Path(folder)
            (root / "explorfm/.venv").mkdir(parents=True)
            (root / "explorfm/.venv/no.py").write_text("secret")
            (root / "explorfm/weights.pt").write_text("excluded")
            (root / "explorfm/local_untracked.py").write_text("value = 2")
            (root / "ckpts").mkdir()
            (root / "ckpts/no.py").write_text("excluded")
            files, _ = prepare.snapshot(root)
            self.assertEqual(list(files), ["explorfm/local_untracked.py"])

    def test_reparse_safety(self):
        # Windows junction creation needs no symlink privilege.
        with tempfile.TemporaryDirectory(dir=prepare.ROOT) as folder:
            root = Path(folder)
            (root / "explorfm").mkdir()
            target = root / "outside"
            target.mkdir()
            link = root / "explorfm/link"
            import os
            if os.name == "nt":
                subprocess.run(["cmd", "/c", "mklink", "/J", str(link), str(target)], check=True, capture_output=True)
            else:
                link.symlink_to(target, target_is_directory=True)
            try:
                with self.assertRaises(ValueError):
                    prepare.safe_path(link / "file.py")
                files, skipped = prepare.snapshot(root)
                self.assertEqual(files, {})
                self.assertIn(["explorfm/link", "link/reparse"], skipped)
            finally:
                if os.name == "nt":
                    link.rmdir()
                else:
                    link.unlink()

    def test_refuse_overwrite_and_source_destination(self):
        with self.assertRaises(ValueError):
            prepare.prepare(Path("C:/Users/Yu/nebula2-wildos"), SRC)
        with self.assertRaises(ValueError):
            prepare.prepare(SRC, SRC / "nested")

    def test_provenance_and_no_weights(self):
        manifest = json.loads((SRC.parent / (SRC.name + ".provenance.json")).read_text())
        self.assertTrue(manifest["source_dirty"])
        self.assertIn("explorfm/explorfm_model.py", manifest["source_status"])
        for rel in manifest["output_files"]:
            self.assertFalse(any(p.lower() in prepare.EXCLUDE for p in Path(rel).parts))
            self.assertNotIn(Path(rel).suffix, (".ckpt", ".pt", ".pth", ".safetensors"))
        rel = "explorfm/explorfm_model.py"
        self.assertEqual(prepare.digest((SRC / rel).read_bytes()), manifest["source_files"][rel]["sha256"])

    def test_reproduced_export_hashes(self):
        original = json.loads((SRC.parent / (SRC.name + ".provenance.json")).read_text())
        other_root = prepare.ROOT / "repro_check/src/nebula2-wildos"
        other = json.loads((other_root.parent / (other_root.name + ".provenance.json")).read_text())
        self.assertEqual(original["output_files"], other["output_files"])
        self.assertEqual(original["source_files"], other["source_files"])

    def test_patch_base_mismatch_rejected_before_writes(self):
        manifest = json.loads((SRC.parent / (SRC.name + ".provenance.json")).read_text())
        with tempfile.TemporaryDirectory(dir=prepare.ROOT) as folder:
            destination = Path(folder) / "new"
            source = Path(folder) / "source"
            source.mkdir()
            with patch.object(prepare, "snapshot", return_value=({}, [])), \
                    patch.object(prepare, "git", return_value=manifest["source_head"]):
                with self.assertRaisesRegex(ValueError, "Patch base mismatch"):
                    prepare.prepare(source, destination)
            self.assertFalse(destination.exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
