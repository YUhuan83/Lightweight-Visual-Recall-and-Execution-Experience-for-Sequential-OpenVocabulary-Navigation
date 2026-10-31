import importlib.util
import io
import pickle
import sys
import types
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent


def load_module():
    spec = importlib.util.spec_from_file_location("explorfm_heads_under_test", HERE / "explorfm_heads.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class StubUnpicklerTest(unittest.TestCase):
    def test_stub_resolves_missing_omegaconf_class(self):
        module = load_module()
        unpickler, shim = module.make_stub_unpickler()

        # Craft a pickle that references omegaconf.DictConfig, then remove the module
        # so a normal load would raise ModuleNotFoundError.
        fake = types.ModuleType("omegaconf")

        class DictConfig:
            def __init__(self):
                self.value = 7

        DictConfig.__module__ = "omegaconf"
        DictConfig.__qualname__ = "DictConfig"
        fake.DictConfig = DictConfig
        sys.modules["omegaconf"] = fake
        try:
            stream = io.BytesIO()
            pickle.dump(DictConfig(), stream)
        finally:
            del sys.modules["omegaconf"]
        stream.seek(0)

        # The test environment may have a real omegaconf installation. Hide it
        # explicitly so the normal unpickler follows the missing-module path.
        previous = sys.modules.get("omegaconf", ...)
        sys.modules["omegaconf"] = None
        try:
            with self.assertRaises(ModuleNotFoundError):
                pickle.load(io.BytesIO(stream.getvalue()))
        finally:
            if previous is ...:
                sys.modules.pop("omegaconf", None)
            else:
                sys.modules["omegaconf"] = previous

        stream.seek(0)
        obj = unpickler(stream).load()
        self.assertEqual(obj.__class__.__name__, "_Stub")

    def test_shim_load_matches_unpickler(self):
        module = load_module()
        _, shim = module.make_stub_unpickler()
        stream = io.BytesIO()
        pickle.dump({"a": [1, 2, 3]}, stream)
        stream.seek(0)
        self.assertEqual(shim.load(stream), {"a": [1, 2, 3]})


if __name__ == "__main__":
    unittest.main()
