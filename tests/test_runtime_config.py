"""Filesystem-only runtime discovery tests; no simulator connection or tick."""
from pathlib import Path
import os
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from runtime_config import ConfigurationError, RuntimeConfig


class RuntimeDiscoveryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def sumo_tree(self):
        home = self.root / "sumo"
        (home / "tools").mkdir(parents=True)
        (home / "bin").mkdir()
        for name in ("sumo", "netconvert", "sumo-gui"):
            (home / "bin" / (name + (".exe" if os.name == "nt" else ""))).touch()
        return home

    def carla_tree(self):
        root = self.root / "carla-runtime"
        root.mkdir()
        (root / ("CarlaUE4.exe" if os.name == "nt" else "CarlaUE4.sh")).touch()
        bridge = root / "Co-Simulation/Sumo"
        for name in ("sumo_integration/bridge_helper.py", "sumo_integration/sumo_simulation.py",
                     "sumo_integration/carla_simulation.py", "run_synchronization.py", "util/netconvert_carla.py"):
            path = bridge / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
        return root, bridge

    def test_cpu_configuration_needs_no_simulator(self):
        self.assertEqual(RuntimeConfig.resolve(environ={}).as_dict(), {
            "carla_root": None, "carla_sumo_root": None, "sumo_home": None,
            "sumo_binary": None, "sumo_gui_binary": None,
            "netconvert_binary": None, "ffmpeg_binary": None})

    def test_explicit_distribution_and_derived_bridge(self):
        home = self.sumo_tree()
        root, bridge = self.carla_tree()
        config = RuntimeConfig.resolve(require_carla=True, require_sumo=True,
            require_bridge=True, require_gui=True,
            environ={"CARLA_ROOT": str(root), "SUMO_HOME": str(home)})
        self.assertEqual(config.carla_sumo_root, bridge)
        self.assertTrue(config.sumo_gui_binary.is_file())

    def test_bridge_override_does_not_require_server_distribution(self):
        home = self.sumo_tree()
        _, bridge = self.carla_tree()
        config = RuntimeConfig.resolve(require_bridge=True,
            environ={"CARLA_SUMO_ROOT": str(bridge), "SUMO_HOME": str(home)})
        self.assertIsNone(config.carla_root)
        self.assertEqual(config.carla_sumo_root, bridge)

    def test_missing_gui_fails_instead_of_fabricating_viewport(self):
        home = self.sumo_tree()
        for path in (home / "bin").glob("sumo-gui*"):
            path.unlink()
        with self.assertRaises(ConfigurationError):
            RuntimeConfig.resolve(require_gui=True, environ={"SUMO_HOME": str(home)})

    def test_missing_runtime_and_explicit_encoder_fail(self):
        with self.assertRaises(ConfigurationError):
            RuntimeConfig.resolve(require_carla=True, environ={})
        with self.assertRaises(ConfigurationError):
            RuntimeConfig.resolve(require_ffmpeg=True,
                environ={"FFMPEG_BINARY": str(self.root / "missing-encoder")})

    def test_resolution_does_not_change_environment_or_sys_path(self):
        home = self.sumo_tree()
        before_env, before_path = dict(os.environ), list(sys.path)
        RuntimeConfig.resolve(require_sumo=True, environ={"SUMO_HOME": str(home)})
        self.assertEqual(dict(os.environ), before_env)
        self.assertEqual(sys.path, before_path)

    def test_missing_bridge_source_is_rejected(self):
        home = self.sumo_tree()
        root, bridge = self.carla_tree()
        (bridge / "run_synchronization.py").unlink()
        with self.assertRaises(ConfigurationError):
            RuntimeConfig.resolve(require_bridge=True,
                environ={"CARLA_ROOT": str(root), "SUMO_HOME": str(home)})


if __name__ == "__main__":
    unittest.main()
