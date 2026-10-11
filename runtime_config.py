"""Explicit runtime discovery; importing this module never starts a simulator.

Only the standard library is required. CARLA's Python wheel is a client API,
not the Unreal runtime. All machine-specific paths come from the environment.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, asdict
from importlib import metadata, util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Mapping

CARLA_VERSION = "0.9.16"
SUMO_VERSION = "1.28.0"


class ConfigurationError(RuntimeError):
    pass


def _path(value: str | None) -> Path | None:
    return Path(os.path.expandvars(value)).expanduser().resolve() if value else None


def _binary(home: Path, name: str) -> Path:
    filename = name + (".exe" if os.name == "nt" else "")
    candidate = home / "bin" / filename
    if not candidate.is_file():
        raise ConfigurationError(f"Missing {name} executable: {candidate}")
    return candidate


def _sumo_package_root() -> Path | None:
    # find_spec locates the separately installed eclipse-sumo binary package;
    # it does not import a simulator API or create a connection.
    try:
        spec = util.find_spec("sumo")
    except (ImportError, ValueError):
        return None
    if spec and spec.submodule_search_locations:
        for location in spec.submodule_search_locations:
            candidate = Path(location).resolve()
            if (candidate / "bin").is_dir() and (candidate / "tools").is_dir():
                return candidate
    return None


@dataclass(frozen=True)
class RuntimeConfig:
    carla_root: Path | None = None
    carla_sumo_root: Path | None = None
    sumo_home: Path | None = None
    sumo_binary: Path | None = None
    sumo_gui_binary: Path | None = None
    netconvert_binary: Path | None = None
    ffmpeg_binary: Path | None = None

    @classmethod
    def resolve(cls, *, require_carla=False, require_sumo=False,
                require_bridge=False, require_gui=False, require_ffmpeg=False,
                environ: Mapping[str, str] | None = None) -> "RuntimeConfig":
        env = os.environ if environ is None else environ
        root = _path(env.get("CARLA_ROOT"))
        bridge = _path(env.get("CARLA_SUMO_ROOT"))
        if bridge is None and root is not None:
            bridge = root / "Co-Simulation" / "Sumo"
        home = _path(env.get("SUMO_HOME"))
        if home is None and (require_sumo or require_bridge or require_gui):
            home = _sumo_package_root()
            if home is None:
                executable = shutil.which("sumo", path=env.get("PATH", ""))
                if executable:
                    home = Path(executable).resolve().parent.parent
        if require_carla:
            if root is None:
                raise ConfigurationError("Set CARLA_ROOT to the matching CARLA 0.9.16 runtime directory.")
            if not any((root / name).is_file() for name in ("CarlaUE4.exe", "CarlaUE4.sh")):
                raise ConfigurationError(f"CARLA_ROOT has no CarlaUE4 launcher: {root}")
        if require_bridge:
            if bridge is None:
                raise ConfigurationError("Set CARLA_SUMO_ROOT, or CARLA_ROOT with Co-Simulation/Sumo available.")
            required = ("sumo_integration/bridge_helper.py", "run_synchronization.py",
                        "sumo_integration/sumo_simulation.py", "sumo_integration/carla_simulation.py",
                        "util/netconvert_carla.py")
            missing = [name for name in required if not (bridge / name).is_file()]
            if missing:
                raise ConfigurationError(f"Official CARLA/SUMO helpers missing under {bridge}: {missing}")
        sumo = gui = netconvert = None
        if require_sumo or require_bridge or require_gui:
            if home is None or not (home / "tools").is_dir():
                raise ConfigurationError("Set SUMO_HOME to a SUMO distribution with bin/ and tools/.")
            sumo, netconvert = _binary(home, "sumo"), _binary(home, "netconvert")
            if require_gui:
                gui = _binary(home, "sumo-gui")
        ffmpeg = None
        if require_ffmpeg:
            # An explicit invalid executable must fail rather than silently use
            # a different encoder. Otherwise use imageio-ffmpeg's bundled binary.
            explicit = env.get("FFMPEG_BINARY") or env.get("IMAGEIO_FFMPEG_EXE")
            if explicit:
                ffmpeg = _path(explicit)
            else:
                try:
                    import imageio_ffmpeg
                    ffmpeg = Path(imageio_ffmpeg.get_ffmpeg_exe()).resolve()
                except ImportError:
                    ffmpeg = _path(shutil.which("ffmpeg", path=env.get("PATH", "")))
            if ffmpeg is None or not ffmpeg.is_file():
                raise ConfigurationError("Install imageio-ffmpeg, or set FFMPEG_BINARY to an existing executable.")
        return cls(root, bridge, home, sumo, gui, netconvert, ffmpeg)

    def activate(self) -> None:
        """Opt-in process environment setup, without CARLA Client or world tick."""
        if self.sumo_home is not None:
            os.environ["SUMO_HOME"] = str(self.sumo_home)
            binary_dir = str(self.sumo_home / "bin")
            existing = os.environ.get("PATH", "").split(os.pathsep)
            if binary_dir not in existing:
                os.environ["PATH"] = os.pathsep.join([binary_dir, *existing])
            tools = str(self.sumo_home / "tools")
            if tools not in sys.path:
                sys.path.insert(0, tools)
        if self.carla_sumo_root is not None:
            bridge = str(self.carla_sumo_root)
            if bridge not in sys.path:
                sys.path.insert(0, bridge)
        if self.ffmpeg_binary is not None:
            os.environ["IMAGEIO_FFMPEG_EXE"] = str(self.ffmpeg_binary)

    def as_dict(self) -> dict:
        return {key: str(value) if value is not None else None
                for key, value in asdict(self).items()}


def installed_versions() -> dict:
    result = {}
    for package in ("carla", "numpy", "Pillow", "imageio-ffmpeg", "traci", "sumolib", "lxml"):
        try:
            result[package] = metadata.version(package)
        except metadata.PackageNotFoundError:
            result[package] = None
    return result


def hidden_process_options() -> dict:
    """Apply hidden Windows launch options only on Windows."""
    if os.name != "nt":
        return {}
    startup = subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = subprocess.SW_HIDE
    return {"startupinfo": startup, "creationflags": subprocess.CREATE_NO_WINDOW}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--carla", action="store_true", help="Check the native runtime launcher; do not launch it.")
    parser.add_argument("--sumo", action="store_true", help="Check SUMO and netconvert executables.")
    parser.add_argument("--bridge", action="store_true", help="Check official CARLA SUMO helper source files.")
    parser.add_argument("--gui", action="store_true", help="Also require the native SUMO-GUI executable.")
    parser.add_argument("--ffmpeg", action="store_true", help="Locate the actual encoder executable.")
    parser.add_argument("--require-client", action="store_true", help="Require installed CARLA API exactly 0.9.16.")
    parser.add_argument("--probe-binaries", action="store_true", help="Run only --version/-version, never a simulation.")
    args = parser.parse_args(argv)
    try:
        config = RuntimeConfig.resolve(require_carla=args.carla, require_sumo=args.sumo,
            require_bridge=args.bridge, require_gui=args.gui, require_ffmpeg=args.ffmpeg)
        versions = installed_versions()
        if args.require_client and versions["carla"] != CARLA_VERSION:
            raise ConfigurationError(f"Install the matching CARLA client explicitly: expected {CARLA_VERSION}, got {versions['carla']}")
        report = {"status": "paths_checked", "python": sys.version.split()[0],
                  "paths": config.as_dict(), "python_packages": versions,
                  "connected_to_simulator": False, "simulation_started": False}
        if args.probe_binaries:
            report["binary_versions"] = {}
            for name, executable, argument in (("sumo", config.sumo_binary, "--version"),
                  ("netconvert", config.netconvert_binary, "--version"),
                  ("ffmpeg", config.ffmpeg_binary, "-version")):
                if executable is not None:
                    result = subprocess.run([str(executable), argument], capture_output=True,
                        text=True, errors="replace", timeout=15, **hidden_process_options())
                    if result.returncode:
                        raise ConfigurationError(f"{name} version probe exited {result.returncode}")
                    report["binary_versions"][name] = (result.stdout or result.stderr).splitlines()[0]
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0
    except (ConfigurationError, OSError, subprocess.SubprocessError) as error:
        print(json.dumps({"status": "configuration_error", "error": str(error)}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
