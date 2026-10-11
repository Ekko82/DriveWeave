# Runtime paths and portable integration

The project reads installation locations from the environment. Importing
`runtime_config` never opens CARLA, connects TraCI, changes a world, or ticks a
simulator. `RuntimeConfig.activate()` explicitly changes only the current
process environment and Python search paths.

| Variable | Purpose |
| --- | --- |
| `CARLA_ROOT` | CARLA 0.9.16 distribution containing CarlaUE4.exe or CarlaUE4.sh |
| `CARLA_SUMO_ROOT` | Official Co-Simulation/Sumo source; defaults to CARLA_ROOT/Co-Simulation/Sumo |
| `SUMO_HOME` | SUMO 1.28.0 distribution with bin/ and tools/; installed eclipse-sumo can be discovered if not supplied |
| `FFMPEG_BINARY` | Optional explicit encoder; IMAGEIO_FFMPEG_EXE is also supported; otherwise imageio-ffmpeg supplies its executable |

Run `python runtime_config.py --carla --sumo --bridge --gui --ffmpeg
--require-client --probe-binaries`. The probes run only --version/-version,
never a simulation. The matching client package is checked from package
metadata, without constructing a CARLA Client.

## Entry-point pattern

For scripts under `carla/`, add the project directory before importing the
helper. This also supports Windows embedded Python's isolated search path:

```python
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from runtime_config import RuntimeConfig

runtime = RuntimeConfig.resolve(require_sumo=True, require_bridge=True)
runtime.activate()
# runtime.sumo_home, runtime.carla_sumo_root, runtime.netconvert_binary
```

Resolve required runtime dependencies after argument parsing where possible,
so --help works before simulator installation. CPU processing should not
require a CARLA runtime; geometric/native API audits need only the matching
Python wheel and the source recording's map.

For a live capture parser, keep operator flags as explicit overrides:

```python
p.add_argument('--sumo-home', type=Path, default=None)
p.add_argument('--carla-sumo', type=Path, default=None)
# After parse_args(), before importing the official integration:
env = dict(os.environ)
if args.sumo_home is not None:
    env['SUMO_HOME'] = str(args.sumo_home)
if args.carla_sumo is not None:
    env['CARLA_SUMO_ROOT'] = str(args.carla_sumo)
runtime = RuntimeConfig.resolve(require_sumo=True, require_bridge=True, environ=env)
args.sumo_home = runtime.sumo_home
args.carla_sumo = runtime.carla_sumo_root
runtime.activate()
```

Use `runtime.sumo_gui_binary` instead of a Windows-only executable suffix.
The provided `hidden_process_options()` returns Windows SW_HIDE /
CREATE_NO_WINDOW flags only on Windows. A SUMO GUI screenshot still requires
a real graphical environment; Linux display/headless operation is unverified.
Do not replace it with a drawn road or claim a missing viewport was recorded.

For video entry points, explicitly resolve and activate
`require_ffmpeg=True` before creating the encoder. An invalid explicit
FFMPEG_BINARY fails instead of falling back silently to a different binary.

Map planners may locate official OpenDrive files under
`runtime.carla_root / 'CarlaUE4/Content/Carla/Maps/OpenDrive'` where present, or
consume a freshly exported map. Store relative source paths and hashes in
plans, with `source_root_env: CARLA_ROOT` for installation-relative sources.
Do not publish original runtime maps or installation inventories.

## Source publication checks

The release owner should call `open_source_check.require_clean(ROOT)` before
authentication, repository creation, commit or upload. The positive allowlist
does not by itself control previously tracked/staged files: refuse any tracked
or staged file outside it, or use a fresh source-only staging repository.
Never publish the complete working directory with `git add .`.

Portable paths and offline tests do not certify a new scene, controller or
video. Actual capture, process exit, source hashes, geometry and video audits
remain required for the selected release version.
