"""Read-only prepublication check of a positive source-file allowlist.

Reports contain file names, line numbers and rule IDs, never matching values.
This is a targeted leak check, not proof that arbitrary code is secret-free.
It never reads directories outside the explicit project allowlist or runs Git.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import stat

ROOT = Path(__file__).resolve().parent
RULES = {
    "machine_specific_user_path": re.compile(r"(?i)(?:[A-Z]:[\\/]+(?:Business|Users)[\\/]+|/[U]sers/|/[h]ome/[^/\s\"'<>]+/)"),
    "github_access_token_literal": re.compile(r"(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{40,})"),
    "openai_key_literal": re.compile(r"sk-(?:proj-)?[A-Za-z0-9_-]{32,}"),
    "private_key_block": re.compile(r"-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----"),
    "aws_access_key_literal": re.compile(r"(?:AKIA|ASIA)[A-Z0-9]{16}"),
    "embedded_password_literal": re.compile(r"(?i)(?:password|api[_-]?key|secret[_-]?key)\s*[=:]\s*[\"'][^\"'\n]{12,}[\"']"),
}
TEXT_SUFFIXES = {".py", ".md", ".txt", ".json", ".toml", ".yaml", ".yml"}
FORBIDDEN_PARTS = {".git", ".venv", "venv", "private", "credentials", "captures",
                   "live-captures", "processed", "sumo-replay", "executions", "research",
                   "captures-v2", "live-captures-v2", "processed-v2", "sumo-replay-v2", "v2-probes",
                   "captures-v3", "processed-v3", "output-v3", "v3-runtime"}


def accepted_preview(root: Path, relative: str, payload: bytes, files: list) -> bool:
    """Permit only explicitly listed, hash-bound small README GIFs.

    Raw media, MP4s and runtime assets remain excluded. The generated sanitized
    versioned scene summary must itself be in the text allowlist, so it is scanned.
    """
    version = 'v3' if relative.endswith('-v3.gif') else 'v2'
    summary_name = 'docs/media/accepted-' + version + '-summary.json'
    if summary_name not in files or not relative.startswith('docs/media/'):
        return False
    if not payload.startswith((b'GIF87a', b'GIF89a')):
        return False
    summary = json.loads(allowed_path(root, summary_name).read_text(encoding='utf-8'))
    if summary.get('schema_version') != 1 or summary.get('version') != version or summary.get('status') not in {
            'accepted_local_media_publication_pending', 'published_verified'}:
        return False
    scenes = summary.get('scenes', [])
    ids = {'highway', 'ramp-entry', 'ramp-exit', 'left-turn', 'roundabout', 'live-highway'}
    if version == 'v3':
        ids.remove('live-highway')
    if len(scenes) != len(ids) or {s.get('id') for s in scenes} != ids:
        return False
    for scene in scenes:
        preview = scene.get('preview', {})
        if preview.get('path') != relative:
            continue
        role = 'chase' if version == 'v3' else 'carla-sumo-four-pane'
        source = next((m for m in scene.get('media', []) if m.get('role') == role), {})
        return (preview.get('sha256') == hashlib.sha256(payload).hexdigest()
                and preview.get('bytes') == len(payload) and preview.get('duration_s') == 6
                and preview.get('size') == [480, 270] and preview.get('fps') == 10
                and preview.get('interpolated_frames') is False
                and preview.get('source_video_sha256') == source.get('sha256')
                and scene.get('requested_action_verified') is True
                and scene.get('full_internal_plan_completed') is True
                and scene.get('manual_review', {}).get('accepted') is True)
    return False


def allowed_path(root: Path, value: str) -> Path:
    if not isinstance(value, str) or not value or "\\" in value or ":" in value:
        raise ValueError("Allowlist requires literal relative POSIX paths")
    relative = PurePosixPath(value)
    if relative.is_absolute() or any(part in {"", ".", ".."} for part in value.split("/")):
        raise ValueError("Unsafe relative allowlist path")
    if any(part.lower() in FORBIDDEN_PARTS for part in relative.parts):
        raise ValueError("Private/generated directory cannot be published as source")
    path = root.joinpath(*relative.parts)
    for parent in (path, *path.parents):
        if parent == root.parent:
            break
        if parent.exists():
            status = parent.lstat()
            if stat.S_ISLNK(status.st_mode) or getattr(status, "st_file_attributes", 0) & 0x400:
                raise ValueError("Symlink or Windows reparse point refused")
        if parent == root:
            break
    if not path.resolve().is_relative_to(root.resolve()):
        raise ValueError("Allowlist path escaped project")
    return path


def verify_allowlist(root: Path = ROOT, manifest_path: Path | None = None) -> dict:
    root = Path(root).resolve()
    manifest = Path(manifest_path) if manifest_path else root / "OPEN_SOURCE_FILES.json"
    data = json.loads(manifest.read_text(encoding="utf-8"))
    files = data.get("files")
    if data.get("schema_version") != 1 or not isinstance(files, list) or not files:
        raise ValueError("Missing or unsupported positive file allowlist")
    if len(files) != len(set(files)):
        raise ValueError("Duplicate allowlist entries")
    findings, hashes = [], {}
    for relative in files:
        try:
            path = allowed_path(root, relative)
            if not path.is_file():
                findings.append({"file": relative, "line": None, "rule": "missing_file"})
                continue
            if path.stat().st_size > 4 * 1024 * 1024:
                findings.append({"file": relative, "line": None, "rule": "source_file_over_4MiB"})
                continue
            payload = path.read_bytes()
            if path.suffix.lower() == '.gif' and accepted_preview(root, relative, payload, files):
                hashes[relative] = hashlib.sha256(payload).hexdigest()
                continue
            if path.suffix.lower() not in TEXT_SUFFIXES and path.name not in {"LICENSE", ".gitignore"}:
                findings.append({"file": relative, "line": None, "rule": "non_source_file"})
                continue
            hashes[relative] = hashlib.sha256(payload).hexdigest()
            text = payload.decode("utf-8-sig")
            for number, line in enumerate(text.splitlines(), 1):
                for name, pattern in RULES.items():
                    if pattern.search(line):
                        findings.append({"file": relative, "line": number, "rule": name})
        except (OSError, UnicodeError, ValueError):
            findings.append({"file": relative, "line": None, "rule": "invalid_or_unreadable_path"})
    return {"schema_version": 1, "status": "pass" if not findings else "fail",
            "checked_files": len(files), "findings": findings,
            "source_sha256": hashes,
            "allowlist_sha256": hashlib.sha256(manifest.read_bytes()).hexdigest(),
            "matched_values_disclosed": False, "git_or_network_used": False,
            "scope": "Explicit source allowlist only; neither private folders nor credential stores read."}


def require_clean(root: Path = ROOT) -> dict:
    report = verify_allowlist(root)
    if report["status"] != "pass":
        raise ValueError("Public-source check failed; review file/line/rule findings with open_source_check.py")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    report = verify_allowlist(manifest_path=args.manifest)
    if args.report:
        target = args.report.resolve()
        if not target.is_relative_to(ROOT / "maintenance" / "reports") or target.suffix != ".json":
            raise ValueError('Local reports must be JSON under maintenance/reports; source overwrite refused')
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key != "source_sha256"}, ensure_ascii=False))
    return 0 if report["status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
