"""Read-only sampled 3D vehicle body audit; no scenario completion claim.

Uses the independently geometry-tested exact convex OBB implementation from
audit_native_actions. Native pitch, roll, yaw, bbox offset and elevation apply.
No CARLA client, world connection, tick or capture edits are performed.
"""
from pathlib import Path
import argparse
import itertools
import json
import math
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from audit_native_actions import file_sha, obb, obb_signed_clearance


def audit(run, execution, out):
    run = run.resolve()
    manifest = json.loads((run / "manifest.json").read_text(encoding="utf-8"))
    frames = [json.loads(line) for line in (run / "frames.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
    collisions = json.loads((run / "collisions.json").read_text(encoding="utf-8"))
    execution = execution.resolve()
    actual_exit = json.loads(execution.read_text(encoding="utf-8"))
    minimum = None
    ego_minimum = None
    pairs = {}
    overlaps = []
    invalid = []
    comparisons = 0
    dt = float(manifest["fixed_delta_seconds"])
    for index, frame in enumerate(frames):
        vehicles = frame["vehicles"]
        boxes = {v["id"]: obb(v) for v in vehicles}
        for vehicle in vehicles:
            box = boxes[vehicle["id"]]
            if not all(math.isfinite(float(value)) for value in box["vertices"].ravel()) or min(box["extent"]) <= 0:
                invalid.append({"frame": frame["frame"], "actor": vehicle["id"]})
        for first, second in itertools.combinations(vehicles, 2):
            comparisons += 1
            clearance = obb_signed_clearance(boxes[first["id"]], boxes[second["id"]])
            detail = {"signed_clearance_m": clearance, "sample": index, "frame": frame["frame"],
                      "time_s": frame["time_s"], "actor_ids": [first["id"], second["id"]],
                      "roles": [first.get("role_name"), second.get("role_name")],
                      "locations_m": [first["transform"]["location_m"], second["transform"]["location_m"]],
                      "rotations_deg": [first["transform"]["rotation_deg"], second["transform"]["rotation_deg"]]}
            if minimum is None or clearance < minimum["signed_clearance_m"]:
                minimum = detail
            if (first.get("ego") or second.get("ego")) and (ego_minimum is None or clearance < ego_minimum["signed_clearance_m"]):
                ego_minimum = detail
            key = "-".join(str(actor) for actor in sorted((first["id"], second["id"])))
            if key not in pairs or clearance < pairs[key]["signed_clearance_m"]:
                pairs[key] = detail
            if clearance < -0.005:
                overlaps.append(detail)
    ranges = []
    for item in overlaps:
        if ranges and ranges[-1]["actor_ids"] == item["actor_ids"] and ranges[-1]["last_frame"] + 1 == item["frame"]:
            ranges[-1]["last_frame"] = item["frame"]
            ranges[-1]["last_time_s"] = item["time_s"]
            ranges[-1]["samples"] += 1
        else:
            ranges.append({"actor_ids": item["actor_ids"], "roles": item["roles"],
                           "first_frame": item["frame"], "last_frame": item["frame"],
                           "first_time_s": item["time_s"], "last_time_s": item["time_s"], "samples": 1})
    hashes = {name: file_sha(run / name) for name in ("manifest.json", "frames.jsonl", "collisions.json")}
    checks = {
        "frames_present_and_count_matches": len(frames) == manifest["frames_captured"] and len(frames) > 0,
        "consecutive_actual_native_frames": all(b["frame"] == a["frame"] + 1 and abs(b["timestamp"] - a["timestamp"] - dt) < .002 for a, b in zip(frames, frames[1:])),
        "valid_native_vehicle_boxes": not invalid,
        "all_vehicle_pairs_tested_each_recorded_frame": comparisons == sum(len(frame["vehicles"]) * (len(frame["vehicles"]) - 1) // 2 for frame in frames),
        "sampled_all_vehicle_bodies_have_at_least_0_05m_clearance": minimum is not None and minimum["signed_clearance_m"] >= .05,
        "no_sampled_vehicle_obb_overlap": not overlaps,
        "native_ego_collision_records_zero": isinstance(collisions, list) and len(collisions) == 0 and manifest.get("collision_count") == 0,
        "actual_capture_process_exit_zero": actual_exit.get("returncode") == 0 and actual_exit.get("native_process_returncode", 0) == 0,
        "execution_matches_current_raw_manifest_and_frames": actual_exit.get("manifest_sha256") == hashes["manifest.json"] and actual_exit.get("frames_jsonl_sha256") == hashes["frames.jsonl"] and Path(actual_exit["capture_run"]).resolve() == run,
        "native_cleanup_completed": manifest.get("native_cleanup_complete") is True,
    }
    report = {"run": str(run), "scenario": manifest["scenario"], "map": manifest["map"],
              "sampled_body_separation_validated": all(checks.values()), "checks": checks,
              "failed_checks": [name for name, good in checks.items() if not good],
              "frames": len(frames), "recorded_duration_s": len(frames) * dt,
              "pair_comparisons": comparisons, "all_vehicle_minimum": minimum, "ego_minimum": ego_minimum,
              "per_pair_minimum": list(pairs.values()), "overlap_samples": len(overlaps),
              "overlap_ranges": ranges, "overlap_examples": overlaps[:20], "invalid_box_examples": invalid[:10],
              "input_sha256": hashes, "execution_file": str(execution), "execution_sha256": file_sha(execution),
              "geometry_source_sha256": file_sha(Path(__file__).parent / "audit_native_actions.py"),
              "auditor_sha256": file_sha(Path(__file__)),
              "geometry_validation": str(Path(__file__).parent / "plans/native-action-auditor-geometry-verification.json"),
              "method": "All native recorded vehicle pairs, transformed native 3D bounding boxes including local center offset, pitch/roll/yaw and height; 15 SAT axes, vertex-box and edge-edge feature distances. Actual child process exit and raw snapshot hashes checked.",
              "limitations": ["Only recorded native times are tested; no continuous-time clearance guarantee.",
                              "Native bounding boxes conservatively enclose vehicle meshes; this is not mesh collision reconstruction.",
                              "Native collision sensor covers ego contacts; sampled boxes additionally cover all recorded vehicle pairs.",
                              "This report only validates body separation and capture provenance. It does not accept an incomplete controller plan or prove scene-specific action completion."],
              "raw_capture_modified": False, "server_connected_or_ticked": False}
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({"report": str(out), "sampled_body_separation_validated": report["sampled_body_separation_validated"],
                      "minimum_all_vehicle_3d_obb_clearance_m": minimum["signed_clearance_m"] if minimum else None,
                      "minimum_ego_3d_obb_clearance_m": ego_minimum["signed_clearance_m"] if ego_minimum else None,
                      "overlap_samples": len(overlaps), "failed_checks": report["failed_checks"]}, indent=2))
    return report["sampled_body_separation_validated"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--execution", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    raise SystemExit(0 if audit(args.run, args.execution, args.out) else 2)
