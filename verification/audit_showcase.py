"""Read-only acceptance checks for genuine SUMO-driven 3D video.

No simulator or renderer is launched. This independently checks stored TraCI
states, Blender world-space pose logs, recorded simulation events and final MP4.
Missing evidence is reported as missing, never silently treated as a pass.
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def wrapped_angle(value):
    return (value + math.pi) % (2 * math.pi) - math.pi


def corners(vehicle):
    half = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]], dtype=float)
    length = vehicle.get("model_length_m", vehicle["length_m"])
    width = vehicle.get("model_width_m", vehicle["width_m"])
    half *= np.array([length / 2, width / 2])
    yaw = vehicle["yaw_rad"]
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    return half @ rotation.T + np.array([vehicle["x"], vehicle["y"]])


def overlap_depth(polygon_a, polygon_b):
    """Separating axis test for convex XY polygons; returns signed overlap."""
    minimum = float("inf")
    for polygon in (polygon_a, polygon_b):
        edges = np.roll(polygon, -1, axis=0) - polygon
        for edge in edges:
            axis = np.array([-edge[1], edge[0]])
            norm = float(np.linalg.norm(axis))
            if norm < 1e-10:
                continue
            axis /= norm
            projection_a = polygon_a @ axis
            projection_b = polygon_b @ axis
            penetration = min(float(projection_a.max()), float(projection_b.max())) - max(
                float(projection_a.min()), float(projection_b.min()))
            if penetration < 0:
                return penetration
            minimum = min(minimum, penetration)
    return minimum


def read_frames(path):
    if path.suffix.lower() == ".jsonl":
        return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    document = json.loads(path.read_text(encoding="utf-8"))
    return document["frames"]


def add_check(report, name, passed, details=None, severity="error"):
    report["checks"][name] = {"status": "pass" if passed else severity,
                              "details": details}


def missing(report, name, reason):
    report["checks"][name] = {"status": "missing", "details": reason}


def evidence_hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def trajectory_checks(frames, report, fps):
    errors = []
    conversion_errors = []
    jumps = []
    heading_jumps = []
    acceleration_outliers = []
    previous = {}
    frame_ids = [frame["frame"] for frame in frames]
    timestamps = [frame["time_s"] for frame in frames]
    add_check(report, "frame_sequence", frame_ids == list(range(frame_ids[0], frame_ids[0] + len(frame_ids))),
              {"count": len(frames), "first": frame_ids[0], "last": frame_ids[-1]})
    timing_error = max(abs(t - (timestamps[0] + index / fps)) for index, t in enumerate(timestamps))
    add_check(report, "presentation_timestamps", timing_error <= 1e-6,
              {"fps": fps, "maximum_error_s": timing_error})
    simulation_origin = frames[0]["sim_time_s"] - frames[0]["time_s"]
    max_sim_time_offset = max(abs((frame["sim_time_s"] - frame["time_s"]) - simulation_origin) for frame in frames)
    add_check(report, "simulation_timestamps", max_sim_time_offset <= 0.0011,
              {"presentation_origin_sim_time_s": simulation_origin,
               "maximum_spacing_error_s": max_sim_time_offset, "tolerance_s": 0.0011})
    maxima = {"center_error_m": 0.0, "yaw_error_rad": 0.0,
              "step_displacement_m": 0.0, "yaw_rate_deg_s": 0.0}
    ego_records = []
    for frame in frames:
        ids = [v["id"] for v in frame["vehicles"]]
        if len(ids) != len(set(ids)):
            errors.append({"frame": frame["frame"], "reason": "duplicate vehicle id"})
        for vehicle in frame["vehicles"]:
            if vehicle.get("ego"):
                ego_records.append((frame["time_s"], vehicle))
            required = ["x", "y", "z", "yaw_rad", "length_m", "width_m", "height_m",
                        "speed_mps", "sumo_front_x", "sumo_front_y", "sumo_angle_deg"]
            if not all(key in vehicle for key in required):
                errors.append({"frame": frame["frame"], "id": vehicle["id"], "reason": "missing required field"})
                continue
            if not all(math.isfinite(float(vehicle[key])) for key in required):
                errors.append({"frame": frame["frame"], "id": vehicle["id"], "reason": "non-finite state"})
                continue
            yaw = wrapped_angle(math.radians(90 - vehicle["sumo_angle_deg"]))
            expected_x = vehicle["sumo_front_x"] - math.cos(yaw) * vehicle["length_m"] / 2
            expected_y = vehicle["sumo_front_y"] - math.sin(yaw) * vehicle["length_m"] / 2
            center_error = math.hypot(vehicle["x"] - expected_x, vehicle["y"] - expected_y)
            yaw_error = abs(wrapped_angle(vehicle["yaw_rad"] - yaw))
            maxima["center_error_m"] = max(maxima["center_error_m"], center_error)
            maxima["yaw_error_rad"] = max(maxima["yaw_error_rad"], yaw_error)
            if center_error > 1e-4 or yaw_error > 1e-6:
                conversion_errors.append({"frame": frame["frame"], "id": vehicle["id"],
                                          "center_error_m": center_error, "yaw_error_rad": yaw_error})
            if min(vehicle["length_m"], vehicle["width_m"], vehicle["height_m"]) <= 0:
                errors.append({"frame": frame["frame"], "id": vehicle["id"], "reason": "non-positive vehicle dimension"})
            if not -0.01 <= vehicle["speed_mps"] <= 60:
                errors.append({"frame": frame["frame"], "id": vehicle["id"], "reason": "out-of-range speed"})
            if abs(vehicle.get("acceleration_mps2", 0.0)) > 12:
                acceleration_outliers.append({"frame": frame["frame"], "id": vehicle["id"],
                                              "acceleration_mps2": vehicle["acceleration_mps2"]})
            old = previous.get(vehicle["id"])
            if old:
                old_frame, old_vehicle = old
                dt = frame["time_s"] - old_frame["time_s"]
                if dt > 0 and frame["frame"] == old_frame["frame"] + 1:
                    # Front-bumper displacement is not distorted by the
                    # center-offset rotation during a curved left turn.
                    displacement = math.hypot(vehicle["sumo_front_x"] - old_vehicle["sumo_front_x"],
                                              vehicle["sumo_front_y"] - old_vehicle["sumo_front_y"])
                    allowance = (max(vehicle["speed_mps"], old_vehicle["speed_mps"]) + 3.0) * dt + 0.08
                    maxima["step_displacement_m"] = max(maxima["step_displacement_m"], displacement)
                    if displacement > allowance:
                        jumps.append({"frame": frame["frame"], "id": vehicle["id"],
                                      "displacement_m": displacement, "allowance_m": allowance})
                    yaw_rate = math.degrees(abs(wrapped_angle(vehicle["yaw_rad"] - old_vehicle["yaw_rad"]))) / dt
                    maxima["yaw_rate_deg_s"] = max(maxima["yaw_rate_deg_s"], yaw_rate)
                    if yaw_rate > 90:
                        heading_jumps.append({"frame": frame["frame"], "id": vehicle["id"], "yaw_rate_deg_s": yaw_rate})
            previous[vehicle["id"]] = (frame, vehicle)
    add_check(report, "finite_valid_vehicle_states", not errors, errors[:30])
    add_check(report, "sumo_front_to_center_and_yaw", not conversion_errors,
              {"maxima": maxima, "outliers": conversion_errors[:30]})
    add_check(report, "no_position_jumps", not jumps, jumps[:30])
    add_check(report, "no_heading_jumps", not heading_jumps, heading_jumps[:30])
    add_check(report, "comfortable_acceleration", not acceleration_outliers,
              acceleration_outliers[:20], severity="warning")
    return ego_records


def collision_checks(frames, report, obstacles=None, label="sumo", penetration_tolerance=0.05):
    overlaps = []
    obstacle_overlaps = []
    obstacle_geometry = []
    obstacle_boxes = []
    if obstacles:
        for obstacle in obstacles:
            polygon = np.asarray(obstacle.get("polygon_xy", obstacle.get("polygon")), dtype=float)
            obstacle_geometry.append(polygon)
            obstacle_boxes.append([polygon[:, 0].min(), polygon[:, 1].min(), polygon[:, 0].max(), polygon[:, 1].max()])
    obstacle_boxes = np.asarray(obstacle_boxes, dtype=float)
    for frame in frames:
        vehicles = frame["vehicles"]
        polygons = [corners(v) for v in vehicles]
        for i, vehicle in enumerate(vehicles):
            for j in range(i + 1, len(vehicles)):
                other = vehicles[j]
                # Fast sphere rejection avoids most OBB tests.
                radius = (math.hypot(vehicle.get("model_length_m", vehicle["length_m"]),
                                    vehicle.get("model_width_m", vehicle["width_m"])) +
                          math.hypot(other.get("model_length_m", other["length_m"]),
                                     other.get("model_width_m", other["width_m"]))) / 2
                if math.hypot(vehicle["x"] - other["x"], vehicle["y"] - other["y"]) > radius:
                    continue
                # The established vehicle root is at tyre-ground contact z.
                if vehicle["z"] + vehicle.get("model_height_m", vehicle["height_m"]) < other["z"] or other["z"] + other.get("model_height_m", other["height_m"]) < vehicle["z"]:
                    continue
                overlap = overlap_depth(polygons[i], polygons[j])
                if overlap > penetration_tolerance:
                    overlaps.append({"frame": frame["frame"], "time_s": frame["time_s"],
                                     "vehicle_a": vehicle["id"], "vehicle_b": other["id"],
                                     "overlap_m": overlap})
            if obstacles:
                bounds = [polygons[i][:, 0].min(), polygons[i][:, 1].min(), polygons[i][:, 0].max(), polygons[i][:, 1].max()]
                nearby = np.flatnonzero((obstacle_boxes[:, 0] <= bounds[2]) & (obstacle_boxes[:, 2] >= bounds[0]) &
                                        (obstacle_boxes[:, 1] <= bounds[3]) & (obstacle_boxes[:, 3] >= bounds[1]))
                for obstacle_index in nearby:
                    obstacle = obstacles[obstacle_index]
                    if vehicle["z"] + vehicle.get("model_height_m", vehicle["height_m"]) < obstacle.get("z_min", 0):
                        continue
                    if vehicle["z"] > obstacle.get("z_max", 100):
                        continue
                    penetration = overlap_depth(polygons[i], obstacle_geometry[obstacle_index])
                    if penetration > penetration_tolerance:
                        obstacle_overlaps.append({"frame": frame["frame"], "id": vehicle["id"],
                                                  "obstacle": obstacle["name"], "overlap_m": penetration})
    add_check(report, f"{label}_vehicle_obb_no_overlap", not overlaps,
              {"penetration_tolerance_m": penetration_tolerance, "overlap_count": len(overlaps),
               "first_overlaps": overlaps[:30]})
    if obstacles:
        add_check(report, f"{label}_obstacle_footprints", not obstacle_overlaps,
                  {"obstacles": len(obstacles), "overlap_count": len(obstacle_overlaps),
                   "first_overlaps": obstacle_overlaps[:30]})
    elif obstacles is not None:
        missing(report, f"{label}_obstacle_footprints", "No physical obstacle footprints were registered; no obstacle intersection test performed")


def pose_checks(trajectory, rendered, report, render_offset_m=(0.0, 0.0, 0.0)):
    expected_ids = [frame["frame"] for frame in trajectory]
    rendered_ids = [frame["frame"] for frame in rendered]
    add_check(report, "render_source_frame_sequence", rendered_ids == expected_ids,
              {"expected_count": len(expected_ids), "rendered_count": len(rendered_ids),
               "expected_first_last": [expected_ids[0], expected_ids[-1]] if expected_ids else [],
               "rendered_first_last": [rendered_ids[0], rendered_ids[-1]] if rendered_ids else []})
    indexed = {frame["frame"]: frame for frame in rendered}
    missing_records = []
    deviations = []
    dimension_deviations = []
    max_position_error = 0.0
    max_yaw_error = 0.0
    for frame in trajectory:
        drawn = indexed.get(frame["frame"])
        if drawn is None:
            missing_records.append({"frame": frame["frame"], "reason": "render frame missing"})
            continue
        cars = {car["id"]: car for car in drawn["vehicles"]}
        for vehicle in frame["vehicles"]:
            actual = cars.get(vehicle["id"])
            if actual is None:
                missing_records.append({"frame": frame["frame"], "id": vehicle["id"]})
                continue
            declared_length_error = abs(actual.get("length_m", float("nan")) - vehicle["length_m"])
            model_length_error = abs(actual.get("model_length_m", actual.get("length_m", float("nan"))) - vehicle["length_m"])
            if not math.isfinite(declared_length_error) or declared_length_error > 0.001 or model_length_error > 0.001:
                dimension_deviations.append({"frame": frame["frame"], "id": vehicle["id"],
                                             "sumo_length_m": vehicle["length_m"],
                                             "render_declared_length_m": actual.get("length_m"),
                                             "actual_model_length_m": actual.get("model_length_m")})
            error = math.sqrt(sum((vehicle[axis] + render_offset_m[index] - actual[axis]) ** 2
                                  for index, axis in enumerate(("x", "y", "z"))))
            yaw_error = abs(wrapped_angle(vehicle["yaw_rad"] - actual["yaw_rad"]))
            max_position_error = max(max_position_error, error)
            max_yaw_error = max(max_yaw_error, yaw_error)
            if error > 0.001 or yaw_error > math.radians(0.01):
                deviations.append({"frame": frame["frame"], "id": vehicle["id"],
                                   "position_error_m": error, "yaw_error_deg": math.degrees(yaw_error)})
    add_check(report, "blender_pose_matches_sumo", not missing_records and not deviations,
              {"declared_sumo_to_render_translation_m": list(render_offset_m),
               "position_tolerance_m": 0.001, "yaw_tolerance_deg": 0.01,
               "max_position_error_m": max_position_error, "max_yaw_error_deg": math.degrees(max_yaw_error),
               "missing": missing_records[:30], "deviations": deviations[:30]})
    add_check(report, "blender_vehicle_lengths_match_current_sumo", not dimension_deviations,
              {"length_tolerance_m": 0.001, "dimension_deviations": dimension_deviations[:30]})


def validation_checks(path, collision_xml, report):
    if path is None:
        missing(report, "sumo_runtime_evidence", "Supply validation.json")
        return
    data = json.loads(path.read_text(encoding="utf-8"))
    add_check(report, "sumo_runtime_version", bool(data.get("sumo_version")), data.get("sumo_version"))
    for key in ("collision_count", "teleport_count"):
        if key not in data:
            missing(report, key, f"{key} absent from validation.json")
        else:
            add_check(report, key, data[key] == 0, data[key])
    add_check(report, "scenario_completion_report", data.get("scenario_completed") is True,
              {"scenario_completed": data.get("scenario_completed"), "events": data.get("events", [])})
    if collision_xml and collision_xml.exists():
        root = ET.parse(collision_xml).getroot()
        collisions = list(root.iter("collision"))
        add_check(report, "sumo_collision_xml", not collisions,
                  {"collision_count": len(collisions), "events": [node.attrib for node in collisions[:20]]})
    else:
        missing(report, "sumo_collision_xml", "Provide SUMO --collision-output XML covering every internal step")


def action_checks(ego_records, report, action_rules, frames=None):
    """Rules are scenario-specific facts supplied before looking at the result."""
    if not action_rules:
        missing(report, "independent_action_completion", "Provide action-rules.json with required edge/lane sequence and yield rules")
        return
    if not ego_records:
        add_check(report, "independent_action_completion", False, "No ego records")
        return
    states = [v for _, v in ego_records]
    times = [t for t, _ in ego_records]
    for kind in ("edge", "lane"):
        expected = action_rules.get(f"required_{kind}_sequence", [])
        actual = []
        for v in states:
            value = v[f"{kind}_id"]
            if not actual or actual[-1] != value:
                actual.append(value)
        cursor = 0
        for value in actual:
            if cursor < len(expected) and value == expected[cursor]:
                cursor += 1
        if expected:
            add_check(report, f"required_{kind}_sequence", cursor == len(expected),
                      {"expected_subsequence": expected, "observed": actual})
    stop_threshold = action_rules.get("yield_below_speed_mps")
    if stop_threshold is not None:
        dt = np.median(np.diff(times)) if len(times) > 1 else 0.0
        slow = [(t, v) for t, v in ego_records if v["speed_mps"] < stop_threshold]
        duration = float(len(slow) * dt)
        add_check(report, "yield_deceleration_phase", duration >= action_rules.get("yield_min_duration_s", 1.0),
                  {"speed_threshold_mps": stop_threshold, "duration_s": duration,
                   "first_s": slow[0][0] if slow else None, "last_s": slow[-1][0] if slow else None})
    if "final_yaw_deg" in action_rules:
        error = abs(wrapped_angle(states[-1]["yaw_rad"] - math.radians(action_rules["final_yaw_deg"])))
        add_check(report, "completed_turn_heading", error <= math.radians(action_rules.get("final_yaw_tolerance_deg", 10)),
                  {"actual_deg": math.degrees(states[-1]["yaw_rad"]), "target_deg": action_rules["final_yaw_deg"]})
    if "end_region" in action_rules:
        region = action_rules["end_region"]
        inside = region["x_min"] <= states[-1]["x"] <= region["x_max"] and region["y_min"] <= states[-1]["y"] <= region["y_max"]
        add_check(report, "reached_exit_region", inside, {"end_xy": [states[-1]["x"], states[-1]["y"]], "region": region})
    overtake_rule = action_rules.get("overtake")
    if overtake_rule and frames:
        # The direction and lane centerlines come from the inspected network.
        # Use geometry, rather than trusting a command or event named "passed".
        axis = overtake_rule.get("axis", "x")
        lateral_axis = "y" if axis == "x" else "x"
        direction = float(overtake_rule.get("direction", 1))
        lane_tolerance = float(overtake_rule.get("lane_center_tolerance_m", 0.15))
        through_center = float(overtake_rule["through_lane_center_m"])
        passing_center = float(overtake_rule["passing_lane_center_m"])
        pairs = []
        for frame in frames:
            cars = {car["id"]: car for car in frame["vehicles"]}
            ego = next((car for car in cars.values() if car.get("ego")), None)
            lead = cars.get(overtake_rule["lead_id"])
            if ego is None or lead is None:
                continue
            if ego["edge_id"] not in overtake_rule["ego_edges"]:
                continue
            if lead["edge_id"] not in overtake_rule["lead_edges"]:
                continue
            ego_forward = math.cos(ego["yaw_rad"]) if axis == "x" else math.sin(ego["yaw_rad"])
            lead_forward = math.cos(lead["yaw_rad"]) if axis == "x" else math.sin(lead["yaw_rad"])
            if min(direction * ego_forward, direction * lead_forward) < 0.98:
                continue
            lead_front = direction * (lead[axis] + lead_forward * lead["length_m"] / 2)
            ego_front = direction * (ego[axis] + ego_forward * ego["length_m"] / 2)
            ego_rear = direction * (ego[axis] - ego_forward * ego["length_m"] / 2)
            pairs.append({"time_s": frame["time_s"], "ego_lane": ego["lane_id"], "lead_lane": lead["lane_id"],
                          "ego_xy": [ego["x"], ego["y"]], "lead_xy": [lead["x"], lead["y"]],
                          "ego_speed_mps": ego["speed_mps"], "lead_speed_mps": lead["speed_mps"],
                          "front_gap_m": ego_front - lead_front, "rear_clearance_m": ego_rear - lead_front,
                          "center_gap_m": direction * (ego[axis] - lead[axis]),
                          "ego_on_through": abs(ego[lateral_axis] - through_center) <= lane_tolerance,
                          "ego_on_passing": abs(ego[lateral_axis] - passing_center) <= lane_tolerance,
                          "lead_on_through": abs(lead[lateral_axis] - through_center) <= lane_tolerance})
        lead_before = [p for p in pairs if p["ego_on_through"] and p["lead_on_through"] and p["front_gap_m"] < -5
                       and p["ego_speed_mps"] > p["lead_speed_mps"] + 0.5]
        observed_lead = lead_before[0] if lead_before else None
        alongside = [p for p in pairs if observed_lead and p["time_s"] > observed_lead["time_s"]
                     and p["ego_on_passing"] and p["lead_on_through"] and abs(p["center_gap_m"]) < 2.0]
        alongside_state = min(alongside, key=lambda p: abs(p["center_gap_m"])) if alongside else None
        passed = [p for p in pairs if alongside_state and p["time_s"] > alongside_state["time_s"]
                  and p["ego_on_passing"] and p["lead_on_through"]
                  and p["rear_clearance_m"] >= overtake_rule.get("minimum_pass_clearance_m", 2.0)]
        passed_state = passed[0] if passed else None
        returned = [p for p in pairs if passed_state and p["time_s"] > passed_state["time_s"]
                    and p["ego_on_through"] and p["lead_on_through"] and p["rear_clearance_m"] > 2.0]
        add_check(report, "slower_lead_actually_ahead_in_target_corridor", observed_lead is not None, observed_lead)
        add_check(report, "overtaking_vehicle_actually_alongside", alongside_state is not None, alongside_state)
        add_check(report, "overtake_full_body_cleared_lead", passed_state is not None, passed_state)
        add_check(report, "returned_to_through_corridor_after_passing", bool(returned), returned[0] if returned else None)
    left_rule = action_rules.get("unprotected_left")
    if left_rule and frames:
        # The rule file defines the real conflict geometry ahead of acceptance:
        # ego enters a specified edge, oncoming rears clear a specified line.
        entering = [(t, v) for t, v in ego_records if v["edge_id"] in left_rule["ego_conflict_edges"]]
        entry_time = entering[0][0] if entering else None
        oncoming_evidence = {}
        clear_line = left_rule["oncoming_clear_line"]
        axis = clear_line["axis"]
        direction = float(clear_line["direction"])
        for vehicle_id in left_rule["oncoming_ids"]:
            history = [(frame["time_s"], car) for frame in frames
                       for car in frame["vehicles"] if car["id"] == vehicle_id]
            cleared = []
            for t, car in history:
                # Check the trailing end, not only vehicle center/front.
                heading_component = math.cos(car["yaw_rad"]) if axis == "x" else math.sin(car["yaw_rad"])
                rear_coordinate = car[axis] - heading_component * car["length_m"] / 2
                if direction * (rear_coordinate - clear_line["value"]) >= 0:
                    cleared.append(t)
            oncoming_evidence[vehicle_id] = {
                "observed": bool(history), "cleared_at_s": cleared[0] if cleared else None,
                "edges_used": list(dict.fromkeys(car["edge_id"] for _, car in history)),
                "start_xy": [history[0][1]["x"], history[0][1]["y"]] if history else None,
                "end_xy": [history[-1][1]["x"], history[-1][1]["y"]] if history else None,
            }
        oncoming_passed = all(item["cleared_at_s"] is not None for item in oncoming_evidence.values())
        yielded_until_clear = entry_time is not None and oncoming_passed and all(
            item["cleared_at_s"] <= entry_time + 0.0011 for item in oncoming_evidence.values())
        add_check(report, "oncoming_traffic_actually_passed", oncoming_passed, oncoming_evidence)
        add_check(report, "unprotected_turn_entered_after_oncoming_cleared", yielded_until_clear,
                  {"ego_entered_conflict_edge_s": entry_time,
                   "clear_line": clear_line, "oncoming": oncoming_evidence})


def video_checks(path, report, expected_fps, expected_resolution, expected_duration, cuts):
    if path is None or not path.exists():
        missing(report, "encoded_video", "Final video not supplied or does not exist")
        return
    import imageio_ffmpeg
    reader = imageio_ffmpeg.read_frames(str(path), pix_fmt="rgb24")
    metadata = next(reader)
    black = []
    abrupt = []
    count = 0
    min_mean = 255.0
    identical_run = 0
    max_identical_run = 0
    previous = None
    width, height = metadata["size"]
    for raw in reader:
        pixels = np.frombuffer(raw, dtype=np.uint8).reshape(height, width, 3)
        # Sample every eighth pixel; decode still visits every encoded frame.
        reduced = pixels[::8, ::8].astype(np.int16)
        mean = float(reduced.mean())
        min_mean = min(min_mean, mean)
        if mean < 2.0 and float(reduced.std()) < 2.0:
            black.append(count)
        if previous is not None:
            delta = float(np.abs(reduced - previous).mean())
            if delta > 45 and not any(abs(count - cut) <= 1 for cut in cuts):
                abrupt.append({"frame": count, "mean_rgb_delta": delta})
            identical_run = identical_run + 1 if delta < 0.02 else 0
            max_identical_run = max(max_identical_run, identical_run)
        previous = reduced
        count += 1
    add_check(report, "video_resolution", list(metadata["size"]) == expected_resolution,
              {"expected": expected_resolution, "actual": metadata["size"]})
    add_check(report, "video_fps", abs(metadata["fps"] - expected_fps) <= 0.001,
              {"expected": expected_fps, "actual": metadata["fps"]})
    expected_count = round(expected_duration * expected_fps)
    add_check(report, "video_duration_and_frame_count", count == expected_count and abs(count / metadata["fps"] - expected_duration) <= 0.001,
              {"decoded_frames": count, "expected_frames": expected_count,
               "duration_s": count / metadata["fps"], "expected_duration_s": expected_duration})
    add_check(report, "no_black_or_undecodable_frames", not black,
              {"decoded_frames": count, "black_frames": black[:30], "minimum_mean_rgb": min_mean})
    add_check(report, "unexpected_visual_cuts", not abrupt,
              {"allowed_cut_frames": cuts, "candidates": abrupt[:30]}, severity="warning")
    add_check(report, "no_long_frozen_frame_runs", max_identical_run <= expected_fps * 2,
              {"max_identical_run_frames": max_identical_run}, severity="warning")


def visible_action_checks(frames, report, rules, fps):
    """Check whether the selected visual range contains the demonstration itself."""
    visual = (rules or {}).get("visible_action_requirements")
    if not visual:
        return
    ego = [(frame["time_s"], car) for frame in frames for car in frame["vehicles"] if car.get("ego")]
    edges = []
    for _, car in ego:
        if not edges or edges[-1] != car["edge_id"]:
            edges.append(car["edge_id"])
    expected = visual.get("edge_sequence", [])
    cursor = 0
    for edge in edges:
        if cursor < len(expected) and edge == expected[cursor]:
            cursor += 1
    if expected:
        add_check(report, "selected_clip_contains_action_route", cursor == len(expected),
                  {"expected_subsequence": expected, "visible_source_edges": edges})
    if "yield_below_speed_mps" in visual:
        threshold = visual["yield_below_speed_mps"]
        slow = [(time, car) for time, car in ego if car["speed_mps"] < threshold]
        duration = len(slow) / fps
        add_check(report, "selected_clip_contains_yield_phase", duration >= visual.get("yield_min_duration_s", 1),
                  {"duration_s": duration, "threshold_mps": threshold,
                   "first_source_s": slow[0][0] if slow else None, "last_source_s": slow[-1][0] if slow else None})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--validation", type=Path)
    parser.add_argument("--collision-xml", type=Path)
    parser.add_argument("--render-poses", type=Path)
    parser.add_argument("--render-offset-m", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        metavar=("X", "Y", "Z"), help="Explicit fixed SUMO-to-Blender root translation, e.g. 0 0 0.008 for road surface")
    parser.add_argument("--omit-trailing-no-ego", action="store_true",
                        help="Explicitly omit only terminal snapshots after ego removal from rendered/video frame expectations")
    parser.add_argument("--first-frame", type=int,
                        help="Inclusive first source frame for pose/video checks; complete source trajectory still validates physics and actions")
    parser.add_argument("--last-frame", type=int,
                        help="Inclusive last source frame for pose/video checks; encoded duration is selected frame count / fps")
    parser.add_argument("--video", type=Path)
    parser.add_argument("--obstacles", type=Path)
    parser.add_argument("--action-rules", type=Path)
    parser.add_argument("--fps", type=float, default=30)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--cut-frames", type=int, nargs="*", default=[],
                        help="Allowed camera cuts identified by source frame ID; mapped to clip frame indexes automatically")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    report = {"checks": {}, "evidence_sha256": {},
              "limits": ["OBB/convex footprint tests probe geometric intersections, not exact mesh intersection.",
                         "Visual cut/frozen checks are review candidates and do not establish camera stability.",
                         "No automatic test proves absence of HUD/text; inspect all key frames.",
                         "Exact vehicle-road contact, tyre contact and occlusion require visual review or mesh geometry."]}
    frames = read_frames(args.trajectory)
    if not frames:
        raise ValueError("trajectory contains no frames")
    ego = trajectory_checks(frames, report, args.fps)
    obstacles = json.loads(args.obstacles.read_text(encoding="utf-8"))["obstacles"] if args.obstacles else None
    collision_checks(frames, report, label="sumo")
    validation_checks(args.validation, args.collision_xml, report)
    action_rules = json.loads(args.action_rules.read_text(encoding="utf-8")) if args.action_rules else None
    action_checks(ego, report, action_rules, frames)
    rendered_expectations = frames
    if args.omit_trailing_no_ego:
        last_visible = max((index for index, frame in enumerate(frames) if any(v.get("ego") for v in frame["vehicles"])), default=-1)
        rendered_expectations = frames[:last_visible + 1]
        report["declared_video_trim"] = {"reason": "SUMO terminal snapshots after ego arrival/removal",
                                          "omitted_frame_ids": [frame["frame"] for frame in frames[last_visible + 1:]],
                                          "omitted_duration_s": (len(frames) - len(rendered_expectations)) / args.fps}
    if args.first_frame is not None or args.last_frame is not None:
        first = args.first_frame if args.first_frame is not None else rendered_expectations[0]["frame"]
        last = args.last_frame if args.last_frame is not None else rendered_expectations[-1]["frame"]
        if first < rendered_expectations[0]["frame"] or last > rendered_expectations[-1]["frame"] or first > last:
            raise ValueError("Requested source frame range is outside the valid rendered trajectory or reversed")
        rendered_expectations = [frame for frame in rendered_expectations if first <= frame["frame"] <= last]
        if len(rendered_expectations) != last - first + 1:
            raise ValueError("Requested source range is not consecutive")
        report["declared_render_source_range"] = {"first_frame": first, "last_frame": last,
                                                  "frame_count": len(rendered_expectations),
                                                  "first_source_time_s": rendered_expectations[0]["time_s"],
                                                  "last_source_time_s": rendered_expectations[-1]["time_s"],
                                                  "encoded_duration_s": len(rendered_expectations) / args.fps,
                                                  "action_checks_use_complete_source_trajectory": True}
    visible_action_checks(rendered_expectations, report, action_rules, args.fps)
    if args.render_poses and args.render_poses.exists():
        drawn = read_frames(args.render_poses)
        pose_checks(rendered_expectations, drawn, report, args.render_offset_m)
        collision_checks(drawn, report, obstacles, label="blender")
        if not args.cut_frames:
            last_shot = None
            for frame in drawn:
                shot = frame.get("camera", {}).get("shot")
                if shot is not None and last_shot is not None and shot != last_shot:
                    args.cut_frames.append(frame["frame"])
                last_shot = shot
    else:
        missing(report, "blender_pose_matches_sumo", "Export evaluated Blender vehicle world poses during frame_set/render")
    if obstacles is None:
        missing(report, "blender_obstacle_footprints", "Provide convex XY footprints and vertical extents for physical roadside obstacles")
    duration = args.duration if args.duration is not None else len(rendered_expectations) / args.fps
    first_source_frame = rendered_expectations[0]["frame"]
    clip_cuts = [frame - first_source_frame for frame in args.cut_frames
                 if first_source_frame < frame <= rendered_expectations[-1]["frame"]]
    report["camera_cut_mapping"] = {"source_frame_ids": args.cut_frames, "encoded_frame_indexes": clip_cuts}
    video_checks(args.video, report, args.fps, [args.width, args.height], duration, clip_cuts)
    for path in (args.trajectory, args.validation, args.collision_xml, args.render_poses,
                 args.video, args.obstacles, args.action_rules):
        if path and path.exists():
            report["evidence_sha256"][str(path.resolve())] = evidence_hash(path)
    status_counts = collections.Counter(check["status"] for check in report["checks"].values())
    report["status_counts"] = dict(status_counts)
    report["automated_evidence_complete"] = not status_counts["error"] and not status_counts["missing"]
    report["overall_status"] = "fail" if status_counts["error"] else ("incomplete" if status_counts["missing"] else "automated_checks_passed_manual_review_needed")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"status": report["overall_status"], "counts": report["status_counts"], "report": str(args.out)}))
    return 1 if status_counts["error"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
