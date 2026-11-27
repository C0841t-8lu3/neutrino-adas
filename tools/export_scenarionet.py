#!/usr/bin/env python3
"""Offline ScenarioNet (.pkl) -> Self-Contained JSON Exporter for Rust/Bevy ADAS Simulator.

Converts ScenarioNet scenario descriptions (Waymo / nuScenes / nuPlan `.pkl` files)
into centered QNX-frame JSON scenes (`assets/scenarios/<id>.json`) so the native Rust
simulator loads HD maps, traffic-light timelines, and surrounding tracks with zero runtime Python.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import pickle
from typing import Any, Dict, List, Tuple


def _round_pt(x: float, y: float, x0: float, y0: float) -> List[float]:
    return [round(float(x) - x0, 3), round(float(y) - y0, 3)]


def _waymo_heading_to_qnx(yaw_rad: float) -> float:
    """Map ScenarioNet ENU yaw (0=+East, +PI/2=+North CCW) to QNX heading (0=+North, +CW)."""
    h = math.pi * 0.5 - float(yaw_rad)
    while h > math.pi:
        h -= 2.0 * math.pi
    while h <= -math.pi:
        h += 2.0 * math.pi
    return round(h, 5)


def _normalize_light_state(raw: Any) -> str:
    s = str(raw or "")
    if "STOP" in s:
        return "RED"
    if "CAUTION" in s:
        return "YELLOW"
    if "GO" in s:
        return "GREEN"
    return "UNKNOWN"


def _classify_line(mf_type: str) -> str:
    t = mf_type.upper()
    if "DOUBLE_YELLOW" in t:
        return "DOUBLE_YELLOW"
    if "YELLOW" in t:
        return "SOLID_YELLOW"
    if "BROKEN" in t:
        return "BROKEN_WHITE"
    if "SOLID_SINGLE_WHITE" in t or "SOLID_WHITE" in t:
        return "SOLID_WHITE"
    if "MEDIAN" in t:
        return "MEDIAN"
    return "BOUNDARY"


def convert_scenario(pkl_path: str) -> Dict[str, Any]:
    with open(pkl_path, "rb") as f:
        sd = pickle.load(f)

    meta = sd.get("metadata", {})
    scenario_id = str(sd.get("id") or meta.get("scenario_id") or os.path.splitext(os.path.basename(pkl_path))[0])
    ts = list(meta.get("ts", []))
    dt_s = round(float(ts[1] - ts[0]), 4) if len(ts) >= 2 else 0.1
    length = int(sd.get("length") or len(ts) or 91)

    tracks = sd.get("tracks", {})
    sdc_id = str(meta.get("sdc_id"))
    if sdc_id not in tracks:
        raise ValueError(f"SDC id {sdc_id!r} missing from tracks in {pkl_path}")

    sdc_track = tracks[sdc_id]
    sdc_state = sdc_track["state"]
    sdc_valid = [bool(v) for v in sdc_state["valid"]]
    first_valid = next((i for i, v in enumerate(sdc_valid) if v), 0)
    last_valid = max((i for i, v in enumerate(sdc_valid) if v), default=length - 1)

    origin_x = float(sdc_state["position"][first_valid][0])
    origin_y = float(sdc_state["position"][first_valid][1])

    # Convert Ego start, goal & ground-truth reference trajectory
    ego_ref_poly: List[List[float]] = []
    for i in range(first_valid, last_valid + 1):
        if sdc_valid[i]:
            pos = sdc_state["position"][i]
            pt = _round_pt(pos[0], pos[1], origin_x, origin_y)
            if not ego_ref_poly or math.hypot(pt[0] - ego_ref_poly[-1][0], pt[1] - ego_ref_poly[-1][1]) >= 0.4:
                ego_ref_poly.append(pt)

    vx0, vy0 = float(sdc_state["velocity"][first_valid][0]), float(sdc_state["velocity"][first_valid][1])
    start_speed = round(math.hypot(vx0, vy0), 3)
    start_h = _waymo_heading_to_qnx(sdc_state["heading"][first_valid])
    goal_pos = sdc_state["position"][last_valid]
    goal_h = _waymo_heading_to_qnx(sdc_state["heading"][last_valid])
    if len(ego_ref_poly) >= 2:
        last_pt = _round_pt(goal_pos[0], goal_pos[1], origin_x, origin_y)
        if ego_ref_poly[-1] != last_pt:
            ego_ref_poly.append(last_pt)

    ego_info = {
        "id": sdc_id,
        "length_m": round(float(sdc_state["length"][first_valid]), 3),
        "width_m": round(float(sdc_state["width"][first_valid]), 3),
        "start_pose": [0.0, 0.0, start_h, start_speed],
        "goal_pose": [*_round_pt(goal_pos[0], goal_pos[1], origin_x, origin_y), goal_h],
        "reference_polyline": ego_ref_poly,
    }

    # Convert HD Map Features
    lanes: List[Dict[str, Any]] = []
    road_lines: List[Dict[str, Any]] = []
    crosswalks: List[Dict[str, Any]] = []
    stop_signs: List[Dict[str, Any]] = []

    for fid, feat in sd.get("map_features", {}).items():
        ftype = str(feat.get("type", ""))
        if ftype.startswith("LANE_"):
            poly_arr = feat.get("polyline", [])
            if len(poly_arr) < 2:
                continue
            pts = [_round_pt(p[0], p[1], origin_x, origin_y) for p in poly_arr]
            widths = feat.get("width")
            if widths is not None and len(widths) > 0:
                avg_w = float(sum(float(w[0]) + float(w[1]) for w in widths) / len(widths))
            else:
                avg_w = 3.5
            spd_kmh = float(feat.get("speed_limit_kmh") or (float(feat.get("speed_limit_mph", 35.0)) * 1.60934))
            lanes.append({
                "id": str(fid),
                "is_bike_lane": "BIKE" in ftype,
                "polyline": pts,
                "width_m": round(max(1.8, avg_w), 2),
                "speed_limit_mps": round(max(4.0, spd_kmh / 3.6), 2),
                "entry_lanes": [str(x) for x in feat.get("entry_lanes", [])],
                "exit_lanes": [str(x) for x in feat.get("exit_lanes", [])],
                "left_neighbor": [str(x) for x in feat.get("left_neighbor", [])],
                "right_neighbor": [str(x) for x in feat.get("right_neighbor", [])],
            })
        elif ftype.startswith("ROAD_LINE_") or ftype.startswith("ROAD_EDGE_"):
            poly_arr = feat.get("polyline", [])
            if len(poly_arr) < 2:
                continue
            # Subsample dense road edges softly (>= 1.2 m spacing) for lightweight fast rendering
            pts: List[List[float]] = []
            for p in poly_arr:
                pt = _round_pt(p[0], p[1], origin_x, origin_y)
                if not pts or math.hypot(pt[0] - pts[-1][0], pt[1] - pts[-1][1]) >= 1.2:
                    pts.append(pt)
            last_p = _round_pt(poly_arr[-1][0], poly_arr[-1][1], origin_x, origin_y)
            if len(pts) == 1 or pts[-1] != last_p:
                pts.append(last_p)
            road_lines.append({
                "id": str(fid),
                "line_type": _classify_line(ftype),
                "polyline": pts,
            })
        elif ftype == "CROSSWALK":
            poly_arr = feat.get("polygon", feat.get("polyline", []))
            if len(poly_arr) >= 3:
                crosswalks.append({
                    "id": str(fid),
                    "polygon": [_round_pt(p[0], p[1], origin_x, origin_y) for p in poly_arr],
                })
        elif ftype == "STOP_SIGN":
            pos = feat.get("position")
            if pos is not None and len(pos) >= 2:
                stop_signs.append({
                    "id": str(fid),
                    "pos": _round_pt(pos[0], pos[1], origin_x, origin_y),
                    "lanes": [str(x) for x in feat.get("lane", [])],
                })

    # Convert Dynamic Traffic Lights
    traffic_lights: List[Dict[str, Any]] = []
    for tl_id, tl in sd.get("dynamic_map_states", {}).items():
        if str(tl.get("type", "")) != "TRAFFIC_LIGHT":
            continue
        sp = tl.get("stop_point")
        if sp is None or len(sp) < 2:
            continue
        raw_states = tl.get("state", {}).get("object_state", [])
        norm_states = [_normalize_light_state(s) for s in raw_states]
        lane_ref = tl.get("lane")
        lane_str = str(lane_ref[0]) if isinstance(lane_ref, (list, tuple)) and lane_ref else (str(lane_ref) if lane_ref is not None else "")
        traffic_lights.append({
            "id": str(tl_id),
            "stop_point": _round_pt(sp[0], sp[1], origin_x, origin_y),
            "lane_id": lane_str,
            "states": norm_states,
        })

    # Convert Surrounding Dynamic Tracks (Vehicles, Trucks, Pedestrians, Cyclists)
    out_tracks: List[Dict[str, Any]] = []
    for tid, tr in tracks.items():
        if str(tid) == sdc_id:
            continue
        obj_type = str(tr.get("type", "VEHICLE")).upper()
        st = tr["state"]
        valids = st["valid"]
        positions = st["position"]
        headings = st["heading"]
        vels = st["velocity"]
        lengths = st["length"]
        widths = st["width"]

        valid_indices = [i for i, v in enumerate(valids) if bool(v)]
        if not valid_indices:
            continue
        l_m = float(lengths[valid_indices[0]])
        w_m = float(widths[valid_indices[0]])

        frames: List[Dict[str, Any]] = []
        for i in range(length):
            vflag = bool(valids[i]) if i < len(valids) else False
            if not vflag:
                continue
            vx, vy = float(vels[i][0]), float(vels[i][1])
            pt = _round_pt(positions[i][0], positions[i][1], origin_x, origin_y)
            frames.append({
                "t": i,
                "x": pt[0],
                "z": pt[1],
                "h": _waymo_heading_to_qnx(headings[i]),
                "spd": round(math.hypot(vx, vy), 3),
            })

        out_tracks.append({
            "id": str(tid),
            "obj_type": obj_type,
            "length_m": round(max(0.5, l_m), 2),
            "width_m": round(max(0.5, w_m), 2),
            "frames": frames,
        })

    return {
        "scenario_id": scenario_id,
        "dataset": str(meta.get("dataset", "waymo")),
        "length": length,
        "dt_s": dt_s,
        "origin_xy": [round(origin_x, 3), round(origin_y, 3)],
        "ego": ego_info,
        "lanes": lanes,
        "road_lines": road_lines,
        "crosswalks": crosswalks,
        "stop_signs": stop_signs,
        "traffic_lights": traffic_lights,
        "tracks": out_tracks,
    }


def export_directory(input_dir: str, output_dir: str) -> List[str]:
    os.makedirs(output_dir, exist_ok=True)
    summary_path = os.path.join(input_dir, "dataset_summary.pkl")
    pkl_files: List[str] = []

    if os.path.isfile(summary_path):
        with open(summary_path, "rb") as f:
            summary = pickle.load(f)
        for fname in sorted(summary.keys()):
            candidate = os.path.join(input_dir, fname)
            if os.path.isfile(candidate):
                pkl_files.append(candidate)

    if not pkl_files:
        pkl_files = sorted(
            p for p in glob.glob(os.path.join(input_dir, "*.pkl"))
            if not os.path.basename(p).startswith(("dataset_summary", "dataset_mapping"))
        )

    exported: List[str] = []
    for pkl_path in pkl_files:
        scene = convert_scenario(pkl_path)
        out_path = os.path.join(output_dir, f"{scene['scenario_id']}.json")
        with open(out_path, "w", encoding="utf-8") as wf:
            json.dump(scene, wf, separators=(",", ":"))
        exported.append(out_path)
        print(
            f"[OK] {scene['scenario_id']} -> {out_path} "
            f"(lanes={len(scene['lanes'])}, lines={len(scene['road_lines'])}, "
            f"xwalks={len(scene['crosswalks'])}, lights={len(scene['traffic_lights'])}, "
            f"tracks={len(scene['tracks'])}, len={scene['length']}@{scene['dt_s']}s)"
        )
    return exported


def main() -> None:
    parser = argparse.ArgumentParser(description="Export ScenarioNet .pkl scenes to self-contained JSON for Rust/Bevy.")
    parser.add_argument("--input-dir", required=True, help="Directory containing dataset_summary.pkl and sd_*.pkl scenarios")
    parser.add_argument("--output-dir", default="assets/scenarios", help="Target directory for exported JSON files")
    args = parser.parse_args()
    exported = export_directory(args.input_dir, args.output_dir)
    print(f"Exported {len(exported)} ScenarioNet scenes cleanly.")


if __name__ == "__main__":
    main()
