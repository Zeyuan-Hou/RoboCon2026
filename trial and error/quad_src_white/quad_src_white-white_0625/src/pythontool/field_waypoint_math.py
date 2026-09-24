"""Field-local manip waypoint geometry: labeled multi-anchor + legacy anchors."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Iterable

# AB pickup zone relative to A1 (+x right, +y forward in field-local frame).
A_FRONT_LOCAL = [
    (0.00, 0.00),
    (0.85, 0.00),
    (1.70, 0.00),
    (2.55, 0.00),
]
B_REAR_LOCAL = [
    (0.00, 0.85),
    (0.85, 0.85),
    (1.70, 0.85),
    (2.55, 0.85),
]
B4_LOCAL = B_REAR_LOCAL[3]
PICKUP_LOCAL_BY_LABEL = {
    "A1": A_FRONT_LOCAL[0],
    "A2": A_FRONT_LOCAL[1],
    "A3": A_FRONT_LOCAL[2],
    "A4": A_FRONT_LOCAL[3],
    "B1": B_REAR_LOCAL[0],
    "B2": B_REAR_LOCAL[1],
    "B3": B_REAR_LOCAL[2],
    "B4": B_REAR_LOCAL[3],
}

# C zone horizontal dx relative to A1; dy = d_c_row (from C1/C4 anchors).
C_DX = [0.075, 0.875, 1.675, 2.475]
C_DX_SPAN = C_DX[3] - C_DX[0]
DROPOFF_LOCAL_X_BY_LABEL = {
    "C1": C_DX[0],
    "C2": C_DX[1],
    "C3": C_DX[2],
    "C4": C_DX[3],
}
LABEL_RE = re.compile(r"\blabel\s*=\s*([A-Za-z0-9_:.-]+)")
SESSION_MARKER = "interactive_waypoint_recorder session"

PICKUP_LABELS = [
    "A1 前排取货1",
    "A2 前排取货2",
    "A3 前排取货3",
    "A4 前排取货4",
    "B1 后排取货1",
    "B2 后排取货2",
    "B3 后排取货3",
    "B4 后排取货4",
]
DROPOFF_LOW_LABELS = [
    "C1 放货1 低层",
    "C2 放货2 低层",
    "C3 放货3 低层",
    "C4 放货4 低层",
]
DROPOFF_HIGH_LABELS = [
    "C1 放货1 高层",
    "C2 放货2 高层",
    "C3 放货3 高层",
    "C4 放货4 高层",
]

# Local entry trajectory endpoints measured from the current navigation-semantic
# field anchors. A1 is local (0, 0), +x follows A1->A4, +y follows pickup rows.
ENTRY_TRAJECTORY_START_LOCAL = (1.298, -1.332)
ENTRY_TRAJECTORY_END_LOCAL = (1.248, 1.189)
ENTRY_TRAJECTORY_DT_S = 0.1
ENTRY_TRAJECTORY_DURATION_S = 2.5


@dataclass(frozen=True)
class AnchorRecord:
    x: float
    y: float
    yaw: float
    label: str | None = None

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw


@dataclass(frozen=True)
class TrajectoryRecord:
    t: float
    x: float
    y: float
    yaw: float


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def average_angle(a: float, b: float) -> float:
    return math.atan2(math.sin(a) + math.sin(b), math.cos(a) + math.cos(b))


def average_yaw(yaws: Iterable[float]) -> float:
    yaws = list(yaws)
    if not yaws:
        return 0.0
    return math.atan2(
        sum(math.sin(y) for y in yaws),
        sum(math.cos(y) for y in yaws),
    )


def local_to_map(
    origin: tuple[float, float],
    local_xy: tuple[float, float],
    theta: float,
    scale: float = 1.0,
) -> tuple[float, float]:
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    dx, dy = local_xy
    rx = dx * cos_t - dy * sin_t
    ry = dx * sin_t + dy * cos_t
    return (
        origin[0] + scale * rx,
        origin[1] + scale * ry,
    )


def solve_theta_ab(a1: tuple[float, float], b4: tuple[float, float]) -> float:
    dx = b4[0] - a1[0]
    dy = b4[1] - a1[1]
    return math.atan2(dy, dx) - math.atan2(B4_LOCAL[1], B4_LOCAL[0])


def solve_theta_c(c1: tuple[float, float], c4: tuple[float, float]) -> float:
    return math.atan2(c4[1] - c1[1], c4[0] - c1[0])


def _normalize_label(label: str | None) -> str | None:
    if label is None:
        return None
    label = label.strip()
    if not label:
        return None
    if ":" in label:
        prefix, value = label.split(":", 1)
        prefix = prefix.lower()
        value = value.upper()
        if prefix in ("pickup", "dropoff"):
            return f"{prefix}:{value}"
        if prefix == "startup":
            return "startup:pickup"
    value = label.upper()
    if value in PICKUP_LOCAL_BY_LABEL:
        return f"pickup:{value}"
    if value in DROPOFF_LOCAL_X_BY_LABEL:
        return f"dropoff:{value}"
    if value in ("STARTUP", "STARTUP_PICKUP"):
        return "startup:pickup"
    return label


def _parse_label(raw: str) -> str | None:
    match = LABEL_RE.search(raw)
    return _normalize_label(match.group(1)) if match else None


def parse_anchor_records_file(path) -> list[AnchorRecord]:
    """Parse anchor rows, preserving optional '# label=...' metadata."""
    records: list[AnchorRecord] = []
    with open(path, "r", encoding="utf-8") as handle:
        for raw in handle:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            data = line.split("#", 1)[0].strip()
            parts = data.split()
            if len(parts) < 3:
                raise ValueError(f"Invalid anchor line (need x y yaw): {raw!r}")
            records.append(
                AnchorRecord(
                    float(parts[0]),
                    float(parts[1]),
                    float(parts[2]),
                    _parse_label(raw),
                )
            )
    return records


def parse_latest_labeled_session(path) -> list[AnchorRecord]:
    """Return the last labeled session if present, else all labeled rows."""
    latest_session: list[AnchorRecord] = []
    all_labeled: list[AnchorRecord] = []
    in_latest_session = False

    with open(path, "r", encoding="utf-8") as handle:
        for raw in handle:
            stripped = raw.strip()
            if stripped.startswith("#") and SESSION_MARKER in stripped:
                latest_session = []
                in_latest_session = True
                continue
            if not stripped or stripped.startswith("#"):
                continue
            data = stripped.split("#", 1)[0].strip()
            parts = data.split()
            if len(parts) < 3:
                raise ValueError(f"Invalid anchor line (need x y yaw): {raw!r}")
            label = _parse_label(raw)
            if label is None:
                if in_latest_session:
                    latest_session = []
                    in_latest_session = False
                continue
            record = AnchorRecord(
                float(parts[0]), float(parts[1]), float(parts[2]), label
            )
            all_labeled.append(record)
            if in_latest_session:
                latest_session.append(record)

    return latest_session if latest_session else all_labeled


def parse_anchor_points_file(path) -> list[tuple[float, float, float]]:
    """Parse example_point.txt lines as (x, y, yaw); skip blanks and # comments."""
    return [record.pose for record in parse_anchor_records_file(path)]


def parse_four_anchors(
    points: list[tuple[float, float, float]],
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    if len(points) < 4:
        raise ValueError(f"Need at least 4 anchor points; got {len(points)}")
    return points[-4], points[-3], points[-2], points[-1]


def parse_startup_pickup_anchor(
    points: list[tuple[float, float, float]],
) -> tuple[float, float, float] | None:
    """5th-from-last row in example_point.txt: startup first-pickup pose."""
    if len(points) < 5:
        return None
    return points[-5]


def parse_startup_pickup_anchor_from_file(path) -> tuple[float, float, float] | None:
    """Prefer labeled startup pickup; fall back to legacy 5th-from-last row."""
    records = parse_latest_labeled_session(path)
    for record in reversed(records):
        if record.label == "startup:pickup":
            return record.pose
    if any(record.label for record in records):
        return None
    return parse_startup_pickup_anchor(parse_anchor_points_file(path))


def apply_startup_pickup_override(
    result: dict,
    startup_row: tuple[float, float, float],
    pickup_index: int,
) -> dict:
    """Replace pickup_points[pickup_index] x/y/yaw with recorded startup anchor."""
    pickup_points = [list(row) for row in result["pickup_points"]]
    if pickup_index < 0 or pickup_index >= len(pickup_points):
        raise ValueError(
            f"startup_pickup_index={pickup_index} out of range "
            f"(num_pickup={len(pickup_points)})"
        )

    x, y, yaw = startup_row
    old_x, old_y, old_yaw, is_front = pickup_points[pickup_index]
    pickup_points[pickup_index] = [x, y, yaw, is_front]
    result = dict(result)
    result["pickup_points"] = [tuple(row) for row in pickup_points]
    result["startup_pickup_override"] = {
        "pickup_index": pickup_index,
        "anchor_map": [x, y],
        "before_map": [old_x, old_y],
        "yaw": yaw,
    }
    return result


def _latest_records_by_label(records: Iterable[AnchorRecord]) -> dict[str, AnchorRecord]:
    latest: dict[str, AnchorRecord] = {}
    for record in records:
        if record.label:
            latest[record.label] = record
    return latest


def _fit_similarity(
    pairs: list[tuple[tuple[float, float], tuple[float, float]]],
) -> tuple[tuple[float, float], float, float]:
    if len(pairs) < 2:
        raise ValueError("Need at least two pairs to fit similarity transform")

    src_cx = sum(src[0] for src, _ in pairs) / len(pairs)
    src_cy = sum(src[1] for src, _ in pairs) / len(pairs)
    dst_cx = sum(dst[0] for _, dst in pairs) / len(pairs)
    dst_cy = sum(dst[1] for _, dst in pairs) / len(pairs)

    den = 0.0
    a_num = 0.0
    b_num = 0.0
    for src, dst in pairs:
        sx = src[0] - src_cx
        sy = src[1] - src_cy
        dx = dst[0] - dst_cx
        dy = dst[1] - dst_cy
        den += sx * sx + sy * sy
        a_num += dx * sx + dy * sy
        b_num += dy * sx - dx * sy

    if den < 1e-9:
        raise ValueError("Degenerate anchor points for similarity transform")

    a = a_num / den
    b = b_num / den
    scale = math.hypot(a, b)
    theta = math.atan2(b, a)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    origin = (
        dst_cx - scale * (src_cx * cos_t - src_cy * sin_t),
        dst_cy - scale * (src_cx * sin_t + src_cy * cos_t),
    )
    return origin, theta, scale


def _fit_residual(
    measured: tuple[float, float],
    predicted: tuple[float, float],
) -> float:
    return math.hypot(measured[0] - predicted[0], measured[1] - predicted[1])


def build_waypoints_from_labeled_records(
    records: Iterable[AnchorRecord],
    *,
    high_layer_y_offset: float = 0.025,
    forward_yaw_override: float | None = None,
    reverse_yaw_override: float | None = None,
) -> dict:
    records = list(records)
    latest = _latest_records_by_label(records)
    pickup_labels = [
        label.split(":", 1)[1]
        for label in latest
        if label.startswith("pickup:") and label.split(":", 1)[1] in PICKUP_LOCAL_BY_LABEL
    ]
    dropoff_labels = [
        label.split(":", 1)[1]
        for label in latest
        if label.startswith("dropoff:") and label.split(":", 1)[1] in DROPOFF_LOCAL_X_BY_LABEL
    ]
    if len(pickup_labels) < 3 or len(dropoff_labels) < 2:
        raise ValueError(
            "Need at least 3 pickup labels and 2 dropoff labels for labeled fit"
        )

    pickup_pairs: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for label in sorted(pickup_labels):
        record = latest[f"pickup:{label}"]
        pickup_pairs.append((PICKUP_LOCAL_BY_LABEL[label], (record.x, record.y)))

    origin, theta_row, scale_ab = _fit_similarity(pickup_pairs)

    reverse_yaws = [latest[f"pickup:{label}"].yaw for label in pickup_labels]
    dropoff_yaws = [latest[f"dropoff:{label}"].yaw for label in dropoff_labels]
    reverse_yaw = normalize_angle(
        reverse_yaw_override
        if reverse_yaw_override is not None
        else average_yaw(reverse_yaws)
    )
    forward_yaw = normalize_angle(
        forward_yaw_override
        if forward_yaw_override is not None
        else average_yaw(dropoff_yaws)
    )

    d_c_values: list[float] = []
    for label in dropoff_labels:
        record = latest[f"dropoff:{label}"]
        _local_x, local_y = map_to_local(
            origin, (record.x, record.y), theta_row, scale_ab
        )
        d_c_values.append(local_y)
    d_c_row = sum(d_c_values) / len(d_c_values)

    pickup_points = build_pickup_parallel(origin, theta_row, scale_ab, reverse_yaw)
    dropoff_low: list[tuple[float, float, float]] = []
    for dx in C_DX:
        x, y = local_to_map(origin, (dx, d_c_row), theta_row, scale_ab)
        dropoff_low.append((x, y, forward_yaw))
    dropoff_high = [
        (x, y + high_layer_y_offset, forward_yaw) for x, y, _ in dropoff_low
    ]
    dropoff_points = dropoff_low + dropoff_high

    pickup_index_by_label = {
        "A1": 0, "A2": 1, "A3": 2, "A4": 3,
        "B1": 4, "B2": 5, "B3": 6, "B4": 7,
    }
    dropoff_index_by_label = {"C1": 0, "C2": 1, "C3": 2, "C4": 3}

    residuals: dict[str, float] = {}
    for label in pickup_labels:
        record = latest[f"pickup:{label}"]
        pred = pickup_points[pickup_index_by_label[label]]
        residuals[f"pickup:{label}"] = _fit_residual((record.x, record.y), (pred[0], pred[1]))
    for label in dropoff_labels:
        record = latest[f"dropoff:{label}"]
        pred = dropoff_low[dropoff_index_by_label[label]]
        residuals[f"dropoff:{label}"] = _fit_residual((record.x, record.y), (pred[0], pred[1]))

    checks = {
        "a1_match_m": residuals.get("pickup:A1", 0.0),
        "b4_match_m": residuals.get("pickup:B4", 0.0),
        "c1_match_m": residuals.get("dropoff:C1", 0.0),
        "c4_match_m": residuals.get("dropoff:C4", 0.0),
    }

    return {
        "model": "labeled_multi_anchor",
        "theta_ab_rad": theta_row,
        "theta_c_rad": theta_row,
        "theta_row_rad": theta_row,
        "theta_ab_deg": math.degrees(theta_row),
        "theta_c_deg": math.degrees(theta_row),
        "theta_row_deg": math.degrees(theta_row),
        "theta_row_delta_rad": 0.0,
        "theta_row_delta_deg": 0.0,
        "scale_ab": scale_ab,
        "d_c_row": d_c_row,
        "row_parallel": True,
        "dist_ab_map_m": scale_ab * math.hypot(B4_LOCAL[0], B4_LOCAL[1]),
        "dist_ab_local_m": math.hypot(B4_LOCAL[0], B4_LOCAL[1]),
        "forward_yaw": forward_yaw,
        "reverse_yaw": reverse_yaw,
        "high_layer_y_offset": high_layer_y_offset,
        "a1_map": [pickup_points[0][0], pickup_points[0][1]],
        "b4_map": [pickup_points[7][0], pickup_points[7][1]],
        "c1_map": [dropoff_low[0][0], dropoff_low[0][1]],
        "c4_low_map": [dropoff_low[3][0], dropoff_low[3][1]],
        "pickup_points": pickup_points,
        "dropoff_points": dropoff_points,
        "checks": checks,
        "labeled_residuals": residuals,
        "labeled_pickup_labels": sorted(pickup_labels),
        "labeled_dropoff_labels": sorted(dropoff_labels),
        "anchor_points_all": [record.pose for record in records],
    }


def load_anchors_from_file(
    path,
    *,
    high_layer_y_offset: float = 0.04,
    forward_yaw_override: float | None = None,
    reverse_yaw_override: float | None = None,
) -> dict:
    """Load labeled multi-anchor records when present, else legacy A1/B4/C1/C4."""
    labeled_records = parse_latest_labeled_session(path)
    latest = _latest_records_by_label(labeled_records)
    pickup_count = sum(1 for label in latest if label.startswith("pickup:"))
    dropoff_count = sum(1 for label in latest if label.startswith("dropoff:"))
    if pickup_count >= 3 and dropoff_count >= 2:
        return build_waypoints_from_labeled_records(
            labeled_records,
            high_layer_y_offset=high_layer_y_offset,
            forward_yaw_override=forward_yaw_override,
            reverse_yaw_override=reverse_yaw_override,
        )

    points = parse_anchor_points_file(path)
    a1_row, b4_row, c1_row, c4_row = parse_four_anchors(points)
    return build_waypoints_from_four_anchors(
        a1_row,
        b4_row,
        c1_row,
        c4_row,
        high_layer_y_offset=high_layer_y_offset,
        forward_yaw_override=forward_yaw_override,
        reverse_yaw_override=reverse_yaw_override,
        anchor_points_all=points,
    )


def map_to_local(
    origin: tuple[float, float],
    point: tuple[float, float],
    theta: float,
    scale: float = 1.0,
) -> tuple[float, float]:
    """Inverse of local_to_map (field-local dx along row, dy forward)."""
    if scale < 1e-9:
        scale = 1.0
    dx = point[0] - origin[0]
    dy = point[1] - origin[1]
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    local_x = (dx * cos_t + dy * sin_t) / scale
    local_y = (-dx * sin_t + dy * cos_t) / scale
    return local_x, local_y


def solve_scale_ab(a1: tuple[float, float], b4: tuple[float, float]) -> float:
    dist_map = math.hypot(b4[0] - a1[0], b4[1] - a1[1])
    dist_local = math.hypot(B4_LOCAL[0], B4_LOCAL[1])
    return dist_map / dist_local if dist_local > 1e-9 else 1.0


def solve_d_c_row(
    a1: tuple[float, float],
    c1: tuple[float, float],
    c4: tuple[float, float],
    theta_row: float,
    scale: float,
) -> float:
    _, local_y_c1 = map_to_local(a1, c1, theta_row, scale)
    _, local_y_c4 = map_to_local(a1, c4, theta_row, scale)
    return 0.5 * (local_y_c1 + local_y_c4)


def build_pickup_parallel(
    a1: tuple[float, float],
    theta_row: float,
    scale: float,
    reverse_yaw: float,
) -> list[tuple[float, float, float, float]]:
    pickup: list[tuple[float, float, float, float]] = []
    for local in A_FRONT_LOCAL:
        x, y = local_to_map(a1, local, theta_row, scale)
        pickup.append((x, y, reverse_yaw, 1.0))
    for local in B_REAR_LOCAL:
        x, y = local_to_map(a1, local, theta_row, scale)
        pickup.append((x, y, reverse_yaw, 0.0))
    return pickup


def build_dropoff_low_parallel(
    a1: tuple[float, float],
    c1: tuple[float, float],
    c4: tuple[float, float],
    theta_row: float,
    scale: float,
    forward_yaw: float,
) -> tuple[list[tuple[float, float, float]], float]:
    d_c = solve_d_c_row(a1, c1, c4, theta_row, scale)
    dropoff_low: list[tuple[float, float, float]] = []
    for dx in C_DX:
        x, y = local_to_map(a1, (dx, d_c), theta_row, scale)
        dropoff_low.append((x, y, forward_yaw))
    return dropoff_low, d_c


def build_waypoints_from_four_anchors(
    a1_row: tuple[float, float, float],
    b4_row: tuple[float, float, float],
    c1_row: tuple[float, float, float],
    c4_row: tuple[float, float, float],
    *,
    high_layer_y_offset: float = 0.025,
    forward_yaw_override: float | None = None,
    reverse_yaw_override: float | None = None,
    anchor_points_all: Iterable[tuple[float, float, float]] | None = None,
) -> dict:
    a1 = (a1_row[0], a1_row[1])
    b4 = (b4_row[0], b4_row[1])
    c1 = (c1_row[0], c1_row[1])
    c4 = (c4_row[0], c4_row[1])

    theta_ab = solve_theta_ab(a1, b4)
    theta_c = solve_theta_c(c1, c4)
    theta_row = average_angle(theta_ab, theta_c)

    if reverse_yaw_override is None:
        reverse_yaw = normalize_angle(average_yaw([a1_row[2], b4_row[2]]))
    else:
        reverse_yaw = normalize_angle(reverse_yaw_override)

    if forward_yaw_override is None:
        forward_yaw = normalize_angle(average_yaw([c1_row[2], c4_row[2]]))
    else:
        forward_yaw = normalize_angle(forward_yaw_override)

    scale_ab = solve_scale_ab(a1, b4)

    pickup_points = build_pickup_parallel(a1, theta_row, scale_ab, reverse_yaw)
    dropoff_low, d_c_row = build_dropoff_low_parallel(
        a1, c1, c4, theta_row, scale_ab, forward_yaw
    )
    dropoff_high = [
        (x, y + high_layer_y_offset, forward_yaw) for x, y, _ in dropoff_low
    ]
    dropoff_points = dropoff_low + dropoff_high

    theta_row_delta = abs(normalize_angle(theta_ab - theta_c))

    checks = {
        "a1_match_m": math.hypot(pickup_points[0][0] - a1[0], pickup_points[0][1] - a1[1]),
        "b4_match_m": math.hypot(pickup_points[7][0] - b4[0], pickup_points[7][1] - b4[1]),
        "c1_match_m": math.hypot(dropoff_low[0][0] - c1[0], dropoff_low[0][1] - c1[1]),
        "c4_match_m": math.hypot(dropoff_low[3][0] - c4[0], dropoff_low[3][1] - c4[1]),
    }

    return {
        "model": "legacy_four_anchor",
        "theta_ab_rad": theta_ab,
        "theta_c_rad": theta_c,
        "theta_row_rad": theta_row,
        "theta_ab_deg": math.degrees(theta_ab),
        "theta_c_deg": math.degrees(theta_c),
        "theta_row_deg": math.degrees(theta_row),
        "theta_row_delta_rad": theta_row_delta,
        "theta_row_delta_deg": math.degrees(theta_row_delta),
        "scale_ab": scale_ab,
        "d_c_row": d_c_row,
        "row_parallel": True,
        "dist_ab_map_m": math.hypot(b4[0] - a1[0], b4[1] - a1[1]),
        "dist_ab_local_m": math.hypot(B4_LOCAL[0], B4_LOCAL[1]),
        "forward_yaw": forward_yaw,
        "reverse_yaw": reverse_yaw,
        "high_layer_y_offset": high_layer_y_offset,
        "a1_map": list(a1),
        "b4_map": list(b4),
        "c1_map": list(c1),
        "c4_low_map": list(c4),
        "pickup_points": pickup_points,
        "dropoff_points": dropoff_points,
        "checks": checks,
        "anchor_points_all": list(anchor_points_all) if anchor_points_all else [],
    }


# Backward-compatible alias for old two-anchor API (delegates if 4 points available).
def build_waypoints_from_anchors(
    a1: tuple[float, float],
    c4_anchor: tuple[float, float],
    **kwargs,
) -> dict:
    raise NotImplementedError(
        "Use load_anchors_from_file with four anchors (A1, B4, C1, C4)"
    )


def flatten_pickup(points: Iterable[tuple[float, float, float, float]]) -> list[float]:
    flat: list[float] = []
    for x, y, yaw, is_front in points:
        flat.extend([x, y, yaw, is_front])
    return flat


def flatten_dropoff(points: Iterable[tuple[float, float, float]]) -> list[float]:
    flat: list[float] = []
    for x, y, yaw in points:
        flat.extend([x, y, yaw])
    return flat


def build_entry_trajectory(
    result: dict,
    *,
    start_local: tuple[float, float] = ENTRY_TRAJECTORY_START_LOCAL,
    end_local: tuple[float, float] = ENTRY_TRAJECTORY_END_LOCAL,
    dt_s: float = ENTRY_TRAJECTORY_DT_S,
    duration_s: float = ENTRY_TRAJECTORY_DURATION_S,
) -> list[TrajectoryRecord]:
    """Generate record_pass from navigation-semantic field anchors."""
    if dt_s <= 0.0:
        raise ValueError("entry trajectory dt_s must be positive")
    if duration_s <= 0.0:
        raise ValueError("entry trajectory duration_s must be positive")

    steps = int(round(duration_s / dt_s))
    if steps < 1:
        raise ValueError("entry trajectory needs at least two samples")

    origin = (float(result["a1_map"][0]), float(result["a1_map"][1]))
    theta = float(result["theta_row_rad"])
    scale = float(result["scale_ab"])
    yaw = normalize_angle(float(result["forward_yaw"]))

    records: list[TrajectoryRecord] = []
    for idx in range(steps + 1):
        alpha = idx / steps
        local_x = start_local[0] + alpha * (end_local[0] - start_local[0])
        local_y = start_local[1] + alpha * (end_local[1] - start_local[1])
        x, y = local_to_map(origin, (local_x, local_y), theta, scale)
        records.append(TrajectoryRecord(idx * dt_s, x, y, yaw))
    return records
