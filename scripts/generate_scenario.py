#!/usr/bin/env python3
"""
Generate a randomised competition scenario for sim-to-real robustness testing.

For a given integer seed, deterministically picks:
  * which corner the robot is dropped into (the "no-tag" corner),
  * which colour gets which of the remaining 3 corners,
  * 3 puck positions inside the arena, away from walls/zones/each other,
  * a randomised yaw for the robot (still roughly facing the arena centre).

Outputs a single yaml that doubles as the spawn_objects config (same schema as
the canonical objects.yaml) plus an extra `robot:` block describing where the
robot should be spawned. The runner script reads `robot:` to pass to roslaunch
and the spawn_objects node consumes the puck/drop_zone blocks unchanged.

Usage:
    python3 scripts/generate_scenario.py --seed 7 --output logs/scenario_7.yaml
"""

import argparse
import math
import random
import sys
from pathlib import Path

import yaml


# Arena bbox: walls at +/- with 0.05 thickness; usable interior approx
ARENA_X = (-1.35, 1.35)
ARENA_Y = (-1.00, 1.00)

# Corner anchor positions (centred 0.20 m inside each corner).
CORNERS = {
    "SE": ( 1.15, -0.85),
    "SW": (-1.15, -0.85),
    "NW": (-1.15,  0.85),
    "NE": ( 1.15,  0.85),
}
CORNER_ORDER = ["SE", "SW", "NW", "NE"]

# Constraints for placing pucks.
WALL_BUFFER     = 0.30  # min distance from any wall
ZONE_BUFFER     = 0.45  # min distance from any drop zone
ROBOT_BUFFER    = 0.50  # min distance from robot spawn
PUCK_SEPARATION = 0.40  # min distance between any two pucks

# Marker IDs are stable per colour (perception expects this mapping).
COLOR_MARKER_ID = {"red": 1, "green": 2, "blue": 3}


def yaw_facing_center(x, y, jitter_rad):
    """Yaw pointing roughly toward (0,0), with a small random offset."""
    return math.atan2(0.0 - y, 0.0 - x) + jitter_rad


def yaw_facing_inward(corner_xy):
    """Yaw of the marker face so it points into the arena (toward the centre)."""
    cx, cy = corner_xy
    return math.atan2(0.0 - cy, 0.0 - cx)


def sample_puck_pose(rng, robot_xy, zones, existing):
    """Reject-sample a puck position that obeys all spacing constraints."""
    x_lo = ARENA_X[0] + WALL_BUFFER
    x_hi = ARENA_X[1] - WALL_BUFFER
    y_lo = ARENA_Y[0] + WALL_BUFFER
    y_hi = ARENA_Y[1] - WALL_BUFFER
    for _ in range(500):
        x = rng.uniform(x_lo, x_hi)
        y = rng.uniform(y_lo, y_hi)
        if math.hypot(x - robot_xy[0], y - robot_xy[1]) < ROBOT_BUFFER:
            continue
        if any(math.hypot(x - zx, y - zy) < ZONE_BUFFER for zx, zy in zones):
            continue
        if any(math.hypot(x - px, y - py) < PUCK_SEPARATION for px, py in existing):
            continue
        return x, y
    raise RuntimeError("Could not sample a puck pose; constraints too tight.")


def build_scenario(seed):
    rng = random.Random(int(seed))

    # 1) Pick the no-tag corner (where the robot is dropped). Seed 0 keeps the
    #    canonical SE corner so the default still matches objects.yaml.
    no_tag_corner = CORNER_ORDER[rng.randrange(0, 4)] if seed != 0 else "SE"

    tag_corners = [c for c in CORNER_ORDER if c != no_tag_corner]

    # 2) Shuffle the colour -> corner assignment among the remaining 3 corners.
    colours = ["red", "green", "blue"]
    rng.shuffle(colours)
    color_to_corner = dict(zip(colours, tag_corners))

    # 3) Robot spawn: jitter the canonical corner pose +/- 0.15 m and yaw +/- 0.4 rad.
    base_x, base_y = CORNERS[no_tag_corner]
    rx = base_x + rng.uniform(-0.15, 0.15)
    ry = base_y + rng.uniform(-0.10, 0.10)
    ryaw = yaw_facing_center(rx, ry, rng.uniform(-0.4, 0.4))

    # 4) Drop zones: one ArUco marker per tagged corner.
    drop_zones = []
    for colour, corner_name in color_to_corner.items():
        cx, cy = CORNERS[corner_name]
        # Push the marker right up against its corner wall.
        marker_yaw = yaw_facing_inward((cx, cy))
        # Blue corner used a 90-deg twist to disambiguate the marker face from
        # the spawn direction; keep the same trick for whichever colour ends up
        # at the corner whose face would otherwise be "behind" the marker.
        drop_zones.append(
            {
                "name": f"zone_{colour}",
                "marker_id": COLOR_MARKER_ID[colour],
                "color": colour,
                "x": float(cx),
                "y": float(cy),
                "z": 0.15,
                "yaw": float(marker_yaw),
            }
        )

    zone_xy = [(z["x"], z["y"]) for z in drop_zones]

    # 5) Pucks: random valid positions, one per colour.
    pucks = []
    placed = []
    for colour in ["red", "green", "blue"]:
        x, y = sample_puck_pose(rng, (rx, ry), zone_xy, placed)
        placed.append((x, y))
        pucks.append(
            {
                "name": f"puck_{colour}",
                "color": colour,
                "x": float(x),
                "y": float(y),
                "z": 0.02,
                "yaw": float(rng.uniform(-math.pi, math.pi)),
            }
        )

    return {
        "robot": {
            "x": float(rx),
            "y": float(ry),
            "z": 0.05,
            "yaw": float(ryaw),
            "no_tag_corner": no_tag_corner,
        },
        "pucks": pucks,
        "drop_zones": drop_zones,
        "_meta": {
            "seed": int(seed),
            "color_to_corner": {c: color_to_corner[c] for c in colours},
        },
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()

    scenario = build_scenario(args.seed)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as fh:
        yaml.safe_dump(scenario, fh, sort_keys=False, default_flow_style=False)

    # Print key=value to stdout so shell scripts can `eval $(...)` for the
    # robot spawn pose without re-parsing yaml.
    robot = scenario["robot"]
    print(f"ROBOT_X={robot['x']:.4f}")
    print(f"ROBOT_Y={robot['y']:.4f}")
    print(f"ROBOT_YAW={robot['yaw']:.4f}")
    print(f"NO_TAG_CORNER={robot['no_tag_corner']}")
    print(f"SCENARIO={args.output}")
    print(f"COLORS={','.join(f'{k}={v}' for k, v in scenario['_meta']['color_to_corner'].items())}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
