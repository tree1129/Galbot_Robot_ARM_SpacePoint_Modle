"""Replay a recorded scene through the non-actuating grasp planner."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .pipeline import GraspCoordinator
from .reachability import DualArmReachability
from .serialization import parse_scene


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", type=Path)
    parser.add_argument("--command", required=True)
    parser.add_argument("--data-dir", type=Path, default=Path("data"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    raw = json.loads(args.scene.read_text(encoding="utf-8"))
    candidates, obstacles, safety, current_tcp = parse_scene(raw)
    plan = GraspCoordinator(DualArmReachability(args.data_dir)).plan(
        args.command, candidates, obstacles, safety, current_tcp
    )
    rendered = json.dumps(plan.to_dict(), ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0 if plan.status == "READY_FOR_SHADOW_REVIEW" else 2


if __name__ == "__main__":
    raise SystemExit(main())
