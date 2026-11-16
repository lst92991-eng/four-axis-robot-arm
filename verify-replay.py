"""Compare new offline outputs with the frozen SCARA handoff. No device I/O."""
import argparse
import hashlib
import json
from pathlib import Path


def sha256(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path, help="output directory of scara_vision_replay")
    parser.add_argument("geometry", type=Path, help="directory containing plane.yml and grid.json outputs")
    options = parser.parse_args()
    baseline = Path(__file__).with_name("refactor-baseline.json")
    reference = json.loads(baseline.read_text(encoding="utf-8"))
    mismatches = []

    for frame in reference["replay"]:
        stem = options.replay / frame["id"]
        record = json.loads(stem.with_suffix(".json").read_text(encoding="utf-8"))
        if sha256(stem.with_suffix(".rgb")) != frame["rgb_sha256"]:
            mismatches.append(stem.name + ": RGB")
        if sha256(stem.with_suffix(".png")) != frame["preview_png_sha256"]:
            mismatches.append(stem.name + ": preview")
        if record["detections"] != frame["detections"]:
            mismatches.append(stem.name + ": detections")

    for name, saved in reference["geometry"].items():
        if sha256(options.geometry / name) != saved["sha256"]:
            mismatches.append(name + ": geometry")

    if mismatches:
        raise SystemExit("Outputs differ from the frozen handoff:\n" + "\n".join(mismatches))
    print(f"PASS {len(reference['replay'])} replay cases and {len(reference['geometry'])} geometry outputs")


if __name__ == "__main__":
    main()
