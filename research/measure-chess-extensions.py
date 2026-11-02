"""Read tracked source only; do not import or execute downloaded projects."""
from pathlib import Path
import argparse
import json
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("snapshots", type=Path)
parser.add_argument("output", type=Path)
args = parser.parse_args()
extensions = {".py", ".c", ".cpp", ".h", ".hpp", ".ino"}
results = []
for repo in sorted(args.snapshots.iterdir()):
    if not (repo / ".git").exists():
        continue
    tracked = subprocess.check_output(
        ["git", "-C", str(repo), "ls-files", "-z"]
    ).decode().split("\0")
    rows = []
    for path in filter(None, tracked):
        if Path(path).suffix not in extensions:
            continue
        raw = (repo / path).read_bytes()
        lines = raw.decode("utf-8-sig", errors="replace").splitlines()
        rows.append({"path": path, "bytes": len(raw), "physical": len(lines),
                     "nonblank": sum(bool(line.strip()) for line in lines)})
    sha = subprocess.check_output(
        ["git", "-C", str(repo), "rev-parse", "HEAD"]
    ).decode().strip()
    results.append({"repository": repo.name.replace("__", "/"), "sha": sha,
                    "snapshot": str(repo), "all_tracked_files": len(list(filter(None, tracked))),
                    "source_files": rows,
                    "python_files": [row for row in rows if row["path"].endswith(".py")]})
args.output.write_text(json.dumps(results, indent=2, ensure_ascii=False), encoding="utf-8")
print(f"Saved {len(results)} repository counts to {args.output}")
