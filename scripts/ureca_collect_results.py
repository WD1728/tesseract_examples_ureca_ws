#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    root = Path(args.root)
    output = Path(args.output)
    rows = []
    headers = None

    for summary in sorted(root.rglob("*.csv")):
        if summary.name.startswith("trajectory") or summary.name.endswith("_seed.csv") or summary.name.endswith("_optimized.csv"):
            continue
        with summary.open(newline="") as fh:
            reader = csv.DictReader(fh)
            if reader.fieldnames is None:
                continue
            if headers is None:
                headers = list(reader.fieldnames)
            for row in reader:
                row["_source_file"] = str(summary)
                rows.append(row)

    if headers is None:
      print("No summary CSV files found.")
      return 1

    headers = headers + ["_source_file"]
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as fh:
      writer = csv.DictWriter(fh, fieldnames=headers)
      writer.writeheader()
      writer.writerows(rows)

    print(f"Wrote {len(rows)} rows to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
