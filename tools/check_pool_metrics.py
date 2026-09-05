#!/usr/bin/env python3
"""Check the box-pool settling criterion from a completed TGS CSV (no GPU needed)."""
import argparse
import csv
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('csv', type=Path)
parser.add_argument('--tail', type=int, default=120)
parser.add_argument('--without-sleep', action='store_true')
args = parser.parse_args()
if args.tail < 1:
    parser.error('--tail must be positive')
with args.csv.open(newline='') as stream:
    rows = list(csv.DictReader(stream))
if len(rows) < args.tail:
    raise SystemExit('Not enough completed samples to check settling')
tail = rows[-args.tail:]
summary = {
    'file': str(args.csv), 'samples': len(rows), 'tail_samples': len(tail),
    'last_frame': int(rows[-1]['frame']),
    'max_speed_mm_s': max(int(row['maxv_mm']) for row in tail),
    'max_contact_pen_mm': max(int(row['pen_mm']) for row in tail),
    'max_deep200_contacts': max(int(row['deep200']) for row in tail),
    'min_sleeping': min(int(row['sleeping']) for row in tail),
}
# The scene defines success as all 432 cubes sleeping with no residual motion.
# maxv is integer mm/s and penetration is capped at 250 mm in narrow phase.
summary['passed'] = (
    all(int(row['solver']) == 3 for row in rows)
    and summary['max_speed_mm_s'] == 0
    and summary['max_deep200_contacts'] == 0
    and (args.without_sleep or summary['min_sleeping'] == 432)
)
print(json.dumps(summary, indent=2))
raise SystemExit(0 if summary['passed'] else 2)
