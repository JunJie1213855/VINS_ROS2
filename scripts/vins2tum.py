#!/usr/bin/env python3
"""Convert VINS vio.csv to TUM format for evo evaluation.

VINS format: timestamp, px, py, pz, qw, qx, qy, qz (comma-separated, no header)
TUM format:  timestamp tx ty tz qx qy qz qw (space-separated)
"""
import sys

def vins_to_tum(input_path: str, output_path: str):
    count = 0
    with open(input_path) as fin, open(output_path, 'w') as fout:
        for line in fin:
            line = line.strip()
            if not line:
                continue
            parts = line.split(',')
            if len(parts) < 8:
                continue
            ts = parts[0]
            px, py, pz = parts[1], parts[2], parts[3]
            qw, qx, qy, qz = parts[4], parts[5], parts[6], parts[7]
            # TUM: timestamp tx ty tz qx qy qz qw
            fout.write(f"{ts} {px} {py} {pz} {qx} {qy} {qz} {qw}\n")
            count += 1
    print(f"Converted {count} poses: {input_path} -> {output_path}")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <vio.csv> <output.tum>")
        sys.exit(1)
    vins_to_tum(sys.argv[1], sys.argv[2])
