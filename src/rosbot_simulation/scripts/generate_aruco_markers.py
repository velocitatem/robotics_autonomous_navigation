#!/usr/bin/env python3

from pathlib import Path
import argparse

import cv2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output_dir", required=True)
    args = parser.parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    for marker_id in (1, 2, 3):
        img = aruco.drawMarker(dictionary, marker_id, 512)
        path = output_dir / f"marker{marker_id}.png"
        cv2.imwrite(str(path), img)
        print(f"[SIM] Generated {path}")


if __name__ == "__main__":
    main()
