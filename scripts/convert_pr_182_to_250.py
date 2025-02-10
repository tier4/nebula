#!/usr/bin/python3

import json
import sys

from PIL import Image
import numpy as np
from ruamel.yaml import YAML


def load_yaml(yaml_path):
    with open(yaml_path, "r") as file:
        yaml = YAML()
        data = yaml.load(file)

    try:
        filters = data["/**"]["ros__parameters"]["point_filters"]
        filters_dict: dict = json.loads(filters)  # Convert string representation to dictionary
        return filters_dict["ring_section_filter"]
    except (KeyError, SyntaxError):
        print("Invalid YAML structure or missing 'ring_section_filter'.")
        sys.exit(1)


def generate_mask(ring_section_filter):
    width, height = 3600, 128
    mask = np.full((height, width), 255, dtype=np.uint8)

    for ring, azi_start, azi_end in ring_section_filter:
        if ring < 0 or ring >= 128:
            raise ValueError(f"Invalid ring: {ring}")

        if azi_end < azi_start:
            mask[ring, int(10 * azi_start) : 3600] = 0  # Set pixels to black
            mask[ring, 0 : int(10 * azi_end)] = 0  # Set pixels to black

        else:
            mask[ring, int(10 * azi_start) : int(10 * azi_end)] = 0  # Set pixels to black

    return mask


def save_png(mask, output_path):
    img = Image.fromarray(mask, mode="L")  # 'L' mode for grayscale
    img.save(output_path)


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <input_yaml>")
        sys.exit(1)

    yaml_path = sys.argv[1]
    output_path = yaml_path.rsplit(".", 1)[0] + ".png"

    ring_section_filter = load_yaml(yaml_path)
    mask = generate_mask(ring_section_filter)
    save_png(mask, output_path)

    print(f"Mask saved to {output_path}")


if __name__ == "__main__":
    main()
