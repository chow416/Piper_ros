#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


COLORS = {
    "button_blue": (255, 80, 40),
    "button_red": (40, 40, 255),
    "button_green": (40, 230, 40),
}


def main():
    parser = argparse.ArgumentParser(description="Render a COCO annotation contact sheet.")
    parser.add_argument("dataset_dir", type=Path)
    parser.add_argument("--count", type=int, default=6)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    annotation_path = args.dataset_dir / "annotations" / "instances_all.json"
    dataset = json.loads(annotation_path.read_text(encoding="utf-8"))
    categories = {item["id"]: item["name"] for item in dataset["categories"]}
    annotations = {}
    for annotation in dataset["annotations"]:
        annotations.setdefault(annotation["image_id"], []).append(annotation)

    selected = dataset["images"][:max(1, args.count)]
    rendered = []
    for record in selected:
        image = cv2.imread(str(args.dataset_dir / "images" / record["file_name"]))
        if image is None:
            raise FileNotFoundError(record["file_name"])
        for annotation in annotations.get(record["id"], []):
            name = categories[annotation["category_id"]]
            x, y, width, height = annotation["bbox"]
            p0 = (int(round(x)), int(round(y)))
            p1 = (int(round(x + width)), int(round(y + height)))
            color = COLORS.get(name, (255, 255, 255))
            cv2.rectangle(image, p0, p1, color, 2)
            cv2.putText(
                image,
                name.replace("button_", ""),
                (p0[0], max(14, p0[1] - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )
        rendered.append(image)

    columns = min(3, len(rendered))
    rows = (len(rendered) + columns - 1) // columns
    cell_height, cell_width = rendered[0].shape[:2]
    sheet = np.zeros((rows * cell_height, columns * cell_width, 3), dtype=np.uint8)
    for index, image in enumerate(rendered):
        row, column = divmod(index, columns)
        sheet[
            row * cell_height:(row + 1) * cell_height,
            column * cell_width:(column + 1) * cell_width,
        ] = image

    output = args.output or args.dataset_dir / "preview.jpg"
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), sheet):
        raise RuntimeError(f"Failed to write {output}")
    print(output)


if __name__ == "__main__":
    main()
