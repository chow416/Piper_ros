"""Checks for the pixel-coordinate and metric contracts used in deployment."""
import json
from pathlib import Path
import tempfile
import unittest

from PIL import Image
import torch

from piper_button_perception.detector import CLASSES, letterbox, restore_boxes
from piper_button_perception.train_detector import ButtonDataset, evaluate


class TrainingContracts(unittest.TestCase):
    def test_letterbox_round_trip_non_square(self):
        for size in [(320, 240), (641, 479), (199, 403)]:
            tensor, geometry = letterbox(Image.new("RGB", size))
            sx, sy, left, top = geometry
            original = torch.tensor([[1., 2., size[0]-2., size[1]-3.]])
            transformed = original.clone()
            transformed[:, [0, 2]] = transformed[:, [0, 2]]*sx+left
            transformed[:, [1, 3]] = transformed[:, [1, 3]]*sy+top
            self.assertEqual(tuple(tensor.shape), (3, 320, 320))
            torch.testing.assert_close(restore_boxes(transformed, geometry, size), original)

    def test_perfect_prediction_and_negative_false_detection(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/"images").mkdir()
            (root/"annotations").mkdir()
            for i in (1, 2):
                Image.new("RGB", (320,240)).save(root/"images"/f"{i}.jpg")
            data = {"images": [{"id":i,"width":320,"height":240,"file_name":f"{i}.jpg"} for i in (1,2)],
                    "categories":[{"id":i,"name":name} for i,name in CLASSES.items()],
                    "annotations":[{"id":1,"image_id":1,"category_id":1,"bbox":[20,30,40,40],
                                    "area":1600,"iscrowd":0}]}
            (root/"annotations"/"instances_test.json").write_text(json.dumps(data))
            dataset = ButtonDataset(root,"test")
            self.assertEqual(tuple(dataset[1][1]["boxes"].shape),(0,4))

            class FixedDetector(torch.nn.Module):
                def forward(self, images):
                    return [{"boxes":torch.tensor([[20.,70.,60.,110.]]),
                             "labels":torch.tensor([1]),"scores":torch.tensor([0.99])}
                            for _ in images]

            metrics, _ = evaluate(FixedDetector(),dataset,batch_size=2)
            self.assertEqual(metrics["per_class"]["button_blue"]["tp"],1)
            self.assertEqual(metrics["per_class"]["button_blue"]["fp"],1)
            self.assertEqual(metrics["per_class"]["button_blue"]["recall"],1)
            self.assertEqual(metrics["median_center_error_px"],0)
            self.assertEqual(metrics["negative_images_with_false_detection"],1)


if __name__ == "__main__":
    unittest.main()
