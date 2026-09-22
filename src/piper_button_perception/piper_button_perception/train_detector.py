"""CPU SSDLite training, validation selection and held-out COCO evaluation."""
import argparse
from contextlib import redirect_stdout
from collections import defaultdict
import hashlib
import io
import json
import math
from pathlib import Path
import random
import time

import numpy as np
from PIL import Image, ImageDraw, ImageEnhance
from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval
import torch
from torch.utils.data import Dataset, DataLoader
import torchvision
from torchvision.ops import box_iou

from .detector import CLASSES, build_model, letterbox, restore_boxes, load_detector


class ButtonDataset(Dataset):
    def __init__(self, root, split, augment=False):
        self.root = Path(root)
        self.annotation_path = self.root / "annotations" / f"instances_{split}.json"
        self.data = json.loads(self.annotation_path.read_text())
        if {c["id"]: c["name"] for c in self.data["categories"]} != CLASSES:
            raise ValueError("Unexpected COCO categories")
        self.images = self.data["images"]
        self.annotations = defaultdict(list)
        for annotation in self.data["annotations"]:
            self.annotations[annotation["image_id"]].append(annotation)
        self.augment = augment

    def __len__(self):
        return len(self.images)

    def original(self, index):
        with Image.open(self.root / "images" / self.images[index]["file_name"]) as image:
            return image.convert("RGB")

    def __getitem__(self, index):
        record = self.images[index]
        image = self.original(index)
        boxes, labels = [], []
        for annotation in self.annotations[record["id"]]:
            x, y, w, h = annotation["bbox"]
            boxes.append([x, y, x + w, y + h])
            labels.append(annotation["category_id"])
        boxes = torch.tensor(boxes, dtype=torch.float32).reshape(-1, 4)
        if self.augment:
            if random.random() < 0.5:
                image = image.transpose(Image.Transpose.FLIP_LEFT_RIGHT)
                boxes[:, [0, 2]] = image.width - boxes[:, [2, 0]]
            image = ImageEnhance.Brightness(image).enhance(random.uniform(0.9, 1.1))
        tensor, (sx, sy, left, top) = letterbox(image)
        boxes[:, [0, 2]] = boxes[:, [0, 2]] * sx + left
        boxes[:, [1, 3]] = boxes[:, [1, 3]] * sy + top
        return tensor, {"boxes": boxes, "labels": torch.tensor(labels, dtype=torch.int64),
                        "image_id": torch.tensor(record["id"])}


def collate(batch):
    return tuple(zip(*batch))


def audit_datasets(datasets):
    """Check split disjointness including exact duplicate image contents."""
    ids, hashes, digest = set(), {}, hashlib.sha256()
    result = {}
    for split, dataset in datasets.items():
        current = {r["id"] for r in dataset.images}
        if len(current) != len(dataset.images) or ids & current:
            raise ValueError("Duplicate image IDs within/across dataset splits")
        ids.update(current)
        digest.update(dataset.annotation_path.read_bytes())
        for index, record in enumerate(dataset.images):
            path = dataset.root / "images" / record["file_name"]
            sha = hashlib.sha256(path.read_bytes()).hexdigest()
            if sha in hashes and hashes[sha] != split:
                raise ValueError("An identical image appears in different splits")
            hashes[sha] = split
            digest.update(sha.encode())
            image = dataset.original(index)
            if image.size != (record["width"], record["height"]):
                raise ValueError(f"Image size mismatch: {path}")
            for ann in dataset.annotations[record["id"]]:
                x, y, w, h = ann["bbox"]
                if not (ann["category_id"] in CLASSES and x >= 0 and y >= 0 and w > 0 and h > 0
                        and x+w <= image.width+1e-4 and y+h <= image.height+1e-4):
                    raise ValueError(f"Invalid box: {ann}")
        result[split] = {"images": len(dataset), "annotations": len(dataset.data["annotations"]),
                         "negative_images": sum(not dataset.annotations[r["id"]] for r in dataset.images)}
    result["sha256"] = digest.hexdigest()
    return result


@torch.inference_mode()
def evaluate(model, dataset, batch_size=8, threshold=0.7):
    model.eval()
    predictions = []
    loader = DataLoader(dataset, batch_size=batch_size, collate_fn=collate, num_workers=0)
    records = {r["id"]: r for r in dataset.images}
    started = time.perf_counter()
    for images, targets in loader:
        outputs = model(list(images))
        for target, output in zip(targets, outputs):
            image_id = int(target["image_id"])
            record = records[image_id]
            width, height = record["width"], record["height"]
            scale = 320 / max(width, height)
            rw, rh = round(width*scale), round(height*scale)
            geometry = (rw/width, rh/height, (320-rw)//2, (320-rh)//2)
            boxes = restore_boxes(output["boxes"], geometry, (width, height))
            for box, label, score in zip(boxes.tolist(), output["labels"].tolist(), output["scores"].tolist()):
                x1, y1, x2, y2 = box
                if x2 > x1 and y2 > y1:
                    predictions.append({"image_id": image_id, "category_id": label,
                                        "bbox": [x1, y1, x2-x1, y2-y1], "score": score})
    elapsed = time.perf_counter() - started
    metrics = {"mAP_50_95": 0.0, "mAP_50": 0.0}
    if predictions:
        with redirect_stdout(io.StringIO()):
            gt = COCO(str(dataset.annotation_path))
            gt.dataset.setdefault("info", {})
            evaluator = COCOeval(gt, gt.loadRes(predictions), "bbox")
            evaluator.evaluate()
            evaluator.accumulate()
            evaluator.summarize()
        metrics.update(mAP_50_95=float(evaluator.stats[0]), mAP_50=float(evaluator.stats[1]))
    per_image = defaultdict(list)
    for p in predictions:
        if p["score"] >= threshold:
            per_image[p["image_id"]].append(p)
    counts = {i: {"tp": 0, "fp": 0, "fn": 0} for i in CLASSES}
    centre_errors, negative_frames, false_negative_frames = [], 0, 0
    for record in dataset.images:
        image_id = record["id"]
        gt_anns = dataset.annotations[image_id]
        if not gt_anns:
            negative_frames += 1
            false_negative_frames += bool(per_image[image_id])
        for category in CLASSES:
            truth = [a["bbox"] for a in gt_anns if a["category_id"] == category]
            dets = sorted((p for p in per_image[image_id] if p["category_id"] == category),
                          key=lambda p: -p["score"])
            unmatched = set(range(len(truth)))
            for p in dets:
                x, y, w, h = p["bbox"]
                match, best_iou = None, 0.5
                for j in unmatched:
                    gx, gy, gw, gh = truth[j]
                    iou = float(box_iou(torch.tensor([[x,y,x+w,y+h]]),
                                        torch.tensor([[gx,gy,gx+gw,gy+gh]]))[0, 0])
                    if iou >= best_iou:
                        match, best_iou = j, iou
                if match is None:
                    counts[category]["fp"] += 1
                else:
                    unmatched.remove(match)
                    counts[category]["tp"] += 1
                    gx, gy, gw, gh = truth[match]
                    centre_errors.append(math.hypot(x+w/2-gx-gw/2, y+h/2-gy-gh/2))
            counts[category]["fn"] += len(unmatched)
    metrics["per_class"] = {
        CLASSES[i]: dict(c, precision=c["tp"]/max(1,c["tp"]+c["fp"]),
                        recall=c["tp"]/max(1,c["tp"]+c["fn"])) for i,c in counts.items()}
    metrics.update(score_threshold=threshold, matching_iou=0.5, negative_images=negative_frames,
                   negative_images_with_false_detection=false_negative_frames,
                   median_center_error_px=float(np.median(centre_errors)) if centre_errors else None,
                   evaluation_seconds=elapsed, images=len(dataset))
    return metrics, predictions


def save_preview(dataset, predictions, path, threshold):
    per_image = defaultdict(list)
    for prediction in predictions:
        if prediction["score"] >= threshold:
            per_image[prediction["image_id"]].append(prediction)
    # Include negatives explicitly, and sample across the held-out set.
    indices = [i for i,r in enumerate(dataset.images) if not dataset.annotations[r["id"]]][:3]
    indices += [i for i in np.linspace(0, len(dataset)-1, 12, dtype=int).tolist() if i not in indices]
    indices = indices[:12]
    sheet = Image.new("RGB", (320*3, 270*4), "#222222")
    colors = {1: "#3388ff", 2: "#ff5555", 3: "#44ee66"}
    for cell, index in enumerate(indices):
        image = dataset.original(index)
        draw = ImageDraw.Draw(image)
        record = dataset.images[index]
        for ann in dataset.annotations[record["id"]]:
            x,y,w,h = ann["bbox"]
            draw.rectangle((x,y,x+w,y+h), outline="white", width=1)
        for p in per_image[record["id"]]:
            x,y,w,h = p["bbox"]
            draw.rectangle((x,y,x+w,y+h), outline=colors[p["category_id"]], width=2)
            draw.text((max(0,x),max(0,y-12)), f'{CLASSES[p["category_id"]][7:]} {p["score"]:.2f}',
                      fill=colors[p["category_id"]], stroke_width=1, stroke_fill="black")
        x, y = (cell%3)*320, (cell//3)*270
        sheet.paste(image.resize((320,240)), (x,y+30))
        ImageDraw.Draw(sheet).text((x+5,y+8), record["file_name"], fill="white")
    sheet.save(path)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+"\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--threads", type=int, default=6)
    parser.add_argument("--lr", type=float, default=0.003)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--patience", type=int, default=10)
    parser.add_argument("--threshold", type=float, default=0.7)
    parser.add_argument("--checkpoint", type=Path, help="Evaluate an existing checkpoint without training")
    parser.add_argument("--initialize", type=Path, help="Fine-tune from an existing button checkpoint")
    parser.add_argument("--unfreeze-backbone", action="store_true", help="Fine-tune the feature extractor as well")
    args = parser.parse_args()
    if args.checkpoint and args.initialize:
        parser.error("--checkpoint is evaluation-only and cannot be combined with --initialize")
    if args.epochs < 1 or args.batch_size < 2 or args.threads < 1:
        parser.error("epochs/threads must be positive; SSDLite training requires batch-size >= 2")
    if args.output.exists() and any(args.output.iterdir()):
        parser.error("Output directory is not empty; choose a new run name to preserve existing results")
    args.output.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(args.threads)
    torch.set_num_interop_threads(1)
    torch.manual_seed(args.seed)
    random.seed(args.seed)
    np.random.seed(args.seed)
    datasets = {s: ButtonDataset(args.dataset, s) for s in ("train","val","test")}
    audit = audit_datasets(datasets)
    config = {k: str(v) if isinstance(v,Path) else v for k,v in vars(args).items()}
    config.update(torch_version=str(torch.__version__), torchvision_version=str(torchvision.__version__),
                  device="cpu", classes=CLASSES, preprocessing="letterbox320_rgb_114",
                  frozen_backbone=not args.unfreeze_backbone, dataset_audit=audit,
                  initialization=str(args.initialize or args.checkpoint or "COCO_V1"))
    write_json(args.output/"run_config.json", config)
    print(json.dumps(config, indent=2), flush=True)
    if args.checkpoint:
        model, checkpoint = load_detector(args.checkpoint)
    else:
        model = build_model(pretrained=not bool(args.initialize), freeze_backbone=not args.unfreeze_backbone)
        if args.initialize:
            initial = torch.load(args.initialize,map_location="cpu",weights_only=True)
            if initial["classes"] != CLASSES or initial["preprocessing"] != "letterbox320_rgb_114":
                raise ValueError("Initialization checkpoint has incompatible classes or preprocessing")
            model.load_state_dict(initial["model"])
        train_dataset = ButtonDataset(args.dataset,"train",augment=True)
        # Avoid a singleton final batch: training BN sees 1x1 feature maps.
        loader = DataLoader(train_dataset, batch_size=args.batch_size, shuffle=True,
                            num_workers=0, collate_fn=collate,
                            drop_last=len(train_dataset) % args.batch_size == 1)
        optimizer = torch.optim.SGD([p for p in model.parameters() if p.requires_grad],
                                    lr=args.lr, momentum=0.9, weight_decay=0.0005)
        scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, args.epochs, eta_min=args.lr*0.05)
        best, stale, history = -1, 0, []
        for epoch in range(1,args.epochs+1):
            started = time.perf_counter()
            model.train()
            if not args.unfreeze_backbone:
                model.backbone.eval()  # Freeze pretrained BN statistics with the backbone.
            loss_sum = defaultdict(float)
            for step, (images, targets) in enumerate(loader,1):
                optimizer.zero_grad(set_to_none=True)
                losses = model(list(images), list(targets))
                loss = sum(losses.values())
                if not torch.isfinite(loss):
                    raise RuntimeError(f"Nonfinite loss at epoch {epoch}, step {step}")
                loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), 10.0)
                optimizer.step()
                for key,value in losses.items():
                    loss_sum[key] += float(value.detach())
                if step % 15 == 0:
                    print(f"epoch {epoch}/{args.epochs} step {step}/{len(loader)} loss={float(loss.detach()):.4f}", flush=True)
            metrics, _ = evaluate(model,datasets["val"],args.batch_size,args.threshold)
            row = {"epoch":epoch,"lr":optimizer.param_groups[0]["lr"],
                   "loss":{k:v/len(loader) for k,v in loss_sum.items()},
                   "validation":metrics,"seconds":time.perf_counter()-started}
            history.append(row)
            write_json(args.output/"history.json",history)
            scheduler.step()
            checkpoint = {"model":model.state_dict(),"classes":CLASSES,"epoch":epoch,
                          "preprocessing":"letterbox320_rgb_114", "validation":metrics,
                          "dataset_sha256":audit["sha256"]}
            torch.save(checkpoint,args.output/"last.pt")
            if metrics["mAP_50_95"] > best:
                best, stale = metrics["mAP_50_95"], 0
                torch.save(checkpoint,args.output/"best.pt")
            else:
                stale += 1
            print(f"EPOCH {epoch}: loss={sum(row['loss'].values()):.4f} "
                  f"val_AP50={metrics['mAP_50']:.4f} val_AP50_95={metrics['mAP_50_95']:.4f} "
                  f"seconds={row['seconds']:.1f} best={best:.4f}",flush=True)
            if stale >= args.patience:
                print(f"Early stopping after {stale} epochs without validation AP improvement",flush=True)
                break
        model, checkpoint = load_detector(args.output/"best.pt")
    # The held-out test split is evaluated only after checkpoint selection is complete.
    metrics, predictions = evaluate(model,datasets["test"],args.batch_size,args.threshold)
    image, _ = datasets["test"][0]
    timings = []
    with torch.inference_mode():
        for i in range(25):
            started = time.perf_counter()
            model([image])
            if i >= 5:
                timings.append((time.perf_counter()-started)*1000)
    metrics.update(selected_epoch=checkpoint["epoch"], single_frame_model_median_ms=float(np.median(timings)),
                   single_frame_model_p95_ms=float(np.percentile(timings,95)))
    metrics["simulation_2d_gates"] = {
        "map50_ge_0_95":metrics["mAP_50"]>=0.95,
        "all_class_recall_ge_0_98":all(c["recall"]>=0.98 for c in metrics["per_class"].values()),
        "median_center_error_le_5px":metrics["median_center_error_px"] is not None and metrics["median_center_error_px"]<=5,
        "no_negative_false_detections":metrics["negative_images"]>0 and metrics["negative_images_with_false_detection"]==0,
    }
    write_json(args.output/"test_metrics.json",metrics)
    write_json(args.output/"test_predictions.json",predictions)
    save_preview(datasets["test"],predictions,args.output/"test_preview.jpg",args.threshold)
    print("FINAL TEST "+json.dumps(metrics,ensure_ascii=False),flush=True)


if __name__ == "__main__":
    main()
