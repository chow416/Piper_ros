"""Choose a deployment score threshold using validation data only, then test."""
import argparse
from pathlib import Path

import torch

from .detector import load_detector
from .train_detector import ButtonDataset, evaluate, save_preview, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--threads", type=int, default=6)
    parser.add_argument("--thresholds", type=float, nargs="+", default=[i/20 for i in range(1,20)])
    args = parser.parse_args()
    if any(not 0 < t < 1 for t in args.thresholds):
        parser.error("Thresholds must be strictly between zero and one")
    if args.output.exists() and any(args.output.iterdir()):
        parser.error("Output directory is not empty")
    args.output.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(args.threads)
    torch.set_num_interop_threads(1)
    model, checkpoint = load_detector(args.checkpoint)
    validation = ButtonDataset(args.dataset,"val")
    candidates = []
    for threshold in sorted(set(args.thresholds)):
        metrics, _ = evaluate(model,validation,threshold=threshold)
        candidates.append(metrics)
        print(f"val threshold={threshold:.2f} min_recall="
              f"{min(c['recall'] for c in metrics['per_class'].values()):.4f} "
              f"negative_false_detections={metrics['negative_images_with_false_detection']}",flush=True)
    def passes(m):
        return (m["negative_images"] > 0 and m["negative_images_with_false_detection"] == 0
                and all(c["precision"] >= 0.98 and c["recall"] >= 0.98 for c in m["per_class"].values()))
    passing = [m for m in candidates if passes(m)]
    if not passing:
        write_json(args.output/"calibration.json",{
            "status":"no_threshold_meets_validation_gates", "candidates":candidates,
            "checkpoint":str(args.checkpoint),"selected_epoch":checkpoint["epoch"]})
        print("No threshold meets validation gates; no deployment threshold selected.",flush=True)
        return
    selected = max(passing,key=lambda m:m["score_threshold"])
    threshold = selected["score_threshold"]
    calibration = {
        "status":"selected_on_validation_only", "threshold":threshold,
        "selection_rule":"highest threshold with per-class precision/recall >= 0.98 and zero negative false detections",
        "checkpoint":str(args.checkpoint),"selected_epoch":checkpoint["epoch"],
        "validation":selected,"candidates":candidates}
    write_json(args.output/"calibration.json",calibration)
    test = ButtonDataset(args.dataset,"test")
    metrics, predictions = evaluate(model,test,threshold=threshold)
    metrics["selected_epoch"] = checkpoint["epoch"]
    metrics["simulation_2d_gates_passed"] = (
        passes(metrics) and metrics["mAP_50"] >= 0.95
        and metrics["median_center_error_px"] is not None
        and metrics["median_center_error_px"] <= 5)
    calibration["status"] = ("passed_simulation_2d_gates" if metrics["simulation_2d_gates_passed"]
                             else "failed_test_gates_do_not_deploy")
    write_json(args.output/"calibration.json",calibration)
    write_json(args.output/"test_metrics.json",metrics)
    write_json(args.output/"test_predictions.json",predictions)
    save_preview(test,predictions,args.output/"test_preview.jpg",threshold)
    print(f"Selected threshold={threshold:.2f} on validation; test metrics: {metrics}",flush=True)


if __name__ == "__main__":
    main()
