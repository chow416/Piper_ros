# Colored elevator button perception plan

> **Archived design note:** perception development was paused on 2026-09-22.
> The planning/control stack must accept an explicit world-frame target and
> must not depend on this design or on `piper_button_perception`.

## Scope

The first perception baseline detects the blue, red, and green physical button
discs in the wrist RGB image.  The neural network is responsible for 2D object
detection and color classification.  Depth and camera calibration are then used
to recover the selected button centre in 3D; TF converts that point into
`base_link` for the planning and control layer.

Color thresholding should be kept as a diagnostic baseline, but the production
ROS interface must not depend on whether the detector is HSV- or CNN-based.

## Recommended first model

- Primary engineering baseline: `ssdlite320_mobilenet_v3_large` from
  TorchVision, fine-tuned for three classes plus background.
- Input: 320 x 320 RGB, letterboxed from the 320 x 240 camera frame.
- Classes: `button_blue`, `button_red`, `button_green`.
- Deployment: export to ONNX and run with OpenVINO on Intel CPU first; test the
  Intel integrated GPU only after CPU inference is stable.
- Fast research alternative: an Ultralytics nano detector.  Check its AGPL or
  commercial licensing obligations before using it in a closed product.

SSDLite is preferred here because localization is required; an image classifier
alone can identify a color but cannot tell the controller where to move.

## Data generation

1. Subscribe to `/wrist_camera/depth/image_raw` and `/camera_info`.
2. Read the separately named Gazebo models `elevator_button_blue`,
   `elevator_button_red`, and `elevator_button_green` from model states.
3. Project each known 3D disc into the image and write COCO or YOLO-format boxes.
4. Generate 3,000-5,000 synthetic frames while randomizing:
   - camera and arm pose;
   - panel position and orientation;
   - light direction and intensity;
   - background/table material;
   - exposure, blur, sensor noise, and partial occlusion.
5. Split by randomization seed/scene, not by adjacent video frames: 70% train,
   15% validation, 15% test.
6. When the real camera is available, label 200-500 real frames and fine-tune.

Hue augmentation must be small because hue is the class label.  Brightness,
contrast, saturation, blur, noise, crop, and occlusion augmentation can be much
stronger.

## Training and acceptance targets

- Initialize from COCO-pretrained weights.
- Train 50-100 epochs with early stopping; use a remote/borrowed GPU if
  available.  CPU training is possible but is not the intended daily workflow.
- Select the checkpoint by validation mAP and per-class recall, not training
  loss alone.
- Initial simulation gates:
  - per-class recall >= 0.98;
  - mAP@0.5 >= 0.95;
  - median 2D centre error <= 5 px;
  - median reconstructed 3D centre error <= 5 mm;
  - zero false target publications in the negative-scene test set.

## ROS 2 inference pipeline

```text
RGB image --> detector --> Detection2DArray
                               |
Depth + CameraInfo ------------+--> robust depth median --> 3D camera point
                                                        --> TF to base_link
task target color --> selector -------------------------> stable ButtonTarget
                                                        --> planning/control
```

Recommended topics:

- input `/wrist_camera/depth/image_raw` (`sensor_msgs/Image`)
- input `/wrist_camera/depth/depth/image_raw` (`sensor_msgs/Image`)
- input `/wrist_camera/depth/camera_info` (`sensor_msgs/CameraInfo`)
- input `/task/target_button_color` (`std_msgs/String`, prototype only)
- output `/perception/buttons_2d` (`vision_msgs/Detection2DArray`)
- output `/perception/selected_button_pose` (`geometry_msgs/PoseStamped`)
- output `/perception/selected_button_normal`
  (`geometry_msgs/Vector3Stamped`)

For the stable interface, define a `ButtonTarget` message containing timestamp,
color/class ID, confidence, pose, panel normal, source pixel, and measured depth.
The planning layer should consume only targets transformed into `base_link` and
newer than a configured timeout.

## Safety and filtering

- Require confidence >= 0.70 and the same class in at least 3 of 5 frames.
- Use the median valid depth from a small central ROI, not one depth pixel.
- Reject depth outside the camera range or with excessive local variance.
- Fit a local plane to depth for the panel normal; use the known simulation
  normal only as the first fallback.
- Never issue a press command directly from one detector frame.
- The control layer must still enforce reachability, collision checking,
  approach distance, speed limits, and contact/force termination.

## Deployment layout

Keep training dependencies out of the normal simulation image.  Use one
training image for PyTorch and a smaller runtime layer containing ROS 2 Humble,
OpenCV/cv_bridge, vision_msgs, TF2, and OpenVINO.  Mount the exported model into
the runtime container read-only and record its version and class map alongside
the ONNX/OpenVINO files.
