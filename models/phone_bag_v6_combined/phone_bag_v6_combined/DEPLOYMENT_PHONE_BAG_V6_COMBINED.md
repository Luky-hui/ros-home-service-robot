# Phone/Bag V6 Combined Deployment

Class order:

- 0: phone
- 1: bag

Model files:

- PT: `<local-training-output>/phone_bag_yolo11n_v6_combined_best.pt`
- ONNX: `<local-training-output>/phone_bag_yolo11n_v6_combined_best.onnx`

Training data:

- Old V5 dataset: `<local-training-dataset>/phone_bag_model_v5_annotated/dataset`
- New near-distance dataset: `<local-training-dataset>/near-distance-phone-bag`
- Empty-label negative images: vegetable combo V2 and V14 anti-false-positive backgrounds

Important class remap:

- Old V5 dataset: `0=phone, 1=bag`
- New near-distance dataset source: `0=bag, 1=phone`
- V6 unified output: `0=phone, 1=bag`

Best validation epoch:

- epoch: 8
- precision: 0.99587
- recall: 0.97415
- mAP50: 0.99450
- mAP50-95: 0.81983

Validation filter check at `conf=0.50`:

- validation images: 153
- positive images: 48
- background images: 105
- detections: 56
- background detections: 0
- class detections: phone 28, bag 28

Runtime near/far filter:

```python
area_ratio = (x2 - x1) * (y2 - y1) / (image_width * image_height)
should_broadcast = confidence >= 0.50 and area_ratio >= 0.015
```

Use `area_ratio >= 0.015` as the first deployment threshold. If the robot misses valid near phones, lower it to `0.010`. If hallway distant detections still get broadcast, raise it to `0.020`.
