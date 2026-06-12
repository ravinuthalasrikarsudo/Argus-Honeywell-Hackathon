# argus_superpoint — learned feature front-end (Pillar 2, ablation C2)

ONNX Runtime SuperPoint keypoint detector (CUDA EP on the RTX 4050, automatic
CPU fallback) feeding external keypoints into the patched VINS-Fusion tracker
(`use_superpoint: 1` in the C2 config).

Models live in `models/superpoint/` (re-fetch with `download_models.sh`).
Throughput ≥ 15 Hz validated on dGPU; run tally confirms SuperPoint drives
~99.9 % of frames when enabled.

**Status — documented negative result.** SuperPoint detections optimise
descriptor-matching repeatability, not LK optical-flow trackability: on long
shuttle runs the C2 init diverges (root cause analysis in
`docs/daily_logs/day5.md`). KLT/Harris (C1) remains the production front-end;
C2 ships behind a flag as the ablation + starting point for a future
descriptor-matching (SuperPoint + LightGlue) front-end.

```bash
bash scripts/run_superpoint.sh          # standalone node
bash scripts/run_vio_superpoint_offline.sh <sensor_bag> <eval_bag>   # C2 pass
```
