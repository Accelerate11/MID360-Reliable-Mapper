# Real MID360 Mapping Workflow

This workflow focuses only on mapping preparation. Navigation and obstacle
avoidance are intentionally out of scope for this stage.

## 1. Start Live Mapping

For a complete mapping experiment with recording, quality logs, and final PCD
export, use:

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/run_real_mid360_mapping_session.sh
```

For a fixed-duration run, pass seconds as the first argument:

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/run_real_mid360_mapping_session.sh 90
```

Start the Livox MID360 driver, FAST-LIO2, the filtered map node, and RViz:

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/start_real_mid360_fastlio_rviz.sh
```

Restart only RViz without touching the driver or FAST-LIO2:

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/start_real_mid360_rviz_only.sh
```

Stop all MID360 mapping processes:

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/stop_mid360_slam.sh
```

## 2. Check Live Mapping Quality

Full pipeline diagnostic:

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/diagnose_real_mid360_mapping_pipeline.sh
```

Fast cloud quality check:

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/check_real_mid360_mapping_quality.sh
```

Expected signs:

- `/cloud_registered_filtered` runs near 10 Hz.
- `/fastlio_denoised_map` receives points.
- Bounds stay in a realistic room-scale range.
- Frame ID is `camera_init`.

## 3. Record a Reproducible Dataset

Run this while live mapping is active:

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/record_real_mid360_mapping_bag.sh
```

The bag is saved under:

```text
/home/accelerate/cuadc_ws/mid360_mapping_bags/
```

Recorded topics:

- `/livox/lidar`
- `/livox/imu`
- `/cloud_registered`
- `/cloud_registered_filtered`
- `/fastlio_denoised_map`
- `/Odometry`
- `/path`
- `/tf`
- `/tf_static`

## 4. Export a PCD Map

Run this while live mapping is active:

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/export_real_mid360_map.sh
```

The PCD and JSON report are saved under:

```text
/home/accelerate/cuadc_ws/mid360_mapping_outputs/
```

## 5. Replay a Bag Offline

Re-run FAST-LIO2 from a recorded raw MID360 bag without using the hardware:

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/replay_real_mid360_mapping_bag.sh \
  /home/accelerate/cuadc_ws/mid360_mapping_sessions/session_xxx/bag/real_mid360_mapping
```

This plays only:

- `/livox/lidar`
- `/livox/imu`

The replay output is saved under:

```text
/home/accelerate/cuadc_ws/mid360_mapping_replays/
```

## 6. Session Summary

Create or refresh a markdown summary for a mapping session:

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/summarize_real_mid360_mapping_session.sh \
  /home/accelerate/cuadc_ws/mid360_mapping_sessions/session_xxx
```

The one-shot session and replay scripts generate `summary.md` automatically.

## 7. Analyze And Compare Maps

Generate a JSON quality report plus a top/3D snapshot from an exported PCD:

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/analyze_real_mid360_map.sh \
  /home/accelerate/cuadc_ws/mid360_mapping_sessions/session_xxx/maps/final_fastlio_denoised_map.pcd
```

Compare two exported maps with a 20 cm voxel overlap metric:

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/compare_real_mid360_maps.sh \
  /home/accelerate/cuadc_ws/mid360_mapping_sessions/session_a/maps/final_fastlio_denoised_map.pcd \
  /home/accelerate/cuadc_ws/mid360_mapping_replays/replay_xxx/maps/replayed_fastlio_denoised_map.pcd
```

Useful comparison fields:

- `jaccard`: shared occupied voxels divided by all occupied voxels.
- `a_coverage_by_b`: how much of map A is reproduced by map B.
- `b_coverage_by_a`: how much of map B is covered by map A.
- `centroid_delta_b_minus_a_m`: coarse drift between map centers.

Evaluate whether a session is ready for downstream map preparation:

```bash
/home/accelerate/cuadc_ws/install/uav_slam_sim/share/uav_slam_sim/scripts/evaluate_real_mid360_mapping_session.sh \
  /home/accelerate/cuadc_ws/mid360_mapping_sessions/session_xxx
```

The result is written to:

```text
mapping_readiness.json
mapping_readiness.md
```

Readiness verdicts:

- `PASS`: usable as a mapping dataset baseline.
- `WARN`: usable for inspection, but coverage/rate/density should be checked.
- `FAIL`: do not use for downstream map preparation until the issue is fixed.

## 8. Recommended Mapping Test Procedure

1. Start a clean session with `run_real_mid360_mapping_session.sh`.
2. Keep MID360 still for 5-10 seconds after startup.
3. Move slowly and avoid sudden rotations.
4. Sweep the room once at constant height.
5. Return near the starting point to observe drift.
6. Press Ctrl+C to stop; the script records a rosbag and exports the final PCD.
7. Review `mapping_readiness.md`, `logs/final_quality.txt`, `maps/final_fastlio_denoised_map.json`, and `maps/final_fastlio_denoised_map.analysis.json`.
8. Compare runs through each session's `summary.md`.

## 9. Map Quality Notes

Good signs:

- Walls look thin rather than heavily doubled.
- The map grows smoothly when MID360 moves.
- The trajectory does not jump.
- No kilometer-scale coordinates appear in the quality report.

Warning signs:

- Repeated `No Effective Points`.
- PCL voxel overflow warnings.
- Map bounds suddenly become hundreds or thousands of meters.
- Dense ghosting after a slow loop.

If warning signs appear, first check LiDAR-IMU extrinsic and time offset before
tuning planning algorithms.
