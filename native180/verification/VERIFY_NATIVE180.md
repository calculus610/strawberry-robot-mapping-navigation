# Native180 regression verification

Run the old and restored builds in separate, fresh local ROS domains. Build both from isolated copies and source only the intended overlay in each shell. The test runner explicitly disables the MID360 driver and RViz. The replay publishes sensor data only. Each command takes a new test output directory.

## Reuse of the original fixtures

- The original `FastLIO原生地图前180实验_20261022/verification/test_projection.py` covers fixed-reference projection, height edges, all-height canvas, exact occupancy, negative coordinates and snapshot replacement.
- `FastLIO原生地图前180实验_20261022/voxel_free_20261022/robot_results/verification/test_free_evidence.py` covers all nine layers, raw endpoint reset, native obstacle priority, blind/endpoint bounds and cache serialization. Use `check_projection_units.py` with the old fixtures and the target build's `libfree_evidence.so` (Linux) or `libfree_evidence.dylib` (Mac). It imports the target's projector modules before running the archived fixtures.
- The archived `check_full_voxel_free.py` and `check_free_ros.py` are earlier ROS fixtures. Their hardcoded workspace and output paths refer to the historical deployment. The new launch and projector checks accept explicit paths.
- The read-only archive `地面静止实验_20261021/recovered/bag/bag_0.db3` contains 708 native `/livox/lidar` CustomMsg packets and 13,887 `/livox/imu` samples over 83.59 s. In a fresh domain replay only those two topics, keeping original message stamps. The two rack-overlay bags have IMU but no raw lidar topic. The November smoke test's different bag uses PointCloud2 and its existing timestamp-preserving conversion path.

## Run on Linux with ROS Humble

For each isolated copy, source ROS Humble, the Livox message package and that copy's `ws/install/local_setup.bash`. Set `ROS_LOCALHOST_ONLY=1` and `ROS_DOMAIN_ID=83` or `84` in each shell. Set `V` to this verification directory, `COPY` to the built copy and `OUT` to a fresh test path. Run one case at a time.

```bash
python3 -B "$V/check_native180_launch.py" --runner "$COPY/run_session.py" --stage original --output "$OUT/original"
python3 -B "$V/check_native180_launch.py" --runner "$COPY/run_session.py" --stage 0 --output "$OUT/stage0"
python3 -B "$V/compare_stage0.py" "$OUT/original" "$OUT/stage0"
```

Use the original overlay for `--stage original` and the restored overlay for `--stage 0`. The comparator checks identical forward scan counts and compares the fixed reference, the absolute occupied-cell coordinates and the native cloud geometry. Wall-clock scheduling and float values at voxel boundaries can change individual representatives. The comparator reports exact occupied-cell overlap and separately requires at least 98% one-cell bidirectional coverage, 4 cm directed P95 cloud distance, at most 5% point-count difference and at most 2 cm ground-origin difference. Both sessions must pass exact, independent saved-map reconstruction.

Run the restored stages independently, with a fresh output each time.

```bash
python3 -B "$V/check_native180_launch.py" --runner "$COPY/run_session.py" --stage 1 --output "$OUT/stage1"
python3 -B "$V/check_native180_launch.py" --runner "$COPY/run_session.py" --stage 2 --output "$OUT/stage2"
python3 -B "$V/check_native180_launch.py" --runner "$COPY/run_session.py" --stage 3 --output "$OUT/stage3"
python3 -B "$V/check_native180_launch.py" --runner "$COPY/run_session.py" --stage 3 --exercise-dynamic --output "$OUT/stage3_panel"
```

The launch harness checks exactly one native source and one optical-pose publisher, exact timestamp pairing, a fixed ground transform matching the static TF, retained near and far raw front observations, absence of rear wall geometry, an independently reproduced live black mask, native canvas and map frame. It verifies that the old independent-map topics lack publishers. Stage 1–3 additionally compare raw candidates and actual pre-matching XYZ against timestamped metrics. The only points eligible for 70% are the finite forward deskewed returns within the 3D range of 3 m. Stage 1 checks the precise ordered sample. The matching-input diagnostics must have no rear or over-range points. The separate original free scan must still include far forward returns.

`check_geometry_boundary.cpp` is a standalone Stage 2 regression for a close pair straddling 3 m. Build it as C++17 with `-I "$COPY/ws/src/fast_lio/src"`. The near return survives only in the frame with raw support. The supporting return stays outside matching in both cases.

The optional Stage 3 panel fixture replaces returns with closer points along the same rays after startup, holds until the panel enters the native tree, then restores the farther background. It requires a native-tree removal metric and disappearance in a later native snapshot. A disappearing two-dimensional cell alone fails this test. For longer acceptance replay compare motion and static geometry at the panel's neighbors, occlusion with no background, a single hit, a sustained static obstacle, wall corners, thin supports, timestamp rollback, rebuild pressure and interrupted shutdown. Short-lived transient rejection needs its own fixture with observations shorter than the admission threshold. Stage 2 diagnostics report shifted and rejected counts. No geometry improvement claim follows from the ROS contract alone.

For an explicit all-height live white-cell fixture, run the installed projector in a fresh domain with no mapper running.

```bash
python3 -B "$V/check_ros_projector_contract.py" \
  --projector "$COPY/ws/install/native_height_projection/lib/native_height_projection/native_height_projector.py" \
  --output "$OUT/projector"
```

This verifies tilted fixed ground, mismatched observation stamps, eight layers staying unknown, the ninth layer turning white, endpoint-only reset, native-black priority, fresh evidence after reset, unchanged canvas and exact saved pixels.

## Saved map and rollback

`check_saved_native180.py SESSION_DIR...` verifies the five required files and uses `projection_stats.json` for the fixed ground transform. It reconstructs every PGM pixel independently from `projection_source_camera_init.pcd` and the nine-layer cache. It checks PGM/YAML dimensions and origin, data semantics, cache bounds, per-state counts and stamps. The complete launch test also calls the isolated `/native_map_save` service, verifies its PCD, and checks shutdown independently recreates the PCD. It requires the saved source PCD to equal the captured native snapshot at the corresponding stamp. The final native PCD may contain a slightly later snapshot.

A failing test can be rerun in a new output directory. The tests leave existing map directories alone. The production rollback is to stop the experimental session normally and start the original isolated native180 entry. No test runs a hardware motion or production service command.

The Mac results as of this handoff are 18/18 original projection/free-evidence fixtures against the restored projector source, and exact saved-map reconstruction on one archived synthetic and three archived field sessions. Linux ROS integration and new stage comparisons remain to be run.
