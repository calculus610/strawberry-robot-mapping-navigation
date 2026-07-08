# zzj_ws

ROS 2 workspace for robot driver and bringup source.

Version-controlled content is kept in `src/` plus workspace-level documentation and helper files.

Generated build outputs are intentionally ignored:

- `build/`
- `install/`
- `log/`

Typical rebuild:

```bash
cd ~/zzj_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

