#!/usr/bin/env bash
# ==========================================================
# zzj_ws 环境配置脚本 - 终极修复版
# 不仅修复 AMENT_PREFIX_PATH，还修复 LD_LIBRARY_PATH
# 确保系统 Nav2 (1.1.20) 优先于 wheeltec_ros2 旧版
# 用法: source ~/zzj_ws/setup_zzj.bash
# ==========================================================

# 先 source 当前工作空间
source /home/wheeltec/zzj_ws/install/setup.bash

# 需要屏蔽的旧 wheeltec_ros2 包名
REMOVE_PKGS=(
    nav2_controller nav2_planner nav2_lifecycle_manager
    nav2_bt_navigator nav2_behaviors nav2_smoother
    nav2_waypoint_follower nav2_waypoint_cycle
    nav2_velocity_smoother nav2_collision_monitor
    nav2_costmap_2d nav2_voxel_grid nav2_map_server
    nav2_rviz_plugins nav2_behavior_tree nav2_amcl
    nav2_core nav2_util nav2_common nav2_msgs
    nav2_smac_planner nav2_navfn_planner nav2_theta_star_planner
    nav2_mppi_controller nav2_regulated_pure_pursuit_controller
    nav2_rotation_shim_controller nav2_constrained_smoother
    nav2_simple_commander nav2_dwb_controller
    dwb_core dwb_critics dwb_plugins dwb_msgs
    nav_2d_utils nav_2d_msgs costmap_queue amcl
    navigation2
)

# -------------------------------------------------------
# 修复 AMENT_PREFIX_PATH：让 wheeltec_ros2 旧包不可见
# -------------------------------------------------------
NEW_AMENT=""
REMOVED_AMENT=""
IFS=':' read -ra ALL_PATHS <<< "$AMENT_PREFIX_PATH"
for p in "${ALL_PATHS[@]}"; do
    pkg_name=$(basename "$p")
    remove=false
    for r in "${REMOVE_PKGS[@]}"; do
        [ "$pkg_name" = "$r" ] && { remove=true; break; }
    done
    if $remove; then
        REMOVED_AMENT="$REMOVED_AMENT${REMOVED_AMENT:+ }$pkg_name"
    else
        NEW_AMENT="$NEW_AMENT${NEW_AMENT:+:}$p"
    fi
done
export AMENT_PREFIX_PATH="$NEW_AMENT"

# -------------------------------------------------------
# 修复 LD_LIBRARY_PATH：移除旧版 nav2/dwb 库路径
# -------------------------------------------------------
OLD_LD="$LD_LIBRARY_PATH"
NEW_LD=""
REMOVED_LD=""
IFS=':' read -ra ALL_LD <<< "$LD_LIBRARY_PATH"
for p in "${ALL_LD[@]}"; do
    # 检查这个路径是否属于被移除的包
    remove=false
    for r in "${REMOVE_PKGS[@]}"; do
        # 如果路径包含 wheeltec_ros2/install/<包名>/
        if [[ "$p" == *"/wheeltec_ros2/install/$r/"* ]]; then
            remove=true
            break
        fi
    done
    if $remove; then
        REMOVED_LD="$REMOVED_LD${REMOVED_LD:+ }$(basename $(dirname $p 2>/dev/null) 2>/dev/null || echo $p)"
    else
        NEW_LD="$NEW_LD${NEW_LD:+:}$p"
    fi
done

# 确保 /opt/ros/humble/lib 在最前面（系统库优先）
NEW_LD="/opt/ros/humble/lib${NEW_LD:+:}${NEW_LD}"
export LD_LIBRARY_PATH="$NEW_LD"

echo "✅ zzj_ws 环境已就绪（完整修复）"
echo "   从 AMENT_PREFIX_PATH 移除: $REMOVED_AMENT"
echo "   从 LD_LIBRARY_PATH 移除旧版库"
