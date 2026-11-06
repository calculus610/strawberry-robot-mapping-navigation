# 原前180手动建图继承与优化

本目录以机器人实际部署的 `/home/wheeltec/manual_native180_20261022` 为基线。
基线源码与现场逐文件一致，备份标签 `native180-original-20261106`，完整备份信息见 `baseline_verified_manifest.json`。
原版前端约定保存在 `ORIGINAL_FRONTEND_CONTRACT.md`。

## 保持的主链与接口

完整帧同步/IMU去畸变后按原安装对应的机体+X裁剪前180°。匹配体素0.20m、原生树体素0.10m。
最终障碍仅来源于原生ikd-tree当前快照；仍为固定地面参考下0.10～1.00m、5cm二维格、九层射线证据、黑格优先。
`/map` 的坐标系为 `native_projection`，位姿来自 `/Odometry`。原有三维服务 `/native_map_save`、会话监督进程与二维/三维/空闲证据保存均保留。

## 版本入口

在机器人新隔离目录 `/home/wheeltec/manual_native180_restored_20261106` 下：

- `start_native_v0_original.sh`：关闭所有新增过滤，作为原版行为对照。
- `start_native_v1_70pct_3m.sh`：前向裁剪后、0.20m匹配体素之前，选取光心3m内的有限点并按输入顺序均匀保留floor(N*70/100)。

- `start_native_v2_smooth.sh`：在V1基础上增加局部加权机体竖直平面拟合与保守极端孤点过滤，仍在匹配前处理。

平滑沿机体XY方向修正，每点最多15mm，保留原点其余字段和机体高度；低支持度、墙角、细杆等条件不足时保留原位。隔离判定使用抽样前的原始有限前向邻点，含3m外近邻，但这些外部点不进入匹配。
合成墙RMS 5.44mm降为2.04mm；墙角、96mm缝、16mm细杆测试通过。此为合成几何证据，不能替代倾斜车体和实际场景验收。

70%的分母是原有雷达预处理和全帧去畸变之后、前180°且3m内的有效点；不等于MID360原始全部回波的70%。
3m约束本帧参与匹配/入树的候选，不裁掉历史地图。九层free仍使用未抽样、未拟合的真实前向观测，允许3m外背景回波用于旧版空闲证据计算。

所有入口接受 `start_rviz:=false`。与原版一致，包装脚本仍检查机器人图形会话、加载隔离环境并拒绝重复传感器/建图进程。
地图仍保存到 `/home/wheeltec/maps/manual_native180/` 下每轮独立的 `native180_vN_*` 目录；后端必须使用输出的 `SESSION_DIR`。
`optimization_profile.json` 记录本轮版本。正常结束仍向本轮监督进程发送SIGINT，并等保存日志和完整结果。

## 验证与回滚

现有高度投影/九层空闲18项回归通过，结果见 `verification/results/`。
机器人完整ROS编译、原版/Stage0等价及各版本离线集成测试待完成；当前SSH链路间歇超时，不能据独立算法测试声称整机验收完成。
回滚：正常结束新版本会话后，执行原入口 `/home/wheeltec/manual_native180_20261022/start_manual_native180.sh`。不删除地图、不修改原版源码或底盘/导航服务。
