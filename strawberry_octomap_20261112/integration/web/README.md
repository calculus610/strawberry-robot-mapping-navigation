# Web 控制识别补丁

`external_manual.patch` 以本目录 `production_external_manual.baseline.py` 为基线。基线 SHA-256 为 `59d0f6b64d20e83c0a8606bf5b8ec2542f59b8b1c2dae1b74045cc6296ebaca9`。`external_manual.candidate.py` 是补丁应用后的候选文件。

补丁已于 2026-11-12 获用户单独批准并部署。部署前后管理器均为 IDLE、无控制租约；没有启动建图、申请控制或发送运动命令。实际 Web 使用由用户验收。部署时先核对实际加载的生产 `external_manual.py` 路径及基线哈希，再在管理器没有活动任务、控制租约或运动时替换该模块并通过 `systemctl --user restart wheeltec-web-manager.service` 重启用户级管理器。回滚使用同一基线文件，并同样在空闲时重启。

补丁新增一条独立会话识别规则。协调器入口、launch 参数、运行状态、PID 与启动时间、会话目录、两个独占锁、ROS 域与本机发现、环境令牌、进程可执行文件和 ROS 节点名称均须一致。只有两条建图就绪标记均已出现，且四个必需节点属于同一次 launch 时，管理器才可按现有流程完成控制重新申请前的检查。速度、租约和任务启动规则由原管理器继续执行。

浏览器现有地图视图订阅 `/map`。独立入口显式使用 `--web-map` 才额外发布该话题，网页绘图与管理器认可控制是两个可独立核验的结果。独立会话保存仍通过 SSH 终端的 Ctrl+C。

## 已部署版本与回滚

- 部署记录：`deployment.json`。
- 原文件备份：`/home/wheeltec/strawberry_octomap_20261112/web_profile_backup_o4frhzb2/external_manual.py`。
- 当前候选 SHA256：`cd3a0eb6470c4b700ef5f5cd4321e2ebff403c89eb94cfbfefec682377ba4216`。
- 回滚前确认无任务/无租约，且生产文件仍为上述候选哈希，避免覆盖用户后续改动。仅用备份恢复原生产 `external_manual.py`，然后 `systemctl --user restart wheeltec-web-manager.service`，确认管理器返回 IDLE。若生产哈希已变化，先人工决定合并，不覆盖。
