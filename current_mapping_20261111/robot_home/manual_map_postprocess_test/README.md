# 原版手动建图 + Ctrl+C 后 A005、Free 冲突与孤立散点筛选

此目录是独立测试工具。原版 FAST-LIO、投影器、配置和 Web 入口未修改。
原版实时地图仍来自 native ikd-tree + 固定地面高度带 + 九层自由射线。
本工具不发布运动指令，不启动底盘控制器，不替换导航地图。

2026-11-11 接入 Free 冲突 v1：原版保存并完成 A005 后，自动把原 Free 中至少3个不同记录帧命中的格转为 Unknown。该阶段保持原有黑格、灰格与0.05m坐标。后续新增孤立散点筛选，再请求机器人本地桌面打开静态对比图。

## 在机器人 SSH 终端启动

    bash /home/wheeltec/manual_map_postprocess_test/start_test.sh

可先执行只读依赖检查（不启动 ROS 节点）：

    bash /home/wheeltec/manual_map_postprocess_test/start_test.sh --check

需要机器人本地桌面 DISPLAY=:0 和原版 Xauthority 可用。脚本会启动原版 Livox 驱动、
FAST-LIO、投影器和机器人本地 RViz。已有雷达或建图进程运行时拒绝启动，请先从原会话正常退出。
机器人保持静止、放在平地，等终端出现 PROJECTION_READY 后再手动驾驶。

建图过程中 RViz 显示原版地图。按一次 Ctrl+C 后等待，勿直接断电：

1. 原版保存最终二维地图与三维点云并退出；
2. 旁路录制器排空精确时间戳配对并封口文件；
3. 原版保存、录制完整性检查通过后启动离线处理；
4. 持久化独立 a005_checkpoint.json，开始 Free 冲突优化并生成 enhanced_conflict/；
5. 对完整最终黑图的原版及A005孤立小域做逐帧证据筛选，保存 enhanced_scatter/；
6. 生成 final_comparison.png；打印 FINISHED 和最终地图绝对路径表示新阶段地图通过核验。图片可用时另打印 COMPARISON 并默认在机器人本地桌面请求打开。

只保存图片、不请求桌面查看器：

    bash /home/wheeltec/manual_map_postprocess_test/start_test.sh --no-preview

若散点阶段取消、超时或失败，打印 SCATTER_FALLBACK_CONFLICT 和 FINISHED_WITH_SCATTER_FALLBACK，回退至已核验的 enhanced_conflict/map.yaml。若Free冲突阶段取消、超时或失败，打印 CONFLICT_FALLBACK_A005 和 FINISHED_WITH_A005_FALLBACK，最终选择回退到已经严格核验的 enhanced/map.yaml。这不代表新阶段成功。A005本身未通过保存核验时仍拒绝成功。

启动/保存期间重复 Ctrl+C 不会再次向原版转发信号。离线处理期间再次按 Ctrl+C 可取消
尚未完成的后处理；已保存原图和观测保留。SSH 断开触发的 HUP 会请求正常停机，但
不保证主机断电、进程 SIGKILL、磁盘硬故障后的自动完成。推荐保持 SSH 终端连接直到结束。

## 输出

每次新建 /home/wheeltec/maps/manual_postprocess_test/日期_随机后缀/：

- original/：原版 map.pgm、map.yaml、投影统计、自由证据、原生 PCD 等；
- observations.bin：紧凑逐帧 XYZ 与精确配对位姿，不是完整原始 rosbag；
- recording.json：帧数、点数、时间戳、参考坐标、SHA256、配对和缺失检查；
- enhanced/map.pgm 与 enhanced/map.yaml：A005结果，继续保留；
- a005_checkpoint.json：A005完成边界与输入指纹，不随主会话状态更新；
- enhanced_conflict/map.pgm 与 map.yaml：Free冲突优化后的中间结果，不覆盖原图、A005或生产导航地图；
- enhanced_conflict/report.json、cells.csv、excluded_hits.csv：新增灰格、弱命中白格及阶段证据；
- enhanced_scatter/map.pgm 与 map.yaml：本次最终测试图；
- enhanced_scatter/report.json、components.csv、changes.csv：筛选证据与每格改动理由；
- final_comparison.png：原版、A005、Free冲突、散点最终图和分色差异；
- comparison.png：相同范围的原图、候选、差异三联图；
- changes.csv：新增占据格的索引、坐标与规则原因；
- report.json：离线算法结果、栅格计数、校验、耗时与本进程峰值 RSS；
- session.json：整个会话完成/失败状态和进程树 RSS 抽样；
- session.log、mapping.log、recorder.log、offline.log：阶段及完整日志。

失败时不宣称增强成功。原始资料保留，不能把旧的周期保存冒充最终保存。
已有 enhanced/ 时离线处理拒绝覆盖。
最终选择以 session.json/final_map 为准。完整新流程成功还须 conflict_status=complete、scatter_status=complete、enhanced_scatter/report.json/status=complete，且产物SHA与地图转换校验通过；enhanced_map字段继续指向A005以兼容旧记录。
桌面预览的requested仅表示已发起打开请求，不等于验证窗口已出现；可用图片的路径以COMPARISON打印。预览失败会打印FINAL_PREVIEW_UNAVAILABLE，不影响已经验证的地图。
对比图中蓝色表示A005将原灰格改黑，紫色表示Free冲突阶段将原白格改灰，橙色表示散点阶段将原版或A005黑格改灰。实时RViz显示原图，优化结果通过结束后生成的静态图片查看。

若只需重新处理一份完整记录，且该会话还没有 enhanced/：

    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -B /home/wheeltec/manual_map_postprocess_test/offline_enhance.py --session /home/wheeltec/maps/manual_postprocess_test/实际会话目录

不需要重新启动 FAST-LIO。失败后的重试保留原图和观测，错误参考或损坏输入仍会拒绝。

## 算法与边界

固定 FAST-LIO 历史位姿；每帧在雷达坐标中按 0.05 m 体素保留首个真实点；
转换到固定地面参考，取离地 0.10–1.00 m 的端点，按 0.05 m 地图格累计命中。
仅将“原灰格且历史端点命中”改黑，原黑格和白格均保留。
地图 origin、resolution、frame 与 YAML 语义一致。A005没有重新估计轨迹，没有重新计算自由射线，
没有墙体膨胀或无证据填线，没有新增自由格。一次合格命中即可补黑，不代表多帧确认静态障碍。

旁路记录所有收到且精确时间戳配对的观测，并非挂接原投影器 accepted-frame 回调。
启动期帧数可能与原投影器接受帧数不同，报告会记录差异及观测/原图时间范围。
DDS 使用兼容原发布者的 BEST_EFFORT；单边丢失、重复、乱序或配对超限会失败。
两条流同时漏掉同一帧时，单凭接收端不能证明或排除丢失；时间间隔异常会报告。
该测试不宣称对真实障碍、门洞或草莓架完成了现场验收。

## 资源与退出保护

逐帧记录和逐帧后处理，不在 RAM 中长期累计历史 XYZ。
历史样本约 127 秒记录 38.3 MiB，按相同点速率约 1.06 GiB/小时；另有原版 PCD/日志开销。
启动要求输出盘至少 2 GiB 空闲，录制中不足 1 GiB 请求停机并拒绝增强。
这不是磁盘用量上限；文件不会自动删除，长期使用需自行管理已保存会话。

配对队列有帧数和字节上限，单帧上限 200000 点；超限明确失败，不偷偷丢帧。
原版停机超时和离线处理超时均报告失败，清理只针对本工具创建的进程及其后代。
使用每角色继承标记和 Linux pidfd 核对进程身份，不按进程名称全局杀进程。

## 回退

停止本次测试后，继续使用原版启动命令：

    bash /home/wheeltec/manual_native180_20261022/start_manual_native180.sh

没有修改需要恢复的生产源码或参数。不自动删除实验资料。
源码/配置基线指纹在 protected_before.json；如果原版后来有意更新，测试工具会拒绝旧指纹，
应重新审查适配，不能直接忽略该检查。

## 验证

verification/offline_historical_a005_strict/validation.json：历史候选逐字节复现、截断、
参考错误、收到帧数不一致、防覆盖及报告提交故障测试。
verification/replay_*：禁用实车驱动和 RViz，在独立 net/IPC/PID 与只读根环境中回放旧包。
FAST-LIO 固定 Log 路径和会话锁映射到测试目录，避免写入原版诊断文件。
verification/audit_*：进程所有权、退出清理等非 ROS 边界验证。


本轮交付验证（2026-11-10）：
- 历史固定输入复算新增 2843 格，与旧候选 PGM/YAML 字节一致；
- 新端到端原版重放记录 1279 帧，独立逐帧 XYZ 摘要、位姿及首末时间戳对账一致；
- 本次原图 6303 黑 / 25677 白 / 1052733 灰；增强 9136 黑 / 25677 白 / 1049900 灰；
- 仅 2833 个原灰格转黑，原黑白不变，YAML 字节一致；
- 从一次 Ctrl+C 到整个流程退出约 7.76 秒；离线计算约 5.15 秒，仅为本样本实测；
- 记录文件 40284064 字节，离线进程峰值 RSS 74520 KiB；
- 录制器进程树 1 Hz 抽样 RSS 峰值 58408 KiB，不代表完整新增内存；
- 原版 52 个受保护文件指纹不变，无真实雷达/运动/RViz 实车运行，无测试节点残留。
详细索引：verification/acceptance.json。现场实物准确性、长时间资源占用仍需手动试车验收。

## Free 冲突 v1 的处理边界

共同时间窗内逐帧复用0.05m传感器首点体素、固定地面和0.10–1.00m高度带；原白格至少3帧命中才进入处理。
P2按多个高度簇/有界站位记录支持，P3完整读取实际回波作射线代理诊断。未标定的射线误差保持不可评估，本版不强制补黑、不新增Free、不删除原黑格。
1或2帧命中的白格仍保留并输出excluded_hits.csv，因此不能保证所有真实障碍都已从白格中消除。
这些统计仅在原建图与录制结束后执行；未改变在线FAST-LIO、原投影器、Nav2或录制格式。

## 输出选择与取消

按一次Ctrl+C后等待。原版、录制、A005、Free冲突、预览依次执行。离线阶段额外信号会取消尚未提交的新阶段；若已完成原子提交，协调器只有完整核验后才采用结果。
Free冲突失败只回退至经核验的A005。损坏A005或原版最终保存失败仍标整次失败，不能把无效回退图当成功。
地图输出采用有锁、不可覆盖的目录提交。反复启动离线worker不会覆盖已有enhanced_conflict；不会自动删来源观测。断电持久性未单独验收。

## 人工复核新阶段

会话必须已有a005_checkpoint.json且尚无enhanced_conflict。固定来源或worker源码指纹不符时拒绝处理，不要手工绕过。

    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -B /home/wheeltec/manual_map_postprocess_test/offline_conflict.py --session /home/wheeltec/maps/manual_postprocess_test/实际会话目录

最终图默认路径：/home/wheeltec/maps/manual_postprocess_test/实际会话目录/enhanced_scatter/map.yaml。
仅生成离线候选；本脚本不会更改Nav2当前所用地图。

## 本次接入验收（2026-11-11）

31项本地校验通过，包括输入损坏、结果防覆盖、取消、超时、回退和预览选择。
机器人上两份固定观测分别复现556、519格Free转Unknown，PGM与已确认实验版逐字节相同。
隔离回放完整旧bag并触发一次Ctrl+C，记录1279帧，停止到退出40.84秒；其中A005约5.09秒、新阶段约31.95秒。
本次重新运行FAST-LIO形成的新观测得到2848格Unknown转Occupied及527格Free转Unknown；新增Free为0、原有Occupied不删、YAML逐字节相同。
新阶段进程峰值RSS约49.86MiB，这是该历史样本的独立进程测量，不代表整个建图系统或更大地图的内存上限。
回放禁用真实驱动、运动控制和桌面；PNG生成与内容校验通过，自动桌面打开仅完成依赖检查，窗口出现仍需实车桌面验证。
52个原版受保护文件哈希未变，无回放进程残留。证据位于/home/wheeltec/offline_experiments/ctrlc_conflict_integration_20261111/。

## 孤立散点测试阶段（2026-11-11，用户明确包括原版占据）

本阶段在完整Free冲突结果上做一次八连通标记，不单独对A005新增掩膜连通，不迭代剥离。
候选为1–3格完整黑域、与域外任意黑格格心欧氏距离至少0.20m、具有完整边界邻域。原版、A005及混合来源一视同仁，不按来源豁免。
使用现有observations.bin、A005原采样与完整录制帧范围。整域按不同记录帧去重，恰1帧支持且无其他帧的相邻一格支持才撤回Unknown；至少2帧保留。零支持也保留并记录未在录制中观察到，不能将观测缺失当成噪声。
只允许本阶段输入的Occupied变Unknown；Free与Unknown逐格保持，resolution/origin/YAML字节一致。不会从缺测推断Free。
单帧小域仍可能是真实细柱或短暂可见障碍，此版为用户授权的筛选测试，未建立真实噪声标签或现场安全保证。黑改灰会影响Nav2膨胀；地图不会自动替换当前Nav2地图。
终端SCATTER_FILTER_COMPLETE打印实际修改格数，零修改也正常。表格给出每域原版/A005来源与保留原因，可据此复核。
散点输入与输出使用独立校验；原A005/Free冲突核心和验证器不变。失败保持已核验的C结果；原始观测与全部中间地图保留。
回滚本次工具更新：无测试会话运行时，在机器人执行

    python3 -I -B /home/wheeltec/offline_experiments/ctrlc_scatter_integration_20261111/deploy_scatter_release.py --rollback

回滚只恢复本次工具文件，遇到后续外部改动会拒绝覆盖，地图与记录不删除。
