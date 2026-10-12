# DriveWeave 交通场景与传感器录制

这一轮增加了明确的前车、后车和邻道交通角色，并保存真实后视相机画面。高速和匝道交通沿连续的原生道路行驶；目标车道间距不足时，主车通过车辆控制调整速度，等待可用间隙。路口队列停车后可以继续接近停止线。

车辆位置与速度由 CARLA 原生物理产生。交通角色采用公开规则，部分邻道车会按主车实际速度控制自己的速度，以构造可复现的交互；代码不通过传送或指定状态来制造交通。

## 录制

先按仓库首页安装 CARLA 0.9.16 客户端、运行时及所需地图，并独立启动服务器。同一时刻仅运行一个同步录制或试跑客户端。

| 计划 | 地图 |
| --- | --- |
| `carla/plans/public-v3/highway.json` | Town04 |
| `carla/plans/public-v3/ramp-entry.json` | Town06 |
| `carla/plans/public-v3/ramp-exit.json` | Town06 |
| `carla/plans/public-v3/left-turn.json` | Town05 |
| `carla/plans/public-v3/roundabout.json` | Town03 |

例如录制匝道驶入：

```powershell
python carla/run_capture_logged.py --log carla/entry-v3.log -- --map Town06 --route-json carla/plans/public-v3/ramp-entry.json --seconds 60 --fps 20 --traffic 0 --chase --rear --instance --png-compression 1 --out carla/captures-v3
```

完整计划结束后会自动停止。预热使用实际车辆控制，录制开始时已经有移动交通。外层日志记录真实进程退出码，并在开始前保存执行源码副本；中止或异常退出的录制不能当作完成素材。

公共计划由实跑计划派生，`publication_provenance` 记录原计划 SHA 和经过规范化的本机路径标签。控制参数、路点与滑行停车标定数表保持原值。标定来源的相对路径是来源标签，控制器使用计划中保存的数表，不要求下载本机原始录制。完整原始数据与执行记录留在录制端，公开仓库包含源码、计划及精选的结果摘要。

## 处理与检查

将 `RUN`、`EXECUTION` 和 `PROCESSED` 替换为本次实际目录。图像及点云处理不需要继续运行仿真服务器。

```powershell
python verification/audit_native_png_integrity.py --run RUN --out png-integrity.json
python carla/process_native_batch_cpu.py --run RUN --out PROCESSED --workers 3
python verification/audit_carla_sensors.py --run RUN --processed PROCESSED --execution EXECUTION --out sensor-audit.json
python carla/verification/audit_motion_quality.py --run RUN --execution EXECUTION --thresholds carla/verification/motion-quality-thresholds-v2.json --verify-instance-pixels --out motion-audit.json
python carla/encode_v3_views.py --run RUN --processed PROCESSED --out VIDEO_OUTPUT --name DriveWeave-scene
python carla/review_v3_media.py --manifest VIDEO_OUTPUT/media-manifest.json --out VIDEO_REVIEW
```

PNG 完整性检查从无损图像重建原始相机通道，与采集回调保存的 SHA 逐帧比较；任一像素不一致都失败。这样可以保留原始像素，而不额外保存一套相同的 BGRA 字节。LiDAR 保留原始返回及处理点对应的来源索引。

编码输出跟车、前视、后视、图像处理、点云处理及同步四视图，均按原速 20 fps 播放。四视图这一版排列为跟车、前视、图像处理和点云处理。图像框采用实例真值，点云采用几何处理；这些结果尚未作为训练模型的输出或驾驶反馈。

运动检查和场景动作检查分别报告。没有碰撞并不表示整个变道过程的动态间距都通过；相机几何候选也不等于目标确实出现在可见像素中。完整视频解码之后仍需查看实际画面。

按实际场景选择动作检查，不将一条场景的通过结果套用到其他录制：

```powershell
# 高速：核对八辆交通角色、真实后车接近、等待、整个换道过程及终点。
python carla/audit_v3_highway.py --run RUN --execution EXECUTION --motion-report motion-audit.json --out highway-actions.json
# 匝道：保存原始逐字节检查；地图只允许经 SHA 证明的 CRLF/LF 等价。
python carla/audit_native_actions_newline_bound.py --run RUN --out ramp-actions.json
# 左转或环岛：从本次实际采集计划读取角色、路点和门槛。
python carla/audit_v3_junction.py --run RUN --execution EXECUTION --motion-report motion-audit.json --out junction-actions.json
```

本次环岛原宽入口区域检查有一项失败，独立的 `audit_v3_entry_corridors.py` 按真实车道与完整车身扫掠区域重新检查实际冲突，并保留原报告。这个补充判断只适用于有对应几何证据的本次录制，不能自动豁免后续运行的失败。

## SUMO 与综合视频

本轮新场景使用 CARLA 原生物理。原有官方 CARLA/SUMO 双向协同录制及同步回放仍单独保留；`v3_traffic` 计划尚未接入实时协同录制器。综合视频中引用原有协同章节时，应在简介中注明其来源和 SUMO 画面属于录制后回放。

`production/assemble_project_video.py` 接受带源视频 SHA、原始帧数和剪辑范围的 JSON，生成 1080p 综合视频及逐段来源清单。它保持原始速度，不插值、不加入画面文字，也不会自动投稿。
