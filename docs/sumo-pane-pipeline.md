# 从新录制制作原生 SUMO 四画面

`carla/build_sumo_pane_scene.py` 只负责已实际通过源验收的 Town05 左转与 Town03 环岛。它不会连接或推进 CARLA，也不发布、清理、升级 current 清单或替代人工审阅。默认只读检查；`--execute` 才启动本机 SUMO GUI 和编码器。

开始前需要同一实际 run 的正常退出凭据、运动审计、动作/路口交互审计、完整传感器审计、native video manifest 与完整视频审计。所有报告与当前源字节 SHA、全部 world frame ID 必须匹配。失败或未完成的录制不能用来生成已验收素材。

每次使用实际 `run/map.xodr` 调用安装目录里的官方 `netconvert_carla.py`，新建独立网络目录，保存转换警告、真实返回码以及 source/converter/network SHA。不能将安装包的 LF 文件哈希替换为录制导出的 CRLF 文件哈希，也不改写已有 raw 文件。这里的两种换行即使解析内容相同，字节哈希仍不同。

本地 scene spec 使用项目相对路径，字段如下。示例字段是格式说明，需要全部换成这次实际存在的源证据；不创建占位视频或虚构审核记录：

```json
{
  "id": "left-turn",
  "run": "carla/captures-v2/ACTUAL_RUN",
  "execution": "carla/executions/ACTUAL_RUN.json",
  "motion_audit": "carla/verification/ACTUAL_MOTION_AUDIT.json",
  "action_audit": "verification/ACTUAL_INTERACTION_AUDIT.json",
  "sensor_audit": "verification/ACTUAL_SENSOR_AUDIT.json",
  "native_video_manifest": "carla/output-v2/left-turn/ACTUAL_NATIVE_INDEX.json",
  "native_video_audit": "verification/ACTUAL_NATIVE_VIDEO_AUDIT.json",
  "network_dir": "carla/sumo-network-v2/left-turn-ACTUAL_RUN",
  "replay_dir": "carla/sumo-replay-v2/left-turn-ACTUAL_RUN",
  "pane_dir": "carla/output-sumo-v2/left-turn-ACTUAL_RUN",
  "sumo_process_file": "carla/sumo-replay-v2/left-turn-ACTUAL_RUN-process.json",
  "sumo_audit": "carla/verification/sumo-sync-left-turn-ACTUAL_RUN.json",
  "pane_audit": "carla/verification/sumo-video-left-turn-ACTUAL_RUN.json",
  "pipeline_receipt": "carla/verification/sumo-pipeline-left-turn-ACTUAL_RUN.json",
  "manual_review": "verification/v2-left-turn-ACTUAL_RUN-manual-review.json"
}
```

环岛替换 `id` 为 `roundabout`，各新输出名同步使用 `roundabout`。所有输出必须不存在；失败的网络、GUI 图像、视频或报告保留，重试使用新的独立输出名。外层 process JSON 与 replay 目录保持为同级文件，避免创建输出目录的先后顺序发生冲突。

```powershell
python carla/build_sumo_pane_scene.py --spec LOCAL_ACTUAL_SCENE_SPEC.json
$env:CARLA_ROOT = 'C:/sim/CARLA-0.9.16'
$env:SUMO_HOME = 'C:/sim/sumo'
python carla/build_sumo_pane_scene.py --spec LOCAL_ACTUAL_SCENE_SPEC.json --execute
```

执行顺序为：实际 raw XODR 转换、全部源帧原生 SUMO GUI 回放、逐帧坐标/PNG/原始 source 映射审计、完整四画面编码、所有成片帧解码与抽样源图比较。任一步非零退出或证据不匹配都会停止，记录真实失败状态，不产生成功审核。

右上角是 **录制后的同步回放**。1280×720 原生 GUI PNG 以实际 native world frame 的八位编号命名；20 Hz 逐帧对应，无裁剪、插值或速度修改。四画面保持原生 Tesla 跟车、SUMO、真值辅助图像处理与真实 LiDAR 处理，不添加标题或 HUD。

执行器完成只表示 `automated_checks_passed_manual_review_needed`。随后查看 pane audit 产生的实际 opening/middle/closing PNG，按 [delivery-assembly.md](delivery-assembly.md) 如实填写样本编号、图像 SHA 和观察结果。不得自动填写 accepted=true 或把三张抽样说成全片人工观看。

完整六场景配置另存 `accepted-media-specs.complete.json` 等新文件，保留已发布 Beta 使用的 `accepted-media-specs.partial.json` 原始字节。全部六项经过 `scene_gate` 后才组装正式交付。README 六个 GIF/正式 MP4 链接只在真实公开 Release 完整 SHA 校验凭据通过后，由现有 [文档收尾入口](verified-release-docs.md) 更新；尚未完成场景继续保持 pending。

本地 scene spec、process/执行凭据、源帧、输出及本机绝对路径不加入 Git 正向清单。公开的是执行器、本文与可移植的 mock 测试。

本次左转与环岛原生 SUMO GUI 的 laneID 元数据有明确表示限制，原严格全车道审计仍保存 FAIL。左转只在一个视口外后台车辆的相连端点样本返回空 laneID；独立 `audit_sumo_sync_boundary_qualified.py` 验证该单样本、端点距离不超过 0.1 米、原生视口外和主车所有 laneID 有效。环岛的 `audit_sumo_sync_round_qualified.py` 独立限定两个仍可见的主车端点样本和一个视口外后台车辆连续 14 样本；全车辆、全主车的 laneID 归属均明确为 false。不得把这两种资格互相套用或描述成全车道审计通过。

资格审计只接受已绑定的全部实际源帧、所有精确位置/朝向/时间和全部原始 PNG SHA；不补写 laneID，不移动车辆，不改变物理、动作、传感器或视频完整性要求。独立原生 GUI 复现实验用于读取真实视口并确认车辆可见，重复渲染的栅格差异单独记录，最多四个像素；PSNR 如实披露，不写虚假的字节相同。原始录制和原始回放截图始终保留其完整 SHA。公开摘要、README 与 release 描述自动包含这些限制；这类 GUI 展示仍是录制后回放。
