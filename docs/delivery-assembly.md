# 验收后生成 v2 交付素材

`carla/assemble_v2_delivery.py` 只处理本地文件，不连接仿真、不上传、不清理。默认检查全部六场景；缺文件、非零实际退出、旧源 SHA、失败报告、裁剪帧或未做人工审阅均拒绝。

配置使用项目相对路径。每项 `id` 为 `highway`、`ramp-entry`、`ramp-exit`、`left-turn`、`roundabout`、`live-highway`，共六项，每项需要：

```json
{
  "id": "highway",
  "run": "carla/captures-v2/ACTUAL_RUN",
  "execution": "carla/executions/ACTUAL_RUN.json",
  "motion_audit": "carla/verification/motion-quality-v2-highway-moving-final.json",
  "action_audit": "verification/carla-v2-highway-moving-action-audit.json",
  "sensor_audit": "verification/carla-v2-highway-sensor-audit.json",
  "native_video_manifest": "carla/output-v2/highway/highway-v2-视频清单.json",
  "native_video_audit": "verification/carla-v2-highway-video-audit.json",
  "sumo_manifest": "carla/sumo-replay-v2/highway/manifest.json",
  "sumo_audit": "carla/verification/sumo-sync-replay-v2-highway.json",
  "pane_manifest": "carla/output-sumo-v2/highway/highway-v2-CARLA与SUMO视频清单.json",
  "pane_audit": "carla/verification/sumo-pane-video-v2-highway.json",
  "manual_review": "verification/v2-highway-manual-review.json",
  "preview_start_sample": 0
}
```

`live-highway` 额外指定实际 `live_audit`；该报告必须绑定本次 `sumo-sync.jsonl`、完整帧数及执行凭据。即使源录制是真实双向协同，四视图中的 SUMO GUI 仍明确标为录制后同步回放。

配置外层为 `{"schema_version":1,"version":"v2","release_dir":"carla/release-v2","scenes":[...]}`。实际 run、报告和人工记录有本机信息，不加入公开源文件清单。

人工记录不能由工具自动审批。查看成片审计实际输出的 opening/middle/closing PNG 后，保存以下字段，至少三张，推荐审计产生的全部五张：

```json
{
  "schema": "native_media_manual_review_v2",
  "scene_id": "highway",
  "accepted": true,
  "reviewer": "ACTUAL_REVIEWER",
  "reviewed_utc": "ACTUAL_TIME_WITH_TIMEZONE",
  "bindings": {
    "capture_manifest_sha256": "ACTUAL_SHA256",
    "source_frames_jsonl_sha256": "ACTUAL_SHA256",
    "pane_manifest_sha256": "ACTUAL_SHA256",
    "pane_video_sha256": "ACTUAL_SHA256"
  },
  "samples": [{
    "sample": 0,
    "native_frame": 0,
    "image": "ACTUAL_DECODED_SAMPLE.png",
    "image_sha256": "ACTUAL_SHA256",
    "observed": "实际看到的车辆、道路及三个其他窗口的对应情况"
  }],
  "presentation": {
    "native_tesla_model_retained": true,
    "native_SUMO_viewport_retained": true,
    "four_panes_legible": true,
    "added_titles_or_HUD": false
  },
  "full_video_manually_watched": false
}
```

实际查看的源帧编号、图像 SHA、观察描述和是否全片观看均须如实填写。人工抽样审阅不能代替所有帧自动解码。上面仅是格式，没有代录任何人工批准。

```powershell
# 检查，不写当前清单或媒体。
python carla/assemble_v2_delivery.py --config LOCAL_ASSEMBLY_CONFIG.json
# 六场景全部通过后，生成 ASCII 文件名的原字节 MP4 副本及真实源 GIF。
python carla/assemble_v2_delivery.py --config LOCAL_ASSEMBLY_CONFIG.json --execute --promote-current
```

GIF 是实际验收 MP4 中连续六秒的解码画面，每隔一个 20 Hz 帧取样为 10 Hz、480×270 GIF；时长仍为六秒，没有插值。原始完整 MP4 不裁剪、不重编码。公共摘要仅构造帧数、相对媒体路径、SHA 和证据范围，不复制原审计的本机路径、命令、日志或自由描述。

`--promote-current` 先将旧 current 清单原始字节保留在永久 `verification/history`，校验备份 SHA，再原子替换维护工具保护的 `carla/output/delivery-manifest.json`。新 current 不引用 history，避免把上版 raw 永久作为当前素材保护。未指定该选项时只产交付目录。

开源 allowlist 只加入工具、本文、实际生成的 `docs/media/*.gif` 和 `docs/media/accepted-v2-summary.json`；不要加入本地 assembly config、current 清单或 execution receipts。发布工具仍需验证 GitHub Release 远程资源 SHA 后才能按审阅声明清理旧版。
