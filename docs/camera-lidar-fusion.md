# 相机与激光雷达后融合

这个流程对 CARLA 原生 RGB 与 LiDAR 录像逐帧处理。相机使用官方 YOLOX-S COCO 预训练 ONNX 模型做目标检测；点云使用实际传感器返回，通过标定投影、视锥内聚类与目标关联，为有足够点云支持的检测框补充距离。原生车辆仍由场景规则控制，融合结果没有回传控制车辆。

综合画面始终保留相机结果、三维点云、点云俯视图、跟车和后视画面。同一目标在图像与点云中使用同色标记。框旁只显示检测类别、置信度、显示编号和有效测距，没有额外说明字幕。没有点云支持或深度存在歧义时保留相机框，不显示距离。

## 输入与处理

1. 相机模型只读取 RGB。按 YOLOX 官方 ONNX 流程做 BGR、640×640 左上对齐填充、输出解码与 NMS，原始结果按 0.3 阈值保存。成片对全部帧、全部类别统一保留置信度不低于 0.75 的结果，减少低置信度误识别，同时会漏掉部分正确的远车和遮挡目标。处理范围为 person、bicycle、car、motorcycle、bus、truck 六类。
2. 点云复用已保存的原生点行索引、降采样与地面分离结果。自车排除使用采集时的自车几何；目标检测与关联不读取实例标签、语义标签或其他车辆真值。
3. 每帧核对 RGB/LiDAR 的原生 frame 与 timestamp，用该帧传感器外参和相机 K 投影。只将位于检测框内的非地面实测点作为关联候选。
4. 视锥内做欧氏聚类，要求至少六个点、足够的像素跨度和合理的观测范围；多个受支持的深度互相冲突时拒绝测距。一条点记录只归属一个检测框。
5. 测距是关联点到 LiDAR 原点距离的中位数，反映观测到的目标表面。点云框表示观测点的包围范围，不代表完整车辆三维尺寸。显示编号来自短时几何匹配，缺测时不绘制预测框或沿用旧距离。

这是预训练图像检测加几何后融合基线。它没有训练联合融合网络，也没有构成自动驾驶感知控制闭环。稀疏点云、遮挡和仿真图像的域差异会造成漏检或错误关联。20 fps 是原速录像播放帧率，离线推理耗时另存于结果中。

## 复现

安装原有 requirements.txt 和额外依赖 requirements-fusion.txt。从 [YOLOX 官方 ONNX 模型页](https://github.com/Megvii-BaseDetection/YOLOX/tree/main/demo/ONNXRuntime) 下载 yolox_s.onnx。权重不包含在仓库中。

本次使用的权重 SHA-256：`c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063`。

```powershell
python -m pip install -r requirements.txt -r requirements-fusion.txt
python carla/fuse_camera_lidar.py --run RUN --processed PROCESSED --out FUSION --model MODEL --threads 3
python verification/audit_camera_lidar_fusion.py --run RUN --processed PROCESSED --fusion FUSION --model MODEL --out AUDIT.json --gt-samples 15
python carla/render_fusion_labels.py --run RUN --processed PROCESSED --fusion FUSION --out DISPLAY --min-confidence 0.75
python production/assemble_fusion_video.py --help
```

`RUN` 是完整原生采集目录，`PROCESSED` 是对应的点云处理目录；`FUSION` 必须是新的独立输出目录。检测框与关联结果保存到 associations.jsonl，每条测距保留原始 LiDAR 点行索引。fusion-manifest.json 绑定采集、处理器、模型、结果文件与源帧的 SHA。

独立审计会重算全部帧的标定投影、测距与源点归属。实例标签和车辆真值只在结果冻结后的诊断中读取，不进入推理。本次原始结果逐帧诊断发现施工隔离栏误检、遮挡处前景点误关联，因此采用了统一的 0.75 展示门槛。门槛是在这组录像上检查后选定的，这组数据不能再作为独立测试集，也不能据此宣称融合准确率或道路安全验证。原始低门槛预测、异常记录和完整场景均保留。

模型处理协议参考 [YOLOX 官方推理示例](https://github.com/Megvii-BaseDetection/YOLOX/blob/main/demo/ONNXRuntime/onnx_inference.py)。外部依赖与许可见 [第三方说明](../THIRD_PARTY_NOTICES.md)。
