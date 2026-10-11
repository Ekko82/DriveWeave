# 实际发布后更新视频链接

`finalize_release_docs.py` 不上传、不联网、不运行 Git 或模拟器，也不清理素材。默认只校验；`--execute` 才更新指定文档。它不接受手写 `publication_verified=true`，只消费 `maintenance.py verify-publication` 实际生成的完整校验凭据。

完整凭据绑定公共 GitHub repository、已发布的 tag、完整 commit SHA、每个资源的真实完整下载字节数和 SHA-256，以及本机 publication manifest 和维护脚本本身的 SHA。本地资源、清单或维护脚本变化后旧凭据失效。请在代码冻结、资源上传后运行：

```powershell
python maintenance.py verify-publication --manifest maintenance/publications/ACTUAL_VERSION.json --report maintenance/reports/ACTUAL_VERSION-publication-verified.json
```

这一步只读 GitHub 与本地文件，不删除文件。后续清理仍执行自己的新远端验证，不能凭文档状态绕过维护门。

四场景 Beta 使用实际验收配置，例如：

```powershell
python finalize_release_docs.py --publication maintenance/publications/ACTUAL_BETA.json --receipt maintenance/reports/ACTUAL_BETA-publication-verified.json --beta-spec LOCAL_FOUR_SCENE_SPEC.json
# 检查通过后写 README 与独立 Beta 公共摘要。
python finalize_release_docs.py --publication maintenance/publications/ACTUAL_BETA.json --receipt maintenance/reports/ACTUAL_BETA-publication-verified.json --beta-spec LOCAL_FOUR_SCENE_SPEC.json --execute
```

Beta 仅接受 `highway`、`ramp-entry`、`ramp-exit`、`live-highway`，每项重新经过完整 `scene_gate`。四个实际 Release MP4 必须与已验收四视图原字节一致；Beta spec 的 SHA 必须属于 publication 的 accepted manifests。工具把左转、环岛标为 pending，不生成 GIF，不升级六场景 current 清单。输出仅为 `README.md` 和 `docs/media/beta-public-summary.json`。

六场景正式版须先通过完整交付组装，见 [delivery-assembly.md](delivery-assembly.md)。正式版不使用 `--beta-spec`：

```powershell
python finalize_release_docs.py --publication maintenance/publications/ACTUAL_STABLE.json --receipt maintenance/reports/ACTUAL_STABLE-publication-verified.json
python finalize_release_docs.py --publication maintenance/publications/ACTUAL_STABLE.json --receipt maintenance/reports/ACTUAL_STABLE-publication-verified.json --execute
```

正式版检查六个场景与 publication 中 pinned 的 accepted delivery 字段一致，实际视频均有远端 SHA 证据，六秒 GIF SHA 与源 MP4 绑定。随后填 `docs/media/preview-index.json` 和公共摘要的真实 URL/tag/commit，更新六个 README GIF/完整视频链接及实际帧数、时长。保留“原生真值图像处理、真实 LiDAR、SUMO GUI 为录制后回放”的范围；live 源采集另有双向协同证据。

文档正常提交到 main，不移动原发布 tag。工具没有 Git 操作。加入公开清单的是工具、本文、可移植测试和实际生成的脱敏摘要；本机 publication/执行/审核凭据不公开。

每次实际新版本发布验证成功后，才按已审阅清理声明处理上版本缓存，没有日历 schedule。`carla/releases` 只能选明确旧版本子目录，不能清理整个父目录。将来 v3 或更高主版本可声明 `old.version="v2"`，选择 `carla/output-v2`、`carla/output-sumo-v2`、`carla/release-v2`；当前 accepted 引用闭包和 keep_paths 始终保护原文件。源码、审计和元数据永久保留，执行只 unlink 已核验的精确二进制文件。
