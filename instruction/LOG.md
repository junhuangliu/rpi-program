# LOG.md - 项目会话日志规则

项目日志采用**项目内自管理**（不依赖 memory 插件）。

- 每次会话开始时，读取当天 daily 日志（`<项目根>/instruction/daily/YYYY-MM-DD.md`，存在则读取，不存在则创建）并向用户汇报
- 重要任务完成后，主动将任务摘要写入当天 daily 日志；将可复用知识写入 `<项目根>/MEMORY.md`
- 项目相关的决策、约定、路径变更，记录到 `<项目根>/MEMORY.md`
- 涉及跨多天的信息时，优先使用 Grep 搜索 `instruction/daily/` 目录，而非逐文件读取