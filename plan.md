# 开发计划

## 任务列表

- [ ] 定义时间与时钟抽象（含 `use_sim_time` 语义）
- [ ] 搭参数系统骨架（schema + 校验 + 热加载）
- [ ] 定义 HAL 抽象接口（传感器 / 执行器 / 总线 / 设备生命周期）
- [ ] 接一个真实驱动跑通「一个传感器 → 一个话题」
- [ ] 实现急停硬路径（独立实时线程 + 超时定时器）

## 当前 Agent State

PLAN_READY

## Before Snapshot

commit hash:   （首次提交）
branch:        main
modified files: （首次初始化）
risk level:    L1

## 模糊点与待确认项

- 本仓接口尚未冻结，任务清单为**方向性**的，落地顺序以 `foray_docs/algorithm_structure.md` §9 演进阶段为准
- 依赖的自研仓尚未建齐，跨仓任务需等对方 `README.md` 明确接口后再启动

## Vector Backend Status

Backend: Markdown
Status:  ready
Environment: 仓库内 Markdown 文档（无外部向量后端）
Index: 本仓 `README.md` / `tree.md` / `decision.md`
Initialization: 2026-02-19
Commit: （首次提交）

## Acceptance Criteria

- [ ] 本仓能独立 `colcon build`（无代码时跳过）
- [ ] CI 绿灯
- [ ] 每个新增模块带独立可执行测试
