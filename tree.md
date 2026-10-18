# 目录结构说明

## 目录结构

```text
.
├── README.md                    本仓定位、接口、上下游
├── AGENTS.md                    AI 协作规范
├── plan.md                      开发计划与进度
├── decision.md                  关键工程决策日志
├── tree.md                      本文件
├── CODEOWNERS                   Review 自动分派
├── .clang-format                组织 C/C++ 格式配置（CI lint-c 依赖）
├── .foray-layer                 所属层与依赖边界（CI 校验）
├── .gitignore
├── docs/
│   ├── time_and_clock.md        时钟域与 use_sim_time 语义
│   ├── parameter_system.md      参数 schema / 校验 / 热加载设计
│   ├── config_stack.md          分层配置、include、env 与导出
│   └── hal_sensor_driver_research.md  海康 / Livox SDK 封装调研
├── foray_platform/              合并包：L1 core + L2 hal（不依赖 ROS）
│   ├── CMakeLists.txt
│   ├── package.xml              build_type=cmake，供 colcon 识别
│   ├── cmake/
│   │   └── foray_platformConfig.cmake.in
│   ├── include/foray/platform/
│   │   ├── time/                Duration · Timestamp · Clock
│   │   ├── param/               schema · snapshot · store · source · manager
│   │   ├── result/              ErrorCode · Error · Result<T>
│   │   ├── container/           有界 SPSC 队列（header-only）
│   │   └── config/              document · resolver · snapshot · exporter · tool
│   ├── src/
│   │   ├── time/clock.cpp
│   │   ├── param/               实现、内部校验与 YAML 工具
│   │   ├── result/error.cpp
│   │   └── config/              resolver · exporter · tool
│   └── test/
│       ├── time/                时钟单测
│       ├── param/               参数系统单测
│       ├── result/              错误码 / Result 单测
│       ├── container/           SPSC 队列单测
│       └── config/              分层 / include / env / 导出单测
└── .github/
    └── workflows/
        └── ci.yml               复制自组织 CI 模板
```

> L2 HAL（同包 `hal` target）与 ROS 适配层按计划逐步加入；当前已落地 `core` target。

## 目录职责

| 路径 | 职责 | 依赖约束 |
|---|---|---|
| `foray_platform/` | L1 core：时间/时钟、参数系统、错误码；L2 hal 按计划加入 | **不得依赖 ROS 2、不得依赖任何业务仓** |
| `docs/` | 设计文档 | — |
| `.foray-layer` | 声明 `layer: L1`、`depends_on_layers: [X2]` | CI 校验依赖方向 |

## 记录约束

MUST NOT 记录以下内容：

- `build/`、`install/`、`log/`、`.git/` 等依赖与产物目录
- 临时文件、缓存文件、日志文件
