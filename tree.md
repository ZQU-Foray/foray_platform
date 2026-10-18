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
│   └── parameter_system.md      参数 schema / 校验 / 热加载设计
├── foray_platform_core/         L1 纯 C++ 核心库（不依赖 ROS）
│   ├── CMakeLists.txt
│   ├── package.xml              build_type=cmake，供 colcon 识别
│   ├── cmake/
│   │   └── foray_platform_coreConfig.cmake.in
│   ├── include/foray/platform/
│   │   ├── time/                Duration · Timestamp · Clock
│   │   └── param/               schema · snapshot · store · source · manager
│   ├── src/
│   │   ├── time/clock.cpp
│   │   └── param/               实现与内部校验
│   └── test/
│       ├── time/                时钟单测
│       └── param/               参数系统单测
└── .github/
    └── workflows/
        └── ci.yml               复制自组织 CI 模板
```

> L2 HAL 与 ROS 适配层尚未开始，落地后在此补充对应目录。

## 目录职责

| 路径 | 职责 | 依赖约束 |
|---|---|---|
| `foray_platform_core/` | L1 平台基础：时间/时钟、参数系统 | **不得依赖 ROS 2、不得依赖任何业务仓** |
| `docs/` | 设计文档 | — |
| `.foray-layer` | 声明 `layer: L1`、`depends_on_layers: [X2]` | CI 校验依赖方向 |

## 记录约束

MUST NOT 记录以下内容：

- `build/`、`install/`、`log/`、`.git/` 等依赖与产物目录
- 临时文件、缓存文件、日志文件
