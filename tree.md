# 目录结构说明

## 布局规则

本仓根目录即 colcon 工作空间根，**源码包全部位于 `src/`**；仓根只保留文档五件套、
`docs/`、`.github/` 与构建产物目录（`build/`、`install/`、`log/`，不入库）。

包内骨架固定为：`include/<ns>/` · `src/` · `test/` · `cmake/` · `launch/` · `config/` ·
`scripts/` · **`apps/`（C++ 可执行小工具）** · **`data/`（polkit 等非源码资产）**。

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
│   ├── hal_architecture.md      HAL 生命周期、帧池与零拷贝
│   ├── hal_sensor_driver_research.md  海康 / Livox SDK 封装调研
│   ├── driver_livox.md          Livox 驱动：运行时、解析、组帧与时间语义
│   ├── driver_hikvision.md      海康 MVS 驱动：接入序列、隔离与时间戳
│   ├── camera_config.md         相机配置键空间、分级与动态参数契约
│   ├── driver_deployment.md     SDK2 / 网络 / PPS-PTP 部署
│   ├── net_bootstrap.md         特权网络自举与 polkit 动作
│   └── sdk_packaging_survey.md  厂商相机 SDK 打包与调用横评
├── src/
│   ├── foray_platform_core/     合并包：L1 core + L2 hal（不依赖 ROS）
│   │   ├── CMakeLists.txt
│   │   ├── package.xml          build_type=cmake，供 colcon 识别
│   │   ├── cmake/
│   │   │   └── foray_platform_coreConfig.cmake.in
│   │   ├── include/foray/platform/
│   │   │   ├── time/            Duration · Timestamp · Clock
│   │   │   ├── param/           schema · snapshot · store · source · manager
│   │   │   ├── result/          ErrorCode · Error · Result<T>
│   │   │   ├── container/       有界 SPSC 队列（header-only）
│   │   │   ├── config/          document · resolver · snapshot · exporter · tool
│   │   │   └── hal/             Device · Camera · Lidar · 帧 · FrameLease/FramePool · ConfigBinder
│   │   ├── src/
│   │   │   ├── time/clock.cpp
│   │   │   ├── param/           实现、内部校验与 YAML 工具
│   │   │   ├── result/error.cpp
│   │   │   ├── config/          resolver · exporter · tool
│   │   │   ├── net/             probe · process · NM/netlink 后端 · plan/ensure_link · bootstrap
│   │   │   └── hal/config_binder.cpp
│   │   ├── test/
│   │   │   ├── time/            时钟单测
│   │   │   ├── param/           参数系统单测
│   │   │   ├── result/          错误码 / Result 单测
│   │   │   ├── container/       SPSC 队列单测
│   │   │   ├── config/          分层 / include / env / 导出单测
│   │   │   ├── net/             MTU / 计划 / 回滚 / 跨桥 / bootstrap 单测
│   │   │   └── hal/             类型 / 帧池与租约 / ConfigBinder / Fake 传感器单测
│   │   ├── apps/foray_net_helper.cpp  特权网络自举 helper（pkexec）
│   │   └── data/polkit/         polkit 动作与可选免密规则
│   ├── foray_platform_driver_livox/  Livox SDK2 驱动（纯 CMake，不依赖 ROS）
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── cmake/
│   │   │   ├── FindLivoxSDK2.cmake  暴露 LivoxSDK2::shared
│   │   │   └── foray_platform_driver_livoxConfig.cmake.in
│   │   ├── include/foray/platform/driver/livox/
│   │   │   └── lidar.hpp        公共工厂（无 SDK 类型）
│   │   ├── src/livox/
│   │   │   ├── lidar.cpp/.hpp   生命周期、配置、解析线程
│   │   │   ├── livox_control.cpp/.hpp  控制面重试 + 读回校验
│   │   │   ├── lidar_public_factory.cpp  公共 make_lidar（后端对象）
│   │   │   ├── livox_runtime.cpp/.hpp  进程级 SDK 单例与 handle 路由
│   │   │   ├── livox_api.hpp    SDK 函数表抽象
│   │   │   ├── livox_api_real.cpp   SDK ON 时的真实实现
│   │   │   ├── livox_api_stub.cpp   SDK OFF 时的 stub
│   │   │   ├── packet_parser.cpp/.hpp  36 字节头与点/IMU 解析
│   │   │   ├── frame_assembler.cpp/.hpp  组帧与逐点时间
│   │   │   └── config_json.cpp/.hpp  Livox 网络 JSON
│   │   ├── apps/
│   │   │   ├── livox_dump.cpp   点云导出（PCD/PPM/流式 stdout）与自检
│   │   │   ├── livox_config.cpp 雷达参数读回/下发/再校验
│   │   │   └── lidar_up.cpp     一键自举：网络 + 配置 + 取帧健康确认
│   │   └── test/livox/          mock 单测 + backend 能力测试
│   └── foray_platform_driver_hikvision/  海康 MVS 驱动（纯 CMake，不依赖 ROS）
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── cmake/
│       │   ├── FindMVS.cmake    暴露 MVS::MvCameraControl（可与官方 CONFIG 互换）
│       │   └── foray_platform_driver_hikvisionConfig.cmake.in
│       ├── include/foray/platform/driver/hikvision/
│       │   └── camera.hpp       公共工厂（无 SDK 类型）
│       ├── src/hikvision/
│       │   ├── camera.cpp/.hpp  生命周期、设备选择、取流线程、帧发布
│       │   ├── camera_bootstrap.cpp/.hpp  相机地址规划（LLA / 同网段 / 复用桥上地址）
│       │   ├── camera_factory.cpp/.hpp  注入 MvsApi 的内部工厂
│       │   ├── camera_public_factory.cpp  公共 make_camera（取后端）
│       │   ├── mvs_api.hpp      MVS 函数表抽象
│       │   ├── mvs_api_real.cpp MVS ON 时的真实实现
│       │   ├── mvs_api_stub.cpp MVS OFF 时的 stub
│       │   ├── mvs_nodes.cpp/.hpp   CameraConfig/CameraTuning → 节点名与值
│       │   ├── mvs_error.cpp/.hpp   MV_E_* → ErrorCode 映射
│       │   └── timestamp.cpp/.hpp   PTP / 标定偏移 / 主机接收三路径换算
│       ├── apps/
│       │   ├── camera_dump.cpp  配置 + 取 N 帧 + 帧率/丢帧统计 + --view 预览
│       │   ├── camera_up.cpp    一键自举：挂桥 + 地址 + MTU + 取帧健康确认
│       │   └── viewer.cpp/.hpp  SDL2 预览窗口（8 位 Mono/Bayer → RGB24）
│       └── test/hikvision/      mock 单测（生命周期/映射/时间戳/地址规划/后端能力）
└── .github/
    └── workflows/
        └── ci.yml               组织 CI 模板 + 目录规范 lint
```

> L2 HAL（同包 `hal` target）已落地；Livox 驱动已落地（mock 全绿，真实 SDK 待部署验证）；
> Hikvision 驱动已落地（mock 全绿，MV-CS004-10GC 实机取流已验证）；ROS 适配层按计划加入。

## 目录职责

| 路径 | 职责 | 依赖约束 |
|---|---|---|
| `src/foray_platform_core/` | L1 core：时间/时钟、参数系统、配置、错误码；L2 hal 抽象 | **不得依赖 ROS 2、不得依赖任何业务仓** |
| `src/foray_platform_driver_livox/` | L2 传感器驱动（HAL 插件） | 只依赖 `foray_platform::hal` / `::net` |
| `src/foray_platform_driver_hikvision/` | L2 传感器驱动（HAL 插件） | 只依赖 `foray_platform::hal` / `::net` |
| `docs/` | 设计文档 | — |
| `.foray-layer` | 声明 `layer: L1`、`depends_on_layers: [X2]` | CI 校验依赖方向 |

## 命名规则

- 包目录名与 `package.xml` 的 `<name>` 必须一致，且**不得与仓同名**（核心包用 `_core` 后缀）。
- 包名格式：`foray_<层职能>[_<角色>]`；角色后缀取封闭词表：`core` `hal` `net` `driver_<vendor>` `ros` `bringup`。
- C++ 可执行小工具进 `apps/`，不使用 `tools/`；非源码资产（polkit/udev/规则）进 `data/`。

## 记录约束

MUST NOT 记录以下内容：

- `build/`、`install/`、`log/`、`.git/` 等依赖与产物目录
- 临时文件、缓存文件、日志文件
