# 厂商相机 SDK 打包与调用调研

> 目的：在实现 `foray_platform_driver_hikvision` 之前，先看别的 RoboMaster 战队
> 把闭源相机 SDK（海康 MVS / 大恒 Galaxy / MindVision）怎么放进工程、怎么在
> CMake 里找到、怎么在机器人上跑起来，用来校准本仓的 SDK 分发与接入方案。
>
> 本文只做横评与结论，不改变 [`driver_hikvision.md`](driver_hikvision.md) 的驱动设计。

## 1. 调研方法

按 RoboMaster 视觉开源仓库的 star 排序与关键词（`MvCameraControl`、`hikrobot`、
`MVCAM_SDK_PATH`、`HikSDK.cmake`）检索，挑出「确实在实车上用海康工业相机」的
仓库，逐仓读取 `CMakeLists.txt`、`cmake/Find*SDK.cmake`、部署脚本与目录结构。
样本以 2024–2026 赛季仍在更新的仓库为主。

## 2. 样本总览

| 仓库 | 队伍 | ★ | SDK 来源 | CMake 集成 | 运行期 |
|---|---|---|---|---|---|
| `TongjiSuperPower/sp_vision_25` | 同济 SuperPower | 699 | 二进制入库 `io/hikrobot/{include,lib/amd64,lib/arm64}` | `target_link_directories` + `target_link_libraries(io MvCameraControl MVSDK usb-1.0)`，`CMAKE_SYSTEM_PROCESSOR` 不匹配即 `FATAL_ERROR` | 靠 `LD_LIBRARY_PATH`；README 同时又要求手动装官方 SDK（**双重来源**） |
| `WUST-RM/awakening` | 武汉科技大学 | 100 | 系统安装 `/opt/MVS` | `cmake/FindHikSDK.cmake` + `find_package(HikSDK QUIET)` + 编译宏 `USE_HikSDK` + imported target `HikSDK::HikSDK`；海康/大恒/MindVision 三家各一个 Find 模块 | `run.sh` 显式 `export MVCAM_SDK_PATH`/`MVCAM_COMMON_RUNENV`/`MVCAM_GENICAM_CLPROTOCOL`/`ALLUSERSPROFILE` + `LD_LIBRARY_PATH=/opt/MVS/lib/64:...` |
| `cv-rmvl/rmvl` | RMVL 视觉库 | — | 系统安装，`find_package` 探测 | **厂商子模块化**：`modules/camera/{hik,mv,galaxy,opt}_camera` 各自 `rmvl_update_build(hik_camera HikSDK_FOUND AND BUILD_rmvl_camera)`；`rmvl_install_sdk(${sdk_name})` 把找到的 SDK 复制进安装目录 | 随安装目录捆绑分发 |
| `Blackjack200/talos26` | — | 18 | 二进制入库 `3dparty/hik_sdk/{include,lib/amd64}`（含 `CommonParameters.ini`） | 子目录 `crates/hardware/hik_camera_driver`，`HIK_SDK_FOUND` 检测 + **stub 回退**（SDK 不可用时编 stub，保证可编译） | `CMAKE_INSTALL_RPATH "$ORIGIN"` + `BUILD_WITH_INSTALL_RPATH`，不靠环境变量 |
| `GKD-RM-Lab/gkd_vision_26` | 西南大学 GKD | 16 | 二进制入库；Docker 镜像内**不含** SDK | 同 `sp_vision_25` | `Dockerfile.dev`（openvino/ubuntu22 基础镜像 + apt 依赖），SDK 由工作区挂载 |
| `SCNU-Vanguard/sp25_qdu_pi` | 华南师范大学 | — | 二进制入库 `hikrobot/lib/arm64` | 同 `sp_vision_25`，但**在 configure 期逐个校验运行时文件存在**，缺则 `FATAL_ERROR` | 显式列举 6 个运行时文件，防 `MV_E_LOAD_LIBRARY (0x8000000c)` |
| `Akaneee126/sentry_omni26` | — | — | 本地 `../hikrobot/lib/{amd64,arm64}` 优先，回退 `/opt/MVS/lib/64` | `find_library(MVS_LIB MvCameraControl PATHS ...)` | — |
| `wuming255/shu_auto_aim_project` | 上海大学 | — | **完全 dlopen** | 无链接期依赖 | `dlopen("libMvCameraControl.so")`，候选路径含 `/opt/MVS/lib/64/libMvCameraControl.so.4.7.0.3`；`dlsym` 逐函数取 |

## 3. 打包范式：四种做法

### 3.1 二进制入库（最常见）

把头文件与 `.so` 直接提交进仓（`io/hikrobot/`、`3dparty/hik_sdk/`），CMake 用相对
路径链接。`sp_vision_25`、`gkd_vision_26`、`sp25_qdu_pi`、`talos26`、`sentry_omni26`
都属这一类。

- 优点：克隆即可编译，零环境配置；CI/新人不踩坑。
- 缺点：仓库膨胀（单主库 amd64 6.9 MB + arm64 6.6 MB，两份即 ~14 MB）；闭源二进制
  与 GPL 组件的许可证边界模糊；升级 SDK 变成一次大 commit；`sp_vision_25` 因此出现
  「仓里有一份、README 又让你装官方包」的双重来源。
- 变体：`talos26` 连 `CommonParameters.ini` 一起入库，说明他们踩过运行时缺文件的坑。

### 3.2 要求手动安装官方包（最省事、最脆）

README 让你自己装 `/opt/MVS` 安装器，CMake 通过环境变量 `MVCAM_SDK_PATH` 找到它。
`WUST-RM/awakening` 的 `cmake/FindHikSDK.cmake` 是代表：用 `$ENV{MVCAM_SDK_PATH}`
（Windows 下 `$ENV{MVCAM_COMMON_RUNENV}`）定位，`find_path` + `find_library` 带
`NO_DEFAULT_PATH`，再导出 imported target。

关键点：**海康官方 Linux 安装器不会自动写 `/etc/profile.d`**，所以每个队都得自己
在 `run.sh` 里 export 那 4–5 个变量。`WUST-RM/awakening/run.sh`：

```bash
export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
export ALLUSERSPROFILE=/opt/MVS/MVFG
export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:$WORK_DIR/lib:$LD_LIBRARY_PATH
```

- 优点：仓库干净，SDK 自己带 udev 规则与客户端工具。
- 缺点：每台机器手工装、版本漂移；新人第一次必然跑不起来。

### 3.3 CMake Find 模块 + 可选编译（最工程化）

`cv-rmvl/rmvl` 与 `WUST-RM/awakening` 都走 `find_package(XxxSDK QUIET)` 路线，把
「有没有 SDK」变成编译期选项：

```cmake
# rmvl/modules/camera/CMakeLists.txt
find_package(HikSDK QUIET)
rmvl_update_build(hik_camera HikSDK_FOUND AND BUILD_rmvl_camera)
rmvl_add_module(hik_camera DEPENDS camera EXTERNAL hiksdk)
```

`awakening` 更进一步，用编译宏把「哪家相机可用」传给源码：

```cmake
find_package(HikSDK QUIET)
if(HikSDK_FOUND)
    list(APPEND BUILD_DEFINITIONS USE_HikSDK)
    list(APPEND BUILD_LIBS HikSDK::HikSDK)
endif()
```

`rmvl` 还多一步 `rmvl_install_sdk()`：构建时把探测到的 `.so` 复制进自己的安装前缀，
让 `make install` 产出一个自包含目录——本质上是 3.1 的三方库版本。

- 优点：无 SDK 也能配置通过（跳过该驱动）；imported target 带 include 路径，用法干净。
- 缺点：Find 模块各家重复造轮子，路径猜测列表越写越长（`awakening` 的
  `FindHikSDK.cmake` 里 `lib/arm`、`lib/arm64`、`lib/aarch64`、`lib/x86`、`lib/x64`、
  `lib/64`、`lib/32` 全列了一遍）。

### 3.4 安装包 + 脚本（本仓路线，样本里没有）

把 SDK 拆到独立仓、由脚本装到前缀、CMake 侧走 `find_package` config 模式。
调研范围内**没有**战队这么做——最接近的是 `rmvl` 的 `rmvl_install_sdk()` 和
`WUST-RM/awakening` 的 `script/install_depences.sh`（但它只装 apt 依赖，不含 SDK）。

## 4. 调用范式：三种

| 范式 | 代表 | 链接期依赖 | 无 SDK 时可启动 |
|---|---|---|---|
| 直接链接（`target_link_libraries ... MvCameraControl`） | `sp_vision_25`、`gkd_vision_26`、`sp25_qdu_pi` | 需要 `.so` 存在 | 否 |
| `find_library` + imported target | `awakening`、`rmvl`、`talos26`、`sentry_omni26` | 需要 `.so` 存在 | 否（但可编 stub） |
| 全 dlopen + `dlsym` | `shu_auto_aim_project` | 无 | 是 |

`talos26` 的 stub 回退值得单独说：SDK 不可用时把 `HIK_USE_SDK` 置 `FALSE`，编译一个
空实现，保证 CI 与跨平台开发不被 SDK 阻断；另有 `HIK_TEST_NO_STUB` 选项强制走真 SDK。

## 5. 运行时依赖：插件与 `CommonParameters.ini`

这是本次调研**最有价值**的一条。`SCNU-Vanguard/sp25_qdu_pi/io/CMakeLists.txt` 的注释
直接写明了 MVS 主库的运行期行为：

```cmake
# [9.21-PI-HIK-SDK] MvCameraControl loads these USB/image modules with dlopen at runtime.
# Fail during CMake configuration instead of reaching MV_E_LOAD_LIBRARY (0x8000000c) on the robot.
set(SP_HIKROBOT_RUNTIME_FILES
    "${SP_HIKROBOT_LIBRARY}"                       # libMvCameraControl.so
    ".../hikrobot/lib/arm64/libMvUsb3vTL.so"
    ".../hikrobot/lib/arm64/libFormatConversion.so"
    ".../hikrobot/lib/arm64/libMediaProcess.so"
    ".../hikrobot/lib/arm64/libMVRender.so"
    ".../hikrobot/lib/arm64/libMvSDKVersion.so"
    ".../hikrobot/lib/arm64/CommonParameters.ini")
foreach(SP_HIKROBOT_RUNTIME_FILE IN LISTS SP_HIKROBOT_RUNTIME_FILES)
  if(NOT EXISTS "${SP_HIKROBOT_RUNTIME_FILE}")
    message(FATAL_ERROR "HikRobot runtime component not found: ${SP_HIKROBOT_RUNTIME_FILE}")
  endif()
endforeach()
```

三点结论：

1. **主库自己 `dlopen` 插件**——`libMvUsb3vTL.so`（USB3 传输层）只是其中之一，图像处理
   类插件（`libFormatConversion.so`、`libMediaProcess.so`、`libMVRender.so`）同样必需。
2. **`CommonParameters.ini` 是运行时必需文件**，不是 `.so`。缺它同样起不来。
   `talos26` 把 `3dparty/hik_sdk/lib/amd64/CommonParameters.ini` 一起入库，是同一结论
   的另一种印证。
3. 缺任何一个 → `MV_E_LOAD_LIBRARY (0x8000000c)`。`sp25_qdu_pi` 干脆把校验提前到
   CMake configure 期，让错误在构建时爆而不是在机器人上爆。

官方安装目录（`/opt/MVS`）对应的布局是：`lib/64`（amd64 主库与插件）、`lib/32`、
`lib/CLProtocol`、`MVFG`、`bin/`（含 `MVS` 客户端）。

## 6. 与本仓方案的对照

本仓当前做法（见 [`driver_hikvision.md` §10](driver_hikvision.md) 与
`foray_ws/scripts/install_mvs_sdk.sh`）：

- 主库与头文件放组织私有仓 `ZQU-Foray/mvs-sdk`，元仓 `.repos` pin 到具体 commit；
- `cmake/MVSConfig.cmake` 提供 `find_package(MVS)` + imported target `MVS::MvCameraControl`；
- 插件不入库，由 `scripts/install-plugins.sh --package <官方包>` 从官方安装包补齐；
- `scripts/make-bundle.sh` 产出离线安装包（`bin/install.sh` 布局）。

相对样本的优势：

| 维度 | 本仓 | 样本常见做法 |
|---|---|---|
| 源码仓体积 | 不含二进制 | 入库 14 MB 起 |
| SDK 定位 | `find_package(MVS)` config 模式 | `target_link_directories` 或手写 Find 模块 |
| 版本锁定 | `.repos` pin commit + `SHA256SUMS` | 二进制入库无版本可查 |
| 多架构 | `MVS_ARCH` 由 `CMAKE_SYSTEM_PROCESSOR` 判定，config 内处理 | 各仓重复 `if(x86_64) ... else()` |
| 插件补齐 | 脚本从官方包提取 + `PLUGINS.sha256` 校验 | 入库或干脆不管 |
| 离线分发 | `make-bundle.sh` 单文件安装包 | 无 |

## 7. 结论与待办

### 7.1 立即要修的缺陷

`scripts/install-plugins.sh` 目前只处理 `*.so`/`*.so.*`，**会漏掉
`CommonParameters.ini`**——按 §5 的结论，这会让「插件装齐了但相机仍起不来」。
需要：

1. 除 `.so` 外，一并安装官方包内的运行时数据文件（`CommonParameters.ini` 及其他
   非 ELF 必需文件），并写入 `PLUGINS.sha256` 覆盖它们；
2. 输出「必需插件」分级清单（`libMvUsb3vTL.so` / `libFormatConversion.so` /
   `libMediaProcess.so` / `libMVRender.so` / `libMvSDKVersion.so`），装完做一次存在性
   自检，缺项明确报出而不是静默通过。

### 7.2 可选增强（来自样本，按性价比排序）

1. **`MVCAM_SDK_PATH` 兼容**：`MVSConfig.cmake` 在 `find_path`/`find_library` 的 `HINTS`
   末尾追加 `$ENV{MVCAM_SDK_PATH}/include`、`$ENV{MVCAM_SDK_PATH}/lib/64`，让「已经装了
   官方包」的机器开箱即用，同时保持私有仓路径优先。
2. **configure 期运行时自检**：`MVSConfig.cmake` 检测到 `libMvCameraControl.so` 但插件
   目录为空时给 `message(WARNING)`，把「能枚举但打不开」提前到构建期（`sp25_qdu_pi` 做法）。
3. **rpath**：安装 `foray_platform` 时可设 `INSTALL_RPATH "$ORIGIN/../lib"`，减少对
   `scripts/env.sh` 的依赖（`talos26` 做法）。
4. **stub 回退**：无 SDK 时编译一个空实现的 `hikvision` 驱动，保证 CI 与跨平台开发可过；
   本仓已有 `FORAY_BUILD_DRIVER_HIKVISION=OFF`，语义接近，暂不必做。
5. **全 dlopen**（`shu_auto_aim_project` 做法）：可做到「无 SDK 也能启动、运行期才失败」。
   本仓已用 `detail::MvsApi` 虚接口隔离，换成 dlopen 只是替换实现层，但会失去编译期
   类型检查——**不采纳**，除非将来需要「同一二进制支持多厂商」。

### 7.3 明确不采纳

- **二进制入库**：与 `decision.md` 2026-10-23 条目冲突（不得入产品源码仓），且会让
  元仓体积翻倍。
- **仅 Docker**：实车部署与 HIL 调试不便，Docker 可作 CI 补充而非唯一路径。

## 8. 参考

- [TongjiSuperPower/sp_vision_25](https://github.com/TongjiSuperPower/sp_vision_25)——`io/CMakeLists.txt`、`io/hikrobot/`
- [WUST-RM/awakening](https://github.com/WUST-RM/awakening)——`cmake/FindHikSDK.cmake`、`script/setup_devenv.sh`、`run.sh`
- [cv-rmvl/rmvl](https://github.com/cv-rmvl/rmvl)——`modules/camera/CMakeLists.txt`、`cmake/FindHikSDK.cmake`
- [Blackjack200/talos26](https://github.com/Blackjack200/talos26)——`crates/hardware/hik_camera_driver/CMakeLists.txt`、`3dparty/hik_sdk/`
- [SCNU-Vanguard/sp25_qdu_pi](https://github.com/SCNU-Vanguard/sp25_qdu_pi)——`io/CMakeLists.txt`（运行时文件清单）
- [GKD-RM-Lab/gkd_vision_26](https://github.com/GKD-RM-Lab/gkd_vision_26)——`Dockerfile.dev`
- [Akaneee126/sentry_omni26](https://github.com/Akaneee126/sentry_omni26)——`hik_camera/CMakeLists.txt`
- [wuming255/shu_auto_aim_project](https://github.com/wuming255/shu_auto_aim_project)——`src/io/mvs_camera.cpp`（dlopen）
- [LemperorD/nav2_ws](https://github.com/LemperorD/nav2_ws)——`scripts/easyMVS.sh`（`MVCAM_SDK_PATH` 用法）
