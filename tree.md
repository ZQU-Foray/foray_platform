# 目录结构说明

## 目录结构

```
.
├── README.md                本仓定位、接口、上下游
├── AGENTS.md                AI 协作规范
├── plan.md                  开发计划与进度
├── decision.md              关键工程决策日志
├── tree.md                  本文件
├── CODEOWNERS               Review 自动分派
├── .foray-layer             所属层与依赖边界（CI 校验）
├── .gitignore
├── package.xml             ROS 2 包元数据
├── CMakeLists.txt           USB CDC 与候选 RoboLink 编解码的无硬件单元测试构建
├── include/foray_platform/
│   ├── usb_cdc_port.hpp     USB CDC 设备接口
│   ├── robolink_crc.hpp     候选协议所用的 CRC-32 算法
│   ├── robolink_frame.hpp   候选保活帧组帧接口
│   ├── robolink_frame_parser.hpp 候选字节流帧解析接口
│   ├── robolink_codec.hpp   零槽/单槽/多槽帧及八类消息载荷接口（含云台姿态）
│   ├── robolink_byte_channel.hpp USB 与帧解析桥接接口
│   └── robolink_host_link.hpp 主机握手、状态/姿态回读与断线状态机接口
├── src/
│   ├── usb_cdc_port.cpp     USB CDC 设备打开、关闭与原始字节读写
│   ├── robolink_crc.cpp     CRC-32/ISO-HDLC 实现
│   ├── robolink_frame.cpp   候选保活帧练习（已离线测试，未作为正式协议启用）
│   ├── robolink_frame_parser.cpp 候选帧解析实现
│   ├── robolink_codec.cpp   零槽/单槽/多槽帧与消息载荷编解码
│   ├── robolink_byte_channel.cpp USB 收发与帧解析桥接
│   └── robolink_host_link.cpp 主机非阻塞运行逻辑
├── test/
│   ├── test_usb_cdc_port.cpp 无硬件设备生命周期与字节收发测试
│   ├── test_robolink_crc.cpp CRC 标准校验值测试
│   ├── test_robolink_frame.cpp 候选保活帧独立黄金字节测试
│   ├── test_robolink_frame_parser.cpp 半帧、连帧、重同步与 CRC 失败测试
│   ├── test_robolink_codec.cpp 消息字段与槽位往返测试
│   ├── test_robolink_byte_channel.cpp 无硬件 USB 字节桥接测试
│   └── test_robolink_host_link.cpp 伪终端握手、回读、超时测试
└── .github/
    └── workflows/
        └── ci.yml           复制自组织 CI 模板
```

> USB CDC、候选 RoboLink 编解码和主机连接状态机可独立构建与测试；工作区尚无装配程序，实际硬件联调未完成。

## 记录约束

MUST NOT 记录以下内容：

- `build/`、`install/`、`log/`、`.git/` 等依赖与产物目录
- 临时文件、缓存文件、日志文件
