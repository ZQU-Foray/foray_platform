# 统一配置解析栈

> 所属层：L1 平台基础 · 代码：`src/foray_platform_core/include/foray/platform/config/`
> 归属：解析机制在 `foray_platform`；机器人相关配置值与装配在 `foray_robots`。

## 1. 为什么放在 `foray_platform`

组织文档的归属已经明确：

- `foray_docs/repository_structure.md` §2：`foray_platform` 的内容包含「参数系统」；
- `foray_docs/algorithm_structure.md` §1.7：L1 包含「参数系统（schema + 校验 + 热加载）」；
- 同文档 §1 把「把 config 拆出独立仓」列为反模式——config 属于装配根；
- `foray_robots` 拥有 `config/shared.yaml` + `config/<robot_type>.yaml`，负责注入配置。

因此：

| 内容 | 归属 |
|---|---|
| YAML/JSON 解析、分层、校验、导出 | `foray_platform`（core target） |
| 兵种/场景的具体值 | `foray_robots`（装配根） |
| 设备能力探测 | 驱动（`open()` 时） |

未来若出现非 C++ 消费者需要稳定绑定、或配置栈需要独立发布节奏，
再按组织判据 C1–C6 评估是否拆出 `foray_config`。

## 2. 组成

| 类型 | 职责 |
|---|---|
| `ConfigLayer` | 一层配置（名称、路径、是否必需） |
| `ConfigDocument` | 有序层集合 + 环境变量覆盖开关 |
| `ConfigResolver` | 加载、include、env 替换、合并、校验 |
| `ConfigSnapshot` | 已校验值 + 每个值的来源（provenance） |
| `ConfigExporter` | dump / Markdown schema / JSON Schema |
| `ConfigTool` | 便捷门面（CLI 与测试复用） |
| `EnvProvider` | 环境变量抽象（进程环境 / 测试注入） |

底层复用现有 `foray_platform_core::param` 的 `ParamSchema`、`ParamValue`、
`ParamStore`，不重复实现类型与范围校验。

## 3. 层与加载顺序

```text
shared.yaml        # 全兵种共享
  → <robot_type>.yaml
  → local/*.yaml   # 本机覆盖（.gitignore）
  → FORAY_<PARAM>  # 环境变量覆盖层
```

- 后加载覆盖先加载；
- 必需层不存在 → 报错；可选层缺失 → 跳过；
- 文件名/数量由装配根决定，解析器不猜测。

## 4. 文件格式

```yaml
include:
  - base/common.yaml
  - base/camera.yaml

camera:
  width: 1280
  serial: "${CAMERA_SERIAL}"
  exposure_us: 2000

lidar:
  host_ip: "192.168.1.5"
  port: 56301
```

规则：

- 顶层 `include`：字符串或字符串序列；路径相对当前文件；
- 循环 include 会报错；同一文件可被多个分支重复 include（后者覆盖前者）；
- 嵌套映射展开为点分名称（`camera.width`）；
- 同一文件内重复键（含 `camera: {width: ...}` 与 `camera.width: ...` 混用）报错；
- 未知键严格报错，防止拼写错误被静默忽略。

## 5. 环境变量

### 5.1 `${VAR}` 替换

- 只对标量文本做替换；未定义变量报错；
- 可整体替换（`serial: "${CAMERA_SERIAL}"`），也可嵌入字符串；
- 替换后按 schema 的期望类型解析（例如 `width: "${WIDTH}"` 仍要求整数）。

### 5.2 `FORAY_<PARAM>` 覆盖层

在所有文件层之后，解析器按 schema 查找：

```text
camera.width  ->  FORAY_CAMERA_WIDTH
lidar.host_ip ->  FORAY_LIDAR_HOST_IP
```

- 变量存在时覆盖文件值；
- 类型/范围仍按 schema 校验；
- provenance 记为 `env:FORAY_CAMERA_WIDTH`。

## 6. 来源追踪

`ConfigSnapshot::provenance` 记录**显式提供过**的参数来自哪个文件或环境变量：

```cpp
const ConfigSnapshot snapshot = ...;
if (const std::string* source = snapshot.source("camera.width")) {
    // /path/to/sentry.yaml 或 env:FORAY_CAMERA_WIDTH
}
```

未显式配置的参数使用 schema 默认值，`source()` 返回 `nullptr`。
provenance 只在配置加载期保留，不进入 HAL 热路径。

## 7. 校验

- `ParamSchema` 负责类型、边界、枚举与默认值；
- `ParamStore` 负责批量校验与 `int -> double` 宽化；
- `ConfigResolver` 一次性返回所有错误（`ParamResult`），便于启动期排查；
- 跨字段约束（如 transport 与 serial/ip 的组合）不属于 `ParamSchema`，
  由 HAL 的 `ConfigBinder` 实现。

## 8. 导出

```cpp
ConfigExporter::dump(snapshot);              // 值 + 来源注释
ConfigExporter::schema_markdown(schema);     // 文档/评审表格
ConfigExporter::schema_json(schema);         // JSON Schema draft 2020-12
```

用途：

- `dump`：启动日志、排障、CI 工件；
- `schema_markdown`：配置文档；
- `schema_json`：编辑器补全/CI 校验。

## 9. CLI

`ConfigTool` 已提供 `validate` / `dump` / `schema_*` 的门面。
可执行文件 `foray_config_tool` 需要真实 schema（HAL 的相机/雷达配置），
在 M6 之后落地；当前由单元测试覆盖。

## 10. 限制

- 现有 `ParamSchema` 不支持数组/动态列表；多传感器用「一传感器一文件」或固定 slot；
- 增删传感器需要重启；曝光/增益等标量可在运行期通过 HAL setter 更新；
- 解析器不探测设备能力，`driver_options` 中的未知节点由驱动在 `open()` 报错；
- 配置栈不处理参数热加载；热加载仍由 `ParamManager` 负责。

## 11. 示例

```cpp
ConfigDocument document;
document.add_layer("shared", "config/shared.yaml");
document.add_layer("robot", "config/sentry.yaml");
document.add_layer("local", "config/local/sentry.yaml", /*required=*/false);

const ConfigResolver resolver(make_platform_schema());
ConfigSnapshot snapshot;
const param::ParamResult result = resolver.load(document, snapshot);
if (!result.ok()) {
    // 启动失败：result.to_string() 包含全部错误
}
```
