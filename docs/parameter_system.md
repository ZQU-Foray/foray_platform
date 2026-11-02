# 参数系统

> 所属层：L1 平台基础 · 对应横切面：X4 可观测与调参
> 代码：`src/foray_platform_core/include/foray/platform/param/`

## 1. 目标

- **类型化**：参数在代码中声明类型、默认值、边界与枚举；
- **校验**：启动期与热加载都执行同一套校验，配置错误立即暴露；
- **热加载**：运行期重新读取配置，整批原子生效，不影响实时线程；
- **可追溯**：每个参数可记录单位与「魔法数字来源」。

## 2. 组成

```
ParamSchema   声明参数（ParamSpec 集合），校验声明本身
ParamStore    保存当前值；不可变快照 + 原子替换
ParamSnapshot 不可变快照；无锁读取，供一个控制周期复用
ParamSource   参数来源抽象（YAML 文件、内存等）
ParamManager  schema + 多来源分层 + 初始加载 / 热加载门面
```

## 3. 声明参数

```cpp
using namespace foray::platform::param;

ParamSchema schema;

ParamSpec speed;
speed.name = "chassis.max_speed";
speed.type = ParamType::kDouble;
speed.default_value = ParamValue(3.0);
speed.description = "底盘最大线速度";
speed.unit = "m/s";
speed.minimum = ParamValue(0.0);
speed.maximum = ParamValue(10.0);
speed.source_note = "2026 规则手册 P.12；2026-03 标定";
if (!schema.add(std::move(speed)).ok()) {
    // schema 定义错误：名称、默认值、边界或枚举非法
}
```

### 3.1 参数名

使用点分标识符，每段匹配 `[A-Za-z_][A-Za-z0-9_]*`：

```
chassis.max_speed
shooter.burst
serial.port
```

点分名称与 YAML 嵌套映射一一对应。

### 3.2 声明校验规则

- 名称合法且唯一；
- 默认值类型与声明类型兼容；
- 默认值满足边界与枚举；
- `minimum` / `maximum` 只允许用于数值类型，且下界不大于上界；
- `int` 参数的边界必须是 `int`（防止静默截断）；
- 枚举值类型一致且不重复（`int` 与 `double` 宽化后相同视为重复）。

## 4. 值类型与转换

支持 `bool`、`int`（`int64_t`）、`double`、`string` 四种标量。

转换规则：

| 来源 → 目标 | 是否允许 |
|---|---|
| `int` → `double` | 允许（无损宽化） |
| `double` → `int` | **拒绝**（会静默截断） |
| 其他跨类型 | 拒绝 |

因此 YAML 中的 `max_speed: 2` 可以赋给 `double` 参数，最终存为 `2.0`；
但 `burst: 2.5` 不能赋给 `int` 参数。

## 5. YAML 来源

```yaml
chassis:
  max_speed: 3.5
  enabled: true
shooter:
  burst: 5
```

等价于点分名称：

```
chassis.max_speed = 3.5
chassis.enabled   = true
shooter.burst     = 5
```

规则：

- 只支持映射与标量，**不支持序列**；
- `bool` 只接受 `true` / `false`；
- `int` 必须是完整的十进制整数；
- `double` 接受整数与小数，必须是有限值；
- 空文件合法，表示不覆盖任何参数；
- **schema 未声明的键 -> 报错**（防止拼写错误被静默忽略）；
- 同一参数出现多次 -> 报错。

## 6. 分层覆盖

`ParamManager` 按来源顺序应用，后者覆盖前者：

```cpp
auto shared = std::make_shared<YamlFileParamSource>("config/shared.yaml");
auto robot = std::make_shared<YamlFileParamSource>("config/sentry.yaml");
ParamManager params(std::move(schema), {shared, robot});

if (!params.load_initial().ok()) {
    // 启动失败：配置错误必须阻断启动
}
```

这对应架构约定的两级配置：

```
config/shared.yaml       <- 全兵种共享（坐标系统一、通用阈值）
   ↑ 覆盖
config/<robot_type>.yaml <- 兵种 / 场景差异
```

## 7. 初始加载与热加载

| 阶段 | 接口 | 允许修改不可热加载参数 |
|---|---|---|
| 启动期 | `load_initial()` | 是 |
| 运行期 | `reload()` | 否（值变化则整批失败） |

热加载流程：

1. 在**非实时线程**调用 `reload()`；
2. 依次重新读取全部来源并合并；
3. 对合并结果做完整校验；
4. 全部成功才原子替换快照；任一失败则保持旧快照。

`ParamManager::reload()` 在 `load_initial()` 成功之前返回 `kNotInitialized`。
重复调用 `load_initial()` 会退化为一次热加载。

## 8. 实时读取

读接口不加锁，热路径应使用快照：

```cpp
void control_cycle() {
    // 每个控制周期只获取一次，周期内复用。
    const auto snapshot = params.snapshot();

    const double max_speed = snapshot->get_double("chassis.max_speed", 0.0);
    const int burst = snapshot->get_int("shooter.burst", 0);
    // ...
}
```

- `snapshot()` 返回 `shared_ptr<const ParamSnapshot>`，读者看到要么全是旧值、
  要么全是新值；
- `find()` 接受 `std::string_view` 且不分配内存；
- `generation()` 在值真正变化时递增，可用于检测配置是否更新；
- `get_*()` 在缺失或类型不符时返回调用方提供的默认值。

写入会复制映射并分配内存，**禁止在实时线程调用**（与 X4「调参不影响实时」一致）。

## 9. 错误处理

批量操作会收集全部错误后一次性返回：

```cpp
const ParamResult result = params.reload();
if (!result.ok()) {
    for (const ParamError& error : result.errors()) {
        // error.code / error.name / error.message
    }
}
```

错误码：

| 错误码 | 含义 |
|---|---|
| `kInvalidSchema` | schema 声明本身非法 |
| `kDuplicateParam` | 同名参数重复 |
| `kUnknownParam` | 配置中出现未声明参数 |
| `kTypeMismatch` | 类型不符 |
| `kOutOfRange` | 超出边界或非有限数值 |
| `kValueNotAllowed` | 不在枚举集合中 |
| `kNotHotReloadable` | 热加载试图修改不可热加载参数 |
| `kSourceFailure` | 来源读取或解析失败 |
| `kNotInitialized` | 未初始加载就调用热加载 |

## 10. 约束与后续

- 不支持序列（数组）参数；需要时再扩展并补充校验规则；
- 不支持参数变更回调；若后续需要，回调必须明确在非实时线程执行；
- ROS 2 参数桥接（`rclcpp` 参数事件 -> `ParamStore`）留到适配层，
  核心库不依赖 ROS；
- 当前 `package.xml` 的许可证字段为 `TODO`，需由组织确认后填写。
