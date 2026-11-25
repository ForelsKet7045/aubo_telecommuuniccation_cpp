# official_dev：机械臂通讯与运动命令

本分支使用官方 ARCS SDK `0.26.0-rc.6+d2cc0fb`，主程序为 `robot_communication`。当前机械臂 IP 为 `192.168.1.125`，子网掩码为 `255.255.255.0`；用户已确认 `read` 可以正常读取，`move --delta-mm 0 0 1` 可以执行并到位。以下命令均在项目根目录的 PowerShell 中执行。

## 编译

需要 Windows x64、Visual Studio 的 C++ 编译工具和 CMake。

```powershell
cmake -S . -B build -A x64
cmake --build build --config Release
```

可执行文件位于 `build/Release/robot_communication.exe`，构建时会将 SDK DLL 复制到同一目录。修改 `main.cpp` 后重新执行第二条命令即可。

## 常用命令

### 查看帮助

```powershell
.\build\Release\robot_communication.exe --help
```

当前支持 `read`、`watch`、`move` 三个子命令；省略子命令时默认执行 `read`。

### read：读取一次当前位姿和关节角

```powershell
.\build\Release\robot_communication.exe read --ip 192.168.1.125
```

输出包括机器人模式、安全状态及一行 JSON 数据：

| 字段 | 含义与单位 |
|---|---|
| `time` | 本机时间戳，秒 |
| `pose_m_rad` | TCP 在基坐标系中的位姿 `[X, Y, Z, RX, RY, RZ]`，位置为米，姿态为 RPY 弧度 |
| `xyz_mm` | 同一 TCP 位置 `[X, Y, Z]`，毫米 |
| `joints_rad` | 六个关节角，弧度 |
| `joints_deg` | 六个关节角，度 |

### watch：持续读取位姿和关节角

下面以 10 Hz 读取 30 秒，按 `Ctrl+C` 可以提前结束：

```powershell
.\build\Release\robot_communication.exe watch --ip 192.168.1.125 --seconds 30 --hz 10
```

`--seconds` 默认 10 秒，必须大于 0；`--hz` 默认 10，允许整数 `1..50`。该命令通过 RTDE 获取反馈，需控制器开放对应服务。

### move：沿基坐标系相对移动

**以下命令会实际驱动机械臂。** 执行前确认目标和路径安全，机器人已上电并处于 `Running`、安全状态为 `Normal`，示教程序已停止且机械臂静止。程序不会自动上电或修改 TCP 配置。

沿基坐标系 Z 正方向移动 **1 mm**，保持当前 TCP 姿态：

```powershell
.\build\Release\robot_communication.exe move --ip 192.168.1.125 --delta-mm 0 0 1
```

沿基坐标系 Z 正方向移动 **10 mm**，最多等待 60 秒：

```powershell
.\build\Release\robot_communication.exe move --ip 192.168.1.125 --delta-mm 0 0 10 --timeout 60
```

沿基坐标系 Z 正方向移动 **50 mm**，指定速度 **50 mm/s**、加速度 **100 mm/s²**，最多等待 60 秒：

```powershell
.\build\Release\robot_communication.exe move --ip 192.168.1.125 --delta-mm 0 0 50 --speed-mm-s 50 --accel-mm-s2 100 --timeout 60
```

执行前确认正 Z 方向有足够空间。每次执行相对移动都会从**当时的位置**重新计算目标；上次超时前可能已移动了一部分，重试时应根据当前位置确定剩余偏移，避免再次移动完整的 50 mm。

`--delta-mm DX DY DZ` 的三个参数单位均为毫米，负数表示对应轴的负方向。偏移基于**基坐标系**，不是工具自身的坐标轴；目标从命令执行时的当前 TCP 位姿计算。

### move：移动到绝对 TCP 位姿

命令格式如下，执行时将占位符替换为实际目标数值：

```text
.\build\Release\robot_communication.exe move --ip 192.168.1.125 --pose X Y Z RX RY RZ --timeout 60
```

`X Y Z` 是基坐标系下的 TCP 位置，单位为**米**；`RX RY RZ` 为 RPY 姿态，单位为**弧度**。可参考 `read` 输出的 `pose_m_rad` 确定目标。一次 `move` 只能指定 `--pose` 或 `--delta-mm` 中的一种。

两种 `move` 均调用直线运动 `moveLine`，无轨迹融合。运动参数如下，均只适用于 `move`：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--speed-mm-s` | `50` | 设定直线速度，毫米/秒，必须为有限正数 |
| `--accel-mm-s2` | `100` | 设定直线加速度，毫米/秒²，必须为有限正数 |
| `--timeout` | `60` | 等待到位的秒数，必须为有限正数 |
| `--verbose` | 关闭 | 开启后约每 200 ms 输出一次运动位姿反馈 |

程序会将速度、加速度从毫米单位转换成 SDK 所需的米单位。默认只输出起点、目标和最终位姿，并在运动前尝试读取并显示 `Controller speed fraction`，不修改控制器速度倍率。例如倍率为 `10%` 时，50 mm/s 的设定速度仍会受到倍率缩放；实际时间还取决于加减速、路径和控制器限制，60 秒并非到位保证。倍率很低或为零时，应检查示教器及控制器设置。

之前的默认速度为 10 mm/s、加速度为 50 mm/s²、超时为 30 秒；本次调整提高速度并延长等待时间，以改善 50 mm 移动超时。运动开始后的超时、异常或 `Ctrl+C` 仍会触发程序请求 `stopLine`；超时日志增加剩余 XYZ 距离，便于判断是未走完还是到位条件未满足。

需要逐帧反馈时，在原命令末尾追加 `--verbose`：

```powershell
.\build\Release\robot_communication.exe move --ip 192.168.1.125 --delta-mm 0 0 1 --verbose
```

此前使用原速度参数的现场验证：`192.168.1.125` 执行 `move --delta-mm 0 0 1` 时，模式为 `Running`、安全状态为 `Normal`；首条运动反馈的 Z 约为 `478.300 mm`，最后为 `479.307 mm`，变化约 `1.007 mm`。首条反馈发生在运动指令发出后，精确到位判断使用程序内部保存的目标；本次最终输出 `Arrived: position error < 0.01 mm, orientation error < 0.0001 rad`。新速度参数的 50 mm 移动、`watch` 和绝对位姿移动仍需现场验证。

本次成功运动的日志开头仍出现 `SDK: login Send exception: [json.exception.parse_error.101] ... last read: '_'`。这条日志没有阻止本次状态读取、运动和到位判断，但其具体原因尚未确认，不能据此认定所有 SDK 接口均兼容。

### 连接参数

| 参数或服务 | 当前默认值 |
|---|---|
| `--ip` | `127.0.0.1`，连接实体机械臂时请显式传入 `192.168.1.125` |
| `--user` | `aubo` |
| `--password` | `123456` |
| `read` / `move` 的 RPC 端口 | `30004` |
| `watch` 的 RTDE 端口 | `30010` |

账号密码如有修改，可在命令末尾追加 `--user 用户名 --password 密码`。主程序的端口固定在代码中，当前没有 `--port`、`--dry-run` 或 `diagnose` 参数。

## 重新编译后的 build 内容要提交 Git 吗？

**日常开发不需要提交 `build/`。** 提交修改过的源代码、`CMakeLists.txt`、README 以及必要的依赖配置即可；其他电脑可按上面的命令重新构建。需要直接分发程序时，可将 Release 可执行文件及所需 DLL 打包为发布附件。

仓库的 `.gitignore` 已有 `/build/`，但目前 Git 中仍跟踪着 67 个 `build/` 文件，包括可执行文件、对象文件和 CMake 缓存。它们此前已加入版本库，所以重新编译仍会显示修改，`.gitignore` 不会自动取消已有跟踪。

如要清理这部分版本跟踪，在项目根目录执行一次：

```powershell
git rm -r --cached -- build
git status --short
```

`--cached` 会保留本机 `build/` 文件，只在 Git 暂存区记录取消跟踪。检查这些删除记录后，随项目修改一起提交即可；此后重新编译的产物会被现有 `.gitignore` 忽略。本文档更新未执行上述取消跟踪操作。

## 历史记录：旧控制器的 8899 通讯验证

以下记录针对此前的 `169.254.138.6` 控制器，与当前 `192.168.1.125` 的读取结果分开记录。独立只读测试程序 `official_sdk_probe` 只执行连接、登录和状态读取，不执行上电、启动、运动、停止或运动参数配置。

### 测试结果

目标控制器：`169.254.138.6:8899`，Server `V4.5.111.71f2ee1-Alpha`。

测试使用本分支 `sdk/lib/aubo_sdk.dll`。SDK 的 `ServiceInterface::getVersion()` 返回 `0.26.0`；DLL 与此前 `0.26.0-rc.6+d2cc0fb` 备份以及 `D:\gitRepo\3DScanner\3rdlibs\windows\aubo-sdk` 的核心库 SHA-256 相同：

```text
a5dbf1744643fef29d14889808bb6a5d5ebdd0f3cdf492a7b212cb31c7698a37
```

| 验证方式 | 结果 |
|---|---|
| 普通 TCP 连接到 8899 | 成功 |
| `RpcClient::connect(ip, 8899)` | 返回 `-7`；JSON-RPC 接口版本查询约 3 秒后超时 |
| `rs_login(handle, ip, 8899)` | 返回 `-1`；内部连接 RTDE 8903 失败 |
| `ServiceInterface::robotServiceLogin(ip, 8899, ...)` | 返回 `-1`；内部连接 RTDE 8903 失败 |
| 旧版 `libserviceinterface.dll` 对照读取同一控制器 | 登录、状态及位姿读取成功 |

直接使用 `RpcClient` 时，SDK 日志显示它向 8899 发出了如下握手请求，未收到有效响应：

```json
{"jsonrpc":"2.0","method":"SystemInfo.getInterfaceVersionCode"}
```

因此，本次结论是：**8899 的 TCP 网络可达，但这份官方 SDK 在控制器当前服务配置下，三种已测试接口均未完成可用的 SDK 会话及数据读取。** 这一结论仅针对上述 DLL 和现场控制器，不代表所有官方历史 SDK 都不兼容。

完整阶段日志、耗时、退出码和旧版库的读取对照记录见 [validation/official-sdk-8899.json](validation/official-sdk-8899.json)。

### 编译只读测试程序并复现旧控制器测试

在项目根目录的 PowerShell 中执行：

```powershell
cmake -S . -B build/official-probe -A x64
cmake --build build/official-probe --config Release --target official_sdk_probe

python tools/run_official_sdk_probe.py --exe build/official-probe/Release/official_sdk_probe.exe --ip 169.254.138.6 --port 8899 --output downloads/official-sdk-probe-8899.json
```

Python 脚本将三种接口放在独立进程中运行，每个进程最多运行 15 秒，完整记录结果。脚本完成不等于 SDK 登录成功，需查看每项的 `exit_code` 和 `read_completed`。

也可以单独测试一种接口：

```powershell
.\build\official-probe\Release\official_sdk_probe.exe rpc --ip 169.254.138.6 --port 8899 --timeout-ms 3000
.\build\official-probe\Release\official_sdk_probe.exe rs --ip 169.254.138.6 --port 8899
.\build\official-probe\Release\official_sdk_probe.exe service --ip 169.254.138.6 --port 8899
```

`rs` 和 `service` 的全部内部连接不一定受 RPC 请求超时控制，建议使用 Python 脚本进行测试，以获得进程超时限制。只读测试程序为显式构建目标，不加入默认构建，不替换原有 `robot_communication` 程序。
