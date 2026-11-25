# official_dev：官方 SDK 的 8899 通讯验证

本分支保留官方 ARCS SDK，增加独立的只读测试程序 `official_sdk_probe`。它只执行连接、登录和状态读取，不执行上电、启动、运动、停止或运动参数配置。

## 本次测试结果

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

## 重新编译和复现

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
