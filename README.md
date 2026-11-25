# AUBO V4.5 控制器通讯

本项目使用旧版 AUBO `rs_*` C 接口连接 TCP **8899**，适用于当前现场控制器。它与 ARCS 的 `RpcClient`（30004）/RTDE（30010）接口不同。

已在以下设备上验证登录、状态读取及连续读取：

- IP：`169.254.138.6`，端口：`8899`
- 示教软件：`V4.5.56_0021799`
- 控制器 Server：`V4.5.111.71f2ee1-Alpha`
- 示教器显示 SDK：`V2.5.3_6e1ea35`

## SDK 来源与版本

旧官网开发包链接已失效。本次使用的是[公开项目保存的旧版 Windows x64 SDK](https://github.com/ziywang50/AubosdkProject/tree/881657eb2274ee8f45ee0cdf92e3aa652b10e874/dependens/auboi5)，**不是官方 V2.5.3 发布包**。`libserviceinterface.dll` 文件版本为 `1.3.0.0`；文件版本不能当作现场示教器的 SDK 版本。已通过真机只读调用验证当前使用的登录、状态读取、位姿读取和姿态转换接口。

原始下载地址、固定仓库版本和每个文件的 SHA-256 记录在 `sdk/source.json`。下载文件保持原样。SDK 原始头文件中的部分中文注释已乱码，不影响接口声明。

`sdk/bin` 内的五个 DLL 需要一起部署。此库由 MinGW 编译，本项目用 MSVC 调用其 C 接口，仅传递固定布局的数据结构；不要直接调用库里的 C++ 类或跨库传递 `std::string`、`std::vector`。

替换前的 SDK、源码及 Release 程序保存在本机 `backups/before-legacy-sdk/`，备份不纳入 Git。

## 编译

需要 Visual Studio C++ x64 工具链、Windows SDK、CMake 3.20 以上。在项目根目录执行：

```powershell
cmake -S . -B build -A x64
cmake --build build --config Release
```

CMake 会将 `sdk/bin/*.dll` 复制到 exe 同目录。默认 SDK 路径是 `sdk`，可以通过 `-DAUBO_SDK_DIR=绝对路径` 修改；新路径必须包含相同的旧版 C 接口和配套库。

## 常用执行命令

以下命令均在项目根目录的 PowerShell 中执行，`--ip` 填机械臂控制器的地址。当前现场地址为 `169.254.138.6`。

```powershell
Set-Location D:\gitRepo\aubo_telecommunication_cpp
```

```powershell
# 读取一次；省略 read 也可以
.\build\Release\robot_communication.exe read --ip 169.254.138.6

# 以 10 Hz 轮询 30 秒，Ctrl+C 提前停止
.\build\Release\robot_communication.exe watch --ip 169.254.138.6 --seconds 30 --hz 10

# 读取原始诊断字段及应用层的运动拦截原因，不执行运动
.\build\Release\robot_communication.exe diagnose --ip 169.254.138.6

# 查看所有参数
.\build\Release\robot_communication.exe --help
```

默认地址是 `127.0.0.1:8899`，实机需提供 `--ip`；可用 `--port` 指定端口。旧版 `rs_login` 不接受用户和密码参数，因此不再提供 `--user`、`--password`。

`read`、`watch` 和 `diagnose` 不执行上电、启动、运动或工具配置操作。`watch` 的 `--hz` 是轮询频率，范围 1–50，不能保证控制器内部反馈以同样频率更新。

`diagnose` 输出原始状态及 `motion_blockers`。`remoteHalt_raw` 是远程关机输入信号，旧头文件没有规定其有效电平；现场控制器在示教器可运动时也返回 `1`，因此不把这个值单独判定为故障，也不反转其极性。软急停、远程急停、碰撞、力控模式及明确硬件故障仍拦截运动，报错会列出具体字段和值。`motion_blockers: []` 仅表示应用层未发现这些拦截条件，不代表控制器已经授权运动。

标准输出为每行一条 JSON，连接和错误信息写入标准错误：

| 字段 | 含义 |
|---|---|
| `time` | 电脑收到本次读取结果时的 Unix 秒数 |
| `frame` | `base`，机器人基座坐标系 |
| `pose_reference` | `flange`，末端法兰 |
| `pose_m_rad` | 法兰 `[x,y,z,roll,pitch,yaw]`，米和弧度 |
| `xyz_mm` | 法兰位置，毫米 |
| `quaternion_wxyz` | 法兰姿态四元数，顺序为 w,x,y,z |
| `joints_rad` / `joints_deg` | 六关节角，弧度/度 |

**位姿语义有变化：这里读取和控制的是法兰，不会自动应用示教器选中的工具 TCP。** 若需要夹爪尖端等 TCP，必须明确配置工具变换后另行适配，不能直接把法兰位置当作工具尖端位置。

退出码：成功 `0`，参数/连接/读取错误 `1`，Ctrl+C `130`。SDK 不兼容时不能只更换端口号。

## 运动命令

保留 `move --pose X Y Z RX RY RZ` 与 `move --delta-mm DX DY DZ`。绝对目标是基座系中的法兰位姿，位移也是沿基座轴计算。速度为 0.01 m/s，线加速度为 0.05 m/s²；姿态速度/加速度限制为 0.1 rad/s 和 0.1 rad/s²，关节速度/加速度限制为 0.2 rad/s 和 0.2 rad/s²。

可先加 `--dry-run` 检查诊断状态并计算目标/逆解，**不发送运动或运动配置指令**：

```powershell
.\build\Release\robot_communication.exe move --ip 169.254.138.6 --delta-mm 0 0 10 --dry-run
```

实际让法兰沿**基座 Z 正方向移动 10 毫米**，保持当前姿态：

```powershell
.\build\Release\robot_communication.exe move --ip 169.254.138.6 --delta-mm 0 0 10
```

`--delta-mm` 后依次为 `DX DY DZ`，单位为毫米。按需要替换上面命令的三个数值，每次仅执行所需的一条命令：

| 期望移动 | `--delta-mm` 参数 |
|---|---|
| 基座 X 正方向 1 mm | `1 0 0` |
| 基座 X 负方向 1 mm | `-1 0 0` |
| 基座 Y 正方向 1 mm | `0 1 0` |
| 基座 Y 负方向 1 mm | `0 -1 0` |
| 基座 Z 正方向 10 mm | `0 0 10` |
| 基座 Z 负方向 10 mm | `0 0 -10` |

指定等待超时，例如最长等待 60 秒：

```powershell
.\build\Release\robot_communication.exe move --ip 169.254.138.6 --delta-mm 0 0 10 --timeout 60
```

移动到绝对法兰位姿的命令格式如下。`X Y Z RX RY RZ` 是占位符，需要替换为实际目标数值；位置单位为**米**，姿态为 **RPY 欧拉角，单位弧度**。可先从 `read` 的 `pose_m_rad` 确认当前位置及单位。

```text
.\build\Release\robot_communication.exe move --ip 169.254.138.6 --pose X Y Z RX RY RZ --dry-run
.\build\Release\robot_communication.exe move --ip 169.254.138.6 --pose X Y Z RX RY RZ
```

默认等待超时为 30 秒；执行过程中按 `Ctrl+C` 会请求停止。

调用前需从示教器完成上电和启动，并停止已有程序。代码检查运动状态、上电状态、急停、碰撞及硬件故障；先对法兰目标求逆解，再通过非阻塞 `rs_move_line` 发送关节目标，异常、超时或 Ctrl+C 时请求 `rs_move_stop`。运动配置使用基座坐标、法兰参考、关闭相对偏移及提前到位。

实机运动由操作者在现场验证，自动化测试使用模拟 SDK。SDK 状态检查不能替代现场对控制权、已有示教程序及工作空间的确认。

## 离线检查

`tests` 中的模拟 SDK 不连接任何设备，可验证参数、单位转换、运动目标类型、异常停止及轮询输出：

```powershell
cmake -S tests -B build/offline-tests -A x64
cmake --build build/offline-tests --config Release
python tests/test_cli.py build/offline-tests/Release/robot_communication_mock.exe
```
