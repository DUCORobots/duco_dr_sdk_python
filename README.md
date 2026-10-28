# DUCO 仿生机器人 Python 二次开发接口

[简体中文](README.md) | [English](README_EN.md)

本项目通过 Thrift RPC 与机器人控制器通信，提供机械臂、腰部和头部的 Python 接口，支持连接管理、状态读取、运动控制、坐标系设置和数字 I/O 等操作。

使用流程：安装依赖 → 配置控制器 IP 和端口 → 读取状态确认连接 → 根据业务需要调用运动或 I/O 接口。不同控制器版本提供的服务、关节数量和接口支持范围可能不同，使用前请与实际设备配置核对。

## 环境准备

使用 Python 3。在项目根目录创建虚拟环境并安装 `six`；项目附带的 Thrift 运行库会导入该依赖。

```bash
python -m venv .venv
```

Windows PowerShell 激活环境：

```powershell
.\.venv\Scripts\Activate.ps1
```

Linux / macOS 激活环境：

```bash
source .venv/bin/activate
```

然后安装依赖：

```bash
python -m pip install -r requirements.txt
```

本 SDK 按源码目录使用，依赖见 [requirements.txt](requirements.txt)。基础同步 RPC 调用使用项目自带的 `lib/thrift/`，无需生成 RPC 代码或安装 Thrift 编译器。`six` 是该运行库使用的兼容性依赖，Python 3 环境也需要安装。

下面的代码和运行命令均假定工作目录为项目根目录；运行 `test.py`、`toolio.py` 时尤其需要注意这一点，它们使用相对路径加载依赖。

## 连接参数

| 部件 | 客户端类 | 示例端口 | 示例关节数量 |
| --- | --- | --- | --- |
| 左臂 | `DucoCobot` | `7003` | 7 |
| 右臂 | `DucoCobot` | `7004` | 7 |
| 腰部 | `DucoCobotWaist` | `7020` | 1 |
| 头部 | `DucoCobotHead` | `7021` | 2 |

本文代码中的 `127.0.0.1` 表示运行 Python 程序的本机，并非机器人的固定地址。请将代码中的 IP，以及示例文件中的 `ROBOT_IP` 或 `ip`，改为实际控制器地址，并确保对应 TCP 端口可达。

每个部件使用独立连接。`test.py` 原片段使用 `DucoCobotWaist` 连接头部端口；新代码应根据控制器实际 RPC 服务选择客户端，不要仅凭端口号混用封装类。

机械臂使能接口为 `enable(block)`，腰部和头部为 `enable(axis_num, block)`；后两者的轴参数取值请按设备协议确认，不要直接套用机械臂调用。

## 最小示例：连接并读取状态

将以下内容保存为项目根目录的 `read_state.py`。该示例只读取信息，不下发运动指令。

```python
import sys

sys.path.append('gen_py')
sys.path.append('lib')

from DucoCobot import DucoCobot
from thrift import Thrift


def main():
    robot = DucoCobot('127.0.0.1', 7003)  # 改为实际控制器 IP。
    try:
        # open() 返回 0 表示成功，-1 表示失败。
        if robot.open() != 0:
            raise ConnectionError('连接失败，请检查 IP、端口和控制器服务')

        print('SDK 版本:', robot.get_version())
        print('控制器版本:', robot.get_robot_version())
        print('机器人状态:', robot.get_robot_state())
        print('实际关节角（rad）:', robot.get_actual_joints_position())
        print('TCP 位姿（m、rad）:', robot.get_tcp_pose())
    finally:
        robot.close()  # 只在整个程序退出时关闭连接。


if __name__ == '__main__':
    try:
        main()
    except Thrift.TException as exc:
        print('RPC 通信异常:', exc)
```

```bash
python read_state.py
```

`get_version()` 返回客户端库版本，`get_robot_version()` 查询控制器版本。`close()` 断开通信，不会自动下使能、下电或复位数字输出。

`get_robot_state()` 返回 `[机器人状态, 程序状态, 安全控制器状态, 操作模式]`，对应枚举定义见 `gen_py/robot/ttypes.py`。先确认读取成功，再进行运动或 I/O 操作。

## 连接与心跳

**建立连接后应在业务运行期间持续复用，不要随单次运动或单侧流程结束而关闭。只在整个应用退出时统一清理连接。**

每个部件使用自己的客户端。心跳放在独立线程中，并使用另一条连接，避免与运动调用共用同一个 Thrift 客户端。多臂应用应保持两侧指令和心跳连接，直到全部业务结束。

`rpc_heartbeat(time=1000)` 的参数单位为 ms。心跳发送周期、RPC 等待时间和失联保护配置需与控制器要求匹配。SDK 将心跳用于远程失联后的停止处理；具体行为以控制器实现和配置为准。心跳异常时应记录错误、停止后续指令，并按业务需要显式调用停止接口，不能只依赖断开连接。

## 基本调用约定

### 阻塞与非阻塞

带 `block` 参数的任务接口通常遵循以下约定，具体以对应方法注释为准：

| 参数 | 返回值 | 使用方式 |
| --- | --- | --- |
| `block=True` | 任务结束时的状态 | 等待调用返回，并判断状态 |
| `block=False` | 任务 ID | 通过 `get_noneblock_taskstate(task_id)` 查询 |

`open()` 的成功返回值是 `0`，运动任务成功结束通常为 `TaskState.ST_Finished`，两者不能混用。融合运动也可能返回 `ST_Interrupt`，应结合具体指令判断。

常用任务状态定义在 `gen_py/robot/ttypes.py`：

| 枚举 | 值 | 含义 |
| --- | --- | --- |
| `ST_Idle` | 0 | 空闲 |
| `ST_Running` | 1 | 执行中 |
| `ST_Paused` | 2 | 暂停 |
| `ST_Stopped` | 3 | 已停止 |
| `ST_Finished` | 4 | 已完成 |
| `ST_Interrupt` | 5 | 中断 |
| `ST_Error` | 6 | 错误 |
| `ST_Illegal` | 7 | 非法任务 |
| `ST_ParameterMismatch` | 8 | 参数不匹配 |

`demo.py` 的 `demo_nonblocking_move()` 展示了任务 ID 轮询方式，并在任务没有正常完成时终止后续演示。

### 单位与坐标系

| 数据或接口 | 单位或含义 |
| --- | --- |
| 关节位置列表 | rad，长度须匹配实际关节数量 |
| 笛卡尔位姿 | `[x, y, z, rx, ry, rz]`，位置 m，姿态 rad |
| `movej`、`movej_pose` 的 `v` / `a` | 系统设定速度 / 加速度的百分比 |
| `movej2`、`movej_pose2` 的 `v` / `a` | rad/s、rad/s² |
| `movel`、`movec` 的 `v` / `a` | m/s、m/s² |
| `speed(percent)` | 全局速度比例，百分比 |
| `speedj` 的速度列表 | rad/s |
| `speedl` 的速度列表 | 前三项 m/s，后三项 rad/s |
| `speedj`、`speedl` 的时间参数 | ms，`-1` 表示持续运行 |

`r` 是轨迹融合参数，`0` 表示不融合；百分比运动接口和物理单位运动接口的 `r` 单位不同，不能直接照搬数值。

`movel()`、`movej_pose()` 等运动接口中的 `tool`、`wobj` 为坐标系名称，`''` 表示当前配置。`cal_fkine()`、`cal_ikine()` 的同名参数则为数值列表：工具偏移和工件位姿均为 `[x, y, z, rx, ry, rz]`；使用当前配置时传空列表 `[]`，不要传坐标系名称。例如，在已连接的 `robot` 上计算正逆解：

```python
joints = robot.get_actual_joints_position()
pose = robot.cal_fkine(joints, [], [])
solution = robot.cal_ikine(pose, joints, [], [])
```

`set_tool_data()`、`set_wobj()` 会影响后续运动使用的配置。`demo.py` 还显式引用 `tcp_1`、`tcp_2`、`user_1`，这些名称必须在控制器中正确配置。

## 基本运动示例

先按现场流程完成上电和状态确认，并核对目标关节范围、工具负载及运动空间。将以下代码保存为项目根目录的 `move_once.py`，修改 `ROBOT_IP` 和 `ARM_PORT` 后运行。该示例直接调用 SDK，不依赖 `demo.py`；它建立指令和心跳连接，使能后让第一关节在当前位置基础上增加 `0.02 rad`，其余关节目标保持不变，不自动返回起点。

```python
import sys
import threading

sys.path.append('gen_py')
sys.path.append('lib')

from DucoCobot import DucoCobot
from gen_py.robot.ttypes import TaskState

ROBOT_IP = '127.0.0.1'  # Replace with your controller IP.
ARM_PORT = 7003


def send_heartbeat(client, stop_event, errors):
    try:
        while not stop_event.is_set():
            client.rpc_heartbeat()
            stop_event.wait(1.0)
    except Exception as exc:
        errors.append(exc)


def check_heartbeat(errors):
    if errors:
        raise RuntimeError('Heartbeat communication failed') from errors[0]


def main():
    robot = DucoCobot(ROBOT_IP, ARM_PORT)
    heartbeat = DucoCobot(ROBOT_IP, ARM_PORT)
    heartbeat.socket.setTimeout(3000)
    stop_event = threading.Event()
    heartbeat_errors = []
    thread = None
    try:
        if robot.open() != 0 or heartbeat.open() != 0:
            raise ConnectionError('Command or heartbeat connection failed')
        thread = threading.Thread(
            target=send_heartbeat, args=(heartbeat, stop_event, heartbeat_errors),
        )
        thread.start()
        try:
            check_heartbeat(heartbeat_errors)
            if robot.enable(True) != TaskState.ST_Finished:
                raise RuntimeError('Enabling the robot failed')

            target = robot.get_actual_joints_position()
            target[0] += 0.02  # Verify the target against joint limits and workspace.
            state = robot.movej2(target, 0.2, 0.5, 0, True)
            check_heartbeat(heartbeat_errors)
            if state != TaskState.ST_Finished:
                raise RuntimeError('Motion did not finish: {}'.format(state))
        except BaseException:
            try:
                robot.stop(True)
            except Exception as exc:
                print('Failed to send stop command:', exc)
            raise
        # Reuse robot and keep the heartbeat running for further operations here.
    finally:
        # Close connections only when the complete application exits.
        stop_event.set()
        if thread is not None and thread.ident is not None:
            thread.join()
        robot.close()
        heartbeat.close()


if __name__ == '__main__':
    main()
```

```bash
python move_once.py
```

示例中的心跳线程只记录异常，主流程在运动前后检查；阻塞运动等待期间不会立即执行主线程检查。多步业务应持续复用 `robot`，并在适当位置检查心跳和任务结果。非阻塞任务的查询方式见 `demo_nonblocking_move()`。

使用持续速度运动时，还需通过 `speed_stop(True)` 结束 `speedj` / `speedl`。下使能和下电分别使用 `disable(True)`、`power_off(True)`，按现场流程调用。

## 运行现有示例

### 综合示例 `demo.py`

先修改文件顶部配置：

```python
ROBOT_IP = '127.0.0.1'
LEFT_ARM_PORT = 7003
RIGHT_ARM_PORT = 7004
JOINT_TRACK_PATH = None
```

然后从项目根目录运行：

```bash
python demo.py
```

`main()` 中的调用决定左右臂的执行顺序，当前为左臂后右臂，双臂并不同时运动。各侧依次演示机器人准备、非阻塞关节运动、坐标系与正逆解、各类运动。错误与日志查询、实时回放片段可按需取消注释启用。

当前程序会先连接双臂，即使只保留一侧运动调用，也仍需要另一侧服务可达。若设备只提供单臂服务，请同步调整 `main()` 中的客户端创建、连接列表、心跳启动和业务调用，仅保留所需侧。

该示例会设置全局速度为 `100%`，并调用 `collision_detect(0)` 关闭碰撞检测。运行前应按现场要求调整速度与碰撞检测等级，完整核对各段运动点位。

需要演示某一类接口时，可在 `run_arm_demo()` 中保留所需步骤；运动步骤依赖前面返回的位姿、关节参考值和坐标系配置，不宜直接跳过这些前置步骤。

### 可选实时轨迹回放

将 `JOINT_TRACK_PATH` 改为轨迹文件路径，并在 `run_arm_demo()` 中取消实时回放片段的注释后启用回放，例如：

```python
JOINT_TRACK_PATH = r'D:\tracks\jointpos.txt'
```

每个非空行包含 7 个空格分隔的关节角，文件单位为度，读取后转换为 rad：

```text
0 0 0 0 0 0 0
0.1 0 0 0 0 0 0
0.2 0 0 0 0 0 0
```

程序会在连接机器人前检查文件；空文件、关节数错误或非有限数值会报错。文件内容仅用于说明格式，实际轨迹须与机器人起点、关节限制和控制周期匹配。

实时回放不是基础接入步骤。当前 SDK 源码将相关接口标记为“不支持”，模式编号注释也与示例调用存在差异；仅在确认所用控制器版本支持后启用，并核对 `demo_realtime()` 中的模式、通信保护参数和等待时长，不要将其视为通用默认配置。

### 多部件示例 `test.py`

```bash
python test.py
```

连接双臂、腰部和头部，当前实际执行的是双臂无限循环 `servoj`。目标关节角持续递增，未设置上限；循环后的 `movej2`、腰部伺服和连接关闭代码在当前流程中不可达。使用前应改为适合现场的有限循环，并采用 `try/finally` 管理连接。

### 末端 I/O 示例 `toolio.py`

```bash
python toolio.py
```

同时连接左右臂，将两侧末端 **DO2 置为高电平**，随后依次读取左臂 DI1、左臂 DO1、右臂 DI1、右臂 DO1。读回通道与写入通道不同；若要确认 DO2，应调用 `get_tool_digital_out(2)`。

当前片段只显式关闭左臂连接，集成时应补齐两侧连接清理；断开连接不会自动复位 DO2。

## 文件说明

| 文件或目录 | 用途 |
| --- | --- |
| `DucoCobot.py` | 机械臂接口封装 |
| `DucoCobotWaist.py` | 腰部接口封装 |
| `DucoCobotHead.py` | 头部接口封装 |
| `gen_py/robot/` | Thrift 生成的 RPC 客户端、数据结构和枚举 |
| `lib/thrift/` | 随项目提供的 Thrift Python 运行库 |
| [demo.py](demo.py) | 机械臂综合示例，包含独立心跳连接和异常清理 |
| [test.py](test.py) | 双臂、腰部、头部连接及关节运动片段 |
| [toolio.py](toolio.py) | 双臂末端数字 I/O 示例 |

一般业务代码调用三个封装类即可；`gen_py/robot/` 中的生成代码应与控制器 RPC 协议配套使用。

## 常见问题

| 现象 | 检查方法 |
| --- | --- |
| `ModuleNotFoundError: No module named 'six'` | 使用运行脚本的同一 Python 环境执行 `python -m pip install six` |
| 找不到 `thrift` 或 `gen_py` | 确认代码目录完整，并从项目根目录运行；集成到其他应用时，将 SDK 根目录及其 `lib/`、`gen_py/` 目录加入模块搜索路径 |
| `open()` 返回 `-1` | 检查控制器 IP、TCP 端口、网络与 RPC 服务；连接失败后不要继续调用接口 |
| 任务返回错误、停止或参数不匹配 | 查看任务状态、`get_last_error()`、使能状态、关节数量及参数单位 |
| 单臂服务可连接，但运行 `demo.py` 失败 | 综合示例默认连接双臂；按上文说明同时调整连接和业务配置 |
| 心跳异常或切换手臂时任务停止 | 检查心跳异常、底层通信超时和控制器提示；确认业务期间没有关闭任一侧的指令或心跳连接 |
| 位姿或运动方向不符合预期 | 检查当前工具/工件坐标系，以及指令中显式指定的坐标系 |
| 非阻塞调用返回后机器人仍在运动 | 返回值是任务 ID，应查询状态；需要停止时调用对应停止接口 |
| 旧片段打印异常时出现 `AttributeError: ... message` | 在 Python 3 中使用 `str(exc)` 或 `print(exc)`，不要依赖 `exc.message` |

更完整的接口签名与参数说明见 [DucoCobot.py](DucoCobot.py)、[DucoCobotWaist.py](DucoCobotWaist.py) 和 [DucoCobotHead.py](DucoCobotHead.py)。
