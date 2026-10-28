# DUCO Robot Python SDK

[简体中文](README.md) | [English](README_EN.md)

Python interfaces for communicating with a DUCO robot controller over Thrift RPC. The SDK provides clients for the robot arms, waist, and head, with methods for reading status, controlling motion, configuring coordinate systems, and accessing digital I/O.

Start by installing the dependencies, setting the controller IP and port, and reading robot status. Then add the motion or I/O operations required by your application. Available services, joint counts, and supported features depend on the controller version and robot configuration.

## Installation

Use Python 3. From the repository root, create a virtual environment:

```bash
python -m venv .venv
```

Activate it in Windows PowerShell:

```powershell
.\.venv\Scripts\Activate.ps1
```

Or in Linux / macOS:

```bash
source .venv/bin/activate
```

Install the dependencies:

```bash
python -m pip install -r requirements.txt
```

The bundled Thrift runtime requires `six`, a Python 2/3 compatibility library. This dependency is still required when using the current runtime with Python 3.

Use the SDK directly from its source directory. The basic synchronous RPC examples use the bundled runtime; you do not need to regenerate RPC code or install the Thrift compiler.

Run the commands and code snippets below from the repository root. In particular, `test.py` and `toolio.py` load dependencies using relative paths.

## Connection Settings

| Component | Client class | Example TCP port | Example joint count |
| --- | --- | --- | --- |
| Left arm | `DucoCobot` | `7003` | 7 |
| Right arm | `DucoCobot` | `7004` | 7 |
| Waist | `DucoCobotWaist` | `7020` | 1 |
| Head | `DucoCobotHead` | `7021` | 2 |

Set the IP address to your controller's address and verify that the required TCP ports are reachable. `127.0.0.1` in this guide refers to the computer running Python, not a fixed robot address. Update the IP in your code and the `ROBOT_IP` or `ip` setting in any example you use.

Each component uses a separate connection. The existing `test.py` fragment uses `DucoCobotWaist` for the head port. Select the client that matches the RPC service provided by your controller rather than assuming the client classes are interchangeable.

The arm uses `enable(block)`. The waist and head use `enable(axis_num, block)`; confirm the axis argument against your device protocol.

## Quick Start: Read Robot Status

Save the following as `read_state.py` in the repository root. This example reads information without issuing motion commands.

```python
import sys

sys.path.append('gen_py')
sys.path.append('lib')

from DucoCobot import DucoCobot
from thrift import Thrift


def main():
    robot = DucoCobot('127.0.0.1', 7003)  # Replace with your controller IP.
    try:
        # open(): 0 means success; -1 means failure.
        if robot.open() != 0:
            raise ConnectionError('Check the controller IP, port, and RPC service')

        print('SDK version:', robot.get_version())
        print('Controller version:', robot.get_robot_version())
        print('Robot state:', robot.get_robot_state())
        print('Actual joints (rad):', robot.get_actual_joints_position())
        print('TCP pose (m, rad):', robot.get_tcp_pose())
    finally:
        # This standalone program is exiting. Keep connections open during business operations.
        robot.close()


if __name__ == '__main__':
    try:
        main()
    except Thrift.TException as exc:
        print('RPC communication error:', exc)
```

```bash
python read_state.py
```

`get_version()` returns the client library version; `get_robot_version()` queries the controller version. Closing a connection does not automatically disable the robot, power it off, or reset digital outputs.

`get_robot_state()` returns `[robot state, program state, safety controller state, operation mode]`. The corresponding enums are defined in `gen_py/robot/ttypes.py`. Confirm that status reads succeed before issuing motion or I/O commands.

## Connection and Heartbeat Lifecycle

**Establish connections once and reuse them throughout the application. Do not close and reopen connections after individual motions or after finishing one arm's sequence.** Close them together when the application exits.

The main example follows this sequence:

1. Create and open four connections: two arm command clients and two heartbeat clients.
2. Start one heartbeat thread per arm. Command and heartbeat threads use separate Thrift clients.
3. Execute the left-arm sequence, then the right-arm sequence, while keeping all connections open.
4. On application exit, signal the heartbeat threads, wait for them to finish, and close the connections.

`run_arm_demo()` accepts an already connected client. It does not open or close connections. `heartbeat_loop()` records communication exceptions for the main flow to check; it does not close or reconnect its client.

According to the SDK comments, `rpc_heartbeat()` allows the controller to stop motion when the remote connection is lost. Actual behavior depends on the controller implementation and configuration. Application error handling should explicitly request a stop rather than relying solely on connection closure.

The `time` parameter of `rpc_heartbeat(time=1000)` is in ms. Match the heartbeat interval, RPC timeout, and communication-loss settings to your controller's requirements. Record heartbeat errors, stop issuing subsequent commands, and request a stop as appropriate for your application.

## API Conventions

### Blocking and Nonblocking Calls

Task methods with a `block` argument generally follow these conventions. Refer to each method's comments for exceptions.

| Argument | Return value | How to use it |
| --- | --- | --- |
| `block=True` | Task state when the call returns | Check the returned state |
| `block=False` | Task ID | Poll `get_noneblock_taskstate(task_id)` |

A successful `open()` returns `0`. A motion that finishes normally usually returns `TaskState.ST_Finished`; these return values are not interchangeable. Blended motions may return `ST_Interrupt`, depending on the command.

Task states are defined in `gen_py/robot/ttypes.py`:

| Enum | Value | Meaning |
| --- | --- | --- |
| `ST_Idle` | 0 | Idle |
| `ST_Running` | 1 | Running |
| `ST_Paused` | 2 | Paused |
| `ST_Stopped` | 3 | Stopped |
| `ST_Finished` | 4 | Finished |
| `ST_Interrupt` | 5 | Interrupted |
| `ST_Error` | 6 | Error |
| `ST_Illegal` | 7 | Illegal task |
| `ST_ParameterMismatch` | 8 | Parameter mismatch |

`demo_nonblocking_move()` in `demo.py` demonstrates task polling. It checks the initial blocking move to zero and rejects a nonblocking task that exits without reaching `ST_Finished`.

### Units and Coordinate Systems

| Data or method | Units or interpretation |
| --- | --- |
| Joint position list | rad; list length must match the device's joint count |
| Cartesian pose | `[x, y, z, rx, ry, rz]`; position in m, orientation in rad |
| `movej` / `movej_pose`: `v`, `a` | Percentages of configured speed and acceleration |
| `movej2` / `movej_pose2`: `v`, `a` | rad/s, rad/s² |
| `movel` / `movec`: `v`, `a` | m/s, m/s² |
| `speed(percent)` | Global speed percentage |
| `speedj`: velocity list | rad/s |
| `speedl`: velocity list | First three entries in m/s; last three in rad/s |
| `speedj` / `speedl`: time | ms; `-1` means continuous operation |

`r` controls trajectory blending; `0` disables blending. Its units differ between percentage-based and physical-unit motion methods, so do not reuse values without checking the method documentation.

Motion methods such as `movel()` and `movej_pose()` accept coordinate system names for `tool` and `wobj`; `''` selects the current configuration. In contrast, `cal_fkine()` and `cal_ikine()` accept numeric lists for these parameters: a tool offset and a workpiece pose, each in `[x, y, z, rx, ry, rz]` format. Pass empty lists `[]` to use the current configuration, not coordinate system names. For an already connected `robot`:

```python
joints = robot.get_actual_joints_position()
pose = robot.cal_fkine(joints, [], [])
solution = robot.cal_ikine(pose, joints, [], [])
```

`set_tool_data()` and `set_wobj()` affect subsequent motion. The example also explicitly uses `tcp_1`, `tcp_2`, and `user_1`; configure these names correctly on the controller.

## Basic Motion Example

Complete the required power-on procedure and check robot state, joint limits, tool load, and the available workspace first. Save the following as `move_once.py` in the repository root and set `ROBOT_IP` and `ARM_PORT` before running it. This standalone example uses the SDK directly, without importing `demo.py`. It opens command and heartbeat connections, enables the arm, and increases the first joint's target by `0.02 rad` from its current position. Other joint targets remain unchanged; the example does not return to the starting position.

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

The heartbeat thread records exceptions; the main flow checks them before and after the motion. These checks do not run immediately while the main thread waits for a blocking motion call. Reuse `robot` for further operations and check heartbeat errors and task results at suitable points. See `demo_nonblocking_move()` for task polling. The example enables the arm but does not power it on.

Use `speed_stop(True)` to end continuous `speedj` or `speedl` motion. Use `disable(True)` and `power_off(True)` according to your application's shutdown procedure.

## Running the Examples

### Main Example: `demo.py`

Set the configuration at the top of the file:

```python
ROBOT_IP = '127.0.0.1'  # Replace with your controller IP.
LEFT_ARM_PORT = 7003
RIGHT_ARM_PORT = 7004
JOINT_TRACK_PATH = None
```

Then run:

```bash
python demo.py
```

The calls in `main()` determine the arm order: currently left arm first, then right arm, without simultaneous motion. Each sequence includes preparation, nonblocking joint motion, coordinate configuration, forward/inverse kinematics, and several motion methods.

The program opens connections to both arms before starting motion. Removing one arm's motion call alone does not remove its connection requirement. For a single-arm service, also adjust client creation, the connection list, heartbeat startup, and business calls in `main()` to retain only the required side.

The example sets global speed to `100%` and calls `collision_detect(0)`, which disables collision detection. Review these settings and all target positions before execution. It also modifies the current tool and workpiece coordinate configurations.

Diagnostic logging and real-time trajectory playback are currently commented out in `run_arm_demo()` and can be enabled as needed. Motion steps depend on previously calculated poses, reference joints, and coordinate settings; retain the prerequisites when selecting individual steps.

### Optional Real-Time Joint Playback

Set `JOINT_TRACK_PATH` and uncomment the playback block in `run_arm_demo()`:

```python
JOINT_TRACK_PATH = r'D:\tracks\jointpos.txt'
```

Each nonempty line must contain seven whitespace-separated joint angles in **degrees**. The loader converts them to radians:

```text
0 0 0 0 0 0 0
0.1 0 0 0 0 0 0
0.2 0 0 0 0 0 0
```

The file is validated before connections are opened. Empty files, incorrect joint counts, and nonfinite values are rejected. The values above illustrate the file format; actual trajectories must match the robot's starting position, joint limits, and control cycle.

The SDK comments mark the real-time interfaces as unsupported, and the original example and SDK comments differ on mode numbering. Playback retains `start_realtime_mode(0, 100, -1)` and a four-second wait from the original example. Confirm controller support, mode numbering, communication-loss settings, and playback duration before enabling it.

### Multiple Components: `test.py`

```bash
python test.py
```

This example connects both arms, the waist, and the head. Its active path continuously sends `servoj` commands to both arms with increasing targets and no upper bound. The later `movej2`, waist servo, and connection-close statements are unreachable while the infinite loop runs. Adapt the loop and add application-exit cleanup before integrating this fragment.

### Tool Digital I/O: `toolio.py`

```bash
python toolio.py
```

This example sets **DO2 high on both arms**, then reads left DI1, left DO1, right DI1, and right DO1. The read channel differs from the written channel; use `get_tool_digital_out(2)` to read back DO2.

The fragment currently closes only the left command connection explicitly. An application should clean up both connections when exiting. Disconnecting does not automatically reset DO2.

## Repository Layout

| File or directory | Description |
| --- | --- |
| [DucoCobot.py](DucoCobot.py) | Robot arm client |
| [DucoCobotWaist.py](DucoCobotWaist.py) | Waist client |
| [DucoCobotHead.py](DucoCobotHead.py) | Head client |
| `gen_py/robot/` | Generated Thrift clients, data structures, and enums |
| `lib/thrift/` | Bundled Thrift Python runtime |
| [demo.py](demo.py) | Main motion example with explicit connections and separate heartbeat threads |
| [test.py](test.py) | Connection and joint-motion examples for both arms, the waist, and the head |
| [toolio.py](toolio.py) | Tool digital I/O example for both arms |
| [requirements.txt](requirements.txt) | Python dependencies |

Application code normally uses the three client classes. Generated code in `gen_py/robot/` must match the controller's RPC protocol.

## Troubleshooting

| Symptom | What to check |
| --- | --- |
| `ModuleNotFoundError: No module named 'six'` | Install `requirements.txt` using the same Python environment that runs the script |
| Cannot import `thrift` or `gen_py` | Check that the source directories are present and run from the repository root; when integrating into another application, add the SDK root and its `lib/` and `gen_py/` directories to the module search path |
| `open()` returns `-1` | Check the controller IP, TCP port, network, and RPC service; do not issue RPC calls after a failed connection |
| Task is stopped or returns an error | Inspect the task state, `get_last_error()`, heartbeat errors, enable state, joint count, and argument units |
| A single-arm connection works but `demo.py` fails | The main example connects both arms; adjust both connection setup and business calls as described above |
| The next arm stops after the previous arm finishes | Keep both command and heartbeat connections open until the entire application finishes |
| Unexpected pose or motion direction | Check the active tool/workpiece frames and explicitly named frames |
| Motion continues after a nonblocking call returns | The return value is a task ID; poll its state or issue the appropriate stop command |
| An older example raises `AttributeError` for `exc.message` | Use `str(exc)` or `print(exc)` in Python 3 |

See [DucoCobot.py](DucoCobot.py), [DucoCobotWaist.py](DucoCobotWaist.py), and [DucoCobotHead.py](DucoCobotHead.py) for method signatures and parameter comments.
