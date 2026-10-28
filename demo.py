# coding: utf-8
"""机械臂综合示例：连接、状态、坐标系、运动及可选实时轨迹回放。

当前按右臂、左臂顺序执行。
指令和心跳连接在 main 中一次建立，整个业务流程复用，程序退出时统一关闭。
本例会使能和运动机器人，并修改当前工具、工件坐标系和全局速度。
运行前确认已上电、运动空间可用，且 tcp_1/tcp_2、user_1 已配置。
关节角单位 rad，笛卡尔位姿 [x, y, z, rx, ry, rz] 的单位为 m、rad。
"""

import math
from pathlib import Path
import sys
import threading
import time

# 按脚本位置加载随项目提供的依赖，支持从其他工作目录启动。
BASE_DIR = Path(__file__).resolve().parent
sys.path.append(str(BASE_DIR / 'gen_py'))
sys.path.append(str(BASE_DIR / 'lib'))
from DucoCobot import DucoCobot
from thrift import Thrift
from gen_py.robot.ttypes import TaskState, RealTimeControlData

ROBOT_IP = '192.168.120.225'  # 当前控制器 IP。
LEFT_ARM_PORT = 7003
RIGHT_ARM_PORT = 7004
LEFT_TARGET_JOINTS = [
    1.382990820616408, 0.7308725647918002, 0.24463438255606249,
    2.0478357248569132, 1.8390740009322621, 0.44459092235325454,
    -0.1613829339740696,
]
# 填写本机轨迹文件路径后开启左臂回放；None 表示跳过。
JOINT_TRACK_PATH = None


def heartbeat_loop(heartbeat, stop_event, errors):
    """独立连接发送心跳，避免与主线程共用同一 Thrift 连接。"""
    try:
        while not stop_event.is_set():
            heartbeat.rpc_heartbeat()  # 默认心跳延时参数 1000 ms。
            stop_event.wait(1.0)  # 可被退出事件唤醒，无需等待整秒。
    except Exception as exc:
        errors.append(exc)
        # 连接由 main 持有，线程不自行关闭或重连；主流程检查异常后处理。


def prepare_robot(robot):
    """查询版本与关节状态，停止旧任务后使能；已上电是前提。"""
    print("rpc version:", robot.get_version())
    print("robot version:", robot.get_robot_version())
    print("joint: ", robot.get_actual_joints_position())

    rlt = robot.stop(True)
    print("stop:", rlt)
    # rlt = robot.power_on(True)
    # print("power_on:", rlt)
    rlt = robot.enable(True)
    print("enable:", rlt)
    # 全局速度比例，单位 %。
    robot.speed(100)
    # 0 关闭碰撞检测，现场应确认是否改用等级 1～5。
    robot.collision_detect(0)
    time.sleep(1)


def demo_nonblocking_move(robot, is_left_arm, heartbeat_errors=None):
    """阻塞回零，再通过任务 ID 轮询非阻塞运动。"""
    # movej2 的速度/加速度单位为 rad/s、rad/s²，r=0 无融合。
    # block=True 返回结束状态；False 返回任务 ID，供后续查询。
    zero_state = robot.movej2([0,0,0,0,0,0,0], 1, 10, 0, True)
    if zero_state != TaskState.ST_Finished:
        raise RuntimeError("回零运动未正常完成，任务状态：{}".format(zero_state))

    target = LEFT_TARGET_JOINTS if is_left_arm else [-q for q in LEFT_TARGET_JOINTS]
    task_id = robot.movej2(target, 2, 10, 0, False)

    time.sleep(0.1)
    state = robot.get_noneblock_taskstate(task_id)
    print("state: ", state)
    while state == TaskState.ST_Running:
        check_heartbeat(heartbeat_errors)
        robot_status = robot.get_robot_status()
        # 状态结构包含 actual_joint_* / target_joint_*，可按需读取位置、速度、加速度。
        # print("actual joints:", robot_status.actual_joint_pos)
        # print("target joints:", robot_status.target_joint_pos)
        # 也可分别调用 get_actual_joints_position()、get_tcp_speed() 等状态接口。
        time.sleep(1)
        state = robot.get_noneblock_taskstate(task_id)
    print("state: ", state)
    if state != TaskState.ST_Finished:
        raise RuntimeError("非阻塞运动未正常完成，任务状态：{}".format(state))


def demo_coordinates(robot):
    """演示工具/工件坐标系配置与正逆运动学，返回位姿和关节参考值。"""
    time.sleep(0.1)
    print("joint: ", robot.get_actual_joints_position())
    print("cart flange: ", robot.get_flange_pose())
    print("cart tcp_base: ", robot.get_tcp_pose())
    # 修改当前工具 TCP 偏移；load 包含质量和质心（kg、m）。
    tcp = robot.get_tcp_offset()
    load = robot.get_tool_load()
    tcp[0] = tcp[0]+0.1
    tcp[1] = tcp[1]-0.1
    robot.set_tool_data("tcptest",tcp,load, [0.0] * 6)

    # 工件坐标系相对基坐标系的位姿；此处将 Z 偏移 -0.1 m。
    wobj = robot.get_wobj()
    wobj[2] = wobj[2]-0.1
    robot.set_wobj("usertest",wobj)

    print("cart tcp_base: ", robot.get_tcp_pose())
    print("cart tcp_world: ", robot.get_tcp_pose_coord("","world"))
    print("cart tcp_current_wobj: ", robot.get_tcp_pose_coord("",""))

    # 空字符串表示当前工具/工件坐标系；逆解用 joint 选取附近解。
    joint = robot.get_actual_joints_position()
    pose = robot.cal_fkine(joint, "", "")
    print("cal_fkine: ", pose)
    joint = robot.cal_ikine(pose, joint, "", "")
    print("cal_ikine: ", joint)
    return pose, joint


def demo_motion(robot, is_left_arm, pose, joint):
    """依次演示关节、直线、圆弧、相对位移、速度及伺服运动。"""
    # movej/movej_pose 的速度和加速度为系统设定值的百分比。
    pose = pose.copy()
    print("movej")
    rlt = robot.movej([0,0,0,0,0,0,0], 50, 100, 0, True)

    print("movej_pose")
    rlt = robot.movej_pose(pose, 50, 100, 0, joint, "", "", True)

    print("movej_pose2")
    pose[0]  = pose[0]+0.05
    rlt = robot.movej_pose2(pose, 1, 1, 0, joint, "", "", True)

    # 直线/圆弧速度单位 m/s；显式工具与工件坐标系须已配置。
    print("movel")
    pose[2]  = pose[2]
    if is_left_arm:
        rlt = robot.movel(pose, 1, 1, 0, [], "tcp_1", "user_1", True, True)
    else:
        rlt = robot.movel(pose, 1, 1, 0, [], "tcp_2", "user_1", True, True)

    # pose1 为圆弧途经点，pose2 为终点；copy 避免修改原位姿。
    print("movec")
    pose1 = pose.copy()
    pose1[2]  = pose1[2]-0.05
    pose2 = pose1.copy()
    pose2[0]  = pose2[0]-0.05
    pose2[5]  = pose2[5]-0.05
    robot.movec(pose1, pose2, 1, 1, 0, 0, [], "", "", True)

    print("move_circle")
    robot.move_circle(pose, pose1, 1, 1, 0, 0, [], "", "", True, True)

    # tcp_move 按工具坐标系偏移，wobj_move 按工件坐标系偏移。
    # 对应的 _2p 接口通过两个位姿定义相对运动。
    print("tcp_move")
    robot.tcp_move([0,-0.05,0,0.05,0,0], 1, 1, 0, "", True)

    print("tcp_move_2p")
    robot.tcp_move_2p(pose2,pose1, 1, 1, 0, "", "", True)

    print("wobj_move")
    robot.wobj_move([0,-0.05,0,0.05,0,0], 1, 1, 0, "", True)

    print("wobj_move_2p")
    robot.wobj_move_2p(pose1,pose2, 1, 1, 0, "", "", True)

    # 关节速度单位 rad/s，持续时间为 1000 ms。
    print("speedj")
    robot.speedj([-0.5,0,0,0.5,0,0,0.2],10,1000,True)

    print("speedl")
    # time=-1 表示持续运动，必须通过 speed_stop 结束。
    try:
        robot.speedl([-0.05,0,0,0.05,0,0],10,-1,False)
        time.sleep(1)
    finally:
        robot.speed_stop(True)

    print("servoj")
    robot.servoj([0,0,0,0,0,0,0], 1, 10, True)
    print("servoj_pose")
    robot.servoj_pose(pose, 1, 10, joint, "", "", True)
    # print("servol")
    # robot.servol(pose2, 1, 10, joint, "", "", True)
    # time.sleep(5)


def demo_diagnostics(robot):
    """查询错误、写日志并修改全局速度比例。"""
    rlt = robot.get_last_error()
    print("last error", rlt)

    robot.log_info("TestInfo")

    # robot.movej2([0,0,0,0,0,0,0], 2, 10, 0, False)
    # time.sleep(1)
    # robot.log_error("TestErr") # 弹窗并暂停任务，按需验证

    # robot.simulation(True,True) # 切换仿真模式，按需验证
    # robot.simulation(False,True)

    speed = robot.get_global_speed()
    print("speed1", speed)
    robot.speed(max(1, speed-10))  # 避免产生非正的速度比例。
    time.sleep(0.1)
    print("speed2", robot.get_global_speed())


def load_joint_track(path):
    """每个非空行应含 7 个以空格分隔的角度值（度），转换为 rad。"""
    track = []
    with open(path, encoding='utf-8') as file:
        for line_number, line in enumerate(file, 1):
            if not line.strip():
                continue
            values = [float(value) for value in line.split()]
            if len(values) != 7 or not all(math.isfinite(q) for q in values):
                raise ValueError('轨迹第 {} 行须为 7 个有限关节角'.format(line_number))
            track.append(RealTimeControlData(
                joint_pos_cmd=[math.radians(q) for q in values],
                joint_vel_cmd=[], joint_torq_cmd=[], cart_pos_tool_wobj_cmd=[],
                cart_vel_tool_wobj_cmd=[], cart_ft_cmd=[], status=True,
            ))
    if not track:
        raise ValueError('轨迹文件不包含有效点位')
    return track


def demo_realtime(robot, track):
    """回放左臂关节轨迹；控制器须支持对应的实时接口。"""
    print('realtime joint:', robot.movej2([0] * 7, 2, 10, 0, True))
    robot.clear_realtime_data_queue()
    robot.realtime_data_enqueue(track, True)
    # 原例 mode=0，与 SDK 注释编号有差异，使用前须核对控制器版本。
    # 保留原例参数：带宽 100 Hz、通信保护参数 -1、等待 4 s。
    try:
        robot.start_realtime_mode(0, 100, -1)
        time.sleep(4)
    finally:
        robot.end_realtime_mode()
    time.sleep(0.1)
    robot.movej2([1] * 7, 2, 10, 0, True)
    time.sleep(1)
    # 空间轨迹可填充 cart_pos_tool_wobj_cmd：前三项 mm 转 m，
    # 后三项度转 rad，其余字段置空；模式编号按控制器版本确认。


def check_heartbeat(errors):
    """阶段之间检查心跳异常，避免在通信失败后继续下发下一组指令。"""
    if errors:
        raise RuntimeError('心跳通信失败') from errors[0]


def run_arm_demo(robot, is_left_arm, heartbeat_errors, track=None):
    """复用已连接的客户端执行单臂流程，不在运动步骤中建立或关闭连接。"""
    errors = heartbeat_errors
    try:
        prepare_robot(robot)
        check_heartbeat(errors)
        demo_nonblocking_move(robot, is_left_arm, errors)
        check_heartbeat(errors)
        pose, joint = demo_coordinates(robot)
        check_heartbeat(errors)
        demo_motion(robot, is_left_arm, pose, joint)
        # demo_diagnostics(robot)
        # if track is not None:
        #     check_heartbeat(errors)
        #     demo_realtime(robot, track)
        check_heartbeat(errors)
    except BaseException:
        try:
            robot.stop(True)
        except Exception as stop_error:
            print('停止运动失败:', stop_error)
        raise


def main():
    """一次建立连接，持续复用；全部业务结束或程序退出时才统一清理。"""
    # 连接与运动前校验可选文件，避免执行到末尾才发现路径无效。
    track = load_joint_track(JOINT_TRACK_PATH) if JOINT_TRACK_PATH is not None else None
    left_arm = DucoCobot(ROBOT_IP, LEFT_ARM_PORT)
    right_arm = DucoCobot(ROBOT_IP, RIGHT_ARM_PORT)
    # 心跳使用独立连接，避免与指令线程共用 Thrift 客户端。
    left_heartbeat = DucoCobot(ROBOT_IP, LEFT_ARM_PORT)
    right_heartbeat = DucoCobot(ROBOT_IP, RIGHT_ARM_PORT)
    left_heartbeat.socket.setTimeout(3000)
    right_heartbeat.socket.setTimeout(3000)

    connections = [
        ('左臂指令', left_arm), ('右臂指令', right_arm),
        ('左臂心跳', left_heartbeat), ('右臂心跳', right_heartbeat),
    ]
    stop_event = threading.Event()
    heartbeat_errors = []
    heartbeat_threads = []
    try:
        # 所有连接先建立成功，再开始下发运动。open：0 成功、-1 失败。
        for name, client in connections:
            if client.open() != 0:
                raise ConnectionError('{}连接失败：{}'.format(name, ROBOT_IP))

        for port, heartbeat in (
            (LEFT_ARM_PORT, left_heartbeat), (RIGHT_ARM_PORT, right_heartbeat),
        ):
            thread = threading.Thread(
                target=heartbeat_loop, args=(heartbeat, stop_event, heartbeat_errors),
                name='heartbeat-{}'.format(port),
            )
            thread.start()
            heartbeat_threads.append(thread)

        run_arm_demo(
            left_arm, is_left_arm=True, heartbeat_errors=heartbeat_errors, track=track,
        )

        run_arm_demo(
            right_arm, is_left_arm=False, heartbeat_errors=heartbeat_errors,
        )
        
    finally:
        # 仅在整个程序退出时清理；单侧运动完成或心跳线程异常均不自行关闭连接。
        stop_event.set()
        for thread in heartbeat_threads:
            thread.join()
        for name, client in connections:
            client.close()
    # 下使能、下电按现场流程显式调用 disable(True)、power_off(True)。


if __name__ == '__main__':
    try:
        main()
    except Thrift.TException as exc:
        print('RPC 通信异常:', exc)
