# coding: utf-8
"""双臂、腰部与头部的连接和关节运动示例。

当前执行路径是双臂连续 servoj；Ctrl+C 可中断循环。
while True 后的 movej2/腰部 servoj 是备选片段，需调整循环后才能执行。
运行前确认设备已上电、使能，目标点和速度适合现场工作空间。
"""

from array import array
import sys
import time
import threading

sys.path.append('gen_py')
sys.path.append('lib')
from DucoCobot import DucoCobot
from DucoCobotWaist import DucoCobotWaist
from thrift import Thrift
from thrift.transport import TSocket
from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol
from gen_py.robot.ttypes import StateRobot, StateProgram, OperationMode, TaskState, MoveJogTaskParam, RealTimeControlData

ip = '127.0.0.1'  # 控制器 IP；远程连接时改为实际地址。
stopheartthread = False
left_arm_port = 7003  # 左臂：7 个关节。
right_arm_port = 7004  # 右臂：7 个关节。
waist_port = 7020  # 本例腰部目标包含 1 个关节值。
head_port = 7021  # 本例头部目标包含 2 个关节值。

def main():
    """创建四个独立连接，演示双臂周期性发送关节目标。"""

    left_arm = DucoCobot(ip, left_arm_port)
    right_arm = DucoCobot(ip, right_arm_port)

    waist = DucoCobotWaist(ip, waist_port)
    # 沿用原例的 DucoCobotWaist 客户端；项目也提供 DucoCobotHead，
    # 头部客户端应与控制器提供的 RPC 服务协议匹配。
    head = DucoCobotWaist(ip, head_port)
    # open()：0 成功、-1 失败。实际使用应逐个检查，参见 demo.py。
    rlt = left_arm.open()
    rlt = right_arm.open()
    rlt = waist.open()
    rlt = head.open()

    # duco_cobot.disable_servo_optimization()
    # 开启关节速度保护、关节位置保护和运动学求解保护，仅作用于左臂连接。
    left_arm.enable_servo_optimization(True,True,True)


    # 目标列表长度须与设备关节数一致，关节角单位 rad。
    pos = 0
    pos_arm = [pos]*7
    pos_waist = [pos]*1
    pos_head = [pos]*2

    # 每轮目标增加 0.00001 rad，以 0.1 s 间隔发送双臂非阻塞伺服指令。
    # 这是持续递增的演示，未设置目标上限；应按实际关节范围调整。
    while True:
        pos = pos+0.00001
        pos_arm = [pos]*7
        # 参数依次为目标关节角、速度、加速度、是否阻塞。
        # False 表示不等待该次运动结束，左右臂调用并不保证同步。
        left_arm.servoj(pos_arm,2,10,False)
        right_arm.servoj(pos_arm,2,10,False)
        time.sleep(0.1)

    # 以下代码在当前无限循环中不可达；演示前需将上方改为有限循环。
    # movej2：关节角 rad，速度 rad/s，加速度 rad/s²，融合半径 0。
    # block=False 返回任务 ID；True 返回任务结束时的状态。
    rlt = left_arm.movej2(pos_arm, 2, 10, 0, False)
    rlt = right_arm.movej2(pos_arm, 2, 10, 0, False)
    rlt = waist.movej2(pos_waist, 2, 10, 0, False)
    rlt = head.movej2(pos_head, 2, 10, 0, True)
    # print("movej2:", rlt)

    pos_waist = [-x for x in pos_waist]
    # 后四项为 kp、kd、速度平滑、加速度平滑参数。
    rlt = waist.servoj(pos_waist,1.5703,3,True,100,40,10,0.5)
    print("servoj:", rlt)

    # 当前循环不会执行到此处；异常退出清理的完整写法见 demo.py。
    left_arm.close()
    right_arm.close()
    waist.close()
    head.close()

if __name__ == '__main__':
    try:
        main()
    except Thrift.TException as tx:
        print('%s' % tx.message)
