# coding: utf-8
"""双臂末端数字 I/O 示例。

文件名虽为 right.py，实际同时连接左臂 7003 和右臂 7004。
执行时会将两侧末端 DO2 置为高电平，确认外接设备允许该操作后运行。
"""

from array import array
import sys
import time
import threading

sys.path.append('gen_py')
sys.path.append('lib')
from DucoCobot import DucoCobot
from thrift import Thrift
from thrift.transport import TSocket
from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol
from gen_py.robot.ttypes import RobotStatus, StateRobot, StateProgram, OperationMode, TaskState, MoveJogTaskParam, RealTimeControlData

ip = '127.0.0.1'  # 控制器地址；远程连接时替换为实际 IP。
stopheartthread = False
port_list = [7003]  # 当前循环连接左臂，循环内额外创建右臂连接。
port = 7003

def main():
    """设置双臂 DO2，随后读取 DI1 和 DO1。"""
    for i  in port_list:
        duco_cobot = DucoCobot(ip, i)
        duco_cobot_r = DucoCobot(ip, 7004)
        # open() 返回 0 表示成功，-1 表示失败；完整检查/清理写法见 demo.py。
        rlt = duco_cobot.open()
        rlt = duco_cobot_r.open()
        # 参数：末端输出通道（1～2）、电平（1 为高）、是否阻塞。
        duco_cobot.set_tool_digital_out(2,1,True)
        duco_cobot_r.set_tool_digital_out(2,1,True)
        # 读取的是通道 1，并非上面设置的 DO2；返回 True/False 为高/低。
        # 如需检查刚写入的输出电平，应读取 get_tool_digital_out(2)。
        # 下列四行依次为左臂 DI1、左臂 DO1、右臂 DI1、右臂 DO1。
        print(duco_cobot.get_tool_digital_in(1))
        print(duco_cobot.get_tool_digital_out(1))
        print(duco_cobot_r.get_tool_digital_in(1))
        print(duco_cobot_r.get_tool_digital_out(1))

        # 原例仅关闭左臂；成对连接的清理方式可参考 demo.py 的 finally。
        # close 只断开通信，不会将此前置高的 DO2 自动复位。
        duco_cobot.close()


if __name__ == '__main__':
    try:
        main()
    except Thrift.TException as tx:
        print('%s' % tx.message)
