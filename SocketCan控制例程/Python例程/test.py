#!/usr/bin/env python3
from __future__ import annotations

import signal
import time

from damiao_socketcan import Can_control_Mode, Control_Mode, DM_Motor_Type, DmActData, Motor_Control


running = True


def signalHandler(signum, _frame):
    global running
    running = False
    print(f"\nInterrupt signal ({signum}) received.")


def main():
    signal.signal(signal.SIGINT, signalHandler)

    canid1 = 0x01
    mstid1 = 0x11
    canid2 = 0x02
    mstid2 = 0x12

    init_data = [
        DmActData(DM_Motor_Type.DM4310,
                 Control_Mode.VEL_MODE, 
                 can_id=canid1, 
                 mst_id=mstid1),
        DmActData(DM_Motor_Type.DM4310,
                 Control_Mode.VEL_MODE, 
                 can_id=canid2, 
                 mst_id=mstid2),
    ]

    robot_ptr1_ = Motor_Control("can0", init_data, Can_control_Mode.canfd) # 根据实际情况修改can接口名

    try:
        while running:
            current_time = time.monotonic()
            desired_duration = 0.01

            for motor_id in range(1, 3): 
                motor = robot_ptr1_.getMotor(motor_id)
                if motor is not None:
                    robot_ptr1_.control_vel(motor, 0.6)
                    robot_ptr1_.poll(0.001)

            print("robot_ptr1_:")
            for motor_id in range(1, 3):
                motor = robot_ptr1_.getMotor(motor_id)
                if motor is None:
                    continue
                print(
                    f"canid is: {motor_id} pos: {motor.Get_Position()} "
                    f"vel: {motor.Get_Velocity()} effort: {motor.Get_tau()} "
                    f"time(s): {motor.getTimeInterval()}"
                )

            sleep_time = desired_duration - (time.monotonic() - current_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
    finally:
        robot_ptr1_.close()
        print("\nThe program exited safely.\n")


if __name__ == "__main__":
    main()
