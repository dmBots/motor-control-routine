from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import select
import socket
import struct
import sys
import time
from typing import Dict, Iterable, List, Optional, Union


CAN_RAW = getattr(socket, "CAN_RAW", 1)
CAN_RAW_FD_FRAMES = getattr(socket, "CAN_RAW_FD_FRAMES", 5)
SOL_CAN_RAW = getattr(socket, "SOL_CAN_RAW", 101)
CAN_SFF_MASK = 0x7FF
CAN_EFF_MASK = 0x1FFFFFFF
CAN_FRAME_FMT = "=IB3x8s"
CANFD_FRAME_FMT = "=IBBBB64s"
CANFD_FRAME_SIZE = struct.calcsize(CANFD_FRAME_FMT)


class DM_Motor_Type(IntEnum):
    DM3507 = 0
    DM4310 = 1
    DM4310_48V = 2
    DM4340 = 3
    DM4340_48V = 4
    DM6006 = 5
    DM6248P = 6
    DM8006 = 7
    DM8009 = 8
    DM10010L = 9
    DM10010 = 10
    DMH3510 = 11
    DMH6215 = 12
    DMS3519 = 13
    DMG6220 = 14
    Num_Of_Motor = 15


class Control_Mode(IntEnum):
    MIT_MODE = 0x000
    POS_VEL_MODE = 0x100
    VEL_MODE = 0x200
    POS_FORCE_MODE = 0x300


class Control_Mode_Code(IntEnum):
    MIT = 1
    POS_VEL = 2
    VEL = 3
    POS_FORCE = 4


class Can_control_Mode(IntEnum):
    can = 0
    canfd = 1


class DM_REG(IntEnum):
    UV_Value = 0
    KT_Value = 1
    OT_Value = 2
    OC_Value = 3
    ACC = 4
    DEC = 5
    MAX_SPD = 6
    MST_ID = 7
    ESC_ID = 8
    TIMEOUT = 9
    CTRL_MODE = 10
    Damp = 11
    Inertia = 12
    hw_ver = 13
    sw_ver = 14
    SN = 15
    NPP = 16
    Rs = 17
    LS = 18
    Flux = 19
    Gr = 20
    PMAX = 21
    VMAX = 22
    TMAX = 23
    I_BW = 24
    KP_ASR = 25
    KI_ASR = 26
    KP_APR = 27
    KI_APR = 28
    OV_Value = 29
    GREF = 30
    Deta = 31
    V_BW = 32
    IQ_c1 = 33
    VL_c1 = 34
    can_br = 35
    sub_ver = 36
    u_off = 50
    v_off = 51
    k1 = 52
    k2 = 53
    m_off = 54
    dir = 55
    p_m = 80
    xout = 81


control_mode_to_code = {
    Control_Mode.MIT_MODE: Control_Mode_Code.MIT,
    Control_Mode.POS_VEL_MODE: Control_Mode_Code.POS_VEL,
    Control_Mode.VEL_MODE: Control_Mode_Code.VEL,
    Control_Mode.POS_FORCE_MODE: Control_Mode_Code.POS_FORCE,
}


@dataclass
class Limit_param:
    Q_MAX: float
    DQ_MAX: float
    TAU_MAX: float


limit_param = [
    Limit_param(12.566, 50.0, 5.0),
    Limit_param(12.5, 30.0, 10.0),
    Limit_param(12.5, 50.0, 10.0),
    Limit_param(12.5, 10.0, 28.0),
    Limit_param(12.5, 20.0, 28.0),
    Limit_param(12.5, 45.0, 12.0),
    Limit_param(12.566, 20.0, 120.0),
    Limit_param(12.5, 45.0, 20.0),
    Limit_param(12.5, 45.0, 54.0),
    Limit_param(12.5, 25.0, 200.0),
    Limit_param(12.5, 20.0, 200.0),
    Limit_param(12.5, 280.0, 1.0),
    Limit_param(12.5, 45.0, 10.0),
    Limit_param(12.5, 2000.0, 2.0),
    Limit_param(12.5, 45.0, 10.0),
]


@dataclass
class DmActData:
    motorType: DM_Motor_Type
    mode: Control_Mode
    can_id: int
    mst_id: int
    pos: float = 0.0
    vel: float = 0.0
    effort: float = 0.0
    cmd_pos: float = 0.0
    cmd_vel: float = 0.0
    cmd_effort: float = 0.0
    kp: float = 0.0
    kd: float = 0.0


@dataclass
class CanFrame:
    can_id: int
    data: bytes
    is_fd: bool


class Motor:
    def __init__(self, motor_type: DM_Motor_Type, ctrl_mode: Control_Mode, can_id: int, master_id: int) -> None:
        self.Can_id = can_id
        self.Master_id = master_id
        self.state_q = 0.0
        self.state_dq = 0.0
        self.state_tau = 0.0
        self.state_err = 0
        self.limit_param = limit_param[int(motor_type)]
        self.Motor_Type = motor_type
        self.mode = ctrl_mode
        self.param_map: Dict[int, Union[float, int]] = {}
        self.last_time_ = time.monotonic()
        self.delta_time_ = 0.0

    def updateTimeInterval(self) -> None:
        now = time.monotonic()
        self.delta_time_ = now - self.last_time_
        self.last_time_ = now

    def getTimeInterval(self) -> float:
        return self.delta_time_

    def receive_data(self, q: float, dq: float, tau: float, err: int = 0) -> None:
        self.state_q = q
        self.state_dq = dq
        self.state_tau = tau
        self.state_err = err

    def GetMotorType(self) -> DM_Motor_Type:
        return self.Motor_Type

    def GetMotorMode(self) -> Control_Mode:
        return self.mode

    def get_limit_param(self) -> Limit_param:
        return self.limit_param

    def GetMasterId(self) -> int:
        return self.Master_id

    def GetCanId(self) -> int:
        return self.Can_id

    def Get_Position(self) -> float:
        return self.state_q

    def Get_Velocity(self) -> float:
        return self.state_dq

    def Get_tau(self) -> float:
        return self.state_tau

    def Get_err(self) -> int:
        return self.state_err

    def set_mode(self, value: Control_Mode) -> None:
        self.mode = value

    def set_param(self, key: int, value: Union[float, int]) -> None:
        self.param_map[int(key)] = value

    def get_param_as_float(self, key: int) -> float:
        value = self.param_map.get(int(key))
        return float(value) if isinstance(value, float) else 0.0

    def get_param_as_uint32(self, key: int) -> int:
        value = self.param_map.get(int(key))
        return int(value) if isinstance(value, int) else 0

    def is_have_param(self, key: int) -> bool:
        return int(key) in self.param_map


class SocketCAN:
    def __init__(self, interface: str, can_mode: Can_control_Mode) -> None:
        self.interface = interface
        self.can_mode = can_mode
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, CAN_RAW)
        if can_mode == Can_control_Mode.canfd:
            self.socket.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
        self.socket.bind((interface,))
        self.socket.setblocking(False)

    def close(self) -> None:
        self.socket.close()

    def write(self, can_id: int, data: Union[bytes, bytearray, List[int]]) -> None:
        payload = bytes(data)
        if self.can_mode == Can_control_Mode.canfd:
            if len(payload) > 64:
                raise ValueError("CAN FD payload cannot exceed 64 bytes")
            frame = struct.pack(CANFD_FRAME_FMT, can_id, len(payload), 0, 0, 0, payload.ljust(64, b"\x00"))
        else:
            if len(payload) > 8:
                raise ValueError("classic CAN payload cannot exceed 8 bytes")
            frame = struct.pack(CAN_FRAME_FMT, can_id, len(payload), payload.ljust(8, b"\x00"))
        self.socket.send(frame)

    def read(self, timeout: float = 0.0) -> Optional[CanFrame]:
        ready, _, _ = select.select([self.socket], [], [], timeout)
        if not ready:
            return None
        raw = self.socket.recv(CANFD_FRAME_SIZE)
        if len(raw) >= CANFD_FRAME_SIZE:
            can_id, length, _flags, _r0, _r1, data = struct.unpack(CANFD_FRAME_FMT, raw[:CANFD_FRAME_SIZE])
            return CanFrame(can_id=can_id & CAN_EFF_MASK, data=data[:length], is_fd=True)
        can_id, length, data = struct.unpack(CAN_FRAME_FMT, raw)
        return CanFrame(can_id=can_id & CAN_EFF_MASK, data=data[:length], is_fd=False)


class Motor_Control:
    def __init__(self, bus_name: str, data_ptr: Iterable[DmActData], can_mode: Can_control_Mode = Can_control_Mode.canfd) -> None:
        self.current_can_mode_ = can_mode
        self.motors: Dict[int, Motor] = {}
        self.read_write_save = False
        self.socket_can_ = SocketCAN(bus_name, can_mode)

        motor_mode = Control_Mode.MIT_MODE
        for data in data_ptr:
            motor_mode = data.mode
            self.addMotor(Motor(data.motorType, data.mode, data.can_id, data.mst_id))

        self.enable_all(motor_mode)
        print("Motor_Control init success!")

    def __enter__(self) -> "Motor_Control":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def close(self) -> None:
        self.disable_all()
        self.socket_can_.close()

    def canframeCallback(self, frame: CanFrame) -> None:
        can_id = frame.data[0] & 0x0F if frame.data else 0
        if self.read_write_save and can_id in self.motors and len(frame.data) >= 8:
            if frame.data[2] in (0x33, 0x55, 0xAA):
                if frame.data[2] in (0x33, 0x55):
                    self.receive_param(frame.data)
                self.read_write_save = False
            return

        motor = self.motors.get(frame.can_id & CAN_SFF_MASK)
        if motor is None or len(frame.data) < 6:
            return

        q_uint = (frame.data[1] << 8) | frame.data[2]
        dq_uint = (frame.data[3] << 4) | (frame.data[4] >> 4)
        tau_uint = ((frame.data[4] & 0x0F) << 8) | frame.data[5]
        limit = motor.get_limit_param()
        motor.receive_data(
            uint_to_float(q_uint, -limit.Q_MAX, limit.Q_MAX, 16),
            uint_to_float(dq_uint, -limit.DQ_MAX, limit.DQ_MAX, 12),
            uint_to_float(tau_uint, -limit.TAU_MAX, limit.TAU_MAX, 12),
            frame.data[0] >> 4,
        )
        motor.updateTimeInterval()

    def poll(self, timeout: float = 0.0) -> Optional[CanFrame]:
        frame = self.socket_can_.read(timeout)
        if frame is not None:
            self.canframeCallback(frame)
        return frame

    def drain(self, max_frames: int = 32) -> None:
        for _ in range(max_frames):
            if self.poll(0.0) is None:
                break

    def refresh_motor_status(self, motor: Motor) -> None:
        can_id = motor.GetCanId()
        self.socket_can_.write(0x7FF, [can_id & 0xFF, (can_id >> 8) & 0xFF, 0xCC, 0x00])

    def disable_all(self) -> None:
        sent = set()
        for motor in self.motors.values():
            if motor.GetCanId() in sent:
                continue
            sent.add(motor.GetCanId())
            for _ in range(5):
                self.control_cmd(motor.GetCanId() + motor.GetMotorMode(), 0xFD)
                time.sleep(0.002)

    def set_zero_position(self, DM_Motor: Motor) -> None:
        self.control_cmd(DM_Motor.GetCanId() + DM_Motor.GetMotorMode(), 0xFE)

    def enable_all(self, mode_: Control_Mode) -> None:
        sent = set()
        for motor in self.motors.values():
            if motor.GetCanId() in sent:
                continue
            sent.add(motor.GetCanId())
            self.switchControlMode(motor, control_mode_to_code[motor.GetMotorMode()])

        for motor_id in list(sent):
            motor = self.motors[motor_id]
            self.read_motor_param(motor, DM_REG.CTRL_MODE)
            self.poll(0.002)
            print(f"id: {motor.GetCanId()} mode is: {motor.get_param_as_uint32(DM_REG.CTRL_MODE)}", file=sys.stderr)

        for motor_id in list(sent):
            motor = self.motors[motor_id]
            for _ in range(5):
                self.control_cmd(motor.GetCanId() + mode_, 0xFC)
                time.sleep(0.002)

    def control_mit(self, DM_Motor: Motor, kp: float, kd: float, q: float, dq: float, tau: float) -> None:
        motor_id = DM_Motor.GetCanId()
        if motor_id not in self.motors:
            print(f"[Error] In control_mit,no motor with id {motor_id} is registered.", file=sys.stderr)
            sys.exit(-1)

        limit = DM_Motor.get_limit_param()
        kp_uint = float_to_uint(kp, 0.0, 500.0, 12)
        kd_uint = float_to_uint(kd, 0.0, 5.0, 12)
        q_uint = float_to_uint(q, -limit.Q_MAX, limit.Q_MAX, 16)
        dq_uint = float_to_uint(dq, -limit.DQ_MAX, limit.DQ_MAX, 12)
        tau_uint = float_to_uint(tau, -limit.TAU_MAX, limit.TAU_MAX, 12)
        data = [
            (q_uint >> 8) & 0xFF,
            q_uint & 0xFF,
            (dq_uint >> 4) & 0xFF,
            ((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F),
            kp_uint & 0xFF,
            (kd_uint >> 4) & 0xFF,
            ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F),
            tau_uint & 0xFF,
        ]
        self.socket_can_.write(motor_id + Control_Mode.MIT_MODE, data)

    def control_pos_vel(self, DM_Motor: Motor, pos: float, vel: float) -> None:
        motor_id = DM_Motor.GetCanId()
        if motor_id not in self.motors:
            print(f"[Error] In control_pos_vel,no motor with id {motor_id} is registered.", file=sys.stderr)
            sys.exit(-1)
        self.socket_can_.write(motor_id + Control_Mode.POS_VEL_MODE, struct.pack("<ff", pos, vel))

    def control_vel(self, DM_Motor: Motor, vel: float) -> None:
        motor_id = DM_Motor.GetCanId()
        if motor_id not in self.motors:
            print(f"[Error] In control_vel,no motor with id {motor_id} is registered.", file=sys.stderr)
            sys.exit(-1)
        self.socket_can_.write(motor_id + Control_Mode.VEL_MODE, struct.pack("<f", vel))

    def receive_param(self, data: Union[bytes, bytearray]) -> None:
        can_id = data[0] & 0x0F
        rid = data[3]
        motor = self.motors.get(can_id)
        if motor is None:
            return

        if self.is_in_ranges(rid):
            value = (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4]
            motor.set_param(rid, value)
            if rid == DM_REG.CTRL_MODE:
                if value == 1:
                    motor.set_mode(Control_Mode.MIT_MODE)
                elif value == 2:
                    motor.set_mode(Control_Mode.POS_VEL_MODE)
                elif value == 3:
                    motor.set_mode(Control_Mode.VEL_MODE)
                elif value == 4:
                    motor.set_mode(Control_Mode.POS_FORCE_MODE)
        else:
            motor.set_param(rid, struct.unpack("<f", bytes(data[4:8]))[0])

    def addMotor(self, DM_Motor: Motor) -> None:
        self.motors[DM_Motor.GetCanId()] = DM_Motor
        self.motors[DM_Motor.GetMasterId()] = DM_Motor

    def read_motor_param(self, DM_Motor: Motor, RID: int) -> float:
        self.read_write_save = True
        can_id = DM_Motor.GetCanId()
        self.socket_can_.write(0x7FF, [can_id & 0xFF, (can_id >> 8) & 0xFF, 0x33, int(RID), 0, 0, 0, 0])
        time.sleep(0.002)
        return 0.0

    def switchControlMode(self, DM_Motor: Motor, mode: Control_Mode_Code) -> bool:
        self.write_motor_param(DM_Motor, DM_REG.CTRL_MODE, [int(mode), 0, 0, 0])
        if DM_Motor.GetCanId() not in self.motors:
            print(f"[Error] In switchControlMode,no motor with id {DM_Motor.GetCanId()} is registered.", file=sys.stderr)
            sys.exit(-1)
        return True

    def change_motor_param(self, DM_Motor: Motor, RID: int, data: float) -> bool:
        payload = int(data).to_bytes(4, byteorder="little", signed=False) if self.is_in_ranges(RID) else struct.pack("<f", data)
        self.write_motor_param(DM_Motor, RID, payload)
        if DM_Motor.GetCanId() not in self.motors:
            print(f"[Error] In change_motor_param,no motor with id {DM_Motor.GetCanId()} is registered.", file=sys.stderr)
            sys.exit(-1)
        return True

    def save_motor_param(self, DM_Motor: Motor) -> None:
        motor_id = DM_Motor.GetCanId()
        self.control_cmd(motor_id + DM_Motor.GetMotorMode(), 0xFD)
        time.sleep(0.01)
        self.read_write_save = True
        self.socket_can_.write(0x7FF, [motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0xAA, 0x01, 0, 0, 0, 0])
        time.sleep(0.1)

    @staticmethod
    def changeMotorLimit(DM_Motor: Motor, P_MAX: float, Q_MAX: float, T_MAX: float) -> None:
        limit_param[int(DM_Motor.GetMotorType())] = Limit_param(P_MAX, Q_MAX, T_MAX)
        DM_Motor.limit_param = limit_param[int(DM_Motor.GetMotorType())]

    def getMotor(self, motor_id: int) -> Optional[Motor]:
        motor = self.motors.get(motor_id)
        if motor is None:
            print(f"[Error] In getMotor,no motor with id {motor_id} is registered.", file=sys.stderr)
        return motor

    def control_cmd(self, can_id: int, cmd: int) -> None:
        self.socket_can_.write(can_id, [0xFF] * 7 + [cmd])

    def write_motor_param(self, DM_Motor: Motor, RID: int, data: Union[bytes, bytearray, List[int]]) -> None:
        self.read_write_save = True
        can_id = DM_Motor.GetCanId()
        payload = bytes(data)
        if len(payload) != 4:
            raise ValueError("write_motor_param data must be 4 bytes")
        self.socket_can_.write(0x7FF, [can_id & 0xFF, (can_id >> 8) & 0xFF, 0x55, int(RID)] + list(payload))
        time.sleep(0.002)

    @staticmethod
    def is_in_ranges(number: int) -> bool:
        return (7 <= int(number) <= 10) or (13 <= int(number) <= 16) or (35 <= int(number) <= 36)


def float_to_uint(value: float, value_min: float, value_max: float, bits: int) -> int:
    value = max(value_min, min(value_max, value))
    span = value_max - value_min
    return int((value - value_min) * ((1 << bits) - 1) / span)


def uint_to_float(value: int, value_min: float, value_max: float, bits: int) -> float:
    span = value_max - value_min
    return float(value) * span / float((1 << bits) - 1) + value_min


