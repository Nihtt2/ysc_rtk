import json
import socket
import struct
import threading
import time
from datetime import datetime
from typing import Optional


class LynxM20Client:
    """
    DEEP Robotics Lynx M20 简易控制封装
    - 默认 UDP: 10.21.31.103:30000
    - 也支持 TCP: 10.21.31.103:30001
    - JSON ASDU + 16字节协议头

    注意:
    1) move() 之前建议先 set_regular_mode()，再 stand()，再 set_gait_basic()
    2) 手册建议 heartbeat >= 1Hz，这里默认按 2Hz 发
    3) 手册建议 move 控制频率约 20Hz
    """

    SYNC_HEADER = b"\xEB\x91\xEB\x90"
    ASDU_JSON = 0x01

    # motion states
    MOTION_IDLE = 0
    MOTION_STAND = 1
    MOTION_SOFT_ESTOP = 2
    MOTION_POWER_ON_DAMPING = 3
    MOTION_SIT = 4
    MOTION_STANDARD = 17  

    # usage modes
    MODE_REGULAR = 0
    MODE_NAVIGATION = 1

    # gaits
    GAIT_BASIC = 1
    GAIT_STAIR = 14

    HEARTBEAT_HZ = 2.0
    MOVE_CONTROL_HZ = 20.0
    HEARTBEAT_INTERVAL_S = 1.0 / HEARTBEAT_HZ
    MOVE_CONTROL_INTERVAL_S = 1.0 / MOVE_CONTROL_HZ

    def __init__(
        self,
        robot_ip: str = "10.21.31.103",
        udp_port: int = 30000,
        tcp_port: int = 30001,
        use_udp: bool = True,
        timeout: float = 1.0,
    ):
        self.robot_ip = robot_ip
        self.udp_port = udp_port
        self.tcp_port = tcp_port
        self.use_udp = use_udp
        self.timeout = timeout

        self._msg_id = 0
        self._hb_thread: Optional[threading.Thread] = None
        self._hb_running = False

        if self.use_udp:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(timeout)
            self.server_addr = (self.robot_ip, self.udp_port)
        else:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(timeout)
            self.sock.connect((self.robot_ip, self.tcp_port))
            self.server_addr = None

    def close(self):
        self.stop_heartbeat()
        try:
            self.sock.close()
        except Exception:
            pass

    def _next_msg_id(self) -> int:
        mid = self._msg_id
        self._msg_id = (self._msg_id + 1) & 0xFFFF
        return mid

    @staticmethod
    def _now_str() -> str:
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    @staticmethod
    def _clamp(v: float, lo: float = -1.0, hi: float = 1.0) -> float:
        return max(lo, min(hi, float(v)))

    def _build_asdu(self, msg_type: int, command: int, items: dict) -> bytes:
        payload = {
            "PatrolDevice": {
                "Type": msg_type,
                "Command": command,
                "Time": self._now_str(),
                "Items": items,
            }
        }
        return json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")

    def _build_apdu(self, asdu: bytes) -> bytes:
        msg_id = self._next_msg_id()
        header = (
            self.SYNC_HEADER
            + struct.pack("<H", len(asdu))       # ASDU length, little endian
            + struct.pack("<H", msg_id)         # Message ID, little endian
            + bytes([self.ASDU_JSON])           # JSON = 0x01
            + b"\x00" * 7                       # reserved
        )
        return header + asdu

    def _send_raw(self, apdu: bytes):
        if self.use_udp:
            self.sock.sendto(apdu, self.server_addr)
        else:
            self.sock.sendall(apdu)

    def send_command(self, msg_type: int, command: int, items: dict):
        asdu = self._build_asdu(msg_type, command, items)
        apdu = self._build_apdu(asdu)
        self._send_raw(apdu)

    # ---------------------------
    # 基础协议封装
    # ---------------------------

    def heartbeat(self):
        """Type=100, Command=100"""
        self.send_command(100, 100, {})

    def start_heartbeat(self, hz: float = HEARTBEAT_HZ):
        """
        启动心跳线程。手册建议 >=1Hz，默认按 2Hz 发。
        """
        if self._hb_running:
            return

        interval = 1.0 / hz if hz > 0 else self.HEARTBEAT_INTERVAL_S
        self._hb_running = True

        def _loop():
            while self._hb_running:
                try:
                    self.heartbeat()
                except Exception as e:
                    print(f"[heartbeat] send failed: {e}")
                time.sleep(interval)

        self._hb_thread = threading.Thread(target=_loop, daemon=True)
        self._hb_thread.start()

    def stop_heartbeat(self):
        self._hb_running = False
        if self._hb_thread is not None:
            self._hb_thread.join(timeout=1.0)
            self._hb_thread = None

    def set_regular_mode(self):
        """Type=1101, Command=5, Mode=0"""
        self.send_command(1101, 5, {"Mode": self.MODE_REGULAR})

    def set_navigation_mode(self):
        """Type=1101, Command=5, Mode=1"""
        self.send_command(1101, 5, {"Mode": self.MODE_NAVIGATION})

    def set_motion_state(self, motion_state: int):
        """Type=2, Command=22"""
        self.send_command(2, 22, {"MotionParam": int(motion_state)})

    def set_gait(self, gait: int):
        """Type=2, Command=23"""
        self.send_command(2, 23, {"GaitParam": int(gait)})

    def set_gait_basic(self):
        self.set_gait(self.GAIT_BASIC)

    def set_gait_stair(self):
        self.set_gait(self.GAIT_STAIR)

    # ---------------------------
    # 你要的高层封装
    # ---------------------------

    def move(
        self,
        vx: float,
        vy: float,
        vyaw: float,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
    ):
        """
        按你想要的接口:
            move(vx, vy, vyaw)

        这里映射到协议:
            X <- vx
            Y <- vy
            Yaw <- vyaw

        所有值范围建议 [-1, 1]
        持续控制时建议按 20Hz 周期调用本方法。
        """
        items = {
            "X": self._clamp(vx),
            "Y": self._clamp(vy),
            "Z": self._clamp(z),
            "Roll": self._clamp(roll),
            "Pitch": self._clamp(pitch),
            "Yaw": self._clamp(vyaw),
        }
        self.send_command(2, 21, items)

    def stop(self):
        """发送零速度停止"""
        self.move(0.0, 0.0, 0.0)

    def stand(self):
        self.set_motion_state(self.MOTION_STAND)

    def sit(self):
        self.set_motion_state(self.MOTION_SIT)

    def idle(self):
        self.set_motion_state(self.MOTION_IDLE)

    def soft_estop(self):
        self.set_motion_state(self.MOTION_SOFT_ESTOP)

    def damping(self):
        self.set_motion_state(self.MOTION_POWER_ON_DAMPING)

    def standard(self):
        self.set_motion_state(self.MOTION_STANDARD)

    def lights(self, front: bool, back: bool):
        """Type=1101, Command=2"""
        self.send_command(1101, 2, {
            "Front": 1 if front else 0,
            "Back": 1 if back else 0,
        })

    # ---------------------------
    # 一个安全启动流程
    # ---------------------------

    def prepare_for_manual_motion(self):
        """
        推荐手动控制前做:
        1. 启动心跳
        2. 切到 Regular mode
        3. 站立
        4. 切 Basic gait
        """
        self.start_heartbeat(hz=self.HEARTBEAT_HZ)
        time.sleep(0.2)

        self.set_regular_mode()
        time.sleep(0.2)

        self.stand()
        time.sleep(1.0)

        self.set_gait_basic()
        time.sleep(0.2)


if __name__ == "__main__":
    dog = LynxM20Client(
        robot_ip="10.21.31.103",
        use_udp=True,   # UDP 更简单；如果你想用 TCP 改成 False
    )

    try:
        dog.prepare_for_manual_motion()

        # 前进 2 秒：20Hz 连续发
        # start = time.time()
        # while time.time() - start < 2.0:
        #     dog.move(0.25, 0.0, 0.0)
        #     time.sleep(dog.MOVE_CONTROL_INTERVAL_S)

        # 横移 2 秒
        # start = time.time()
        # while time.time() - start < 2.0:
        #     dog.move(0.0, -0.5, 0.0)
        #     time.sleep(dog.MOVE_CONTROL_INTERVAL_S)

        # 原地转 2 秒
        # start = time.time()
        # while time.time() - start < 2.0:
        #     dog.move(0.0, 0.0, 0.25)
        #     time.sleep(dog.MOVE_CONTROL_INTERVAL_S)

        # 停止也连续发几帧
        dog.stop()
        time.sleep(dog.MOVE_CONTROL_INTERVAL_S)

        dog.sit()
        time.sleep(1.0)

    finally:
        dog.stop()
        time.sleep(dog.MOVE_CONTROL_INTERVAL_S)
        dog.close()
