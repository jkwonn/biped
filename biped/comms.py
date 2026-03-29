"""talks to the robot hardware over serial and the vision pipeline over udp"""
import json
import socket
import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class RobotCommand:
    """command packet from the vision pipeline"""
    yaw: float = 0.0
    forward: float = 0.0
    head_pitch: float = 0.0
    head_yaw: float = 0.0
    tracking: bool = False
    target_id: int = -1

    def to_dict(self) -> dict:
        return {
            "yaw": round(self.yaw, 3),
            "forward": round(self.forward, 3),
            "head_pitch": round(self.head_pitch, 3),
            "head_yaw": round(self.head_yaw, 3),
            "tracking": self.tracking,
            "target_id": self.target_id,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict())

    @classmethod
    def from_json(cls, data: str) -> "RobotCommand":
        d = json.loads(data)
        return cls(**d)


class UDPSender:
    """sends command packets over udp"""
    def __init__(self, ip: str = "127.0.0.1", port: int = 8888):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = (ip, port)

    def send(self, cmd: RobotCommand):
        try:
            self.sock.sendto(cmd.to_json().encode("utf-8"), self.target)
        except OSError:
            pass

    def close(self):
        self.sock.close()


class UDPReceiver:
    """listens for command packets over udp (for testing)"""
    def __init__(self, port: int = 8888):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))

    def receive(self) -> RobotCommand:
        data, _ = self.sock.recvfrom(1024)
        return RobotCommand.from_json(data.decode("utf-8"))

    def close(self):
        self.sock.close()


class SerialLink:
    """
    usb serial connection to the daisy seed firmware.

    protocol (text lines at 115200 baud):
        computer to daisy:
            P <ch0> <ch1> ... <ch11>   set all servo pulses (us)
            S <ch> <pulse_us>          set one servo
            C                          center all servos to 1500us

        daisy to computer:
            H <uptime_ms>             heartbeat
            I <pitch> <roll> <yaw>    imu data (radians)
            BIPED OK <n>              startup confirmation
    """

    def __init__(self, port: str, baudrate: int = 115200):
        import serial
        self.ser = serial.Serial(port, baudrate, timeout=0.01)
        self.imu_pitch = 0.0
        self.imu_roll = 0.0
        self.imu_yaw = 0.0
        self.connected = False

        # wait for the daisy to send its startup message
        deadline = time.time() + 3.0
        while time.time() < deadline:
            line = self._readline()
            if line and line.startswith("BIPED OK"):
                self.connected = True
                print(f"connected to daisy seed on {port}")
                break
        if not self.connected:
            print(f"warning: no startup message from {port} (continuing anyway)")
            self.connected = True

    def send_pulses(self, pulses: list[int]):
        """sends pulse widths for all 12 servo channels at once"""
        msg = "P " + " ".join(str(p) for p in pulses) + "\n"
        self.ser.write(msg.encode("ascii"))

    def send_single(self, channel: int, pulse_us: int):
        """sets a single servo channel to a specific pulse width"""
        self.ser.write(f"S {channel} {pulse_us}\n".encode("ascii"))

    def center_all(self):
        """tells the daisy to center all servos to 1500us"""
        self.ser.write(b"C\n")

    def poll(self) -> Optional[str]:
        """reads any pending messages from the daisy and updates imu state"""
        last = None
        while True:
            line = self._readline()
            if not line:
                break
            if line.startswith("I "):
                parts = line.split()
                if len(parts) >= 4:
                    self.imu_pitch = float(parts[1])
                    self.imu_roll = float(parts[2])
                    self.imu_yaw = float(parts[3])
            last = line
        return last

    def close(self):
        self.ser.close()

    def _readline(self) -> Optional[str]:
        try:
            raw = self.ser.readline()
            if raw:
                return raw.decode("ascii", errors="replace").strip()
        except Exception:
            pass
        return None


def run_mock_receiver(port: int = 8888):
    """prints incoming udp commands in a loop. for testing without hardware"""
    recv = UDPReceiver(port)
    print(f"mock receiver listening on port {port}...")
    print("run 'python -m biped vision --send-udp' in another terminal.\n")

    try:
        while True:
            cmd = recv.receive()
            status = "TRACK" if cmd.tracking else "SEARCH"
            print(
                f"[{status}] yaw={cmd.yaw:+.3f} fwd={cmd.forward:+.3f} "
                f"head=({cmd.head_yaw:+.3f}, {cmd.head_pitch:+.3f})"
            )
    except KeyboardInterrupt:
        print("\nreceiver stopped.")
    finally:
        recv.close()
