#!/usr/bin/env python3
"""Wait until the vehicle's systems are reachable, before the drivers start.

    ros2 run bluerov2_bringup preflight.py --timeout 180 pi imu dvl nvidia

Checks (any subset, in any order):

    pi      BlueOS on the Raspberry Pi answers ping (192.168.2.2)
    imu     ArduSub sends its heartbeat (MAVLink system 1) to UDP 14551, the port the
            IMU driver uses. Run this before the IMU driver starts: two programs on one
            UDP port share its packets.
    dvl     the DVL answers ping (192.168.2.128) and its TCP port 16171 is open
    nvidia  the multi-camera computer answers ping (10.42.0.5) and its camera software
            answers camera_status (checked with `multicam.py status`, on CycloneDDS)

Every 3 s the checks that are not OK yet are retried. A table is printed whenever
something changes, and a reminder every 15 s. Exit 0 when everything is OK, 1 when
--timeout expires first. A check that became OK is not repeated.
"""

import argparse
import socket
import subprocess
import sys
import time

PI_IP = "192.168.2.2"
DVL_IP, DVL_PORT = "192.168.2.128", 16171
NVIDIA_IP = "10.42.0.5"
MAVLINK_PORT = 14551
VEHICLE_SYSID = 1

ORDER = ("pi", "imu", "dvl", "nvidia")
LABELS = {
    "pi": ("Pi (BlueOS)", PI_IP),
    "imu": ("IMU (ArduSub)", f"heartbeat on UDP {MAVLINK_PORT}"),
    "dvl": ("DVL", f"{DVL_IP}:{DVL_PORT}"),
    "nvidia": ("NVIDIA (multicam)", NVIDIA_IP),
}


def ping(ip: str) -> bool:
    return subprocess.run(
        ["ping", "-c", "1", "-W", "1", ip],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    ).returncode == 0


def tcp_open(ip: str, port: int) -> bool:
    try:
        socket.create_connection((ip, port), timeout=2).close()
        return True
    except OSError:
        return False


def mavlink_frames(data: bytes):
    """Yield (sysid, msgid) for every MAVLink v1/v2 frame in a datagram."""
    i = 0
    while i < len(data):
        if data[i] == 0xFD and i + 10 <= len(data):          # MAVLink 2
            length, incompat = data[i + 1], data[i + 2]
            sysid = data[i + 5]
            msgid = int.from_bytes(data[i + 7:i + 10], "little")
            yield sysid, msgid
            i += 12 + length + (13 if incompat & 0x01 else 0)
        elif data[i] == 0xFE and i + 6 <= len(data):         # MAVLink 1
            length = data[i + 1]
            yield data[i + 3], data[i + 5]
            i += 8 + length
        else:
            i += 1


class HeartbeatListener:
    def __init__(self, port: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)

    def heard(self, seconds: float) -> bool:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                data = self.sock.recv(4096)
            except BlockingIOError:
                time.sleep(0.05)
                continue
            if any(sysid == VEHICLE_SYSID and msgid == 0 for sysid, msgid in mavlink_frames(data)):
                return True
        return False

    def close(self):
        self.sock.close()


def nvidia_software_ok() -> bool:
    return subprocess.run(
        ["ros2", "run", "bluerov2_bringup", "multicam.py", "--timeout", "4", "status"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    ).returncode == 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Wait until the vehicle's systems are reachable.")
    parser.add_argument("checks", nargs="+", choices=ORDER)
    parser.add_argument("--timeout", type=float, default=180.0, help="seconds (default 180)")
    args, _ros_args = parser.parse_known_args()
    checks = [c for c in ORDER if c in args.checks]

    status = {c: "waiting" for c in checks}
    ok = {c: False for c in checks}
    listener = HeartbeatListener(MAVLINK_PORT) if "imu" in checks else None
    start = time.monotonic()
    last_table = ""
    last_print = 0.0

    def table(elapsed: float) -> str:
        lines = [f"--- preflight, {elapsed:5.0f} s ---"]
        for c in checks:
            name, target = LABELS[c]
            lines.append(f"  {name:<19}{target:<24}{'OK' if ok[c] else '..'}  {status[c]}")
        return "\n".join(lines)

    try:
        while True:
            elapsed = time.monotonic() - start
            if "pi" in checks and not ok["pi"]:
                ok["pi"] = ping(PI_IP)
                status["pi"] = f"ping OK after {elapsed:.0f} s" if ok["pi"] else "no ping reply"
            if "imu" in checks and not ok["imu"]:
                ok["imu"] = listener.heard(1.5)
                status["imu"] = (f"heartbeat after {elapsed:.0f} s" if ok["imu"]
                                 else "no ArduSub heartbeat yet")
            if "dvl" in checks and not ok["dvl"]:
                if not ping(DVL_IP):
                    status["dvl"] = "no ping reply"
                elif not tcp_open(DVL_IP, DVL_PORT):
                    status["dvl"] = f"ping OK, TCP {DVL_PORT} not open yet"
                else:
                    ok["dvl"] = True
                    status["dvl"] = f"ping + TCP OK after {elapsed:.0f} s"
            if "nvidia" in checks and not ok["nvidia"]:
                if not ping(NVIDIA_IP):
                    status["nvidia"] = "no ping reply"
                elif not nvidia_software_ok():
                    status["nvidia"] = "ping OK, camera software not answering yet"
                else:
                    ok["nvidia"] = True
                    status["nvidia"] = f"ping + camera software OK after {elapsed:.0f} s"

            elapsed = time.monotonic() - start
            current = table(elapsed)
            body = current.split("\n", 1)[1]
            if body != last_table or elapsed - last_print >= 15:
                print(current, flush=True)
                last_table, last_print = body, elapsed

            if all(ok.values()):
                print(f"PREFLIGHT PASSED: {', '.join(checks)} ready after {elapsed:.0f} s", flush=True)
                return 0
            if elapsed >= args.timeout:
                missing = [LABELS[c][0] for c in checks if not ok[c]]
                print(f"PREFLIGHT FAILED after {elapsed:.0f} s: not ready: {', '.join(missing)}",
                      flush=True)
                return 1
            time.sleep(3)
    finally:
        if listener:
            listener.close()


if __name__ == "__main__":
    sys.exit(main())
