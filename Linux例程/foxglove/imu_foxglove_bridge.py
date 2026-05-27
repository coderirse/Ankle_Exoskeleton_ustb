#!/usr/bin/env python3
"""
Read HiPNUC HI91 IMU from serial and stream to Foxglove via WebSocket.

Foxglove 话题与 HI91 字段对应；注册 JSON Schema 后 Plot 可选取字段。
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass

import serial

from hipnuc_decoder import (
    HI91_EULER_JSON_SCHEMA,
    HI91_JSON_SCHEMA,
    HipnucDecoder,
    Hi91Packet,
)

TMP_CONFIG_FILE = "/tmp/hihost_config.tmp"
DEFAULT_PORT = 8765
DISPLAY_UPDATE_INTERVAL = 0.05  # 与 hihost read 相同


@dataclass
class Hi91FoxgloveChannels:
    """与 HI91 各字段一一对应的 Foxglove 话题。"""

    frame: object  # Channel /hi91 整帧
    acc: object  # Vector3Channel /hi91/acc
    gyr: object  # Vector3Channel /hi91/gyr
    mag: object  # Vector3Channel /hi91/mag
    quat: object  # QuaternionChannel /hi91/quat
    euler: object  # Channel /hi91/euler
    system_time: object
    temperature: object
    air_pressure: object
    pps_sync_stamp: object
    attitude: object  # FrameTransformsChannel /hi91/attitude


def _scalar_channel(name: str, prop_type: str) -> object:
    from foxglove import Channel

    return Channel(
        f"/hi91/{name}",
        schema={
            "type": "object",
            "additionalProperties": False,
            "properties": {name: {"type": prop_type}},
        },
    )


def create_hi91_channels() -> Hi91FoxgloveChannels:
    from foxglove import Channel
    from foxglove.channels import (
        FrameTransformsChannel,
        QuaternionChannel,
        Vector3Channel,
    )

    return Hi91FoxgloveChannels(
        frame=Channel("/hi91", schema=HI91_JSON_SCHEMA),
        acc=Vector3Channel("/hi91/acc"),
        gyr=Vector3Channel("/hi91/gyr"),
        mag=Vector3Channel("/hi91/mag"),
        quat=QuaternionChannel("/hi91/quat"),
        euler=Channel("/hi91/euler", schema=HI91_EULER_JSON_SCHEMA),
        system_time=_scalar_channel("system_time", "integer"),
        temperature=_scalar_channel("temperature", "integer"),
        air_pressure=_scalar_channel("air_pressure", "number"),
        pps_sync_stamp=_scalar_channel("pps_sync_stamp", "integer"),
        attitude=FrameTransformsChannel("/hi91/attitude"),
    )


def publish_hi91(channels: Hi91FoxgloveChannels, p: Hi91Packet, ts: int) -> None:
    from foxglove.messages import FrameTransform, FrameTransforms, Quaternion, Vector3

    channels.frame.log(p.as_hihost_json(), log_time=ts)

    channels.acc.log(
        Vector3(x=p.acc[0], y=p.acc[1], z=p.acc[2]),
        log_time=ts,
    )
    channels.gyr.log(
        Vector3(x=p.gyr[0], y=p.gyr[1], z=p.gyr[2]),
        log_time=ts,
    )
    channels.mag.log(
        Vector3(x=p.mag[0], y=p.mag[1], z=p.mag[2]),
        log_time=ts,
    )
    channels.quat.log(
        Quaternion(x=p.quat[1], y=p.quat[2], z=p.quat[3], w=p.quat[0]),
        log_time=ts,
    )
    channels.euler.log(
        {"pitch": p.pitch, "roll": p.roll, "yaw": p.yaw},
        log_time=ts,
    )
    channels.system_time.log({"system_time": p.system_time}, log_time=ts)
    channels.temperature.log({"temperature": p.temperature}, log_time=ts)
    channels.air_pressure.log({"air_pressure": p.air_pressure}, log_time=ts)
    channels.pps_sync_stamp.log({"pps_sync_stamp": p.pps_sync_stamp}, log_time=ts)

    channels.attitude.log(
        FrameTransforms(
            transforms=[
                FrameTransform(
                    parent_frame_id="world",
                    child_frame_id="hi91",
                    rotation=Quaternion(
                        x=p.quat[1],
                        y=p.quat[2],
                        z=p.quat[3],
                        w=p.quat[0],
                    ),
                )
            ]
        ),
        log_time=ts,
    )


def load_probed_config() -> tuple[str | None, int | None]:
    try:
        with open(TMP_CONFIG_FILE, "r", encoding="utf-8") as f:
            parts = f.read().strip().split()
            if len(parts) >= 2:
                return parts[0], int(parts[1])
    except OSError:
        pass
    return None, None


def safe_serial_read(ser: serial.Serial) -> bytes:
    """避免 pyserial 'readiness but no data' 导致崩溃。"""
    try:
        waiting = ser.in_waiting
        if waiting > 0:
            data = ser.read(waiting)
            if data:
                return data
        return ser.read(1)
    except serial.SerialException:
        time.sleep(0.01)
        return b""


def enable_device_output(ser: serial.Serial) -> None:
    cmd = b"AT+EOUT=1\r\n"
    ser.reset_input_buffer()
    ser.write(cmd)
    deadline = time.monotonic() + 0.5
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = safe_serial_read(ser)
        if chunk:
            buf.extend(chunk)
            if b"OK" in buf:
                return
        else:
            time.sleep(0.01)
    print("Warning: AT+EOUT=1 did not return OK; continuing anyway.", file=sys.stderr)


def print_hihost_terminal(packet: Hi91Packet, frame_rate: int) -> None:
    """终端输出与 hihost read 相同：清屏 + JSON + 帧率。"""
    sys.stdout.write("\033[H\033[J")
    print(packet.format_hihost_terminal())
    print(f"\nFrame Rate: {frame_rate} fps")
    sys.stdout.flush()


def log_time_ns(packet: Hi91Packet) -> int:
    return int(packet.system_time) * 1_000_000 if packet.system_time else time.time_ns()


def run_bridge(port: str, baud: int, host: str, ws_port: int, *, show_terminal: bool) -> None:
    try:
        import foxglove
    except ImportError as exc:
        print(
            "Missing dependency: pip install foxglove-sdk pyserial\n"
            "Or: ./run.sh",
            file=sys.stderr,
        )
        raise SystemExit(1) from exc

    channels = create_hi91_channels()

    print(f"Opening {port} @ {baud} ...")
    with serial.Serial(port, baud, timeout=0.02) as ser:
        enable_device_output(ser)
        decoder = HipnucDecoder()

        print(f"Foxglove WebSocket: ws://127.0.0.1:{ws_port}")
        print("Topics (HI91 fields):")
        for name in (
            "/hi91",
            "/hi91/acc",
            "/hi91/gyr",
            "/hi91/mag",
            "/hi91/quat",
            "/hi91/euler",
            "/hi91/system_time",
            "/hi91/temperature",
            "/hi91/air_pressure",
            "/hi91/pps_sync_stamp",
            "/hi91/attitude",
        ):
            print(f"  {name}")
        if show_terminal:
            print("Terminal: hihost-style JSON (same as `hihost read`).")
        print("Press Ctrl+C to stop.\n")

        server = foxglove.start_server(host=host, port=ws_port)
        try:
            frame_count = 0
            frame_rate = 0
            last_fps_time = time.monotonic()
            last_display_time = time.monotonic()
            latest_packet: Hi91Packet | None = None

            while True:
                chunk = safe_serial_read(ser)
                if not chunk:
                    time.sleep(0.001)
                    continue

                for b in chunk:
                    frame = decoder.feed(b)
                    if frame is None or frame.hi91 is None:
                        continue

                    latest_packet = frame.hi91
                    publish_hi91(channels, frame.hi91, log_time_ns(frame.hi91))
                    frame_count += 1

                now = time.monotonic()
                fps_elapsed = now - last_fps_time
                if fps_elapsed >= 1.0:
                    frame_rate = int(frame_count / fps_elapsed)
                    frame_count = 0
                    last_fps_time = now

                if (
                    show_terminal
                    and latest_packet is not None
                    and (now - last_display_time) >= DISPLAY_UPDATE_INTERVAL
                ):
                    print_hihost_terminal(latest_packet, frame_rate)
                    last_display_time = now
        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            server.stop()


def parse_args() -> argparse.Namespace:
    probed_port, probed_baud = load_probed_config()
    parser = argparse.ArgumentParser(
        description="Stream HiPNUC HI91 IMU to Foxglove (field names match hihost read)",
    )
    parser.add_argument("-p", "--port", default=probed_port or "/dev/ttyACM0")
    parser.add_argument("-b", "--baud", type=int, default=probed_baud or 115200)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--ws-port", type=int, default=DEFAULT_PORT)
    parser.add_argument(
        "-q",
        "--quiet",
        action="store_true",
        help="不打印终端 JSON（仅 Foxglove）",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not os.path.exists(args.port):
        print(f"Serial port not found: {args.port}", file=sys.stderr)
        print("Run: sudo ../build/hihost probe", file=sys.stderr)
        raise SystemExit(1)
    run_bridge(
        args.port,
        args.baud,
        args.host,
        args.ws_port,
        show_terminal=not args.quiet,
    )


if __name__ == "__main__":
    main()
