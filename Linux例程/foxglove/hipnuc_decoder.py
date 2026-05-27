"""
HiPNUC binary protocol decoder (HI91 / HI92 / HI81).
Ported from ../lib/hipnuc_dec.c for use by the Foxglove bridge.
"""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass
from typing import Optional

CHSYNC1 = 0x5A
CHSYNC2 = 0xA5
CH_HDR_SIZE = 6
HIPNUC_MAX_RAW_SIZE = 256
GRAVITY = 9.8

HIPNUC_ID_HI91 = 0x91
HIPNUC_ID_HI92 = 0x92
HIPNUC_ID_HI81 = 0x81

# Foxglove 需注册 JSON Schema，Plot 面板才能列出字段名（否则只有话题名 /hi91）
HI91_JSON_SCHEMA: dict = {
    "title": "HI91",
    "type": "object",
    "additionalProperties": False,
    "properties": {
        "type": {"type": "string"},
        "pps_sync_stamp": {"type": "integer"},
        "temperature": {"type": "integer"},
        "air_pressure": {"type": "number"},
        "system_time": {"type": "integer"},
        "acc": {
            "type": "array",
            "items": {"type": "number"},
            "minItems": 3,
            "maxItems": 3,
        },
        "gyr": {
            "type": "array",
            "items": {"type": "number"},
            "minItems": 3,
            "maxItems": 3,
        },
        "mag": {
            "type": "array",
            "items": {"type": "number"},
            "minItems": 3,
            "maxItems": 3,
        },
        "pitch": {"type": "number"},
        "roll": {"type": "number"},
        "yaw": {"type": "number"},
        "quat": {
            "type": "array",
            "items": {"type": "number"},
            "minItems": 4,
            "maxItems": 4,
        },
    },
}

HI91_EULER_JSON_SCHEMA: dict = {
    "title": "HI91_euler",
    "type": "object",
    "additionalProperties": False,
    "properties": {
        "pitch": {"type": "number"},
        "roll": {"type": "number"},
        "yaw": {"type": "number"},
    },
}


@dataclass
class Hi91Packet:
    """HI91 浮点 IMU 数据帧，字段与 hipnuc_dec.h / 编程手册 3.3.1 一致。"""

    pps_sync_stamp: int = 0  # ms，手册 offset 1
    temperature: int = 0  # °C
    air_pressure: float = 0.0  # Pa
    system_time: int = 0  # ms，上电后本地时间戳
    acc_g: tuple[float, float, float] = (0.0, 0.0, 0.0)  # G，模块原始单位
    gyr: tuple[float, float, float] = (0.0, 0.0, 0.0)  # deg/s
    mag: tuple[float, float, float] = (0.0, 0.0, 0.0)  # uT
    roll: float = 0.0  # deg
    pitch: float = 0.0  # deg
    yaw: float = 0.0  # deg
    quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)  # w,x,y,z

    @property
    def acc(self) -> tuple[float, float, float]:
        """m/s²，与 hihost read / hipnuc_dump_packet 显示一致 (G × 9.8)。"""
        return (
            self.acc_g[0] * GRAVITY,
            self.acc_g[1] * GRAVITY,
            self.acc_g[2] * GRAVITY,
        )

    def as_hihost_terminal_dict(self) -> dict:
        """与 hihost read 终端打印字段完全一致（hipnuc_dump_packet）。"""
        return {
            "type": "HI91",
            "system_time": self.system_time,
            "acc": [round(self.acc[0], 3), round(self.acc[1], 3), round(self.acc[2], 3)],
            "gyr": [round(self.gyr[0], 3), round(self.gyr[1], 3), round(self.gyr[2], 3)],
            "mag": [round(self.mag[0], 3), round(self.mag[1], 3), round(self.mag[2], 3)],
            "pitch": round(self.pitch, 2),
            "roll": round(self.roll, 2),
            "yaw": round(self.yaw, 2),
            "quat": [
                round(self.quat[0], 3),
                round(self.quat[1], 3),
                round(self.quat[2], 3),
                round(self.quat[3], 3),
            ],
            "air_pressure": round(self.air_pressure, 1),
        }

    def format_hihost_terminal(self) -> str:
        """hihost read 风格的终端输出文本。"""
        return json.dumps(self.as_hihost_terminal_dict(), indent=2)

    def as_hihost_json(self) -> dict:
        """Foxglove /hi91 整帧（含 temperature、pps_sync_stamp）。"""
        data = self.as_hihost_terminal_dict()
        data["pps_sync_stamp"] = self.pps_sync_stamp
        data["temperature"] = self.temperature
        return data


@dataclass
class HipnucFrame:
    hi91: Optional[Hi91Packet] = None
    packet_type: Optional[str] = None


def _crc16(initial: int, data: bytes) -> int:
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            temp = (crc << 1) & 0xFFFF
            if crc & 0x8000:
                temp ^= 0x1021
            crc = temp
    return crc & 0xFFFF


def _parse_hi91(payload: bytes, offset: int) -> tuple[Hi91Packet, int]:
    # struct hi91_t (packed): tag B, status H, temp b, pad0, air_pressure f,
    # system_time I, acc[3] f, gyr[3] f, mag[3] f, roll/pitch/yaw f, quat[4] f
    (
        _tag,
        pps_sync_stamp,
        temp,
        air_pressure,
        system_time,
        ax,
        ay,
        az,
        gx,
        gy,
        gz,
        mx,
        my,
        mz,
        roll,
        pitch,
        yaw,
        qw,
        qx,
        qy,
        qz,
    ) = struct.unpack_from("<BhBfI16f", payload, offset)
    return (
        Hi91Packet(
            pps_sync_stamp=pps_sync_stamp,
            temperature=temp,
            air_pressure=air_pressure,
            system_time=system_time,
            acc_g=(ax, ay, az),
            gyr=(gx, gy, gz),
            mag=(mx, my, mz),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            quat=(qw, qx, qy, qz),
        ),
        offset + 76,
    )


class HipnucDecoder:
    """Byte-stream decoder matching hipnuc_input() behavior."""

    def __init__(self) -> None:
        self._buf = bytearray(HIPNUC_MAX_RAW_SIZE)
        self._nbyte = 0
        self._payload_len = 0

    def feed(self, byte: int) -> Optional[HipnucFrame]:
        if self._nbyte == 0:
            self._buf[0] = self._buf[1] if len(self._buf) > 1 else 0
            self._buf[1] = byte
            if not (self._buf[0] == CHSYNC1 and self._buf[1] == CHSYNC2):
                return None
            self._nbyte = 2
            return None

        self._buf[self._nbyte] = byte
        self._nbyte += 1

        if self._nbyte == CH_HDR_SIZE:
            self._payload_len = struct.unpack_from("<H", self._buf, 2)[0]
            if self._payload_len > HIPNUC_MAX_RAW_SIZE - CH_HDR_SIZE:
                self._nbyte = 0
                return None

        if self._nbyte < CH_HDR_SIZE or self._nbyte < self._payload_len + CH_HDR_SIZE:
            return None

        frame = bytes(self._buf[: self._nbyte])
        self._nbyte = 0

        crc = 0
        crc = _crc16(crc, frame[: CH_HDR_SIZE - 2])
        crc = _crc16(crc, frame[CH_HDR_SIZE : CH_HDR_SIZE + self._payload_len])
        expected = struct.unpack_from("<H", frame, CH_HDR_SIZE - 2)[0]
        if crc != expected:
            return None

        return self._parse_payload(frame[CH_HDR_SIZE : CH_HDR_SIZE + self._payload_len])

    def _parse_payload(self, payload: bytes) -> Optional[HipnucFrame]:
        ofs = 0
        hi91: Optional[Hi91Packet] = None

        while ofs < len(payload):
            tag = payload[ofs]
            if tag == HIPNUC_ID_HI91 and ofs + 76 <= len(payload):
                hi91, ofs = _parse_hi91(payload, ofs)
            elif tag in (HIPNUC_ID_HI92, HIPNUC_ID_HI81):
                # Skip extended packets in this bridge; HI91 is the common IMU output.
                break
            else:
                ofs += 1

        if hi91 is None:
            return None
        return HipnucFrame(hi91=hi91, packet_type="HI91")
