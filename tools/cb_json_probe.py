#!/usr/bin/env python3
"""Fetch the CyberBeast JSON endpoint descriptor (MsgType 0x24/0x25) over SocketCAN.

Protocol 4.8:
  request  (0x24): bytes 0-3 = offset (uint32 little-endian)
  response (0x25): first frame = metadata {0x00,0x00, TotalLength(u32 LE), VersionCRC(u16 LE)}
                   data frames   = {ChunkOffset(u16 LE), 6 bytes JSON}
  device sends at most 50 frames per cycle, so the host walks the offset.

Usage: python3 cb_json_probe.py [iface] [out.json]
"""

import json
import socket
import struct
import sys
import time

IFACE = sys.argv[1] if len(sys.argv) > 1 else "slcan0"
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/cyberbeast_endpoints.json"

MASTER, NODE = 0x01, 0x01
EFF = 0x8000_0000
PRI_S, MT_S, DEST_S, SRC_S = 26, 18, 10, 2
CONFIG = 4
MT_JSON_READ, MT_JSON_DATA = 0x24, 0x25
PAYLOAD_PER_FRAME = 6
FRAMES_PER_CYCLE = 50


def mk_id(pri, mt, dest, src, seq=0):
    return ((pri & 7) << PRI_S) | ((mt & 0xFF) << MT_S) | ((dest & 0xFF) << DEST_S) | ((src & 0xFF) << SRC_S) | (seq & 3)


def send(sock, can_id, data):
    data = bytes(data).ljust(8, b"\x00")[:8]
    sock.send(struct.pack("=IB3x8s", can_id | EFF, len(data), data))


def drain(sock, window):
    sock.settimeout(window)
    end = time.time() + window
    out = []
    while time.time() < end:
        try:
            buf = sock.recv(16)
        except socket.timeout:
            break
        can_id, dlc, data = struct.unpack("=IB3x8s", buf[:16])
        can_id &= 0x1FFF_FFFF
        if ((can_id >> MT_S) & 0xFF) == MT_JSON_DATA:
            out.append(data[:dlc])
    return out


def fetch(sock, offset):
    send(sock, mk_id(CONFIG, MT_JSON_READ, NODE, MASTER), struct.pack("<I", offset) + b"\x00" * 4)
    time.sleep(0.05)
    return drain(sock, 0.8)


def main():
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((IFACE,))

    first = fetch(sock, 0)
    if not first:
        print("no JSON_DESC_DATA frames received")
        return 1
    meta = first[0]
    print("metadata frame:", meta.hex(" "))
    total = struct.unpack("<I", meta[2:6])[0]
    crc = struct.unpack("<H", meta[6:8])[0]
    print(f"TotalLength={total} VersionCRC=0x{crc:04X}")

    chunks = {}
    for frame in first[1:]:
        chunks[struct.unpack("<H", frame[0:2])[0]] = frame[2:]

    offset = 0
    requests = 1
    while offset < total and requests < 400:
        offset = min(total, max(chunks) + PAYLOAD_PER_FRAME) if chunks else 0
        frames = fetch(sock, offset)
        requests += 1
        for frame in frames:
            if frame[0:2] == b"\x00\x00" and frame[2:6] == meta[2:6]:
                continue  # metadata frame repeated
            chunks[struct.unpack("<H", frame[0:2])[0]] = frame[2:]
        if not frames:
            print(f"device stopped answering at offset {offset}")
            break

    blob = bytearray(b" " * total)
    for off, data in chunks.items():
        blob[off:off + len(data)] = data
    text = bytes(blob).rstrip(b"\x00 ").decode("utf-8", "replace")
    print(f"collected {len(chunks)} chunks, {len(text)} chars, {requests} requests")

    try:
        tree = json.loads(text)
    except Exception as exc:  # noqa: BLE001
        print("JSON parse failed:", exc)
        print(text[:400])
        with open(OUT, "w", encoding="utf-8") as fh:
            fh.write(text)
        return 1

    with open(OUT, "w", encoding="utf-8") as fh:
        json.dump(tree, fh, indent=1, ensure_ascii=False)

    found = []

    def walk(node, path=""):
        if isinstance(node, dict):
            if "id" in node and isinstance(node.get("id"), int):
                found.append((path, node.get("id"), node.get("type"), node.get("name")))
            for key, value in node.items():
                walk(value, key if not path else f"{path}.{key}")
        elif isinstance(node, list):
            for item in node:
                walk(item, path)

    walk(tree)
    print(f"parsed JSON: {len(found)} endpoints, top-level keys: {list(tree)[:6]}")
    keys = ("torque_constant", "requested_state", "current_state", "error", "pos_gain", "current_lim",
            "cpr", "control_mode", "input_mode", "vel_limit", "bus_voltage", "fet_thermistor",
            "motor_thermistor", "serial_number", "dc_bus", "brake", "heartbeat")
    for name, eid, etype, _ in sorted(found, key=lambda item: item[1]):
        if any(k in name for k in keys):
            print(f"  id=0x{eid:04X} ({eid:5d}) type={etype:<8} {name}")
    print(f"full JSON written to {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
