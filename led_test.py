#!/usr/bin/env python3
"""
led_test.py - set a Smartgloves ring LED green, yellow, red, off (2 s each) over RCSP.

Uses the RCSP framing and commands from glove_bridge.py (RcspWorker; glove_bridge.py is not
modified): ListDevices -> GetDeviceInfo (left/right) -> SetColorLed {DeviceId, LedId "RingLed",
Red, Green, Blue}. Every step prints its result; on failure it prints the exact error and exits 1.

  python led_test.py                  # right glove, 2 s per colour
  python led_test.py --hand left --seconds 1
  python led_test.py --port 45451 --host 127.0.0.1

Run it while the driver (rokoko-sdk / rkk-hand-solver) is running. If glove_bridge.py runs at the
same time, start it with --no-rcsp, otherwise it sets the LED too (blue idle / green engaged).
Test without gloves:  python fake_rgmp_server.py   (its window prints "[fake] LED right: rgb(...)")
Standard library only.
"""
from __future__ import annotations

import argparse
import queue
import socket
import sys
import time

from glove_bridge import RcspWorker

SEQUENCE = [("green", (0, 255, 0)), ("yellow", (255, 180, 0)), ("red", (255, 0, 0)), ("off", (0, 0, 0))]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=45451)
    ap.add_argument("--hand", default="right", choices=["right", "left"])
    ap.add_argument("--seconds", type=float, default=2.0, help="time per colour")
    a = ap.parse_args()

    w = RcspWorker(a.host, a.port, queue.Queue(), lambda m: print(f"  log: {m}"))
    step = "connect"
    try:
        sock = socket.create_connection((a.host, a.port), timeout=5.0)
        print(f"[1] connected to RCSP {a.host}:{a.port}")
        step = "ListDevices"
        devices = w.command(sock, "ListDevices").get("Devices", [])
        print(f"[2] ListDevices: {len(devices)} device(s)")
        for d in devices:
            print(f"      DeviceId={d.get('DeviceId')} type={d.get('DeviceType')} "
                  f"bootloader={d.get('IsBootloader')}")
        step = "GetDeviceInfo"
        dev = None
        for d in devices:
            if d.get("DeviceType") != "Smartgloves" or d.get("IsBootloader"):
                continue
            info = w.command(sock, "GetDeviceInfo", {"DeviceId": d["DeviceId"]})
            is_left = (info.get("Smartgloves") or {}).get("IsLeft")
            side = "left" if is_left else "right"
            print(f"[3] DeviceId={d['DeviceId']} is the {side} glove")
            if side == a.hand:
                dev = d["DeviceId"]
        if dev is None:
            print(f"FAIL at {step}: no {a.hand} Smartglove in ListDevices (is it on and connected to the driver?)")
            sys.exit(1)
        step = "SetColorLed"
        for name, (r, g, b) in SEQUENCE:
            resp = w.command(sock, "SetColorLed", {"DeviceId": dev, "LedId": "RingLed",
                                                   "Red": r, "Green": g, "Blue": b})
            print(f"[4] LED {name:6s} rgb({r},{g},{b}) -> OK {resp if resp else ''}")
            time.sleep(a.seconds)
        sock.close()
    except (OSError, ConnectionError, RuntimeError, TimeoutError, ValueError) as e:
        print(f"FAIL at {step}: {type(e).__name__}: {e}")
        sys.exit(1)
    print("PASS: all commands acknowledged. Check that the ring LED really showed green, yellow, red, then off.")


if __name__ == "__main__":
    main()
