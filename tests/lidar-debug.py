#!/usr/bin/env python3
"""
TF-Luna lidar debug tool — two modes:

  MODE 1 — Via FC USB (MAVLink):
      python3 lidar-debug.py
      Reads DISTANCE_SENSOR + STATUSTEXT from the flight-controller USB port.
      The firmware emits "L rx=N fr=N ck=N d=N a=N" via STATUSTEXT every 500 ms.

  MODE 2 — Direct raw serial (USB-UART adapter wired straight to the TF-Luna):
      python3 lidar-debug.py --direct /dev/ttyUSB0
      Parses raw 9-byte TF-Luna frames, prints distance + amplitude + validity.
      Use this to verify the lidar hardware independently of the firmware.

What each STATUSTEXT counter means:
  rx=0              → FC receives NO bytes at all.
                      Check: common ground, wiring, TF-Luna pins 5&6 must FLOAT
                      (grounding them switches the sensor to I2C — UART goes silent).
  rx>0  fr=0  ck=0  → Bytes arrive but no 0x59 0x59 sync found.
                      Likely baud mismatch — TF-Luna default is 115200.
  rx>0  fr=0  ck>0  → Sync found but checksum keeps failing.
                      Electrical noise or baud off-by-one.
  fr>0  ck=0        → Clean frames. If d=800 and a<100 the sensor just sees
                      nothing in range — point it at a wall < 4 m away.
  fr>0  d<800  a≥100 → Working correctly.  The firmware should output valid=true.
"""

import argparse
import sys
import time
import struct

FC_VID = 0x1209
FC_PID = 0x5741
TFLUNA_BAUD = 115_200
MAV_BAUD = 115_200

# MAVLink orientation codes used in DISTANCE_SENSOR
ORIENT = {25: "DOWN", 0: "FORWARD", 2: "RIGHT", 6: "LEFT"}


# ─── helpers ─────────────────────────────────────────────────────────────────

def find_fc_port():
    """Return the serial port for the FC (VID:PID 0x1209:0x5741) or None."""
    import serial.tools.list_ports
    for p in serial.tools.list_ports.comports():
        if p.vid == FC_VID and p.pid == FC_PID:
            return p.device
    return None


def ts():
    return time.strftime("%H:%M:%S")


# ─── MODE 1: MAVLink via FC USB ───────────────────────────────────────────────

def run_mavlink():
    port = find_fc_port()
    if port is None:
        print("FC not found (VID:PID 0x1209:0x5741). Is it plugged in and enumerated?")
        sys.exit(1)
    print(f"[{ts()}] FC found on {port}  (Ctrl-C to quit)\n")

    from pymavlink import mavutil
    mav = mavutil.mavlink_connection(port, baud=MAV_BAUD)

    print("Waiting for heartbeat …")
    mav.wait_heartbeat(timeout=10)
    print(f"[{ts()}] Heartbeat OK  system={mav.target_system}  component={mav.target_component}\n")

    print("─" * 60)
    print("Listening for DISTANCE_SENSOR (left lidar) and STATUSTEXT …")
    print("─" * 60)

    while True:
        msg = mav.recv_match(
            type=["DISTANCE_SENSOR", "STATUSTEXT"],
            blocking=True,
            timeout=5,
        )
        if msg is None:
            print(f"[{ts()}] (no message for 5 s — FC still connected?)")
            continue

        t = msg.get_type()

        if t == "DISTANCE_SENSOR":
            orient = msg.orientation
            label = ORIENT.get(orient, f"orient={orient}")
            dist = msg.current_distance   # cm
            valid = dist < msg.max_distance
            flag = "OK" if valid else "OOR"   # out-of-range
            print(
                f"[{ts()}] DISTANCE_SENSOR  {label:8s}  "
                f"dist={dist:4d} cm  ({dist/100:.2f} m)  [{flag}]  "
                f"min={msg.min_distance} max={msg.max_distance}"
            )

        elif t == "STATUSTEXT":
            text = msg.text.rstrip("\x00").strip()
            if text.startswith("L "):          # our lidar diagnostic line
                parts = _parse_lidar_status(text)
                print(f"\n[{ts()}] LIDAR DIAG  {text}")
                if parts:
                    rx, fr, ck, d, a = parts
                    print(_diagnose(rx, fr, ck, d, a))
                print()
            else:
                print(f"[{ts()}] STATUS  {text}")


def _parse_lidar_status(text):
    """Return (rx_bytes, frames, ck_errors, dist_cm, amplitude) or None."""
    try:
        fields = {}
        for tok in text.split():
            if "=" in tok:
                k, v = tok.split("=", 1)
                fields[k] = int(v)
        return (
            fields["rx"], fields["fr"], fields["ck"],
            fields["d"],  fields["a"],
        )
    except Exception:
        return None


def _diagnose(rx, fr, ck, d, a):
    AMP_MIN = 100
    MAX_CM  = 800

    if rx == 0:
        verdict = (
            "  !! rx=0 — FC receives NO bytes from the lidar.\n"
            "     Check:\n"
            "       • Common ground between TF-Luna and FC\n"
            "       • TF-Luna TX  →  R6 pad on FC  (PC7, USART6 RX)\n"
            "       • TF-Luna pins 5 & 6 must be FLOATING — if grounded\n"
            "         the sensor switches to I2C mode and UART goes silent\n"
            "       • Lidar is powered (5 V, ≥ 130 mA)"
        )
    elif fr == 0 and ck == 0:
        verdict = (
            "  !! rx>0 but no frames decoded and no checksum errors.\n"
            "     The 0x59 0x59 sync header is never found.\n"
            "     Likely cause: baud-rate mismatch.  TF-Luna default = 115200.\n"
            "     Run  --direct /dev/ttyUSBx  with a USB-UART adapter to\n"
            "     confirm what the lidar is actually sending."
        )
    elif ck > 0 and fr == 0:
        verdict = (
            f"  !! {ck} checksum errors, 0 good frames.\n"
            "     Bytes arrive but the frames are corrupt.\n"
            "     Likely: electrical noise, too-long wires, or a baud mismatch\n"
            "     that accidentally matches the 0x59 0x59 header."
        )
    elif fr > 0 and ck > 0:
        verdict = (
            f"  ~  {fr} good frames + {ck} checksum errors — mostly working\n"
            "     but some corruption.  Check wire routing / shielding."
        )
    elif fr > 0 and a < AMP_MIN:
        verdict = (
            f"  ~  Frames decoding OK but amplitude={a} < {AMP_MIN}.\n"
            "     The sensor sees no valid return — point it at a wall < 4 m.\n"
            "     Amplitude < 100 means the target is too far, too dark,\n"
            "     or the lidar is blocked."
        )
    elif fr > 0 and d >= MAX_CM:
        verdict = (
            f"  ~  Frames OK, amplitude={a} ≥ {AMP_MIN} but dist={d} cm (max).\n"
            "     Sensor is working but nothing is within 8 m."
        )
    else:
        verdict = f"  ✓  Lidar healthy: dist={d} cm  amplitude={a}"

    return verdict


# ─── MODE 2: direct raw serial ───────────────────────────────────────────────

def run_direct(port_path):
    import serial

    print(f"Opening {port_path} at {TFLUNA_BAUD} baud  (Ctrl-C to quit)\n")
    try:
        ser = serial.Serial(port_path, TFLUNA_BAUD, timeout=2)
    except serial.SerialException as e:
        print(f"Cannot open {port_path}: {e}")
        sys.exit(1)

    AMP_MIN  = 100
    MAX_CM   = 800
    MIN_CM   = 20

    print("─" * 60)
    print("Reading raw TF-Luna frames (9 bytes each) …")
    print("─" * 60)

    rx_bytes     = 0
    good_frames  = 0
    ck_errors    = 0
    buf          = bytearray()
    last_print   = time.time()
    stats_interval = 3.0   # print running stats every 3 s

    try:
        while True:
            chunk = ser.read(64)
            if not chunk:
                print(f"[{ts()}] (timeout — no data from lidar)")
                continue

            rx_bytes += len(chunk)
            buf.extend(chunk)

            # Scan for 0x59 0x59 headers in the buffer
            while len(buf) >= 9:
                if buf[0] != 0x59:
                    buf.pop(0)
                    continue
                if buf[1] != 0x59:
                    buf.pop(0)
                    continue
                # We have a candidate frame
                frame = buf[:9]
                csum = sum(frame[:8]) & 0xFF
                if csum != frame[8]:
                    ck_errors += 1
                    buf.pop(0)        # resync
                    continue

                # Valid frame
                good_frames += 1
                dist = struct.unpack_from("<H", frame, 2)[0]
                amp  = struct.unpack_from("<H", frame, 4)[0]
                raw_temp = struct.unpack_from("<H", frame, 6)[0]
                temp_c = raw_temp / 8.0 - 256.0

                valid = (amp >= AMP_MIN and amp != 0xFFFF
                         and MIN_CM <= dist <= MAX_CM)
                flag = "VALID" if valid else "INVALID"

                print(
                    f"[{ts()}] frame={good_frames:5d}  "
                    f"dist={dist:4d} cm ({dist/100:.2f} m)  "
                    f"amp={amp:5d}  temp={temp_c:5.1f}°C  [{flag}]"
                )
                if not valid:
                    if amp < AMP_MIN:
                        print(f"          → amplitude {amp} < {AMP_MIN}: no target or too far")
                    elif amp == 0xFFFF:
                        print("          → amplitude=0xFFFF: saturation (target too close/reflective)")
                    elif dist < MIN_CM:
                        print(f"          → dist {dist} cm < {MIN_CM} cm: below minimum range")
                    elif dist > MAX_CM:
                        print(f"          → dist {dist} cm > {MAX_CM} cm: beyond max range")

                buf = buf[9:]    # consume the frame

            # Periodic stats
            now = time.time()
            if now - last_print >= stats_interval:
                print(
                    f"\n  --- Stats: rx_bytes={rx_bytes}  "
                    f"good_frames={good_frames}  "
                    f"ck_errors={ck_errors} ---\n"
                )
                last_print = now

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print(
            f"\nFinal: rx_bytes={rx_bytes}  "
            f"good_frames={good_frames}  "
            f"ck_errors={ck_errors}"
        )


# ─── entry point ─────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="TF-Luna lidar debug tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument(
        "--direct",
        metavar="PORT",
        default=None,
        help=(
            "Read raw TF-Luna frames from PORT (e.g. /dev/ttyUSB0) using a "
            "USB-UART adapter wired straight to the lidar. "
            "Without this flag the tool reads MAVLink from the FC USB port."
        ),
    )
    args = ap.parse_args()

    if args.direct:
        run_direct(args.direct)
    else:
        run_mavlink()


if __name__ == "__main__":
    main()
