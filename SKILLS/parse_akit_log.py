#!/usr/bin/env python3
"""Real WPILOG (.wpilog) parser for AdvantageKit / DataLogManager logs.

Implements the WPILib Data Log File Format Specification v1.0
(allwpilib/wpiutil/doc/datalog.adoc). Pure stdlib — no bindings needed.

Every number this script prints is computed from the bytes of the log file.
It never prints canned metrics; if an analysis can't run (entry missing,
unsupported type), it says so and the claim stays unverified.

Usage:
  python SKILLS/parse_akit_log.py <log.wpilog>                 # entry summary
  python SKILLS/parse_akit_log.py <log.wpilog> --grep Vision   # filter entries
  python SKILLS/parse_akit_log.py <log.wpilog> --pose-entry "/RealOutputs/Odometry/Robot"
  python SKILLS/parse_akit_log.py <log.wpilog> --dump "/RealOutputs/Vision/IMUMode"

Note: entry names include their leading slash exactly as printed in the
summary table (e.g. "/RealOutputs/Vision/IMUMode", not "Vision/IMUMode").
--dump/--pose-entry do an exact match against that name.

SIM mode writes a WPILOGWriter to logs/ (Robot.java), same as REAL/REPLAY,
so a headless or interactive sim run produces a .wpilog here too.
"""

import argparse
import math
import struct
import sys
from pathlib import Path

# Windows consoles/pipes often default to cp1252, which can't encode emoji.
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except (AttributeError, ValueError):
        pass

CONTROL_START, CONTROL_FINISH, CONTROL_SET_METADATA = 0, 1, 2


class Entry:
    def __init__(self, entry_id, name, type_str, metadata):
        self.id = entry_id
        self.name = name
        self.type = type_str
        self.metadata = metadata
        self.records = []  # (timestamp_us, payload bytes)


def read_varint(buf, off, length):
    return int.from_bytes(buf[off:off + length], "little"), off + length


def read_string4(buf, off):
    n, off = read_varint(buf, off, 4)
    return buf[off:off + n].decode("utf-8", errors="replace"), off + n


def parse_log(buf):
    if len(buf) < 12 or buf[:6] != b"WPILOG":
        raise ValueError("not a WPILOG file (bad magic)")
    version = int.from_bytes(buf[6:8], "little")
    extra_len = int.from_bytes(buf[8:12], "little")
    off = 12 + extra_len

    active = {}       # id -> Entry currently accepting data records
    all_entries = []  # every Start creates a new Entry; Finish only deactivates,
                      # because the spec allows entry-ID reuse after a Finish
    skipped = 0
    while off < len(buf):
        bitfield = buf[off]
        id_len = (bitfield & 0x3) + 1
        size_len = ((bitfield >> 2) & 0x3) + 1
        ts_len = ((bitfield >> 4) & 0x7) + 1
        off += 1
        entry_id, off = read_varint(buf, off, id_len)
        payload_size, off = read_varint(buf, off, size_len)
        timestamp, off = read_varint(buf, off, ts_len)
        payload = buf[off:off + payload_size]
        if len(payload) < payload_size:
            print(f"⚠️  truncated record at byte {off}; stopping parse.")
            break
        off += payload_size

        if entry_id == 0:
            ctrl = payload[0]
            if ctrl == CONTROL_START:
                target = int.from_bytes(payload[1:5], "little")
                p = 5
                name, p = read_string4(payload, p)
                type_str, p = read_string4(payload, p)
                metadata, p = read_string4(payload, p)
                e = Entry(target, name, type_str, metadata)
                active[target] = e
                all_entries.append(e)
            elif ctrl == CONTROL_FINISH:
                active.pop(int.from_bytes(payload[1:5], "little"), None)
            # SetMetadata: nothing needed for our analyses
        elif entry_id in active:
            active[entry_id].records.append((timestamp, payload))
        else:
            skipped += 1

    return version, all_entries, skipped


def decode_scalar(type_str, payload):
    try:
        if type_str == "double" and len(payload) == 8:
            return struct.unpack("<d", payload)[0]
        if type_str == "float" and len(payload) == 4:
            return struct.unpack("<f", payload)[0]
        if type_str == "int64" and len(payload) == 8:
            return struct.unpack("<q", payload)[0]
        if type_str == "boolean" and len(payload) == 1:
            return bool(payload[0])
        if type_str == "string":
            return payload.decode("utf-8", errors="replace")
        if type_str == "double[]" and len(payload) % 8 == 0:
            return list(struct.unpack(f"<{len(payload)//8}d", payload))
        if type_str == "boolean[]":
            return [bool(b) for b in payload]
        if type_str == "int64[]" and len(payload) % 8 == 0:
            return list(struct.unpack(f"<{len(payload)//8}q", payload))
        if type_str == "float[]" and len(payload) % 4 == 0:
            return list(struct.unpack(f"<{len(payload)//4}f", payload))
        if type_str == "string[]" and len(payload) >= 4:
            n = int.from_bytes(payload[:4], "little")
            off, out = 4, []
            for _ in range(n):
                s, off = read_string4(payload, off)
                out.append(s)
            return out
    except (struct.error, IndexError, UnicodeDecodeError):
        pass
    return None  # unsupported / struct types stay raw


def decode_pose(type_str, payload):
    """Pose2d as struct:Pose2d (24 bytes: x, y, theta doubles) or double[3]."""
    if type_str == "struct:Pose2d" and len(payload) == 24:
        return struct.unpack("<3d", payload)
    if type_str == "double[]" and len(payload) == 24:
        return struct.unpack("<3d", payload)
    return None


def pose_jump_analysis(entry, threshold):
    poses = []
    for ts, payload in entry.records:
        p = decode_pose(entry.type, payload)
        if p is not None:
            poses.append((ts, p))
    if len(poses) < 2:
        print(f"❌ Cannot analyze '{entry.name}': {len(poses)} decodable pose "
              f"sample(s) (type '{entry.type}'). Claim stays UNVERIFIED.")
        return 1
    worst = (0.0, None)
    over = 0
    for (t0, a), (t1, b) in zip(poses, poses[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if d > threshold:
            over += 1
        if d > worst[0]:
            worst = (d, t1)
    print(f"\n📈 Pose-jump analysis: '{entry.name}' "
          f"({len(poses)} samples, {poses[0][0]/1e6:.2f}s → {poses[-1][0]/1e6:.2f}s)")
    print(f"   max frame-to-frame translation delta: {worst[0]:.4f} m "
          f"at t={worst[1]/1e6:.2f}s")
    print(f"   deltas over {threshold:.2f} m threshold: {over}")
    if over:
        print("   ❌ FAIL: pose snapping present in this log.")
        return 1
    print("   ✅ PASS: no frame-to-frame jump exceeded the threshold in this log.")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--grep", help="only list entries whose name contains this")
    ap.add_argument("--dump", help="print decoded values for this exact entry name")
    ap.add_argument("--limit", type=int, default=20, help="max values to dump")
    ap.add_argument("--pose-entry",
                    help="entry name to run pose-jump analysis on")
    ap.add_argument("--jump-threshold", type=float, default=0.15,
                    help="max acceptable frame-to-frame pose delta, meters")
    args = ap.parse_args()

    path = Path(args.log)
    if not path.is_file():
        print(f"❌ log file not found: {path}")
        sys.exit(1)
    try:
        version, all_entries, skipped = parse_log(path.read_bytes())
    except ValueError as e:
        print(f"❌ {e}")
        sys.exit(1)

    # Merge same-name restarts (entry-ID reuse) and sort each entry's records —
    # the spec does not guarantee records appear in timestamp order.
    entries = {}
    for e in all_entries:
        m = entries.get(e.name)
        if m is not None and m.type == e.type:
            m.records.extend(e.records)
        else:
            entries[e.name if m is None else f"{e.name} ({e.type})"] = e
    for e in entries.values():
        e.records.sort(key=lambda r: r[0])

    total = sum(len(e.records) for e in entries.values())
    all_ts = [ts for e in entries.values() for ts, _ in e.records]
    span = f"{min(all_ts)/1e6:.2f}s → {max(all_ts)/1e6:.2f}s" if all_ts else "empty"
    print(f"🔍 {path.name}: WPILOG v{version >> 8}.{version & 0xFF}, "
          f"{len(entries)} entries, {total} data records, span {span}"
          + (f", {skipped} orphan records" if skipped else ""))

    shown = sorted(entries.values(), key=lambda e: -len(e.records))
    if args.grep:
        shown = [e for e in shown if args.grep.lower() in e.name.lower()]
    print(f"\n{'records':>8}  {'type':<22} name")
    for e in shown[:40]:
        print(f"{len(e.records):>8}  {e.type:<22} {e.name}")
    if len(shown) > 40:
        print(f"   ... {len(shown) - 40} more (use --grep to filter)")

    rc = 0
    by_name = entries

    if args.dump:
        e = by_name.get(args.dump)
        if e is None:
            print(f"❌ no entry named '{args.dump}'")
            rc = 1
        else:
            print(f"\n📋 {e.name} ({e.type}), first {args.limit} of {len(e.records)}:")
            for ts, payload in e.records[:args.limit]:
                val = decode_scalar(e.type, payload)
                print(f"   t={ts/1e6:9.3f}s  "
                      f"{val if val is not None else payload.hex()[:48] + ' (raw)'}")

    if args.pose_entry:
        e = by_name.get(args.pose_entry)
        if e is None:
            print(f"❌ no entry named '{args.pose_entry}'; pose analysis UNVERIFIED.")
            rc = 1
        else:
            rc = max(rc, pose_jump_analysis(e, args.jump_threshold))

    sys.exit(rc)


if __name__ == "__main__":
    main()
