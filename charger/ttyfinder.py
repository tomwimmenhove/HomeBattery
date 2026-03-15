#!/usr/bin/env python3
"""
ttyfinder.py

Utility functions to find which /dev/tty* device corresponds to a USB
device by vendor:product id (VID:PID). Intended to be imported by
other scripts in the same directory.

API:
    find_tty_for_vidpid(vidpid, patterns=None, timeout=0, poll_interval=0.1)
        -> returns a single device path string like '/dev/ttyUSB0' or None.

    find_all_ttys_for_vidpid(vidpid, patterns=None)
        -> returns a list of dicts with details for every match.

vidpid may be either 'vvvv:pppp' or two separate hex pieces separated
(e.g. '1a86:7523' or '1a86', '7523' via helper).
"""

from pathlib import Path
import glob
import time
import os
from typing import List, Optional, Dict, Tuple

DEFAULT_PATTERNS = ['/dev/ttyUSB*', '/dev/ttyACM*']

def _read_txt(p: Path) -> Optional[str]:
    try:
        return p.read_text().strip().lower()
    except Exception:
        return None

def _find_vid_pid_in_ancestors(sysfs_path: Path) -> Tuple[Optional[str], Optional[str], Optional[Path]]:
    """
    Walk up from sysfs_path to /sys looking for idVendor and idProduct files.
    Return (vid, pid, matched_path) or (None, None, None).
    """
    p = sysfs_path.resolve()
    sys_root = Path('/sys').resolve()
    while True:
        idv = (p / 'idVendor')
        idp = (p / 'idProduct')
        if idv.exists() and idp.exists():
            vid = _read_txt(idv)
            pid = _read_txt(idp)
            return vid, pid, p
        if p == sys_root or p.parent == p:
            break
        p = p.parent
    return None, None, None

def find_all_ttys_for_vidpid(vidpid: str, patterns: Optional[List[str]] = None) -> List[Dict]:
    """
    Return list of matches. Each match is a dict:
      {
        'dev': '/dev/ttyUSB0',
        'tty': 'ttyUSB0',
        'sysfs_match': '/sys/devices/.../1-1.3',
        'idVendor': '1a86',
        'idProduct': '7523',
        'busid': '1-1.3'  # when available
      }
    """
    if patterns is None:
        patterns = DEFAULT_PATTERNS

    # Normalize input
    vp = vidpid.strip().lower()
    if ':' in vp:
        want_vid, want_pid = (x.zfill(4) for x in vp.split(':', 1))
    else:
        raise ValueError("vidpid must be in form 'vvvv:pppp' (hex).")

    results = []
    seen = set()

    for pattern in patterns:
        for devpath in glob.glob(pattern):
            if not os.path.exists(devpath):
                continue
            dev = Path(devpath)
            name = dev.name  # e.g. ttyUSB0
            sys_class_tty = Path('/sys/class/tty') / name

            # primary sysfs device location for tty is /sys/class/tty/<name>/device
            sysfs_tty = sys_class_tty / 'device'
            if not sysfs_tty.exists():
                # try the class entry itself (some layouts differ)
                if not sys_class_tty.exists():
                    continue
                # sometimes the device symlink is not present; skip if we can't find the device
                # (this is rare for usb serial)
                continue

            vid, pid, matched = _find_vid_pid_in_ancestors(sysfs_tty)
            if vid is None or pid is None:
                continue

            vid_norm = vid.zfill(4)
            pid_norm = pid.zfill(4)
            if vid_norm == want_vid and pid_norm == want_pid:
                busid = matched.name if matched is not None else None
                key = (str(dev), str(matched))
                if key in seen:
                    continue
                seen.add(key)
                results.append({
                    'dev': str(dev),
                    'tty': name,
                    'sysfs_match': str(matched) if matched is not None else None,
                    'idVendor': vid,
                    'idProduct': pid,
                    'busid': busid,
                })
    return results

def find_tty_for_vidpid(vidpid: str, patterns: Optional[List[str]] = None,
                        timeout: float = 0.0, poll_interval: float = 0.1) -> Optional[str]:
    """
    Return the first matching /dev path for vidpid (e.g. '1a86:7523').
    If timeout > 0, poll until timeout seconds for a match to appear.
    Returns string like '/dev/ttyUSB0' or None if not found.
    """
    deadline = time.time() + float(timeout) if timeout and timeout > 0 else None
    while True:
        matches = find_all_ttys_for_vidpid(vidpid, patterns)
        if matches:
            return matches[0]['dev']
        if deadline is None:
            return None
        if time.time() >= deadline:
            return None
        time.sleep(poll_interval)


# Simple CLI for debugging if run directly
if __name__ == '__main__':
    import argparse, json
    ap = argparse.ArgumentParser(description="Find /dev/tty* for a vendor:product")
    ap.add_argument('vidpid', help="vendor:product e.g. 1a86:7523")
    ap.add_argument('--json', action='store_true')
    ap.add_argument('--timeout', type=float, default=0.0, help='seconds to wait for device (default 0)')
    args = ap.parse_args()

    dev = find_tty_for_vidpid(args.vidpid, timeout=args.timeout)
    if dev is None:
        print(f"No match for {args.vidpid}", flush=True)
        raise SystemExit(2)
    if args.json:
        print(json.dumps({'dev': dev}))
    else:
        print(dev)
