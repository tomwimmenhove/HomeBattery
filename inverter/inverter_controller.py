#!/usr/bin/env python3
"""
inverter_controller_sse.py

Listen to an SSE stream that emits DSMR telegram JSON objects and print
the integer wattage the inverter should output to reach net-zero.

Key behavior:
- Forces HTTP/1.1 (http2=False) and sets Accept: text/event-stream
- Tolerant SSE parsing: supports "data: ..." lines and raw JSON lines separated by blank lines
- Keeps last_output_w state and computes:
    desired = last_output_w + (import_kW - export_kW) * 1000
  then clamps to [min_output_w, max_output_w]
- Sends (prints) a new integer watt value to stdout only when change >= hysteresis
- Logs human-readable state to stderr
- Auto-reconnects with exponential backoff
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import sys
from typing import AsyncGenerator, List

import httpx


# ---------- SSE helpers ----------
async def sse_event_generator(resp: httpx.Response) -> AsyncGenerator[str, None]:
    """
    Yield payload strings for each SSE event.

    - Accepts standard SSE "data:" lines (possibly multiple per event).
    - Accepts non-standard raw JSON lines (server sends JSON\n\n).
    - Yields combined payload string when a blank line is received.
    """
    buf: List[str] = []
    async for raw_line in resp.aiter_lines():
        if raw_line is None:
            continue
        line = raw_line.rstrip("\r\n")
        # skip SSE comments
        if line.startswith(":"):
            continue
        if line == "":
            # blank line -> event boundary
            if buf:
                payload = "\n".join(buf)
                buf = []
                yield payload
            else:
                continue
        else:
            if line.startswith("data:"):
                buf.append(line[5:].lstrip())
            else:
                # treat non-data line (likely raw JSON) as payload content
                buf.append(line)


# ---------- JSON helpers ----------
def safe_get_power_kW(telegram: dict, key: str) -> float:
    """
    Extract numeric value from telegram['fields'][key], defensively.
    Accepts structures like:
      fields[key] == [[{"value": 0.583, "unit":"kW"}], ...]
    Returns 0.0 on any failure.
    """
    try:
        fields = telegram.get("fields", {})
        block = fields.get(key)
        if not block:
            return 0.0
        first = block[0]
        # case: first is a list whose first element has 'value'
        if isinstance(first, list) and first:
            cand = first[0]
            if isinstance(cand, dict) and "value" in cand:
                return float(cand["value"])
        # case: first is a dict with 'value'
        if isinstance(first, dict) and "value" in first:
            return float(first["value"])
    except Exception:
        pass
    return 0.0


# ---------- main controller ----------
async def run_controller(
    url: str,
    max_output_w: int,
    min_output_w: int,
    hysteresis_w: int,
    initial_output_w: int,
    send_initial: bool = False,
):
    logger = logging.getLogger("inverter_controller")
    last_output_w = float(initial_output_w)

    headers = {"Accept": "text/event-stream"}
    backoff = 1.0
    max_backoff = 30.0

    # Optionally send initial value immediately
    if send_initial:
        out_val = int(round(last_output_w))
        print(out_val, flush=True)
        logger.info("Initial command SENT -> %d W", out_val)

    while True:
        logger.info("Connecting to SSE %s (http2 disabled)", url)
        try:
            async with httpx.AsyncClient(http2=False, headers=headers, timeout=None) as client:
                async with client.stream("GET", url) as resp:
                    resp.raise_for_status()
                    logger.info("Connected (status=%s). Listening for events...", resp.status_code)
                    backoff = 1.0  # reset backoff on success
                    async for payload in sse_event_generator(resp):
                        payload = (payload or "").strip()
                        if not payload:
                            continue

                        # Try parse JSON payload; if not JSON, just log and skip
                        try:
                            telegram = json.loads(payload)
                        except Exception:
                            logger.debug("Non-JSON payload received; printing raw and continuing.")
                            # Print raw payload to stderr for inspection but skip control logic
                            print(payload, file=sys.stderr, flush=True)
                            continue

                        # Extract import/export (kW)
                        import_kW = safe_get_power_kW(telegram, "active_power_import_kW")
                        export_kW = safe_get_power_kW(telegram, "active_power_export_kW")

                        net_kW = import_kW - export_kW
                        delta_w = net_kW * 1000.0  # convert kW -> W
                        unclamped_desired = last_output_w + delta_w
                        desired = max(min_output_w, min(max_output_w, unclamped_desired))

                        change = desired - last_output_w
                        abs_change = abs(change)

                        # timestamp for logging
                        ts = telegram.get("received_at")
                        if not ts:
                            # try fallback into fields/meter_timestamp
                            try:
                                ts = telegram.get("fields", {}).get("meter_timestamp", [[{"timestamp": ""}]])[0][0].get("timestamp", "")
                            except Exception:
                                ts = ""

                        logger.info(
                            "ts=%s import=%.3f kW export=%.3f kW net=%.3f kW delta=%.0f W unclamped=%.0f W clipped=%.0f W last=%.0f W change=%.0f W",
                            ts, import_kW, export_kW, net_kW, delta_w, unclamped_desired, desired, last_output_w, change,
                        )

                        if abs_change >= hysteresis_w:
                            out_val = int(round(desired))
                            # stdout: raw integer — the only machine-readable command output
                            print(out_val, flush=True)
                            last_output_w = float(out_val)
                            logger.info("Command SENT -> %d W (hysteresis %d W)", out_val, hysteresis_w)
                        else:
                            logger.debug("Change %.0f W < hysteresis %d W — not sending", abs_change, hysteresis_w)

        except (httpx.ConnectError, httpx.ReadTimeout, httpx.RemoteProtocolError, httpx.HTTPError) as e:
            logger.error("Connection / HTTP error: %s", e)
        except asyncio.CancelledError:
            logger.info("Cancelled; exiting")
            raise
        except Exception as e:
            logger.exception("Unexpected error: %s", e)

        # reconnect/backoff
        logger.info("Reconnecting in %.1f seconds...", backoff)
        await asyncio.sleep(backoff)
        backoff = min(backoff * 2.0, max_backoff)


# ---------- CLI ----------
def parse_args():
    p = argparse.ArgumentParser(description="Inverter controller (SSE, hysteresis, stdout raw watts)")
    p.add_argument("--url", "-u", default="http://localhost:8000/stream", help="SSE stream URL")
    p.add_argument("--max", "-M", dest="max_output", type=int, default=700, help="Maximum inverter output (W)")
    p.add_argument("--min", "-m", dest="min_output", type=int, default=0, help="Minimum inverter output (W)")
    # don't use -h short option (reserved for help). use -H instead.
    p.add_argument("--hysteresis", "-H", dest="hysteresis", type=int, default=50, help="Hysteresis in watts (default 50)")
    p.add_argument("--initial", "-i", dest="initial", type=int, default=0, help="Initial last_output in watts")
    p.add_argument("--send-initial", action="store_true", help="Send initial value immediately at startup (print to stdout)")
    p.add_argument("--verbose", "-v", action="store_true", help="Verbose logging to stderr")
    return p.parse_args()


def main():
    args = parse_args()
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(stream=sys.stderr, level=level, format="%(asctime)s %(levelname)s %(message)s")

    if args.min_output > args.max_output:
        print("Error: min output cannot be greater than max output", file=sys.stderr)
        sys.exit(2)

    try:
        asyncio.run(
            run_controller(
                url=args.url,
                max_output_w=args.max_output,
                min_output_w=args.min_output,
                hysteresis_w=args.hysteresis,
                initial_output_w=args.initial,
                send_initial=args.send_initial,
            )
        )
    except KeyboardInterrupt:
        print("Interrupted", file=sys.stderr)
        sys.exit(0)


if __name__ == "__main__":
    main()
