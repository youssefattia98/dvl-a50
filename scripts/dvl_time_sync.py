#!/usr/bin/env python3
"""Synchronize a Water Linked A50 clock before starting its ROS driver."""

import argparse
import datetime
import json
import sys
import time
import urllib.error
import urllib.request


MAX_CLOCK_SKEW_SECONDS = 5.0


def request_json(url, *, data=None, timeout=3.0):
    body = None if data is None else json.dumps(data).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=body,
        headers={"Content-Type": "application/json"},
        method="GET" if body is None else "POST",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        payload = response.read()
        return json.loads(payload) if payload else {}


def enabled(value):
    return value.strip().lower() in {"1", "true", "yes", "on"}


def clock_skew_seconds(status):
    system_time = status.get("system_time", "")
    if not system_time:
        return None
    parsed_time = datetime.datetime.fromisoformat(system_time.replace("Z", "+00:00"))
    return abs(
        (datetime.datetime.now(datetime.timezone.utc) - parsed_time).total_seconds()
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", required=True)
    parser.add_argument("--enabled", default="true")
    args = parser.parse_args()

    if not enabled(args.enabled):
        print("[dvl_time_sync] Clock synchronization disabled", flush=True)
        return 0

    base_url = f"http://{args.ip}/api/v1/integration/time"
    status_url = f"{base_url}/status"
    try:
        status = request_json(status_url)
        skew = clock_skew_seconds(status)
        clock_is_fresh = skew is not None and skew <= MAX_CLOCK_SKEW_SECONDS
        if status.get("ntp_synced", False) and clock_is_fresh:
            print(
                f"[dvl_time_sync] DVL clock is synchronized by "
                f"{status.get('ntp_synced_to', 'NTP')} (skew {skew:.3f} s)",
                flush=True,
            )
            return 0

        if status.get("ntp_synced", False):
            request_json(
                f"{base_url}/ntp/force-sync",
                data={"timeout_seconds": 10},
                timeout=15.0,
            )
            status = request_json(status_url)
            skew = clock_skew_seconds(status)
            if skew is not None and skew <= MAX_CLOCK_SKEW_SECONDS:
                print(
                    f"[dvl_time_sync] Forced DVL NTP synchronization "
                    f"(skew {skew:.3f} s)",
                    flush=True,
                )
                return 0
            raise RuntimeError(
                "DVL reports NTP synchronization but its clock remains stale"
            )

        now = datetime.datetime.now(datetime.timezone.utc).isoformat()
        request_json(f"{base_url}/manual", data={"now": now})
        print(f"[dvl_time_sync] Synchronized DVL clock to this computer: {now}", flush=True)

        # A time change can briefly restart the DVL web service. Wait for it so
        # the data driver does not mistake that restart for a disconnected DVL.
        for _ in range(20):
            try:
                status = request_json(status_url, timeout=1.0)
                skew = clock_skew_seconds(status)
                if skew is None or skew <= MAX_CLOCK_SKEW_SECONDS:
                    return 0
                time.sleep(0.5)
                continue
            except (OSError, urllib.error.URLError, TimeoutError):
                time.sleep(0.5)
        print("[dvl_time_sync] DVL web service is still restarting; continuing", flush=True)
    except Exception as error:
        # A missing DVL must never prevent the bridge and sonar from launching.
        print(f"[dvl_time_sync] DVL clock sync skipped: {error}", file=sys.stderr, flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
