#!/usr/bin/env python3
"""Synchronize a Water Linked A50 clock before starting its ROS driver."""

import argparse
import datetime
import json
import sys
import time
import urllib.error
import urllib.request


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", required=True)
    parser.add_argument("--enabled", default="true")
    args = parser.parse_args()

    if not enabled(args.enabled):
        print("[dvl_time_sync] Clock synchronization disabled", flush=True)
        return 0

    base_url = f"http://{args.ip}/api/v1/time"
    try:
        status = request_json(base_url)
        if status.get("ntp_synchronized", False):
            print(
                f"[dvl_time_sync] DVL clock is synchronized by "
                f"{status.get('ntp_server', 'NTP')}",
                flush=True,
            )
            return 0

        now = datetime.datetime.now(datetime.timezone.utc).isoformat()
        request_json(f"{base_url}/manual", data={"time": now})
        print(f"[dvl_time_sync] Synchronized DVL clock to this computer: {now}", flush=True)

        # A time change can briefly restart the DVL web service. Wait for it so
        # the data driver does not mistake that restart for a disconnected DVL.
        for _ in range(20):
            try:
                request_json(base_url, timeout=1.0)
                return 0
            except (OSError, urllib.error.URLError, TimeoutError):
                time.sleep(0.5)
        print("[dvl_time_sync] DVL web service is still restarting; continuing", flush=True)
    except Exception as error:
        # A missing DVL must never prevent the bridge and sonar from launching.
        print(f"[dvl_time_sync] DVL clock sync skipped: {error}", file=sys.stderr, flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
