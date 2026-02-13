#!/usr/bin/env python3
"""
Test client for the UR5 External Control API.

Exercises every endpoint with example payloads so you can quickly verify
the server is working.  Run with --dry-run to skip actual motion commands.

Usage:
    python3 test_external_api.py                   # full test
    python3 test_external_api.py --dry-run          # read-only endpoints only
    python3 test_external_api.py --base-url http://192.168.1.50:5050
"""

import argparse
import json
import sys
import urllib.request
import urllib.error


def req(method: str, url: str, body: dict | None = None) -> dict:
    """Send an HTTP request and return the parsed JSON response."""
    data = json.dumps(body).encode() if body else None
    r = urllib.request.Request(url, data=data, method=method)
    r.add_header("Content-Type", "application/json")
    r.add_header("Accept", "application/json")
    try:
        with urllib.request.urlopen(r, timeout=15) as resp:
            return json.loads(resp.read())
    except urllib.error.HTTPError as e:
        return json.loads(e.read())
    except Exception as e:
        return {"success": False, "message": str(e)}


def pprint(label: str, data: dict):
    print(f"\n{'─'*60}")
    print(f"  {label}")
    print(f"{'─'*60}")
    print(json.dumps(data, indent=2))


def main():
    ap = argparse.ArgumentParser(description="Test the UR5 External Control API")
    ap.add_argument("--base-url", default="http://localhost:5050", help="API base URL")
    ap.add_argument("--dry-run", action="store_true", help="Only test read-only endpoints")
    args = ap.parse_args()
    base = args.base_url.rstrip("/")

    # ── Read-only endpoints ───────────────────────────────────────────────
    pprint("GET /api/commands", req("GET", f"{base}/api/commands"))
    pprint("GET /api/status",   req("GET", f"{base}/api/status"))

    if args.dry_run:
        print("\n✅ Dry-run complete — skipping motion commands.")
        return

    # ── Program control ───────────────────────────────────────────────────
    pprint(
        "POST /api/program/execute",
        req("POST", f"{base}/api/program/execute", {"program": "move_to_home.prog"}),
    )

    import time
    time.sleep(2)

    pprint(
        "POST /api/program/pause",
        req("POST", f"{base}/api/program/pause"),
    )

    time.sleep(1)

    pprint(
        "POST /api/program/resume",
        req("POST", f"{base}/api/program/resume"),
    )

    time.sleep(1)

    pprint(
        "POST /api/program/stop",
        req("POST", f"{base}/api/program/stop"),
    )

    # ── Gripper ───────────────────────────────────────────────────────────
    pprint(
        "POST /api/gripper  (open)",
        req("POST", f"{base}/api/gripper", {"action": "open"}),
    )
    time.sleep(1)

    pprint(
        "POST /api/gripper  (close)",
        req("POST", f"{base}/api/gripper", {"action": "close"}),
    )
    time.sleep(1)

    pprint(
        "POST /api/gripper  (position 0.5)",
        req("POST", f"{base}/api/gripper", {"action": "position", "position": 0.5}),
    )

    # ── Named position ────────────────────────────────────────────────────
    pprint(
        "POST /api/move/named  (Home)",
        req("POST", f"{base}/api/move/named", {"name": "Home", "duration": 3.0}),
    )

    time.sleep(4)

    pprint(
        "POST /api/move/named  (TableCenter — Cartesian via cuRobo)",
        req("POST", f"{base}/api/move/named", {"name": "TableCenter"}),
    )

    # ── Raw joints ────────────────────────────────────────────────────────
    time.sleep(4)
    pprint(
        "POST /api/move/joints",
        req("POST", f"{base}/api/move/joints", {
            "positions": [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],
            "duration": 3.0,
        }),
    )

    # ── Cartesian pose ────────────────────────────────────────────────────
    time.sleep(4)
    pprint(
        "POST /api/move/pose",
        req("POST", f"{base}/api/move/pose", {
            "position": [0.4, 0.0, 0.4],
            "quaternion": [0.0, 1.0, 0.0, 0.0],
        }),
    )

    print("\n✅ All tests complete.")


if __name__ == "__main__":
    main()
