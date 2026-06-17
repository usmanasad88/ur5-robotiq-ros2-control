#!/bin/bash
# ============================================================
# quest_wireless.sh — set up / restore wireless ADB to the Quest
# ============================================================
# Wireless ADB (adb tcpip 5555) only needs USB ONCE per Quest power session.
# After that, reconnects are over-the-air. This script handles both cases.
#
# Usage:
#   ./quest_wireless.sh            # auto: USB present → arm tcpip + connect + print IP
#                                  #       USB absent  → try to reconnect last/known IP
#   ./quest_wireless.sh <ip>       # reconnect to a known IP (no USB needed, same boot)
#
# On success it prints the line to copy, e.g.:
#   export QUEST_IP=192.168.18.133
# then run:  QUEST_IP=$QUEST_IP ./run_quest_rtde_teleop.sh ...
# ============================================================

set -uo pipefail
PORT=5555
CACHE="${HOME}/.cache/quest_wireless_ip"

have_adb() { command -v adb >/dev/null 2>&1; }
if ! have_adb; then echo "ERROR: adb not found. sudo apt install android-tools-adb"; exit 1; fi

adb start-server >/dev/null 2>&1

# A USB device shows a serial WITHOUT a ':' ; a wireless one looks like ip:port
usb_serial() {
    adb devices | awk 'NR>1 && $2=="device" && $1 !~ /:/ {print $1; exit}'
}
wireless_connected() {  # arg: ip
    adb devices | grep -q "^$1:$PORT[[:space:]]\+device"
}

try_connect() {  # arg: ip  → arms-free reconnect (same power session)
    local ip="$1"
    echo "Connecting to $ip:$PORT ..."
    adb connect "$ip:$PORT" >/dev/null 2>&1
    if wireless_connected "$ip"; then
        echo "$ip" > "$CACHE"
        echo "Wireless ADB OK."
        echo
        echo "  export QUEST_IP=$ip"
        return 0
    fi
    return 1
}

IP_ARG="${1:-}"

# Case 1: explicit IP given → just reconnect (no USB)
if [[ -n "$IP_ARG" ]]; then
    try_connect "$IP_ARG" && exit 0
    echo "Reconnect to $IP_ARG failed (Quest may have powered off → needs USB once)."
    exit 1
fi

# Case 2: USB present → arm tcpip, read IP, connect (the one-time-per-boot step)
USB="$(usb_serial)"
if [[ -n "$USB" ]]; then
    echo "USB device $USB found — arming wireless ADB ..."
    IP="$(adb -s "$USB" shell ip route 2>/dev/null | grep -oE 'src [0-9.]+' | awk '{print $2}' | head -1)"
    if [[ -z "$IP" ]]; then echo "Could not read Quest wlan0 IP (is Wi-Fi on?)"; exit 1; fi
    adb -s "$USB" tcpip "$PORT" >/dev/null 2>&1
    sleep 2
    try_connect "$IP" && { echo "  (you can unplug USB now)"; exit 0; }
    echo "tcpip armed but connect failed — check Quest and host are on the same network."
    exit 1
fi

# Case 3: no USB, no IP arg → try the cached IP
if [[ -f "$CACHE" ]]; then
    CACHED="$(cat "$CACHE")"
    echo "No USB; trying last known IP $CACHED ..."
    try_connect "$CACHED" && exit 0
fi
echo "No USB device and no reachable wireless Quest."
echo "Connect the Quest via USB once and re-run, or pass a known IP:"
echo "  ./quest_wireless.sh 192.168.18.133"
exit 1
