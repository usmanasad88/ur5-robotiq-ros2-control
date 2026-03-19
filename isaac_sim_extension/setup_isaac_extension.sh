#!/usr/bin/env bash
# setup_isaac_extension.sh
#
# Installs the UR5 HCDT extension into Isaac Sim by:
#   1. Copying the Python package into isaacsim.examples.interactive
#   2. Registering it as a python module in that extension's extension.toml
#   3. Exporting UR_WS_PATH in your shell RC so USD assets can be found
#
# Usage:
#   ./setup_isaac_extension.sh [--isaac-sim-path /path/to/isaac-sim] [--pull]
#
# Default Isaac Sim path: ~/isaac-sim-standalone-5.0.0-linux-x86_64
#
# The script is idempotent: running it multiple times has no ill effect.
# Use --pull to copy changes from the Isaac Sim installation back to the source directory.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_PKG="${SCRIPT_DIR}/ur_robotiq_cortex/ur_robotiq_cortex"   # Python package source

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
ISAAC_SIM_PATH=""
PULL_TO_SOURCE=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --isaac-sim-path)
            ISAAC_SIM_PATH="$2"
            shift 2
            ;;
        --pull)
            PULL_TO_SOURCE=1
            shift
            ;;
        -h|--help)
            grep '^#' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

# ---------------------------------------------------------------------------
# Auto-detect Isaac Sim
# ---------------------------------------------------------------------------
if [[ -z "${ISAAC_SIM_PATH}" ]]; then
    DEFAULT_PATH="${HOME}/isaac-sim-standalone-5.0.0-linux-x86_64"
    if [[ -d "${DEFAULT_PATH}" ]]; then
        ISAAC_SIM_PATH="${DEFAULT_PATH}"
    else
        echo "ERROR: Could not auto-detect Isaac Sim. Pass --isaac-sim-path."
        exit 1
    fi
fi

if [[ ! -d "${ISAAC_SIM_PATH}" ]]; then
    echo "ERROR: Isaac Sim directory not found: ${ISAAC_SIM_PATH}"
    exit 1
fi
echo "[setup] Found Isaac Sim at: ${ISAAC_SIM_PATH}"

# ---------------------------------------------------------------------------
# Paths inside the isaacsim.examples.interactive extension
# ---------------------------------------------------------------------------
INTERACTIVE_EXT="${ISAAC_SIM_PATH}/exts/isaacsim.examples.interactive"
INTERACTIVE_PKG="${INTERACTIVE_EXT}/isaacsim/examples/interactive"
TOML="${INTERACTIVE_EXT}/config/extension.toml"

if [[ ! -f "${TOML}" ]]; then
    echo "ERROR: extension.toml not found at ${TOML}"
    exit 1
fi

# ---------------------------------------------------------------------------
# Copy the Python package
# ---------------------------------------------------------------------------
DEST="${INTERACTIVE_PKG}/ur_robotiq_cortex"

if [[ "${PULL_TO_SOURCE}" == "1" ]]; then
    echo "[setup] Pulling changes from Isaac Sim back to source..."
    echo "[setup] Copying from: ${DEST}"
    echo "[setup] To: ${SRC_PKG}"
    
    if [[ ! -d "${DEST}" ]]; then
        echo "ERROR: Destination directory does not exist: ${DEST}"
        exit 1
    fi
    
    rm -rf "${SRC_PKG}"
    cp -r "${DEST}" "${SRC_PKG}"
    
    # Remove any cached files copied over
    find "${SRC_PKG}" -name "__pycache__" -type d -exec rm -rf {} +
    
    # Revert intra-package imports in the source copy
    for f in "${SRC_PKG}"/*.py; do
        if [[ -f "$f" ]]; then
            sed -i \
                's/from isaacsim\.examples\.interactive\.ur_robotiq_cortex/from ur_robotiq_cortex/g' \
                "$f"
            sed -i \
                's/import isaacsim\.examples\.interactive\.ur_robotiq_cortex/import ur_robotiq_cortex/g' \
                "$f"
        fi
    done
    
    echo "[setup] Reverted intra-package imports in source."
    echo "=== Pull complete ==="
    exit 0
fi

echo "[setup] Copying Python package to: ${DEST}"
rm -rf "${DEST}"
cp -r "${SRC_PKG}" "${DEST}"
echo "[setup] Copy complete."

# ---------------------------------------------------------------------------
# Fix intra-package imports to use the full dotted path
# ---------------------------------------------------------------------------
# The package now lives at isaacsim.examples.interactive.ur_robotiq_cortex.*
# so bare "from ur_robotiq_cortex." imports must become fully qualified.
for f in "${DEST}/ur_robotiq_cortex.py" "${DEST}/ur_robotiq_cortex_extension.py"; do
    if [[ -f "$f" ]]; then
        sed -i \
            's/from ur_robotiq_cortex\./from isaacsim.examples.interactive.ur_robotiq_cortex./g' \
            "$f"
        sed -i \
            's/import ur_robotiq_cortex\./import isaacsim.examples.interactive.ur_robotiq_cortex./g' \
            "$f"
    fi
done

# Update __init__.py to use the fully qualified path
cat > "${DEST}/__init__.py" << 'PYEOF'
from isaacsim.examples.interactive.ur_robotiq_cortex.ur_robotiq_cortex_extension import URRobotiqCortexExtension
PYEOF

echo "[setup] Fixed intra-package imports."

# ---------------------------------------------------------------------------
# Register the module in extension.toml (idempotent)
# ---------------------------------------------------------------------------
MODULE_NAME="isaacsim.examples.interactive.ur_robotiq_cortex"

if grep -qF "${MODULE_NAME}" "${TOML}"; then
    echo "[setup] Module already registered in extension.toml: ${MODULE_NAME}"
else
    echo "[setup] Registering module in extension.toml: ${MODULE_NAME}"
    # Insert the new [[python.module]] block just before the [[test]] section
    awk -v entry="[[python.module]]\nname = \"${MODULE_NAME}\"" \
        '/^\[\[test\]\]/ && !done { print entry; print ""; done=1 } { print }' \
        "${TOML}" > "${TOML}.tmp" && mv "${TOML}.tmp" "${TOML}"
    echo "[setup] extension.toml updated."
fi

# ---------------------------------------------------------------------------
# Export UR_WS_PATH so Isaac Sim can find the USD assets
# ---------------------------------------------------------------------------
UR_WS_PATH="$(cd "${SCRIPT_DIR}/.." && pwd)"   # parent of isaac_sim_extension = ur_ws

SHELL_RC="${HOME}/.bashrc"
if [[ "${SHELL}" == *zsh* ]]; then
    SHELL_RC="${HOME}/.zshrc"
fi

if grep -qF "UR_WS_PATH" "${SHELL_RC}" 2>/dev/null; then
    echo "[setup] UR_WS_PATH already set in ${SHELL_RC}"
else
    {
        echo ""
        echo "# Added by ur_ws/isaac_sim_extension/setup_isaac_extension.sh"
        echo "export UR_WS_PATH=\"${UR_WS_PATH}\""
    } >> "${SHELL_RC}"
    echo "[setup] Added UR_WS_PATH to ${SHELL_RC}"
fi

export UR_WS_PATH="${UR_WS_PATH}"

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
echo "=== Setup complete ==="
echo "  Installed to   : ${DEST}"
echo "  extension.toml : ${TOML}"
echo "  UR_WS_PATH     : ${UR_WS_PATH}"
echo ""
echo "Start Isaac Sim normally. 'UR5 HCDT' will appear in"
echo "Isaac Examples > Cortex after Isaac Sim loads."
echo ""
echo "If USD assets are not found, verify that the following files exist:"
echo "  \${UR_WS_PATH}/isaac_standalone/Collected_ur10e_robotiq2f-140_ROS/ur5_robotiq2f-85.usd"
echo "  \${UR_WS_PATH}/isaac_standalone/Collected_ur10e_robotiq2f-140_ROS/ur10e_robotiq2f-140_ROS.usd"
echo ""
echo "To change the robot model (default: ur5):"
echo "  export UR_ROBOT_TYPE=ur10   # before launching Isaac Sim"
echo ""
echo "To re-run after editing source files in ur_ws:"
echo "  ./setup_isaac_extension.sh"
