#!/usr/bin/env bash
# setup_isaac_extension.sh
#
# Installs the UR5 HCDT extension into Isaac Sim by:
#   1. Copying the Python package into isaacsim.examples.interactive
#   2. Registering it as a python module in that extension's extension.toml
#   3. Exporting UR_WS_PATH in your shell RC so USD assets can be found
#
# Usage:
#   ./setup_isaac_extension.sh [--isaac-sim-path /path/to/isaac-sim] [--pull] [--uninstall]
#
# Default: auto-detects Isaac Sim 6 (source build at ~/Repos/isaacsim).
#
# Revert to Isaac Sim 5.0.0 (backup on BigD):
#   ./setup_isaac_extension.sh --isaac-sim-path /media/mani/BigD/isaac-sim-standalone-5.0.0-linux-x86_64
# The script auto-detects the layout: Isaac Sim 6 keeps extensions under
# source/extensions/, Isaac Sim 5.0 under exts/.
#
# Isaac Sim 6 (source build) symlinks each extension's config/ and isaacsim/
# folders into _build, so copying our package into the source tree takes effect
# immediately -- no rebuild required.
#
# The script is idempotent: running it multiple times has no ill effect.
#   --pull       Copy changes from the Isaac Sim installation back to this source dir.
#   --uninstall  Remove the package and its extension.toml registration.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_PKG="${SCRIPT_DIR}/ur_robotiq_cortex/ur_robotiq_cortex"   # Python package source

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
ISAAC_SIM_PATH=""
PULL_TO_SOURCE=0
UNINSTALL=0
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
        --uninstall)
            UNINSTALL=1
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
# Auto-detect Isaac Sim (prefer v6 source tree, fall back to a v5 install)
# ---------------------------------------------------------------------------
has_interactive_ext() {
    [[ -d "$1/source/extensions/isaacsim.examples.interactive" ]] || \
    [[ -d "$1/exts/isaacsim.examples.interactive" ]]
}

if [[ -z "${ISAAC_SIM_PATH}" ]]; then
    for CAND in "${HOME}/Repos/isaacsim" "${HOME}/isaacsim" \
                "/media/mani/BigD/isaac-sim-standalone-5.0.0-linux-x86_64" \
                "${HOME}/isaac-sim-standalone-5.0.0-linux-x86_64"; do
        if has_interactive_ext "${CAND}"; then
            ISAAC_SIM_PATH="${CAND}"
            break
        fi
    done
    if [[ -z "${ISAAC_SIM_PATH}" ]]; then
        echo "ERROR: Could not auto-detect Isaac Sim. Pass --isaac-sim-path."
        exit 1
    fi
fi

if ! has_interactive_ext "${ISAAC_SIM_PATH}"; then
    echo "ERROR: No isaacsim.examples.interactive extension found under: ${ISAAC_SIM_PATH}"
    exit 1
fi
echo "[setup] Found Isaac Sim at: ${ISAAC_SIM_PATH}"

# ---------------------------------------------------------------------------
# Resolve the isaacsim.examples.interactive extension path for this layout
#   Isaac Sim 6  -> source/extensions/...   launch via _build/.../release/isaac-sim.sh
#   Isaac Sim 5  -> exts/...                launch via isaac-sim.sh at the root
# ---------------------------------------------------------------------------
if [[ -d "${ISAAC_SIM_PATH}/source/extensions/isaacsim.examples.interactive" ]]; then
    INTERACTIVE_EXT="${ISAAC_SIM_PATH}/source/extensions/isaacsim.examples.interactive"
    LAUNCH_HINT="${ISAAC_SIM_PATH}/_build/linux-x86_64/release/isaac-sim.sh"
else
    INTERACTIVE_EXT="${ISAAC_SIM_PATH}/exts/isaacsim.examples.interactive"
    LAUNCH_HINT="${ISAAC_SIM_PATH}/isaac-sim.sh"
fi

INTERACTIVE_PKG="${INTERACTIVE_EXT}/isaacsim/examples/interactive"
TOML="${INTERACTIVE_EXT}/config/extension.toml"
MODULE_NAME="isaacsim.examples.interactive.ur_robotiq_cortex"
DEST="${INTERACTIVE_PKG}/ur_robotiq_cortex"

if [[ ! -f "${TOML}" ]]; then
    echo "ERROR: extension.toml not found at ${TOML}"
    exit 1
fi

# ---------------------------------------------------------------------------
# Uninstall
# ---------------------------------------------------------------------------
if [[ "${UNINSTALL}" == "1" ]]; then
    echo "[setup] Uninstalling..."
    rm -rf "${DEST}"
    echo "[setup] Removed ${DEST}"
    # Drop the [[python.module]] block for our module from extension.toml.
    awk -v mod="${MODULE_NAME}" '
        $0 == "[[python.module]]" {
            buf=$0; getline nxt;
            if (nxt ~ mod) { getline blank; if (blank != "") print blank; next }
            else { print buf; print nxt; next }
        }
        { print }
    ' "${TOML}" > "${TOML}.tmp" && mv "${TOML}.tmp" "${TOML}"
    echo "[setup] Removed module registration from extension.toml"
    echo "=== Uninstall complete ==="
    exit 0
fi

# ---------------------------------------------------------------------------
# Pull changes from the Isaac Sim installation back to source
# ---------------------------------------------------------------------------
if [[ "${PULL_TO_SOURCE}" == "1" ]]; then
    echo "[setup] Pulling changes from Isaac Sim back to source..."
    echo "[setup] Copying from: ${DEST}"
    echo "[setup] To:           ${SRC_PKG}"

    if [[ ! -d "${DEST}" ]]; then
        echo "ERROR: Destination directory does not exist: ${DEST}"
        exit 1
    fi

    rm -rf "${SRC_PKG}"
    cp -r "${DEST}" "${SRC_PKG}"
    find "${SRC_PKG}" -name "__pycache__" -type d -exec rm -rf {} +

    # Revert fully-qualified intra-package imports back to bare ones in source.
    for f in "${SRC_PKG}"/*.py; do
        [[ -f "$f" ]] || continue
        sed -i 's/from isaacsim\.examples\.interactive\.ur_robotiq_cortex/from ur_robotiq_cortex/g' "$f"
        sed -i 's/import isaacsim\.examples\.interactive\.ur_robotiq_cortex/import ur_robotiq_cortex/g' "$f"
    done

    echo "[setup] Reverted intra-package imports in source."
    echo "=== Pull complete ==="
    exit 0
fi

# ---------------------------------------------------------------------------
# Copy the Python package
# ---------------------------------------------------------------------------
echo "[setup] Copying Python package to: ${DEST}"
rm -rf "${DEST}"
cp -r "${SRC_PKG}" "${DEST}"
find "${DEST}" -name "__pycache__" -type d -exec rm -rf {} +
echo "[setup] Copy complete."

# ---------------------------------------------------------------------------
# Fix intra-package imports to use the full dotted path
# ---------------------------------------------------------------------------
# The package now lives at isaacsim.examples.interactive.ur_robotiq_cortex.* so
# bare "from ur_robotiq_cortex." imports must become fully qualified.
for f in "${DEST}"/*.py; do
    [[ -f "$f" ]] || continue
    sed -i 's/from ur_robotiq_cortex\./from isaacsim.examples.interactive.ur_robotiq_cortex./g' "$f"
    sed -i 's/import ur_robotiq_cortex\./import isaacsim.examples.interactive.ur_robotiq_cortex./g' "$f"
done

# Rewrite __init__.py to the fully qualified path.
cat > "${DEST}/__init__.py" << 'PYEOF'
from isaacsim.examples.interactive.ur_robotiq_cortex.ur_robotiq_cortex_extension import URRobotiqCortexExtension
PYEOF

echo "[setup] Fixed intra-package imports."

# ---------------------------------------------------------------------------
# Register the module in extension.toml (idempotent)
# ---------------------------------------------------------------------------
# Kit discovers extensions via extension.toml; each [[python.module]] block names
# a module imported at startup. Adding ours makes Kit import it, which runs
# __init__.py -> URRobotiqCortexExtension and registers the example in the browser.
if grep -qF "${MODULE_NAME}" "${TOML}"; then
    echo "[setup] Module already registered in extension.toml: ${MODULE_NAME}"
else
    echo "[setup] Registering module in extension.toml: ${MODULE_NAME}"
    # Insert a new [[python.module]] block just before the first [[test]] section.
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
if [[ "${SHELL:-}" == *zsh* ]]; then
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
echo "Start Isaac Sim:"
echo "  ${LAUNCH_HINT}"
echo "(or from the workspace root:  ./launch_isaac_ros2.sh )"
echo ""
echo "'UR5 HCDT' will appear in Isaac Examples > Cortex after Isaac Sim loads."
echo ""
echo "If USD assets are not found, verify that the following files exist:"
echo "  \${UR_WS_PATH}/isaac_standalone/Collected_ur10e_robotiq2f-140_ROS/ur5_robotiq2f-85.usd"
echo "  \${UR_WS_PATH}/isaac_standalone/Collected_ur10e_robotiq2f-140_ROS/ur10e_robotiq2f-140_ROS.usd"
echo ""
echo "To change the robot model (default: ur5):"
echo "  export UR_ROBOT_TYPE=ur10   # before launching Isaac Sim"
echo ""
echo "To re-run after editing source files in ur_ws:  ./setup_isaac_extension.sh"
echo "To uninstall:                                   ./setup_isaac_extension.sh --uninstall"
