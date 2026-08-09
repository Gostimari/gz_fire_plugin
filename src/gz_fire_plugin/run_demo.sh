#!/usr/bin/env bash
# run_demo.sh — Fire plugin demo launcher
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# Plugin path set karo
PLUGIN_LIB="$SCRIPT_DIR/install/gz_fire_plugin/lib/gz_fire_plugin"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PLUGIN_LIB:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

WORLD="$SCRIPT_DIR/src/gz_fire_plugin_demo/worlds/demo_cube_fire.sdf"

echo ""
echo "══════════════════════════════════════"
echo "  Fire Plugin Demo"
echo "  Plugin path: $PLUGIN_LIB"
echo "  World: demo_cube_fire.sdf"
echo ""
echo "  Gazebo khulne ke baad ALAG terminal mein:"
echo "  source install/setup.bash"
echo "  gz topic -t /fire/trigger -m gz.msgs.Boolean -p 'data: true'"
echo "══════════════════════════════════════"
echo ""

gz sim -r "$WORLD"
