#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════
#  build.sh — gz_fire_plugin workspace builder
#  WSL2 + Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic
# ═══════════════════════════════════════════════════════════════
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo ""
echo "══════════════════════════════════════"
echo "  gz_fire_plugin — Build"
echo "══════════════════════════════════════"

# 1. Source ROS 2 Jazzy
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
  echo "[OK] ROS 2 Jazzy sourced"
else
  echo "[ERR] /opt/ros/jazzy/setup.bash not found"
  exit 1
fi

# 2. Check gz
if ! command -v gz &>/dev/null; then
  echo "[ERR] 'gz' not found. Install: sudo apt install gz-harmonic"
  exit 1
fi
echo "[OK] Gazebo: $(gz --version 2>&1 | head -1)"

# 3. Install dev libraries
echo ""
echo "[...] Installing dev dependencies..."
sudo apt-get install -y \
  libgz-sim8-dev \
  libgz-plugin2-dev \
  libgz-transport13-dev \
  libgz-msgs10-dev \
  libgz-math7-dev \
  libsdformat14-dev \
  python3-colcon-common-extensions \
  2>/dev/null

# 4. Build
echo ""
echo "[...] colcon build..."
colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --packages-select gz_fire_plugin gz_fire_plugin_demo

echo ""
echo "══════════════════════════════════════"
echo "  Build DONE!"
echo ""
echo "  Ab ye karo:"
echo ""
echo "  source install/setup.bash"
echo ""
echo "  # Terminal 1 — Simulation launch:"
echo "  ./run_demo.sh"
echo ""
echo "  # Terminal 2 — Fire ON:"
echo "  gz topic -t /fire/trigger -m gz.msgs.Boolean -p 'data: true'"
echo ""
echo "  # Fire OFF:"
echo "  gz topic -t /fire/trigger -m gz.msgs.Boolean -p 'data: false'"
echo "══════════════════════════════════════"
