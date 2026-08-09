# 🔥 gz_fire_plugin

**Gazebo Harmonic Fire Simulation Plugin for ROS 2 Jazzy (Ubuntu 24.04)**

A fire simulation system plugin for **Gazebo Sim 8 / Gazebo Harmonic** that enables realistic fire, smoke, heat propagation, ignition, progressive burning, and object damage within a simulated environment.

Developed by **Rahul Gupta** and **Aachal Sharma** for robotics research and simulation.

---

## ✨ Features

- 🔥 Fire and flame particle simulation
- 💨 Smoke and particle emission
- 🌡️ Heat propagation in configurable cone
- 🎯 Distance and direction based ignition
- 🪵 Progressive object burning
- 📈 Continuous burn damage accumulation
- 🧱 Object destruction after sufficient damage
- 🎨 Burn / charring visual effects
- ⚙️ Fully configurable via SDF parameters
- 🧩 Gazebo System plugin architecture (ISystemConfigure + ISystemPreUpdate + ISystemUpdate)
- 🚀 Designed for Gazebo Harmonic / Gazebo Sim 8

---

## 🖥️ Compatibility

| Component           | Version      |
|---------------------|--------------|
| Gazebo Sim          | 8.x          |
| Tested Version      | 8.11.0       |
| Gazebo Distribution | Harmonic     |
| ROS 2               | Jazzy        |
| Operating System    | Ubuntu 24.04 |

---

## 🧠 How It Works

```
        🔥 Fire Source (fire_source_link)
                    │
                    ▼
        Distance / Direction Check (cone)
                    │
                    ▼
              Heat Transfer
                    │
                    ▼
                 Ignition
            (ignition_heat threshold)
                    │
                    ▼
          Progressive Burning
                    │
                    ▼
           Damage Accumulation
             (damage_rate)
                    │
                    ▼
          Charring / Burn Mark Visual
                    │
                    ▼
           Object Destruction
            (destroy_damage threshold)
```

---

## 📁 Project Structure

```
gz_fire_plugin/
├── README.md
├── build.sh                              ← build script
├── run_demo.sh                           ← demo launcher
└── src/
    ├── gz_fire_plugin/                   ← Core plugin package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/
    │   │   └── gz_fire_plugin/
    │   │       └── FirePlugin.hh         ← Plugin header
    │   └── src/
    │       └── FirePlugin.cc             ← Plugin implementation
    └── gz_fire_plugin_demo/              ← Demo world and launch
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   └── fire_demo.launch.py
        ├── urdf/
        │   └── fire_nozzle.urdf
        └── worlds/
            └── demo_cube_fire.sdf
```

---

## 🏗️ Building

### Prerequisites

ROS 2 Jazzy aur Gazebo Harmonic already installed hone chahiye.

```bash
# ROS 2 Jazzy check
source /opt/ros/jazzy/setup.bash
ros2 --version

# Gazebo check
gz sim --version
```

### Step 1 — Dev dependencies install karo

```bash
sudo apt-get install -y \
  ros-jazzy-gz-sim-vendor \
  ros-jazzy-gz-plugin-vendor \
  ros-jazzy-gz-transport-vendor \
  ros-jazzy-gz-msgs-vendor \
  ros-jazzy-gz-math-vendor \
  ros-jazzy-gz-cmake-vendor \
  python3-colcon-common-extensions
```

### Step 2 — Repo clone karo

```bash
cd ~
git clone https://github.com/Aachal-Sharma/gz_fire_plugin.git fire_ws
cd fire_ws
```

### Step 3 — Source karo

```bash
source /opt/ros/jazzy/setup.bash
```

### Step 4 — Build karo

```bash
colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --packages-select gz_fire_plugin
```

### Step 5 — Workspace source karo

```bash
source ~/fire_ws/install/setup.bash
```

### Step 6 — Plugin path set karo

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/fire_ws/install/gz_fire_plugin/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

---

## 🚀 Running the Demo

```bash
gz sim -r ~/fire_ws/src/gz_fire_plugin/src/gz_fire_plugin_demo/worlds/demo_cube_fire.sdf
```

### Fire ON karo (alag terminal mein)

```bash
source /opt/ros/jazzy/setup.bash
source ~/fire_ws/install/setup.bash

gz topic -t /fire -m gz.msgs.Boolean -p "data: true"
```

### Fire OFF karo

```bash
gz topic -t /fire -m gz.msgs.Boolean -p "data: false"
```

### Fire state monitor karo

```bash
gz topic -e -t /fire/state
```

---

## ⚙️ SDF Plugin Configuration

Apne robot ke URDF/SDF mein plugin is tarah attach karo:

```xml
<plugin
    filename="libFirePlugin.so"
    name="gz::sim::systems::FirePlugin">

    <!-- Fire source link — jiska +Z flame direction hai -->
    <!-- <fire_link>fire_source_link</fire_link> -->

    <!-- Cone geometry -->
    <cone_half_angle>0.785</cone_half_angle>   <!-- radians = 45 deg -->
    <cone_max_range>2.0</cone_max_range>        <!-- metres -->

    <!-- Flame intensity (0.0 - 2.0) -->
    <intensity>1.0</intensity>

    <!-- Heat parameters -->
    <ignition_heat>100.0</ignition_heat>   <!-- heat units to ignite -->
    <heat_rate>10.0</heat_rate>            <!-- heat units per second -->

    <!-- Damage parameters -->
    <damage_rate>0.5</damage_rate>         <!-- damage per second while burning -->
    <destroy_damage>100.0</destroy_damage> <!-- total damage before destruction -->

    <!-- Destroy object after burn completes -->
    <destroy_after_burn>true</destroy_after_burn>

    <!-- Control topics -->
    <fire_topic>/fire</fire_topic>
    <state_topic>/fire/state</state_topic>

    <!-- Start fire immediately on launch -->
    <start_on>true</start_on>

</plugin>
```

### Parameter Reference

| Parameter          | Type    | Default | Description                              |
|--------------------|---------|---------|------------------------------------------|
| `cone_half_angle`  | double  | 0.785   | Half angle of fire cone in radians       |
| `cone_max_range`   | double  | 2.0     | Max reach of fire in metres              |
| `intensity`        | double  | 1.0     | Flame intensity scale (0.0 to 2.0)       |
| `ignition_heat`    | double  | 100.0   | Heat threshold to ignite an object       |
| `heat_rate`        | double  | 10.0    | Heat applied per second to objects       |
| `damage_rate`      | double  | 0.5     | Damage per second while object is burning|
| `destroy_damage`   | double  | 100.0   | Total damage threshold for destruction   |
| `destroy_after_burn` | bool  | true    | Remove object after damage threshold     |
| `fire_topic`       | string  | /fire   | Boolean topic to toggle fire ON/OFF      |
| `state_topic`      | string  | /fire/state | Topic to publish current fire state  |
| `start_on`         | bool    | false   | If true, fire starts immediately         |

---

## 🔥 Fire Simulation Behavior

The plugin simulates fire interaction progressively — objects do not disappear immediately.

1. **Fire Generation** — Fire source produces flame, heat, and smoke particle effects continuously.
2. **Fire Exposure Check** — Objects within configured range and cone direction are evaluated each simulation step.
3. **Heat Transfer** — Exposed burnable objects gradually accumulate heat at `heat_rate` per second.
4. **Ignition** — When accumulated heat reaches `ignition_heat`, the object starts burning.
5. **Progressive Burning** — Object continues burning while exposed, rather than instant destruction.
6. **Damage Accumulation** — Burning increases the object damage at `damage_rate` per second.
7. **Charring / Burn Effects** — Visual burn marks and charring appear as damage increases.
8. **Destruction** — When damage reaches `destroy_damage`, object is removed from simulation.

---

## 📡 Topics

| Topic        | Type              | Description                        |
|--------------|-------------------|------------------------------------|
| `/fire`      | `gz.msgs.Boolean` | Send `true` to ignite, `false` to extinguish |
| `/fire/state`| `gz.msgs.Boolean` | Current fire state (ON / OFF)      |

---

## 👥 Contributors

This project was developed through equal contributions from:

| Name          | GitHub                                               |
|---------------|------------------------------------------------------|
| Rahul Gupta   | [@topguns837](https://github.com/Rahulcva)         |
| Aachal Sharma | [@Aachal-Sharma](https://github.com/Aachal-Sharma)   |

Both contributors contributed equally to the fire simulation system design, implementation, integration, testing, and demonstration.

---

## 🌐 Repository

[https://github.com/Aachal-Sharma/gz_fire_plugin](https://github.com/Aachal-Sharma/gz_fire_plugin)

---

## 🙏 Acknowledgement

This project is developed for the **Gazebo Sim / Gazebo Harmonic** simulation ecosystem and aims to provide a reusable fire and burning simulation capability for robotics research and simulation.

---

## 📄 License

Apache-2.0 — see [LICENSE](./LICENSE) for details.
