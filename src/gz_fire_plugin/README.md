# gz_fire_plugin 🔥

**Gazebo Harmonic fire/flame simulation plugin for ROS 2 Jazzy (Ubuntu 24.04)**

Ye plugin reference repo [`gz_sim_spray_painting_plugin`](https://github.com/topguns837/gz_sim_spray_painting_plugin)
ke architecture ko follow karta hai — lekin fire/flame simulation ke liye.

---

## Features

- 🔥 **Particle-based fire visuals** — orange→red gradient flame using Gazebo's native particle emitter
- 📡 **Boolean topic toggle** — `/fire/trigger` se fire ON/OFF karo
- 🌡️ **Heat-ray cone scanning** — nozzle link ke +Z axis mein cone-shaped heat rays
- 📢 **State publisher** — `/fire/state` pe current fire status
- ⚙️ **Fully configurable** via SDF parameters

---

## Prerequisites

| Software | Version |
|----------|---------|
| Ubuntu   | 24.04 (Noble / Jazzy) |
| ROS 2    | Jazzy |
| Gazebo   | Harmonic (gz-sim8) |
| Compiler | GCC 13 / C++17 |

### WSL2 pe install karo

```bash
# ROS 2 Jazzy
sudo apt install ros-jazzy-desktop

# Gazebo Harmonic
sudo apt install gz-harmonic

# Build tools
sudo apt install python3-colcon-common-extensions
```

---

## Build karo

```bash
git clone <this-repo> gz_fire_plugin_ws
cd gz_fire_plugin_ws

chmod +x build.sh
./build.sh

source install/setup.bash
```

---

## Usage

### 1. Demo launch karo

```bash
ros2 launch gz_fire_plugin_demo fire_demo.launch.py
```

### 2. Fire trigger karo (alag terminal mein)

```bash
source install/setup.bash

# Fire ON
gz topic -t /fire/trigger -m gz.msgs.Boolean -p "data: true"

# Fire OFF
gz topic -t /fire/trigger -m gz.msgs.Boolean -p "data: false"
```

### 3. State monitor karo

```bash
gz topic -e -t /fire/state
```

---

## Plugin SDF Parameters

Apne robot ke URDF/SDF mein plugin add karo:

```xml
<plugin filename="libFirePlugin.so"
        name="gz::sim::systems::FirePlugin">

  <!-- Required: link jiska +Z flame direction hai -->
  <fire_link>my_fire_nozzle_link</fire_link>

  <!-- Optional (defaults shown) -->
  <cone_half_angle>20.0</cone_half_angle>     <!-- degrees -->
  <cone_max_range>2.0</cone_max_range>         <!-- metres -->
  <fire_color>1.0 0.4 0.0 1.0</fire_color>    <!-- RGBA orange -->
  <particle_rate>200</particle_rate>            <!-- particles/sec -->
  <intensity>1.0</intensity>                   <!-- 0.0 – 2.0 -->
  <num_rays>20</num_rays>                      <!-- heat-rays per scan -->
  <paint_interval_steps>10</paint_interval_steps>
  <fire_topic>/fire/trigger</fire_topic>
</plugin>
```

### GZ_SIM_SYSTEM_PLUGIN_PATH set karo

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/install/gz_fire_plugin/lib/gz_fire_plugin:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

---

## Project Structure

```
gz_fire_plugin/
├── build.sh                          # One-shot build script
├── README.md
└── src/
    ├── gz_fire_plugin/               # Core plugin package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/
    │   │   └── gz_fire_plugin/
    │   │       └── FirePlugin.hh     # Plugin header
    │   └── src/
    │       └── FirePlugin.cc         # Plugin implementation
    └── gz_fire_plugin_demo/          # Demo worlds & launch
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   └── fire_demo.launch.py
        ├── urdf/
        │   └── fire_nozzle.urdf      # Standalone nozzle model
        └── worlds/
            └── demo_cube_fire.sdf    # Demo world
```

---

## Architecture (kaise kaam karta hai)

```
FirePlugin::Configure()
    ├── SDF parameters parse karo
    ├── fire_link entity resolve karo
    ├── ParticleEmitter entity create karo (initially OFF)
    ├── /fire/trigger topic subscribe karo
    └── /fire/state topic advertise karo

FirePlugin::PreUpdate()   [har simulation step]
    └── fireOn state change pe:
        ├── ParticleEmitter component update karo (ON/OFF)
        └── /fire/state pe publish karo

FirePlugin::Update()       [har N steps]
    └── fireOn == true ho to:
        └── Heat-ray cone scan karo (+Z axis mein)
```

---

## Apne Robot mein integrate karo

```xml
<!-- apne robot URDF mein -->
<joint name="arm_to_nozzle" type="fixed">
  <parent link="arm_end_effector_link"/>
  <child  link="fire_source_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<link name="fire_source_link"/>

<gazebo>
  <plugin filename="libFirePlugin.so"
          name="gz::sim::systems::FirePlugin">
    <fire_link>fire_source_link</fire_link>
    <fire_topic>/my_robot/fire</fire_topic>
    <intensity>1.5</intensity>
  </plugin>
</gazebo>
```

---

## License

Apache-2.0
