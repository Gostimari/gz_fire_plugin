# 🔥 Gazebo Harmonic Fire Simulation Plugin

A fire simulation system plugin for **Gazebo Sim / Gazebo Harmonic** that enables fire, smoke, heat propagation, ignition, progressive burning, and object damage within a simulated environment.

The plugin is designed to provide a configurable and reusable fire simulation capability for robotics research and simulation.

---

## ✨ Features

- 🔥 Fire and flame simulation
- 💨 Smoke and particle emission
- 🌡️ Heat propagation
- 🔥 Distance- and direction-based ignition
- 🪵 Progressive object burning
- 📈 Continuous burn damage accumulation
- 🧱 Object destruction after sufficient damage
- 🎨 Burn / charring visual effects
- 🎯 Configurable fire range and cone angle
- ⚙️ Configurable heat and damage parameters
- 🧩 Gazebo System plugin architecture
- 🚀 Designed for Gazebo Harmonic / Gazebo Sim 8

---

## 🖥️ Compatibility

| Component | Version |
|-----------|---------|
| Gazebo Sim | 8.x |
| Tested Version | 8.11.0 |
| Gazebo Distribution | Harmonic |
| ROS 2 | Jazzy |
| Operating System | Ubuntu 24.04 |

---

## 🧠 How It Works

The fire system continuously evaluates objects around the fire source and applies heat and burning effects progressively.

```text
              🔥 Fire Source
                    │
                    ▼
          Distance / Direction Check
                    │
                    ▼
              Heat Transfer
                    │
                    ▼
                 Ignition
                    │
                    ▼
          Progressive Burning
                    │
                    ▼
           Damage Accumulation
                    │
                    ▼
            Charring / Burn Mark
                    │
                    ▼
           Object Destruction


           Object Destruction

---

## ⚙️ Configuration

The fire behavior can be configured through the SDF plugin parameters.

Example:

```xml
<plugin
    filename="libFirePlugin.so"
    name="gz::sim::systems::FirePlugin">

    <cone_half_angle>0.785</cone_half_angle>
    <cone_max_range>2.0</cone_max_range>

    <intensity>1.0</intensity>

    <ignition_heat>100.0</ignition_heat>
    <heat_rate>10.0</heat_rate>

    <damage_rate>0.5</damage_rate>
    <destroy_damage>100.0</destroy_damage>

    <destroy_after_burn>true</destroy_after_burn>

    <fire_topic>/fire</fire_topic>
    <state_topic>/fire/state</state_topic>

    <start_on>true</start_on>
</plugin>



---

## 🏗️ Building

### 1. Source ROS 2 Jazzy

```bash
source /opt/ros/jazzy/setup.bash

 ###  2. Build the Plugin
cd ~/fire_ws

colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --packages-select gz_fire_plugin

### Source the Workspace
source ~/fire_ws/install/setup.bash

export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/fire_ws/install/gz_fire_plugin/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH

### run
gz sim -r ~/fire_ws/src/gz_fire_plugin/src/gz_fire_plugin_demo/worlds/demo_cube_fire.sdf

