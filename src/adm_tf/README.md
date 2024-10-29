## How to build
Unzip the downloaded archive in the ros2 workspace and compile:
```
colcon build --packages-select adm_tf
```

## How to launch
Source the workspace and launch:

**Run in shell 1**:

> In the first shell, start the component container:

```bash
source install/local_setup.bash
ros2 run adm_tf adm_static_publisher_main
```

**Run in shell 2**:

> In the second shell load device_au_radar_node node

```bash
source install/local_setup.bash
ros2 run adm_tf adm_listener_main
