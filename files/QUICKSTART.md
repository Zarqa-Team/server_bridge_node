
```bash
cd ~/ws_sensor_combined
colcon build --packages-select server_node
source install/setup.bash
```

## Run
```bash
# Terminal 1
ros2 run server_node bridge

# Terminal 2 (optional - monitor topics)
ros2 topic list
ros2 topic echo /competition/server_time
```

