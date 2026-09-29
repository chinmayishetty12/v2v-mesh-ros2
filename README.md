# v2v-mesh-ros2

ROS 2 (Python) V2V mesh communication node for a mining-vehicle safety system, built for Smart India Hackathon 2026 (Problem Statement 26007). This is my part of the team project.

## What it does
- Simulates a local V2V mesh over UDP (127.0.0.1:5005), broadcasting at 10 Hz
- Simulates network conditions: ~18 ms latency and 2% packet drop
- Converts neighbouring vehicles' positions into relative coordinates in the ego vehicle's `base_link` frame
- Publishes them live on `/v2v/neighbor_vehicles` so the AI Risk Prediction and Sensor Fusion modules can subscribe, e.g. for blind corners or fog where cameras/LiDAR fail

## Run
Requires ROS 2 installed and sourced in each terminal.

```bash
# Terminal 1
python3 v2v_mesh_node.py --ros-args -p vehicle_id:=4

# Terminal 2
ros2 topic echo /v2v/neighbor_vehicles
```
## Message format
`std_msgs/String`: `ID:<id> | Rel_X:<m> | Rel_Y:<m>`

## Note
Fleet positions are hard-coded simulated states, not real sensor data.
