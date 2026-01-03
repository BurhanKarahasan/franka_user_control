# Robot status message published at regular intervals

std_msgs/Header header

# Current pose
geometry_msgs/Pose current_pose

# Current joint positions (radians)
float64[7] joint_positions

# Current joint velocities (rad/s)
float64[7] joint_velocities

# Current Cartesian velocity
float64[6] cartesian_velocity  # [vx, vy, vz, wx, wy, wz]

# Robot state
bool is_moving
string control_mode  # 'velocity', 'position', 'idle'

# Safety status
bool emergency_stop_active
bool collision_detected
