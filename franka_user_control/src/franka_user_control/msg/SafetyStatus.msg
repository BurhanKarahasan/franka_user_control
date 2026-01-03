# Safety monitoring status

std_msgs/Header header

# Velocity limits
bool velocity_limit_exceeded
float64 current_velocity         # Current Cartesian velocity magnitude (m/s)
float64 max_allowed_velocity     # Maximum allowed velocity (m/s)

# Force/torque monitoring
bool force_limit_exceeded
float64[6] current_forces        # Current Cartesian forces/torques
float64[6] force_thresholds      # Current safety thresholds

# Workspace limits
bool workspace_limit_violated
geometry_msgs/Point current_position
string violated_boundary         # 'x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max', or 'none'

# Overall safety state
bool is_safe                     # False if any safety limit is violated
string[] active_warnings         # List of active safety warnings
