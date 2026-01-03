# Controller internal state

std_msgs/Header header

# Control mode
string mode  # 'velocity', 'position', 'idle'

# Position control state
bool motion_in_progress
float64 motion_progress          # 0.0 to 1.0
float64 estimated_time_remaining # seconds

# Velocity control state
float64[6] commanded_velocity    # Last commanded velocity [vx, vy, vz, wx, wy, wz]
float64[6] actual_velocity       # Actual measured velocity

# Impedance settings
string impedance_mode            # 'stiff', 'medium', 'soft', 'custom'
float64[6] impedance_values      # Current impedance [Kx, Ky, Kz, Krx, Kry, Krz]
