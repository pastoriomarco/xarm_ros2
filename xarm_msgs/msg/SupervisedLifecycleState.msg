# Restricted supervised state published by the sole xarm_ros2 SDK owner.
#
# Steady-clock values are meaningful only to processes on the same host. A
# consumer must additionally bound state age from its local receive time.

string owner_id
string session_id
uint64 generation
int64 source_steady_time_ns
int64 fresh_until_steady_time_ns

bool transport_connected
bool report_connected
bool identity_verified
bool process_restart_required
bool report_received
bool position_valid
bool command_gate_open
int64 command_gate_valid_until_steady_time_ns
bool hardware_component_active
bool position_ever_initialized
bool stationary
int64 stationary_since_steady_time_ns

int16 identity_axis
int16 identity_device_type
int16 state
int16 mode
int32 command_count
int32 brake_mask
int32 servo_enable_mask
int32 error_code
int32 warning_code

uint64 report_sample_count
uint64 joint_read_attempt_count
uint64 joint_read_success_count
uint64 joint_write_attempt_count
uint64 lifecycle_command_attempt_count
uint64 shutdown_controller_attempt_count
int32 last_joint_read_return_code
int32 last_joint_write_return_code
