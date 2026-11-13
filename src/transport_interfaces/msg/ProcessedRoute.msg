# Processing timestamp and coordinate frame (map).
std_msgs/Header header
# Copied from input Path; not a guaranteed planner request ID or map version.
builtin_interfaces/Time source_stamp
# Unique processor output ID; invalid outputs also have a new ID.
string route_id
# Geometry validity ONLY: not authorization to drive or collision certification.
bool valid
string reason

# Start + corners + goal. For one-point routes, contains one pose.
# Intermediate orientations describe outgoing segments, NOT measured vehicle yaw.
geometry_msgs/Pose[] waypoints
transport_interfaces/RouteSegment[] segments
float64 total_length
# Preserved normalized final goal yaw. One-point route may still need rotation.
float64 goal_yaw
