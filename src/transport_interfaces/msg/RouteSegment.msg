# Map-frame axis direction. All distances are metres, angles radians.
uint8 POS_X=0
uint8 POS_Y=1
uint8 NEG_X=2
uint8 NEG_Y=3

uint32 index
geometry_msgs/Point start
geometry_msgs/Point end
uint8 axis
float64 heading
float64 length

# First segment: false. Its initial turn requires the actual robot heading.
# Other segments: signed turn from previous segment, including possible U-turn.
bool has_previous_turn
float64 turn_from_previous
