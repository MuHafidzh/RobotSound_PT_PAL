# JoyAndro Controller Message
# Timestamp
std_msgs/Header header

# Analog stick (axes)
float32 x        # Left/Right: -1.0 (right) to +1.0 (left)
float32 y        # Up/Down: -1.0 (down) to +1.0 (up)

# Button states (bool pressed/released)
bool button_x       # X button
bool button_circle  # O button
bool button_triangle # Triangle button
bool button_square   # Square button