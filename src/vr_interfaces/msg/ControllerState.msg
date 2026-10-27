# One Touch controller as sent by the Quest app (unity/RobotRegistration/QuestStatePublisher.cs).
bool tracked                  # false = not positionally tracked (pose is stale)
geometry_msgs/Pose pose       # controller anchor in vr_world (x = pointing direction, y left, z up)
float32 index_trigger         # 0..1
float32 hand_trigger          # 0..1 (grip)
float32 thumbstick_x          # -1..1, right positive (OVRInput convention)
float32 thumbstick_y          # -1..1, forward/up positive
bool button_primary           # A (right) / X (left)
bool button_secondary         # B (right) / Y (left)
bool thumbstick_pressed
bool menu                     # menu / start button (left controller only)
