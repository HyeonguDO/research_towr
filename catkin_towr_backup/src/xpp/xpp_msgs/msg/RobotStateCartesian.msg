# The state of a robot expressed in the Cartesian frame

duration                time_from_start   # global time along trajectory

# Position, velocity and acceleration of the base expressed in world frame
# The orientation quaternion maps base to world frame.
State6d                 base              # base pos/vel/acc in world

StateLin3d[]            ee_motion         # endeffector pos/vel/acc in world
geometry_msgs/Vector3[] ee_forces         # endeffector forces expressed in world
bool[]                  ee_contact        # True if the foot is touching the environment

# geometry_msgs/Vector3[] ee_tensions         # tension forces expressed in world      NEW  메세지 변수 추가로 인한 오류인 것 확실
State6d ee_tensions_test         # tension forces expressed in world      NEW  TEST TEST



