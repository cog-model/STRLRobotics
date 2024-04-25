from turtlebot_mover import TurtlebotMover
from time import sleep
mover = TurtlebotMover(max_angular_speed=0.2, trajectory_type='circle', ground_z=4.0)
mover.move_to(7, -8.5)
mover.move_to(8, -7.5)
mover.move_to(8, 5)
mover.move_to(7, 6)
mover.move_to(4, 6)
