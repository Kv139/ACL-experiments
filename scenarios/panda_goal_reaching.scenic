from library import *

from numpy import random

#room_region = BoxRegion(dimensions=(2,2,3), position=(0,0,0.68005))
room_region = BoxRegion(dimensions=(2,2,2), position=(0,0,0.0))
workspace = Workspace(room_region)


# ego = new Franka with position (0,0,0)
# # # target = new TARGET on Floor

# # obj = new circle_obj with position(.5,.5,.05)

# # distractor = new distractor_obj  in wor

terminate after 15*300 steps # 15*STEPS