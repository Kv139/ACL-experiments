"""
Generate a room for the i-roomba create vacuum
"""
from scenic.core.external_params import VerifaiRange

from vacuum_lib import *
from itertools import combinations
no_verifai = True
param verifaiSamplerType = 'mab'
dnev = 1.59576912 # double normal ev, |[-dnev, dnev]| has the same ev as |N(0, 1)|
dpi = 6.28318531

## Scene Layout ##

# Create room region and set it as the workspace
room_region = RectangularRegion(0 @ 0, 0, 5.09, 5.09)
workspace = Workspace(room_region)

# Create floor and walls
floor = new Floor
wall_offset = floor.width/2 + 0.04/2 + 1e-4
right_wall = new Wall at (wall_offset, 0, 0.25), facing toward floor
left_wall = new Wall at (-wall_offset, 0, 0.25), facing toward floor
front_wall = new Wall at (0, wall_offset, 0.25), facing toward floor
back_wall = new Wall at (0, -wall_offset, 0.25), facing toward floor

param start  = new Point in workspace

# Place vacuum on floor

ego = new Vacuum at globalParameters.start, on floor
record (ego.x, ego.y, ego.z) as EgoPosition

# Create a "safe zone" around the vacuum so that it does not start stuck
safe_zone = CircularRegion(ego.position, radius=1)

# # Create a living room region where we will place living room furniture
living_room_region = RectangularRegion(-1.25 @ 0, 0, 5, 5).difference(safe_zone)

param ideal_offset_X = Range(-dnev*.05, dnev*.05)
param ideal_offset_Y = Range(-dnev*.05, dnev*.05)

param ideal_pos = new Point in living_room_region

couch = new Couch on floor, at globalParameters.ideal_pos offset by (globalParameters.ideal_offset_X @ globalParameters.ideal_offset_Y),
    facing away from left_wall, 
    with regionContainedIn living_room_region, 
 

ideal_pos = new OrientedPoint ahead of couch by 0.336, facing away from couch
coffee_table = new CoffeeTable on floor, facing away from couch, with regionContainedIn living_room_region

param toy_1 = new Point in living_room_region
param toy_2 = new Point in living_room_region
param toy_3 = new Point in living_room_region


new Toy on floor, at globalParameters.toy_1
new Toy on floor, at globalParameters.toy_2
new Toy on floor, at globalParameters.toy_3


# # # Create a dining room region where we will place dining room furniture
dining_room_region = RectangularRegion(1.25 @ 0, 0, 5, 5).difference(safe_zone)

param dining_table_point = new Point in dining_room_region

dining_table = new DiningTable on floor, at globalParameters.dining_table_point, with size .1
ideal_pos = new OrientedPoint behind dining_table by -0.1, facing toward dining_table

chair_1 = new DiningChair on floor, at ideal_pos    


