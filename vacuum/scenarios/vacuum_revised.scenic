"""
Generate a room for the i-roomba create vacuum
"""
from scenic.core.external_params import VerifaiRange

from vacuum_lib import *

param verifaiSamplerType = 'halton'
dnev = 1.59576912 # double normal ev, |[-dnev, dnev]| has the same ev as |N(0, 1)|
dpi = 6.28318531

## Scene Layout ##

# Create room region and set it as the workspace
room_region = RectangularRegion(0 @ 0, 0, 3.59, 3.59)
workspace = Workspace(room_region)

# Create floor and walls
floor = new Floor
wall_offset = floor.width/2 + 0.04/2 + 1e-4
right_wall = new Wall at (wall_offset, 0, 0.25), facing toward floor
left_wall = new Wall at (-wall_offset, 0, 0.25), facing toward floor
front_wall = new Wall at (0, wall_offset, 0.25), facing toward floor
back_wall = new Wall at (0, -wall_offset, 0.25), facing toward floor


ego = new Vacuum at (1,1), on floor
record (ego.x, ego.y, ego.z) as EgoPosition

# Create a "safe zone" around the vacuum so that it does not start stuck
safe_zone = CircularRegion(ego.position, radius=1)

    # Create a living room region where we will place living room furniture
living_room_region = RectangularRegion(-1.0 @ 0, 0, 3.5, 3.5).difference(safe_zone)
ideal_pos = new OrientedPoint ahead of left_wall by 0.335
couch = new Couch on floor, facing VerifaiRange((-dnev*5) deg, (dnev*5) deg) relative to ideal_pos, with regionContainedIn living_room_region,
                at ideal_pos offset by (VerifaiRange(-dnev*.05, dnev*.05) @ VerifaiRange(-dnev*.5, dnev*.5))
ideal_pos = new OrientedPoint ahead of couch by 0.336, facing away from couch
coffee_table = new CoffeeTable on floor, facing VerifaiRange((-dnev*5) deg, (dnev*5) deg) relative to ideal_pos, with regionContainedIn living_room_region,
                at ideal_pos offset by (VerifaiRange(-dnev*.05, dnev*.05) @ VerifaiRange(-dnev*.05, dnev*.05))

new Toy on floor, at (VerifaiRange(-1.5, 1.5), VerifaiRange(-1.5, 1.5), 10)
new Toy on floor, at (VerifaiRange(-1.5, 1.5), VerifaiRange(-1.5, 1.5), 10)
new Toy on floor, at (VerifaiRange(-1.5, 1.5), VerifaiRange(-1.5, 1.5), 10)
# new Toy on floor, at (VerifaiRange(-1.5, 1.5), VerifaiRange(-1.5, 1.5), 10)
# new Toy on floor, at (VerifaiRange(-1.5, 1.5), VerifaiRange(-1.5, 1.5), 10)

# # Create a dining room region where we will place dining room furniture
# dining_room_region = RectangularRegion(1.25 @ 0, 0, 5, 5).difference(safe_zone)

# dining_table = new DiningTable on floor, at (VerifaiRange(-2.5, 2.5) @ VerifaiRange(-2.5, 2.5)), with size .1
# ideal_pos = new OrientedPoint behind dining_table by -0.1, facing toward dining_table
# chair_1 = new DiningChair on floor, 
#                 facing VerifaiRange((-dnev*10) deg, (dnev*10) deg) relative to ideal_pos,
#                 at ideal_pos offset by (VerifaiRange(-dnev*.05, dnev*.05) @ VerifaiRange(-dnev*.05, dnev*.05)),
#                 with regionContainedIn dining_room_region     
# ideal_pos = new OrientedPoint left of dining_table by -0.1, facing toward dining_table
# chair_3 = new DiningChair on floor, 
#                 facing VerifaiRange((-dnev*10) deg, (dnev*10) deg) relative to ideal_pos,
#                 at ideal_pos offset by (VerifaiRange(-dnev*.05, dnev*.05) @ VerifaiRange(-dnev*.05, dnev*.05)), 
#                 with regionContainedIn dining_room_region


