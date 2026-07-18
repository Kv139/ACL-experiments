param map = localPath('../../CARLA/Town01.xodr')
param carla_map = 'Town01'

model scenic.simulators.metadrive.model

param time_step = 1.0/10
param verifaiSamplerType = 'halton'
param use2DMap = True

# param extra_cars = 0
# param brake_checkers = 0
# param tailgaters = 0
# param merging_cars = 0
# param obstacles = 0

param distractor_cars = 0
param ego_speed = 0
param ego_on_straight = 0 # if 1 then its pawns on intersection
param ego_intersection_has_cars = 0
param distance_to_nearest = 0
param distance_average_distractor = 0
param cluster_same = 0 ## have cluster of cars on lane
param different_lane = 0 ##if nearest car is on same lane or differnt lnae? is there a better parameter?


param select_road = VerifaiOptions([*network.roads])
param select_lane = VerifaiOptions([*network.lanes])


param distance_to_extra ##cant exceed that subsetttttt!!!
# param different_side = 0

# param avg_merge_dist = 0
# param avg_brake_dist = 0
# param avg_tail_dist = 0
# param avg_extra_dist = 0
# param avg_obstacel_dist = 0

# param extra_speed = 0
# param 
# param merge_speed = 0
# param tail_speed = 0


import numpy as np
TERMINATE_TIME = 40 / globalParameters.time_step
######brakign
behavior brakeChecking(distanceToTrigger = 10):
	try:
		do FollowLaneBehavior(target_speed=5)
	interrupt when (distance from self to ego) < distanceToTrigger:
		while True:
			print ("BREAKKK")
			take SetBrakeAction(1), SetThrottleAction(0)

##tailgater
behavior tailGate(target_gap = 4):
    while True:
		if distance from self to ego > target_gap:
			take SetThrottleAction(0.01), SetBrakeAction(0)
		else:
			take SetThrottleAction(0), SetBrakeAction(1) 


def get_nearest_centerline(obj):
	min_dist = np.inf
	for lane in network.lanes:
		dist = distance to lane
		if dist < min_dist:
			min_dist = dist
			centerline = lane.centerline
	return centerline



start = Uniform(*globalParameters.select_lane.centerline.points)
start2 = Uniform(*globalParameters.distractor_lane.centerline.points)

start = (start[0] @ start[1])
start2 = (start2[0] @ start2[1])

ego = new Car on start, facing roadDirection, with observation 0, with cte 0 
distractor = new Car on start2, with behavior DriveAvoidingCollisions(target_speed=10, avoidance_threshold=12)


#--------new stuf
# for i in range(globalParameters.brake_checkers):
#     new Car ahead of ego by Range(15, 20) + i * 20, facing roadDirection, with behavior brakeChecking(distanceToTrigger=Range(10, 15))

# for i in range(globalParameters.tailgaters):
#     new Car behind ego by Range(10, 20) + i * 20, facing roadDirection, with behavior tailGate(target_gap=Range(3, 5))


for i in range(globalParameters.brake_checkers):
    lane = Uniform(*network.lanes)
    x = new OrientedPoint on lane.centerline
    new Car at x, with behavior brakeChecking(distanceToTrigger=Range(10, 15))

for i in range(globalParameters.tailgaters):
    lane = Uniform(*network.lanes)
    x = new OrientedPoint on lane.centerline
    new Car at x, with behavior tailGate(target_gap=Range(3, 5))

prev_merge = distractor
for i in range(globalParameters.merging_cars):
    prev_merge = new Car ahead of prev_merge by 20, with behavior FollowLaneBehavior(target_speed=10, laneToFollow=None, is_oppositeTraffic=False)

for i in range(globalParameters.obstacles):
    lane = Uniform(*network.lanes)
    spot = new OrientedPoint on lane.centerline
    cardegree = Uniform(1.0, -1.0) * Range(45, 90) deg
    new Car at spot, facing cardegree relative to roadDirection, with allowCollisions True

for i in range(globalParameters.extra_cars):
    random_lane = Uniform(*network.lanes)
    random_point = Uniform(*random_lane.centerline.points)
    random_pos = (random_point[0] @ random_point[1])
    random_speed = Range(12, 15)
    new Car on random_pos, with behavior DriveAvoidingCollisions(target_speed=random_speed, avoidance_threshold=12)



monitor DrivingReward(obj):
	curr_lane = obj._lane
	while True:

		if curr_lane is None or not curr_lane.containsPoint(obj.position):
			curr_lane = obj._lane
		
		ego.previous_coordinates = obj.position
		lane = obj._lane

		if lane:
			centerline = lane.centerline	
		else:
			centerline = get_nearest_centerline(obj)

		if obj._lane:
			ego.lane_heading = lane._defaultHeadingAt(ego.position)
			orientation_error = np.abs(ego.heading - lane._defaultHeadingAt(ego.position))
			ego.orientation_error = orientation_error

			if orientation_error > .05:
				orientation_error =  max(-orientation_error, -3)
			else:
				orientation_error = 1 # postive reward for the correct orientation

		nearest_line_points = centerline.nearestSegmentTo(obj.position)
		nearest_line_segment = PolylineRegion(nearest_line_points)
		
		cte = min(abs(distance to nearest_line_segment),1)
		if cte < .2: 
			cte = 0
		speed_reward = max(0.5 * ego.speed, 2) # postiive reward for driving fast

		dist_reward = distance to ego.previous_coordinates

		reward =  -cte + speed_reward + orientation_error + dist_reward

		ego.reward = reward

		wait
	

require monitor DrivingReward(ego)