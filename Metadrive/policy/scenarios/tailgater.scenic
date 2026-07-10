param map = localPath('../../CARLA/Town01.xodr')
param carla_map = 'Town01'

model scenic.simulators.metadrive.model

param time_step = 1.0/10
param verifaiSamplerType = 'halton'
param use2DMap = True

param extra_cars = 1

import numpy as np
TERMINATE_TIME = 40 / globalParameters.time_step

"""
Setting global params for the road, land, starting and stoping
Scene defining variables should be global params in order to allow for
easily mutating later
"""
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

# TODO fix params -- need more variabiltiy and ensure that modified scenes make!

param select_road = VerifaiOptions([*network.roads])
param distractor_road = VerifaiOptions([*network.roads])

param select_lane = VerifaiOptions([*network.lanes])
param distractor_lane = VerifaiOptions([*network.lanes])

start = Uniform(*globalParameters.select_lane.centerline.points)
start2 = Uniform(*globalParameters.distractor_lane.centerline.points)

start = (start[0] @ start[1])
start2 = (start2[0] @ start2[1])

ego = new Car on start, facing roadDirection, with observation 0, with cte 0 
distractor = new Car on start2, with behavior DriveAvoidingCollisions(target_speed=10, avoidance_threshold=12)
tailgater = new Car behind ego by 15, facing roadDirection, with behavior tailGate(target_gap=Range(3,5))
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