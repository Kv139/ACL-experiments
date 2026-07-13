param map = localPath('../../CARLA/Town01.xodr')
param carla_map = 'Town01'

model scenic.simulators.metadrive.model

param time_step = 1.0/10
param verifaiSamplerType = 'halton'
param use2DMap = True

param extra_cars = 1
param crosser_speed = Range(4,10)
import numpy as np
TERMINATE_TIME = 40 / globalParameters.time_step

"""
Setting global params for the road, land, starting and stoping
Scene defining variables should be global params in order to allow for
easily mutating later
"""

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

param spawn_intersection = VerifaiOptions([*network.intersections])
intersection_lane = Uniform(*globalParameters.spawn_intersection.incomingLanes)

crossing = Uniform(*globalParameters.spawn_intersection.maneuvers)
crossing2 = Uniform(*globalParameters.spawn_intersection.maneuvers)

crossing_start = crossing.startLane.centerline.points[0]
crossing_start = (crossing_start[0] @ crossing_start[1])

crossing2_start = crossing2.startLane.centerline.points[0]
crossing2_start = (crossing2_start[0] @ crossing2_start[1])



start = Uniform(*intersection_lane.centerline.points)
start2 = Uniform(*globalParameters.distractor_lane.centerline.points)

start = (start[0] @ start[1])
start2 = (start2[0] @ start2[1])

ego = new Car on start, facing roadDirection, with observation 0, with cte 0 
distractor = new Car on start2, with behavior DriveAvoidingCollisions(target_speed=10, avoidance_threshold=12)

crosser = new Car on crossing_start, facing roadDirection, with behavior FollowTrajectoryBehavior(trajectory=[crossing.startLane, crossing.connectingLane, crossing.endLane], target_speed=globalParameters.crosser_speed)
crosser2 = new Car on crossing2_start, facing roadDirection, with behavior FollowTrajectoryBehavior(trajectory=[crossing2.startLane, crossing2.connectingLane, crossing2.endLane], target_speed=globalParameters.crosser_speed)