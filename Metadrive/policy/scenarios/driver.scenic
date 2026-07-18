param map = localPath('../../CARLA/Town01.xodr')
param carla_map = 'Town01'
import random

model scenic.simulators.metadrive.model

param time_step = 1.0/10
param verifaiSamplerType = 'halton'
param use2DMap = True

# param extra_cars = 0

param distractor_cars = 0
param has_brake_checker = 0
param intersection_cars = 0

param ego_speed = 0
param ego_on_intersection = 0
param intersection_has_cars = 0
param has_cluster = 0 ###has cluster fo cars in front of ego


param distance_to_nearest = 5
param cluster_dist = 5

param maximum_distractor_speed = 10
param brake_dist = 20
param inter_shuffle_seed = DiscreteRange(0, 2**31 - 1)


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

lane_inter_obj = {} ##what intersection is each lane in
for intersection in network.intersections:
    for lane in intersection.incomingLanes:
        lane_inter_obj[lane] = intersection

inter_lane_obj = list(lane_inter_obj.keys()) or list(network.lanes)
param select_lane = VerifaiOptions([*network.lanes])

if globalParameters.ego_on_intersection == 1:
    ego_lane = Uniform(*inter_lane_obj)
else:
    ego_lane = globalParameters.select_lane

if globalParameters.ego_on_intersection == 1 and ego_lane in lane_inter_obj:
    ego_inter = lane_inter_obj[ego_lane]
    ego_inter_lanes = list(ego_inter.incomingLanes)
else:
    ego_inter = None
    ego_inter_lanes = []


start = Uniform(*ego_lane.centerline.points)
start = (start[0] @ start[1])

ego = new Car on start, facing roadDirection, with speed globalParameters.ego_speed, with observation 0, with cte 0


#---------distractor cars
dist_cars_id = list(range(globalParameters.distractor_cars))
# random.shuffle(dist_cars_id)
# inter_id = dist_cars_id[:globalParameters.intersection_cars]
dist_cars_id = list(range(globalParameters.distractor_cars))
rng = random.Random(globalParameters.inter_shuffle_seed)  
rng.shuffle(dist_cars_id)
inter_id = dist_cars_id[:globalParameters.intersection_cars]

cluster_size = 3

other_counter = 0
cluster_placed = 1 if globalParameters.has_brake_checker == 1 else 0
brake_spawned = False

if globalParameters.has_brake_checker == 1:
    new Car ahead of ego by globalParameters.brake_dist, facing roadDirection, with behavior brakeChecking(distanceToTrigger=Range(10, 15))
    brake_spawned = True

for i in range(globalParameters.distractor_cars):
    if (i in inter_id):
        if (globalParameters.ego_on_intersection == 1 and globalParameters.intersection_has_cars == 1 and len(ego_inter_lanes) > 0 and i == min(inter_id)):
            int_lane = Uniform(*ego_inter_lanes)
        else:
            int_lane = Uniform(*inter_lane_obj)
        pt = Uniform(*int_lane.centerline.points)
        pos = (pt[0] @ pt[1])
        new Car at pos, with behavior DriveAvoidingCollisions(target_speed=Range(2, globalParameters.maximum_distractor_speed), avoidance_threshold=12)
    elif other_counter == 0:
        new Car ahead of ego by globalParameters.distance_to_nearest, facing roadDirection, with behavior DriveAvoidingCollisions(target_speed=Range(0, globalParameters.maximum_distractor_speed), avoidance_threshold=12)
        other_counter += 1
        cluster_placed += 1
    elif globalParameters.has_cluster == 1 and cluster_placed < cluster_size:
        new Car ahead of ego by globalParameters.distance_to_nearest + globalParameters.cluster_dist * cluster_placed, facing roadDirection, with behavior DriveAvoidingCollisions(target_speed=Range(0, globalParameters.maximum_distractor_speed), avoidance_threshold=12)
        other_counter += 1
        cluster_placed += 1
    else:
        min_dist = globalParameters.distance_to_nearest
        random_lane = Uniform(*[l for l in network.lanes if not str(l.uid).startswith('lane_:')])
        random_point = Uniform(*random_lane.centerline.points)
        d_car = new Car on (random_point[0] @ random_point[1]), with behavior DriveAvoidingCollisions(target_speed=Range(2, globalParameters.maximum_distractor_speed), avoidance_threshold=12)
        require (distance from ego to d_car) > min_dist
        other_counter += 1


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