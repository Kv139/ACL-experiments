param map = localPath('../CARLA/Town01.xodr')
param carla_map = 'Town01'

model scenic.simulators.metadrive.model

param time_step = 1.0/10
param verifaiSamplerType = 'halton'
param render = 0
param use2DMap = True

import numpy as np
TERMINATE_TIME = 40 / globalParameters.time_step

"""
Setting global params for the road, land, starting and stoping
Scene defining variables should be global params in order to allow for
easily mutating later
"""

def get_nearest_centerline(obj):
	min_dist = np.inf
	for lane in globalParameters.select_road.lanes:
		dist = distance to lane
		if dist < min_dist:
			min_dist = dist
			centerline = lane.centerline
	return centerline

# TODO fix params -- need more variabiltiy and ensure that modified scenes make!
param select_road = Uniform(*network.roads)
param select_lane = Uniform(*globalParameters.select_road.lanes)

param start_x = Uniform(*globalParameters.select_lane.centerline.points[0])
param start_y = Uniform(*globalParameters.select_lane.centerline.points[1])


start = (globalParameters.start_x @ globalParameters.start_y)

ego = new Car on start, with observation 0, with cte 0 
ego.previous_coordinates = [0,0]

monitor DrivingReward(obj, select_lane=globalParameters.select_lane):
	while True:
		ego.previous_coordinates = obj.position
		# if select_lane:
		# 	centerline = select_lane.centerline
		# else:
		lane = obj._lane
		if lane:
			centerline = lane.centerline	
		else:
			centerline = get_nearest_centerline(obj)

		nearest_line_points = centerline.nearestSegmentTo(obj.position)
		nearest_line_segmenet = PolylineRegion(nearest_line_points)
		
		cte = abs(distance to nearest_line_segmenet)
		if cte < .1: 
			reward=1
		else:
			reward =  -cte + 0.1 * ego.speed
		
		ego.reward = reward
		wait
	

require monitor DrivingReward(ego,None)