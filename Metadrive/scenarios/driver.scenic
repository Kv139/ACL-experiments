param map = localPath('../CARLA/Town01.xodr')
param carla_map = 'Town01'

model scenic.simulators.metadrive.model

param time_step = 1.0/10
param verifaiSamplerType = 'halton'
param render = 1
param use2DMap = True

TERMINATE_TIME = 40 / globalParameters.time_step

"""
Setting global params for the road, land, starting and stoping
Scene defining variables should be global params in order to allow for
easily mutating later
"""

param select_road = Uniform(*network.roads)
param select_lane = Uniform(*globalParameters.select_road.lanes)

param start_x = Uniform(*globalParameters.select_lane.centerline.points[0])
param start_y = Uniform(*globalParameters.select_lane.centerline.points[1])


start = (globalParameters.start_x @ globalParameters.start_y)

id = 0
ego = new Car on start 

ego.previous_coordinates = [0,0]


monitor DrivingReward(obj, select_lane=None):
	ego.previous_coordinates = obj.position
	if select_lane:
		centerline = select_lane.centerline
	else:
		centerline = obj.lane.centerline
	
	center_line_dist = centerline.signedDistanceTo(obj.position)

	ego.reward = ego.speed - center_line_dist
	wait
	#* distance from self to centerline

	# print(f'Computed reward was {ego.reward}')
	

require monitor DrivingReward(ego,None)




# TODO add distractors such as pedestrians etc
# right_sidewalk = network.laneGroupAt(ego)._sidewalk
# new Pedestrian on visible right_sidewalk

def true_dist(car1,car2):
	bp1 = car1._boundingPolygon
	bp2 = car2._boundingPolygon
	return bp1.distance(bp2)

terminate when (distance from ego to start) > 760