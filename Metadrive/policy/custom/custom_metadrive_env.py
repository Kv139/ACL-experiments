from metadrive.envs import BaseEnv
from metadrive.manager.sumo_map_manager import SumoMapManager
from metadrive.obs.observation_base import DummyObservation
from metadrive.obs.state_obs import LidarStateObservation

from metadrive.utils import clip, Config


class DriveEnv(BaseEnv):

    def reward_function(self, agent):
        """
        Override this func to get a new reward function
        :param vehicle_id: id of BaseVehicle
        :return: reward
        """
        self.done = False
        vehicle = self.agents["default_agent"]
        step_info = dict()

        # Reward for moving forward in current lane
        if vehicle.lane in vehicle.navigation.current_ref_lanes:
            current_lane = vehicle.lane
        else:
            current_lane = vehicle.navigation.current_ref_lanes[0]
       
        positive_road = 1
        long_last, _ = current_lane.local_coordinates(vehicle.last_position)
        long_now, lateral_now = current_lane.local_coordinates(vehicle.position)

        # reward for lane keeping, without it vehicle can learn to overtake but fail to keep in lane
        if self.config["use_lateral_reward"]:
            lateral_factor = clip(1 - 2 * abs(lateral_now) / vehicle.navigation.get_current_lane_width(), 0.0, 1.0)
        else:
            lateral_factor = 1.0

        reward = 0.0
        reward += self.config["driving_reward"] * (long_now - long_last) * lateral_factor * positive_road
        reward += self.config["speed_reward"] * (vehicle.speed_km_h / vehicle.max_speed_km_h) * positive_road

        step_info["step_reward"] = reward

        if self._is_arrive_destination(vehicle):
            reward = +self.config["success_reward"]
            self.done = True
            step_info["arrive_dest"] = True
        elif self._is_out_of_road(vehicle):
            reward = -self.config["out_of_road_penalty"]
            self.done = True
            step_info["out_of_road"] = True
        elif vehicle.crash_vehicle:
            reward = -self.config["crash_vehicle_penalty"]
            self.done = True
            step_info["crash_vehicle"] = True
        elif vehicle.crash_object:
            reward = -self.config["crash_object_penalty"]
            self.done = True
            step_info["crash_object"] = True
        elif vehicle.crash_sidewalk:
            reward = -self.config["crash_sidewalk_penalty"]
            self.done = True
            step_info["crash_sidewalk"] = True

        return reward, step_info

    def cost_function(self, agent):
        """Dummy cost function."""
        return 0, {}

    def done_function(self, agent):
        """Dummy done function."""
        return self.done, {}

    def get_single_observation(self):
        """Dummy observation function."""
        return LidarStateObservation(self.config)

    def setup_engine(self):
        """Setup the engine for MetaDrive."""
        super().setup_engine()
        self.engine.register_manager(
            "map_manager", SumoMapManager(self.config["sumo_map"])
        )

    @staticmethod
    def _is_arrive_destination(vehicle):
        long, lat = vehicle.navigation.final_lane.local_coordinates(vehicle.position)
        flag = (vehicle.navigation.final_lane.length - 5 < long < vehicle.navigation.final_lane.length + 5) and (
            vehicle.navigation.get_current_lane_width() / 2 >= lat >=
            (0.5 - vehicle.navigation.get_current_lane_num()) * vehicle.navigation.get_current_lane_width()
        )
        return flag

    def _is_out_of_road(self, vehicle):
        ret = not vehicle.on_lane
        if self.config["out_of_route_done"]:
            ret = ret or vehicle.out_of_route
        elif self.config["on_continuous_line_done"]:
            ret = ret or vehicle.on_yellow_continuous_line or vehicle.on_white_continuous_line or vehicle.crash_sidewalk
        if self.config["on_broken_line_done"]:
            ret = ret or vehicle.on_broken_line
        return ret