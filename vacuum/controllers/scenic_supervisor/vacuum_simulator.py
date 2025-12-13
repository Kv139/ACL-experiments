"""Interface to Webots for dynamic simulations.

This interface is intended to be instantiated from inside the controller script
of a Webots `Robot node`_ with the ``supervisor`` field set to true. Such a
script can create a `WebotsSimulator` (passing in a reference to the supervisor
node) and then call its `simulate` method as usual to run a simulation. For an
example, see :file:`examples/webots/generic/controllers/scenic_supervisor.py`.

Scenarios written for this interface should use our generic Webots world model
:doc:`scenic.simulators.webots.model` or a model derived from it. Objects which
are instances of `WebotsObject` will be matched to Webots nodes; see the model
documentation for details.

.. _Robot node: https://www.cyberbotics.com/doc/reference/robot
"""

from collections import defaultdict
import ctypes
import math
from os import path
import tempfile
from textwrap import dedent

import numpy as np
import trimesh

from scenic.core.regions import MeshVolumeRegion
from scenic.core.simulators import Simulation, Simulator
from scenic.core.type_support import toOrientation
from scenic.core.vectors import Vector
from scenic.simulators.webots.utils import ENU, WebotsCoordinateSystem
from controller import DistanceSensor

from trimesh.creation import box

import yaml

import math, numpy as np
from trimesh.proximity import closest_point
from trimesh.proximity import ProximityQuery
episodes = 0
saved_stepcount = 0
class CustomWebotsSimulator(Simulator):
    """`Simulator` object for Webots.

    Args:
        supervisor: Supervisor node handle from the Webots Python API.
    """
    episode_count = 0
    current_simulation = None
    last_avg_return = None
    
    def __init__(self, supervisor):
        super().__init__()
        self.supervisor = supervisor
        topLevelNodes = supervisor.getRoot().getField("children")
        worldInfo = None
        for i in range(topLevelNodes.getCount()):
            child = topLevelNodes.getMFNode(i)
            if child.getTypeName() == "WorldInfo":
                worldInfo = child
                break
        if not worldInfo:
            raise RuntimeError("Webots world does not contain a WorldInfo node")
        system = worldInfo.getField("coordinateSystem").getSFString()
        self.coordinateSystem = WebotsCoordinateSystem(system)

    def createSimulation(self, scene, **kwargs):
        self.episode_count += 1
        self.current_simulation = CustomWebotsSimulation(
            scene, self.supervisor, coordinateSystem=self.coordinateSystem, **kwargs
        )
        return self.current_simulation


class CustomWebotsSimulation(Simulation):
    """`Simulation` object for Webots.

    Attributes:
        supervisor: Webots supervisor node used for the simulation. This is
            exposed for the use of scenarios which need to call Webots APIs
            directly; e.g. :scenic:`simulation().supervisor.setLabel({...})`.
    """
    def __init__(self, scene, supervisor, coordinateSystem=ENU, *, timestep, **kwargs):
        #room data
        self.room_width = 3.5    
        self.room_length = 3.5   
        self.granularity = 0.1    
        self.total_spaces = (2 * np.floor(self.room_width / (2*self.granularity)) + 1)**2 - 4 #-4 for each of the corners
        self.obj_dims = []
        
        #collisions & collision detection
        self.collisions = 0
        self.discrete_collisions = 0
        self.colliding = False
        self.collision_safeguard = 0
        self.inter_penalty = False
        self.prox_checks = []
        self.spheres = []
        
        #metrics and rewards
        self.best_coverage = 0,0
        self.covered_spaces = []
        self.coverage_timesteps = []
        self.invalid_action = False
        self.total_reward = 0
        
        #simulation data
        self.time_elapsed = 0
        self.total_reward = 0
        self.total_steps = 0
        
        #i dont know
        self.supervisor = supervisor
        self.coordinateSystem = coordinateSystem
        self.mode2D = scene.compileOptions.mode2D
        self.nextAdHocObjectId = 1
        self.usedObjectNames = defaultdict(lambda: 0)
        self.timestep = supervisor.getBasicTimeStep() / 500 if timestep is None else timestep
        # directory to store proto files for adhoc webots objects
        self.tmpMeshDir = tempfile.mkdtemp()
        self.supervisor_node = self.supervisor.getSelf()

        #device inputs
        self.left_motor = self.supervisor.getDevice("right wheel motor")
        self.right_motor = self.supervisor.getDevice("left wheel motor")

        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        self.velocity_ranges = [0,16.129]

        self.enable_sensors = False
        self.actions = [0,0]
        self.ms = round(1000 * self.timestep)
        self.LIDAR = self.supervisor.getDevice("LIDAR")
        self.vroom_reward = False
        self.collision_penalty =False

        #observation space
        self.observation = {
            "velocity": np.zeros(2), 
            "position": np.zeros(2),
            # "sensor":   np.zeros(7),
            "lidar" :   np.zeros(shape=(128,)),
            "rotation": 0,
            "coverage": 0
        } # TODO Need to fix obs and initialziation        
        
        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        super().setup()
        # Reset Webots simulation
        self.supervisor.simulationResetPhysics()
        self.compute_total_tiles()


    def createObjectInSimulator(self, obj):
        if not hasattr(obj, "webotsName"):
            return  # not a Webots object

        # Find the name of the Webots node for this object.
        name = None
        if obj.webotsAdhoc is not None:
            # Dynamically generate object from Scenic object
            objectRawMesh = obj.shape.mesh
            objectScaledMesh = MeshVolumeRegion(
                mesh=objectRawMesh,
                dimensions=(obj.width, obj.length, obj.height),
            ).mesh
            objFilePath = path.join(self.tmpMeshDir, f"{self.nextAdHocObjectId}.obj")
            trimesh.exchange.export.export_mesh(objectScaledMesh, objFilePath)

            name = self._getAdhocObjectName(self.nextAdHocObjectId)
            protoName = (
                "ScenicObjectWithPhysics" if isPhysicsEnabled(obj) else "ScenicObject"
            )
          
 
            objFilePath = str(objFilePath).replace("\\", "\\\\")# Temporary fix, not sure if this is the right way to do this? hmm

            protoDef = dedent(
                f"""
                DEF {name} {protoName} {{
                    url "{objFilePath}"
                }}
                """
            )
            rootNode = self.supervisor.getRoot()
            rootChildrenField = rootNode.getField("children")
            rootChildrenField.importMFNodeFromString(-1, protoDef)
            self.nextAdHocObjectId += 1
        else:
            if obj.webotsName:
                name = obj.webotsName
            else:
                ty = obj.webotsType
                if not ty:
                    raise RuntimeError(f"object {obj} has no webotsName or webotsType")
                nextID = self.usedObjectNames[ty]
                self.usedObjectNames[ty] += 1
                if nextID == 0 and self.supervisor.getFromDef(ty):
                    name = ty
                else:
                    name = f"{ty}_{nextID}"

        # Get handle to Webots node.
        webotsObj = self.supervisor.getFromDef(name)
        if webotsObj is None:
            raise SimulationCreationError(f"Webots object {name} does not exist in world")
        obj.webotsObject = webotsObj
        obj.webotsName = name

        # Set the fields of the Webots object:

        # position
        if self.mode2D:  # 2D compatibility mode
            # Set initial elevation if unspecified
            if obj.elevation is None:
                pos = webotsObj.getField("translation").getSFVec3f()
                spos = self.coordinateSystem.positionToScenic(pos)
                obj.elevation = spos[2]

            # Overwrite Z value with elevation
            pos = self.coordinateSystem.positionFromScenic(
                Vector(obj.position.x, obj.position.y, obj.elevation) + obj.positionOffset
            )
            webotsObj.getField("translation").setSFVec3f(pos)
        else:
            pos = self.coordinateSystem.positionFromScenic(
                obj.position + obj.positionOffset
            )
            webotsObj.getField("translation").setSFVec3f(pos)

        # orientation
        offsetOrientation = toOrientation(obj.rotationOffset)
        webotsObj.getField("rotation").setSFRotation(
            self.coordinateSystem.orientationFromScenic(
                obj.orientation, offsetOrientation
            )
        )

        # density
        densityField = getFieldSafe(webotsObj, "density")
        if densityField is not None:
            if obj.density is None:
                # Get initial value for property if unspecified
                obj.density = densityField.getSFFloat()
            else:
                densityField.setSFFloat(float(obj.density))

        # battery
        battery = getattr(obj, "battery", None)
        if battery:
            if not isinstance(battery, (tuple, list)) or len(battery) != 3:
                raise TypeError(f'"battery" of {name} does not have 3 components')
            field = webotsObj.getField("battery")
            field.setMFFloat(0, battery[0])
            field.setMFFloat(1, battery[1])
            field.setMFFloat(2, battery[2])

        # customData
        customData = getattr(obj, "customData", None)
        if customData:
            if not isinstance(customData, str):
                raise TypeError(f'"customData" of {name} is not a string')
            webotsObj.getField("customData").setSFString(customData)

        # controller
        if obj.controller:
            controllerField = webotsObj.getField("controller")
            curCont = controllerField.getSFString()
            if obj.controller != curCont:
                # the following operation also causes the controller to be restarted
                controllerField.setSFString(obj.controller)
            elif obj.resetController:
                webotsObj.restartController()
        if hasattr(obj, 'width') and hasattr(obj, 'length'):
            name = (obj.webotsName or obj.webotsType or "")
            if "SCENIC_ADHOC7" in name:
                for _ in range(4):
                    self.obj_dims.append((0.1, 0.1))
            else:
                self.obj_dims.append((float(obj.width), float(obj.length)))
            
            
    def compute_total_tiles(self):
        room_area = self.room_width * self.room_length
        object_area = sum(width * length for width, length in self.obj_dims)
        cleanable_area = room_area - object_area

        tile_area = self.granularity ** 2
        total_tiles = int(cleanable_area / tile_area)
        self.total_spaces = total_tiles
        #print(f"Computed total cleanable tiles: {total_tiles}")
                
    def get_coverage_metric(self):
        # Number of unique positions visited
        covered_count = len(self.covered_spaces)
        # Coverage ratio (fraction of total spaces covered)
        coverage_ratio = covered_count / self.total_spaces
        # Optionally: return both count and percentage
        return covered_count, coverage_ratio          
                
    def step(self): # action should be some low level control commands for the robot
        if not self.enable_sensors: 
               # print("Protections failed sensors were not initialized before calling") 
               # TODO more elegant fix here, sensor need to be adaquetly initialized before the simlation begins stepping
                self.init_step()

        self.total_steps += 1
        global saved_stepcount
        saved_stepcount += 1
        # if(saved_stepcount % 100 == 0):
        #     print(f"Saved step count: {saved_stepcount}")
        self.pos = self.granularity * np.round(np.array(self.supervisor_node.getPosition()[:2]) / self.granularity)
        # TODO Normalize observation space, docmumnet sensor value ranges, and signals for crashing etc...
        raw_lidar = np.array(self.LIDAR.getRangeImage(), dtype=np.float64)
        
        cleaned_lidar = np.nan_to_num(raw_lidar,nan=.5,posinf=.5,neginf=0.00)
        self.observation = {
            "velocity": np.array([self.actions[0], self.actions[1]]),
            "position": np.array([self.pos[0]/1.75, self.pos[1]/1.75]),
            "lidar"   :    cleaned_lidar,
            "rotation": self.objects[0].heading,
            "coverage": self.best_coverage[1]
        }
        self.transform_vel()
        self.left_motor.setVelocity(self.actions[0]) 
        self.right_motor.setVelocity(self.actions[1])
        self.supervisor.step(self.ms)

        covered_count, coverage_ratio = self.get_coverage_metric()
        if coverage_ratio > self.best_coverage[1]:
            self.best_coverage = covered_count, coverage_ratio

        
    def getObjects(self):
        for obj in self.objects:
            if "unnamed floor" in str(obj).lower() or "unnamed vacuum" in str(obj).lower():
                continue
            x, y, z = obj.position
            yaw = obj.heading
            c, s = math.cos(yaw), math.sin(yaw)
            # 4×4 yaw+translate
            T = np.array([
                [ c, -s, 0, x],
                [ s,  c, 0, y],
                [ 0,  0, 1, z],
                [ 0,  0, 0, 1]
            ])
            base = obj.shape._mesh                     
            dims = (obj.width, obj.length, obj.height)  
            mesh = MeshVolumeRegion(mesh=base, dimensions=dims).mesh
            mesh_in_world = mesh.copy()
            mesh_in_world.apply_transform(T)
            self.prox_checks.append(ProximityQuery(mesh_in_world))
            self.spheres.append([obj.position] + [mesh_in_world.bounding_sphere.primitive.radius])

    def init_step(self):
        """
        Initialize all the sensors and devices on the robot
        """
        self.LIDAR.enable(self.ms)

        self.supervisor.step(self.ms) # Need to step the simulation once after initializing the sensors!
        pos = self.granularity * np.round(np.array(self.supervisor_node.getPosition()[:2]) / self.granularity) #need to verify
        self.pos = pos # initialize the position
        self.enable_sensors = True
        self.getObjects()


    def getProperties(self, obj, properties):
        webotsObj = getattr(obj, "webotsObject", None)
        if not webotsObj:  # static object with no Webots counterpart
            return {prop: getattr(obj, prop) for prop in properties}

        pos = webotsObj.getField("translation").getSFVec3f()
        x, y, z = self.coordinateSystem.positionToScenic(pos)
        lx, ly, lz, ax, ay, az = webotsObj.getVelocity()
        vx, vy, vz = self.coordinateSystem.positionToScenic((lx, ly, lz))
        velocity = Vector(vx, vy, vz)
        speed = math.hypot(*velocity)
        angularSpeed = math.hypot(ax, ay, az)

        offsetOrientation = toOrientation(obj.rotationOffset)
        globalOrientation = self.coordinateSystem.orientationToScenic(
            webotsObj.getField("rotation").getSFRotation(), offsetOrientation
        )
        yaw, pitch, roll = obj.parentOrientation.localAnglesFor(globalOrientation)

        values = dict(
            position=Vector(x, y, z),
            velocity=velocity,
            speed=speed,
            angularSpeed=angularSpeed,
            angularVelocity=Vector(ax, ay, az),
            yaw=yaw,
            pitch=pitch,
            roll=roll,
            elevation=z,
        )

        if hasattr(obj, "battery"):
            field = webotsObj.getField("battery")
            val = (field.getMFFloat(0), obj.battery[1], obj.battery[2])
            values["battery"] = val

        return values
    
    def metric(self):

        avg_reward = self.total_reward / self.total_steps if self.total_steps > 0 else 0
        exploration = len(self.covered_spaces)
        collision_rate = self.collisions / self.total_steps if self.total_steps > 0 else 0

        score = avg_reward - 10 * collision_rate + 0.1 * exploration

        return {
            "total_reward": self.total_reward,
            "average_reward": avg_reward,
            "steps": self.total_steps,
            "time_elapsed": self.time_elapsed,
            "collision_count": self.collisions,
            "exploration_score": exploration,
            "final_score": score
        }

    def destroy(self):
        global episodes
        episodes += 1
        print(f"Episode number: {episodes}")
        print(f"Covered {self.best_coverage[0]} cells out of {self.total_spaces} ({self.best_coverage[1]*100:.2f}%)")


        for i in range(1, self.nextAdHocObjectId):
            name = self._getAdhocObjectName(i)
            node = self.supervisor.getFromDef(name)
            if node is not None: # ensure that the node actually exisits in the simulation before destroying it
                node.remove()
            self.supervisor.step(self.ms) # TODO this fixe crashing error on repeated reset calls! I DO NOT KNOW WHY.... temp fix, need to figure out underlying cause
    def _getAdhocObjectName(self, i: int) -> str:
        return f"SCENIC_ADHOC_{i}"
    

    def get_coverage_reward(self, granularity, pos):
            reward = 0
            #important parameter
            radius = .335/2
            x_range = np.arange(pos[0] - radius - granularity, pos[0] + radius + granularity, granularity)
            y_range = np.arange(pos[1] - radius - granularity, pos[1] + radius + granularity, granularity)
            x_range_combined, y_range_combined = np.meshgrid(x_range, y_range, indexing="xy")
            mask = (x_range_combined - pos[0])**2 + (y_range_combined - pos[1])**2 <= radius**2
            circle_points = [
                (
                    round(granularity * round(x / granularity), 3),
                    round(granularity * round(y / granularity), 3)
                )
                for x, y in np.vstack((x_range_combined[mask],
                                    y_range_combined[mask])).T
            ]
            for point in circle_points:
                if(point not in self.covered_spaces):
                    reward += .5
                    self.covered_spaces.append(point)
                    self.coverage_timesteps.append(self.total_steps)
            if reward == 0:
                reward += -.001
            return reward

    def checkCollisions(self):
        minDist = 0.01
        if np.any(self.observation['lidar'] < minDist):
                return True
        for i in range(len(self.prox_checks)):
            if math.dist(self.spheres[i][0], self.records["EgoPosition"][len(self.records["EgoPosition"]) - 1][1]) > .335/2 + self.spheres[i][1] + minDist:
                continue  
            if(abs(self.prox_checks[i].signed_distance(np.array([self.records["EgoPosition"][len(self.records["EgoPosition"]) - 1][1]]))) < .335/2 + minDist):
                return True
        return False

    def get_reward(self): # "any dummy for now will be okay"
        """
        Calculate the reward based off of the current state
        """
        pos = self.granularity * np.round(np.array(self.supervisor_node.getPosition()[:2]) / self.granularity) #need to verify
        pos = tuple(pos.tolist())
        reward = self.get_coverage_reward(self.granularity, [pos[0], pos[1]])
        
        # if np.all(self.observation["velocity"] > 0):
        #     reward += .2 # small reward for driving forwa
        
        if (self.checkCollisions()): # if any distance sensor is low penalize
            if self.collision_penalty:
                reward += -.001
        if (self.vroom_reward):
            reward += .05

        self.total_reward += reward
        return reward

    def get_info(self):
        """
        Any information about the system/state that should be retained
        """
        cleaned_cells = len(self.covered_spaces)
        total_cleanable_tiles = self.total_spaces
        coverage_percent = (cleaned_cells / total_cleanable_tiles) * 100 if total_cleanable_tiles > 0 else 0

        return {
            "cleaned_cells": cleaned_cells,
            "total_cleanable_tiles": total_cleanable_tiles,
            "coverage": coverage_percent,
            "collisions": self.collisions,
            "discrete_collisions": self.discrete_collisions
        }
     
     
    def get_obs(self):
        """
        Return the current state of the enviroment
        """
        return self.observation
    
    def transform_vel(self): 
        """
        Maps the actors actions from (-1,1) to the actual motor range
        of the robot system
        """
        self.actions[0] = self.actions[0] * self.velocity_ranges[1] 
        self.actions[1] = self.actions[1] * self.velocity_ranges[1]

        if np.all(self.actions > 10):
            self.vroom_reward = True
        else:
            self.vroom_reward = False
    
        if np.all(self.actions < 3):
            self.collision_penalty = True
        else:
            self.collision_penalty = False

    
    def get_truncation(self):
        if self.collision_safeguard > 50:
            return True
        else:
            return False

def getFieldSafe(webotsObject, fieldName):
    """Get field from webots object. Return null if no such field exists.

    Needed to workaround this issue (https://github.com/cyberbotics/webots/issues/5646)

    Args:
        webotsObject: webots object
        fieldName: name of the field to look for

    Returns:
        Field|None: Field object if the field with the given name exists. None otherwise.
    """

    field = webotsObject.getField(fieldName)
    # this seems to always return some object, but return None if field is None
    if field is None:
        return None

    # if field is valid, it has a valid pointer
    if isinstance(field._ref, ctypes.c_void_p) and field._ref.value is not None:
        # then the field is valid and we return the reference
        return field

    # if the pointer points to None, then the field does not exist on this object
    return None


def isPhysicsEnabled(webotsObject):
    """Whether or not physics is enabled for this `WebotsObject`"""
    if webotsObject.webotsAdhoc is None:
        return webotsObject
    if isinstance(webotsObject.webotsAdhoc, dict):
        return webotsObject.webotsAdhoc.get("physics", True)
    raise TypeError(f"webotsAdhoc must be None or a dictionary")
