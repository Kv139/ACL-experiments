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

class WebotsFrankaSimulator(Simulator):
    """`Simulator` object for Webots.

    Args:
        supervisor: Supervisor node handle from the Webots Python API.
    """

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
        return WebotsFrankaSimulation(
            scene, self.supervisor, coordinateSystem=self.coordinateSystem, **kwargs
        )
    

class WebotsFrankaSimulation(Simulation):
    """`Simulation` object for Webots.

    Attributes:
        supervisor: Webots supervisor node used for the simulation. This is
            exposed for the use of scenarios which need to call Webots APIs
            directly; e.g. :scenic:`simulation().supervisor.setLabel({...})`.
    """

    def __init__(self, scene, supervisor, coordinateSystem=ENU, *, timestep, **kwargs):
        self.supervisor = supervisor
        self.coordinateSystem = coordinateSystem
        self.mode2D = scene.compileOptions.mode2D
        self.nextAdHocObjectId = 1
        self.usedObjectNames = defaultdict(lambda: 0)

        self.timestep = supervisor.getBasicTimeStep() / 1000 if timestep is None else timestep
        # directory to store proto files for adhoc webots objects
        self.tmpMeshDir = tempfile.mkdtemp()

        self.supervisor_node = self.supervisor.getSelf()

        # Grab motor/sensor objects (named in the proto)
        sensor_names = [f"panda_joint{i}_sensor" for i in range(1,8)]
        self.postition_sensors = [self.supervisor.getDevice(name) for name in sensor_names]
        
        motor_names = [f"panda_joint{i}" for i in range(1,8)]
        self.motors = [self.supervisor.getDevice(name) for name in motor_names]



        self.total_reward = 0

        self.enable_sensors = False
        self.ms = round(1000 * self.timestep)

        self.target = [0,0,0]

        self.observation = [self.target[0], 
                            self.target[1],
                            self.target[2],
                            0,0,0,0,0,0,0]

        self.actions = np.zeros(7)

        super().__init__(scene, timestep=timestep, **kwargs)

    def setup(self):
        super().setup()
        # Reset Webots simulation
        self.supervisor.simulationResetPhysics()
 

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

    def step(self): # action should be some low level control commands for the robot
        if not self.enable_sensors: 
               # print("Protections failed sensors were not initialized before calling") 
               # TODO more elegant fix here, sensor need to be adaquetly initialized before the simlation begins stepping
                self.init_step()

        self.observation = [sensor.getValue() for sensor in self.postition_sensors]
        
        for action,motor in zip(self.actions, self.motors):
            print(action)
            motor.setVelocity(action*0.032)

        self.supervisor.step(self.ms)


    def init_step(self):
        """
        Initialize all the sensors and devices on the robot
        """

        # initialize motors
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0)

        # initialize Sensors
        for sensor in self.postition_sensors:
            sensor.enable(self.ms)


        self.supervisor_node.enablePoseTracking(self.ms)

        self.supervisor.step(self.ms) # Need to step the simulation once after initializing the sensors!
       
        target_obj = self.supervisor.getFromDef("sphere")
        if target_obj:
            pos = np.array(target_obj.getField('translation').getSFVec3f, dtype=np.float32) #sphere is rotationaly invariant, pose won't matter here
            self.pos = pos # initialize the position
        else:
            print(f"Object not found")

        
        self.enable_sensors = True


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

    def destroy(self):
        # Destroy adhoc objects generated at the beginning of the simulation      
        for i in range(1, self.nextAdHocObjectId):
            name = self._getAdhocObjectName(i)
            node = self.supervisor.getFromDef(name)
            node.remove()
        self.supervisor.step(self.ms) # TODO this fixe crashing error on repeated reset calls! I DO NOT KNOW WHY.... temp fix, need to figure out underlying cause
    
    def _getAdhocObjectName(self, i: int) -> str:
        return f"SCENIC_ADHOC_{i}"

    def get_reward(self): # "any dummy for now will be okay"
        """
        Calculate the reward based off of the current state
        """

        return 0

    
    def get_info(self):
        """
        Any information about the system/state that should be retained
        """
        return {}
     
    def get_obs(self):
        """
        Return the current state of the enviroment
        """
        return self.observation
    
    def get_truncation(self): 
        """ 
        checks flag for early termination you will need to modify you ScenicGym class to check this condition!
            
        """
        return False
    
    def sampler_feedback(self): 
        """
        sends simulation feedback to the run-loop for samplers 
        """
        return 0

            

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