model scenic.simulators.webots.model

"""
Here we can define objects to be included in the Scenic program. Objects should 
inherit from the WebotsObject class. Custom meshes can be used for this as well
as base shapes from Scenic as shown above. 

Read here for a complete list of the WebotsObject attributes:
https://docs.scenic-lang.org/en/latest/modules/scenic.simulators.webots.model.html#module-scenic.simulators.webots.model
"""

class Floor(Object):
    width: 2
    length: 2
    height: 0.01
    position: (0,0,0.0)
    color: [0.785, 0.785, 0.785]

class centerObj(WebotsObject):
    webotsName: "CENTER"
    width: 0.000001
    length: 0.000001
    height: 0.000003
    position: (0,0,0.5)
    shape: CylinderShape()

class OBSTACLE(WebotsObject):
    webotsName: "OBSTACLE"
    width: 0.02
    length: 0.02
    height: 0.1
    shape: CylinderShape()

class TARGET(WebotsObject):
    webotsAdhoc: {'physics': False}
    webotsName: "TARGET"
    width: 0.02
    length: 0.02
    height: 0.1
    shape: CylinderShape()

class circle_obj(WebotsObject):
    shape: SpheroidShape()
    width: .1
    length: .1
    height: .1
    webotsAdhoc: {'physics': False}
    is_target: True


class distractor_obj(WebotsObject):
    shape: SpheroidShape()
    width: .1
    length: .1
    height: .1
    webotsAdhoc: {'physics': True}
    is_target: False

class Franka(WebotsObject):
    webotsName: "PANDA"
    shape: CylinderShape()
    width: .022 # TODO find the accurate dimensions
    heigth: .022
    length: .022 
    customData: "../scenarios/panda_goal_reaching.scenic"
    Supervisor: True
    controller: "scenic_supervisor"
    resetController: False
    reward: 0

