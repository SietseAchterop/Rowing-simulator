import opensim as osim
import sys
import math
pi = math.pi

"""
Inertia of solid cylinder of radiusr, height h and mass m
I_z = 1/2 mr^2
I_x = I_y = 1/12 m(3r_2 + h^2)
"""


bbaan = osim.Model()

bbaan.setName("Pusher")
bbaan.setGravity(osim.Vec3(0, -9.90665, 0))

"""   The bodies with geometry   """

# Upper leg
I_x = (1/2)*1*0.05**2
I_yz = 1/12*1*(3*0.05**2+6**2)
print(I_x, I_yz)
upper = osim.Body("The_upper_leg",
                  1.0,
                  osim.Vec3(0, 0, 0),
                  osim.Inertia(I_x, I_yz, I_yz, 0, 0, 0))

upperGeometry = osim.Cylinder(0.1, 1)
upper.attachGeometry(upperGeometry)
upperGeometry.setColor(osim.Vec3(0.5, 0.5, 1))

# Lower leg
lower = osim.Body("The_lower_leg",
                  1.0,
                  osim.Vec3(0, 0, 0),
                  osim.Inertia(I_x, I_yz, I_yz, 0, 0, 0))

lowerGeometry = osim.Cylinder(0.1, 1.5)
lower.attachGeometry(lowerGeometry)
lowerGeometry.setColor(osim.Vec3(1, 0, 1))

# the foot
blade = osim.Body()
blade.setName('TheBlade')
blade.setMass(1)
blade.setInertia(osim.Inertia(1, 1, 1, 0, 0, 0))

# boot voorlopig een brick, later een meshfile van een boot
bladeGeometry = osim.Brick(osim.Vec3(0.03, 0.6, 0.6))
blade.attachGeometry(bladeGeometry)
bladeGeometry.setColor(osim.Vec3(1, 1, 1))


"""  The joints   """

baseJoint = osim.PinJoint("baseJoint",
                          bbaan.getGround(),
                          osim.Vec3(5, 0.525, 0),
                          osim.Vec3(0, 0, 0),
                          upper,
                          osim.Vec3(0, -1, 0),
                          osim.Vec3(0, 0, 0))
coord = baseJoint.updCoordinate()
coord.setName('baseangle')
#coord.set_range(0, -1.4)
coord.setRangeMin(-1.4)
coord.set_range(1,  1.0)
# coord.setDefaultValue(math.radians(-2.027))

act_1 = osim.CoordinateActuator('baseangle')
act_1.setName('act_1')
bbaan.addForce(act_1)

controller = osim.PrescribedController()

controller.addActuator(act_1)
controller.prescribeControlForActuator('act_1', osim.Constant(0.0))

lowerJoint = osim.PinJoint("lowerJoint",
                           upper,
                           osim.Vec3(0, 1, 0),
                           osim.Vec3(0, 0, 0),
                           lower,
                           osim.Vec3(0, -1.5, 0),
                           osim.Vec3(0, 0, pi/2))


coord = lowerJoint.updCoordinate()

coord.setName('kneeangle')
coord.set_range(0, -1.2)
coord.set_range(1,  pi/2)
# coord.setDefaultValue(math.radians(-50.0))

act_2 = osim.CoordinateActuator('kneeangle')
act_2.setName('act_2')
bbaan.addForce(act_2)

controller.addActuator(act_2)
controller.prescribeControlForActuator('act_2', osim.Constant(0.0))

bladeJoint = osim.PinJoint("bladeJoint",
                           lower,
                           osim.Vec3(0, 1.5, 0),
                           osim.Vec3(0, 0, 0),
                           blade,
                           osim.Vec3(0, 0, 0),
                           osim.Vec3(0, 0, 0))

coord = bladeJoint.updCoordinate()
coord.setName('bladeangle')
coord.set_range(0,  0)
coord.set_range(1,  pi/2)
# coord.setDefaultValue(math.radians(55.0))

act_3 = osim.CoordinateActuator('bladeangle')
act_3.setName('act_3')
bbaan.addForce(act_3)

controller.addActuator(act_3)
controller.prescribeControlForActuator('act_3', osim.Constant(0.0))

bbaan.addController(controller)

bbaan.addBody(upper)
bbaan.addBody(lower)
bbaan.addBody(blade)
bbaan.addJoint(baseJoint)
bbaan.addJoint(lowerJoint)
bbaan.addJoint(bladeJoint)

# markers
marker = osim.Marker('mbasej', bbaan.getGround(), osim.Vec3(5, 0.525, 0))
bbaan.addMarker(marker)
marker = osim.Marker('mbladej', lower, osim.Vec3(0, 1.5, 0))
bbaan.addMarker(marker)
marker = osim.Marker('mend', blade, osim.Vec3(0, 0.6, 0))
bbaan.addMarker(marker)


reporter = osim.ConsoleReporter()
reporter.set_report_time_interval(1.0)
bbaan.addComponent(reporter)

bbaan.finalizeConnections()
bbaan.printToXML("Pusher.osim")

# uncomment the rest to directly get the simulation
"""
bbaan.setUseVisualizer(True)
state = bbaan.initSystem()

print(state.getY())
finalState = osim.simulate(bbaan, state, 10)

# Analyze a simulation.
print(coord.getValue(finalState))
model.realizePosition(finalState)
print(model.calcMassCenterPosition(finalState))
model.realizeAcceleration(finalState)
print(joint.calcReactionOnParentExpressedInGround(finalState))

"""
