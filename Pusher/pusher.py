import opensim as osim
import sys
import math
pi = math.pi


bbaan = osim.Model()

bbaan.setName("Pusher")
bbaan.setGravity(osim.Vec3(0, -9.90665, 0))

"""   The bodies with geometry   """
theCourse = osim.Body("The_rowing_course",
                      1.0,
                      osim.Vec3(0, 0, 0),
                      osim.Inertia(1, 1, 1, 0, 0, 0))

theCourseGeometry = osim.Brick(osim.Vec3(20, 0.05, 3.5))
theCourse.attachGeometry(theCourseGeometry)
theCourseGeometry.setColor(osim.Vec3(0, 0, 1))

theBoat = osim.Body()
theBoat.setName('TheBoat')
theBoat.setMass(50)
theBoat.setInertia(osim.Inertia(7, 7, 45, 0, 0, 0))

# boot voorlopig een brick, later een meshfile van een boot
theBoatGeometry = osim.Brick(osim.Vec3(3, 0.1, 0.3))
theBoat.attachGeometry(theBoatGeometry)
theBoatGeometry.setColor(osim.Vec3(0, 1, 1))

# Upper leg
upper = osim.Body("The_upper_leg",
                  1.0,
                  osim.Vec3(0, 0, 0),
                  osim.Inertia(1, 1, 1, 0, 0, 0))

upperGeometry = osim.Cylinder(0.1, 1)
upper.attachGeometry(upperGeometry)
upperGeometry.setColor(osim.Vec3(0.5, 0.5, 1))

# Lower leg
lower = osim.Body("The_lower_leg",
                  1.0,
                  osim.Vec3(0, 0, 0),
                  osim.Inertia(1, 1, 1, 0, 0, 0))

lowerGeometry = osim.Cylinder(0.1, 1.5)
lower.attachGeometry(lowerGeometry)
lowerGeometry.setColor(osim.Vec3(1, 0, 1))

# the foot
blade = osim.Body()
blade.setName('TheBlade')
blade.setMass(5)
blade.setInertia(osim.Inertia(1, 1, 1, 0, 0, 0))

# boot voorlopig een brick, later een meshfile van een boot
bladeGeometry = osim.Brick(osim.Vec3(0.025, 0.3, 0.3))
blade.attachGeometry(bladeGeometry)
bladeGeometry.setColor(osim.Vec3(1, 1, 1))


"""  The joints   """
courseJoint = osim.WeldJoint("courseJoint",
                             bbaan.getGround(),
                             osim.Vec3(0, 0.05, 0),
                             osim.Vec3(0, 0, 0),
                             theCourse,
                             osim.Vec3(0, 0, 0),
                             osim.Vec3(0, 0, 0))
                        
# later customjoint
boatJoint = osim.FreeJoint("boatJoint",
                           theCourse,
                           osim.Vec3(0, 0.2, 0),
                           osim.Vec3(0, 0, 0),
                           theBoat,
                           osim.Vec3(0, 0, 0),
                           osim.Vec3(0, 0, 0))
coord = boatJoint.upd_coordinates(0)
coord.set_clamped(True)
coord = boatJoint.upd_coordinates(1)
coord.set_clamped(True)
coord = boatJoint.upd_coordinates(2)
coord.set_clamped(True)
coord = boatJoint.upd_coordinates(3)
coord.set_clamped(True)
coord = boatJoint.upd_coordinates(4)
coord.set_clamped(True)
coord = boatJoint.upd_coordinates(5)
coord.set_clamped(True)

                        
baseJoint = osim.PinJoint("baseJoint",
                          theBoat,
                          osim.Vec3(2, 0.1, 0),
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
act_1.setName('baseangle')
bbaan.addForce(act_1)

controller = osim.PrescribedController()

controller.addActuator(act_1)
controller.prescribeControlForActuator('baseangle', osim.Constant(0.0))

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
act_2.setName('kneeangle')
bbaan.addForce(act_2)

controller.addActuator(act_2)
controller.prescribeControlForActuator('kneeangle', osim.Constant(0.0))

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
act_3.setName('bladeangle')
bbaan.addForce(act_3)

controller.addActuator(act_3)
controller.prescribeControlForActuator('bladeangle', osim.Constant(0.0))

bbaan.addController(controller)


""" contact geometry  """

baan = osim.ContactMesh('box40_7_0.1_weinig.stl',
                        osim.Vec3(0, 0, 0),
                        osim.Vec3(pi/2, 0, 0),
                        theCourse)
baan.setName("baantje")
bbaan.addContactGeometry(baan)


boot = osim.ContactMesh('box6_0.6_0.2.stl',
                        osim.Vec3(0, 0, 0),
                        osim.Vec3(pi/2, 0, 0),
                        theBoat)
boot.setName("bootje")
bbaan.addContactGeometry(boot)


blad = osim.ContactMesh('box0.05_0.45_0.45.stl',
                        osim.Vec3(0, 0, 0),
                        osim.Vec3(0, -pi/2, 0),
                        blade)
blad.setName("bladje")
bbaan.addContactGeometry(blad)

# Define Contact Force Parameters
"""
stiffness           = 1000000;
dissipation         = 2.0;
staticFriction      = 0.8;
dynamicFriction     = 0.4;
viscousFriction     = 0.4;
transitionVelocity  = 0.2;
"""
stiffness           = 1000000;
dissipation         = 1.0;
staticFriction      = 0.001;
dynamicFriction     = 0.001;
viscousFriction     = 0.001;
transitionVelocity  = 0.02;

e_1 = osim.ElasticFoundationForce()
e_1.setName('Course')
e_1.addGeometry('bootje')
e_1.addGeometry('baantje')
e_1.setStiffness(stiffness)
e_1.setDissipation(dissipation)
e_1.setStaticFriction(staticFriction)
e_1.setDynamicFriction(dynamicFriction)
e_1.setViscousFriction(viscousFriction)
e_1.setTransitionVelocity(transitionVelocity)
bbaan.addForce(e_1)

stiffness           = 50000;
dissipation         = 3.0;
staticFriction      = 1.0;
dynamicFriction     = 1.0;
viscousFriction     = 0.84;
transitionVelocity  = 0.1;

e_2 = osim.ElasticFoundationForce()
e_2.setName('Blad')
e_2.addGeometry('bladje')
e_2.addGeometry('baantje')
e_2.setStiffness(stiffness)
e_2.setDissipation(dissipation)
e_2.setStaticFriction(staticFriction)
e_2.setDynamicFriction(dynamicFriction)
e_2.setViscousFriction(viscousFriction)
e_2.setTransitionVelocity(transitionVelocity)
bbaan.addForce(e_2)


bbaan.addBody(theCourse)
bbaan.addBody(theBoat)
bbaan.addBody(upper)
bbaan.addBody(lower)
bbaan.addBody(blade)
bbaan.addJoint(courseJoint)
bbaan.addJoint(boatJoint)
bbaan.addJoint(baseJoint)
bbaan.addJoint(lowerJoint)
bbaan.addJoint(bladeJoint)

# markers
marker = osim.Marker('mbasej', theBoat, osim.Vec3(5, 0.3, 0))
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
