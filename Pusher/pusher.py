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
                     osim.Inertia(0, 0, 0))

theCourseGeometry = osim.Brick(osim.Vec3(20, 0.05, 2))
theCourse.attachGeometry(theCourseGeometry)
theCourseGeometry.setColor(osim.Vec3(0,0,1))

theBoat = osim.Body()
theBoat.setName('TheBoat')
theBoat.setMass(1)
theBoat.setInertia(osim.Inertia(1,1,1,0,0,0))

# boot voorlopig een brick, later een meshfile van een boot
theBoatGeometry = osim.Brick(osim.Vec3(6, 0.3, 0.8))
theBoat.attachGeometry(theBoatGeometry)
theBoatGeometry.setColor(osim.Vec3(0,1,1))

# Lower leg
upper = osim.Body("The_upper_leg",
                     1.0,
                     osim.Vec3(0, 0, 0),
                     osim.Inertia(0, 0, 0))

upperGeometry = osim.Cylinder(0.1, 1)
upper.attachGeometry(upperGeometry)
upperGeometry.setColor(osim.Vec3(0.5,0.5,1))

# Lower leg
lower = osim.Body("The_lower_leg",
                     1.0,
                     osim.Vec3(0, 0, 0),
                     osim.Inertia(0, 0, 0))

lowerGeometry = osim.Cylinder(0.1, 1.5)
lower.attachGeometry(lowerGeometry)
lowerGeometry.setColor(osim.Vec3(1,0,1))

# the foot
blade = osim.Body()
blade.setName('TheBlade')
blade.setMass(1)
blade.setInertia(osim.Inertia(1,1,1,0,0,0))

# boot voorlopig een brick, later een meshfile van een boot
bladeGeometry = osim.Brick(osim.Vec3(0.03, 0.6, 0.6))
blade.attachGeometry(bladeGeometry)
bladeGeometry.setColor(osim.Vec3(1,1,1))


"""  The joints   """
courseJoint = osim.WeldJoint("courseJoint",
                        bbaan.getGround(),
                        osim.Vec3(0, 0.025, 0),
                        osim.Vec3(0, 0, 0),
                        theCourse,
                        osim.Vec3(0, 0, 0),
                        osim.Vec3(0, 0, 0))
                        
boatJoint = osim.FreeJoint("boatJoint",
                        theCourse,
                        osim.Vec3(0, 0.5, 0),
                        osim.Vec3(0, 0, 0),
                        theBoat,
                        osim.Vec3(0, 0, 0),
                        osim.Vec3(0, 0, 0))
                        
baseJoint = osim.PinJoint("baseJoint",
                        theBoat,
                        osim.Vec3(5, 0.3, 0),
                        osim.Vec3(0, 0, 0),
                        upper,
                        osim.Vec3(0, -1, 0),
                        osim.Vec3(0, 0, 0))
coord_1 = baseJoint.updCoordinate()
coord_1.setName('baseangle')
coord_1.set_range(0, -1.4)
coord_1.set_range(1,  1.0)

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


coord_2 = lowerJoint.updCoordinate()

coord_2.setName('lowerangle')
coord_2.set_range(0, -1.2)
coord_2.set_range(1,  pi/2)


act_2 = osim.CoordinateActuator('lowerangle')
act_2.setName('act_2')
bbaan.addForce(act_2)

controller.addActuator(act_2)
controller.prescribeControlForActuator('act_2', osim.Constant(0.0))

bladeJoint = osim.PinJoint("bladeJoint",
                        lower,
                        osim.Vec3(0, 1.6, 0),
                        osim.Vec3(0, 0, 0),
                        blade,
                        osim.Vec3(0, 0, 0),
                        osim.Vec3(0, 0, 0))

coord_3 = bladeJoint.updCoordinate()
coord_3.setName('bladeangle')
coord_3.set_range(0,  0)
coord_3.set_range(1,  pi/2)

act_3 = osim.CoordinateActuator('bladeangle')
act_3.setName('act_3')
bbaan.addForce(act_3)

controller.addActuator(act_3)
controller.prescribeControlForActuator('act_3', osim.Constant(0.0))

bbaan.addController(controller)


""" contact geometry  """
theCourseContactSpace = osim.ContactHalfSpace(osim.Vec3(0, 0.1, 0),
                                              osim.Vec3(0, 0, -pi/2),    #-pi/2),
                                              theCourse)
theCourseContactSpace.setName("baantje")
bbaan.addContactGeometry(theCourseContactSpace)

# voorlopig 4 spheres op de onderste hoekpunten
#   een vlak beter? maar hoe?
corner_1 = osim.ContactSphere()
corner_1.setRadius(0.1)
corner_1.setLocation(osim.Vec3(6, -0.3, -0.8))
corner_1.setFrame(theBoat)
corner_1.setName('corner_1')
bbaan.addContactGeometry(corner_1)

corner_2 = osim.ContactSphere()
corner_2.setRadius(0.1)
corner_2.setLocation(osim.Vec3(-6, -0.3, -0.8))
corner_2.setFrame(theBoat)
corner_2.setName('corner_2')
bbaan.addContactGeometry(corner_2)

corner_3 = osim.ContactSphere()
corner_3.setRadius(0.1)
corner_3.setLocation(osim.Vec3(6, -0.3, 0.8))
corner_3.setFrame(theBoat)
corner_3.setName('corner_3')
bbaan.addContactGeometry(corner_3)

corner_4 = osim.ContactSphere()
corner_4.setRadius(0.1)
corner_4.setLocation(osim.Vec3(-6, -0.3, 0.8))
corner_4.setFrame(theBoat)
corner_4.setName('corner_4')
bbaan.addContactGeometry(corner_4)

blade_1 = osim.ContactSphere()
blade_1.setRadius(0.1)
blade_1.setLocation(osim.Vec3(0.05, 0.6, 0.6))
blade_1.setFrame(blade)
blade_1.setName('blade_1')
bbaan.addContactGeometry(blade_1)

blade_2 = osim.ContactSphere()
blade_2.setRadius(0.1)
blade_2.setLocation(osim.Vec3(0.05, 0.6, -0.6))
blade_2.setFrame(blade)
blade_2.setName('blade_2')
bbaan.addContactGeometry(blade_2)

blade_3 = osim.ContactSphere()
blade_3.setRadius(0.1)
blade_3.setLocation(osim.Vec3(0.05, -0.6, 0.6))
blade_3.setFrame(blade)
blade_3.setName('blade_3')
bbaan.addContactGeometry(blade_3)

blade_4 = osim.ContactSphere()
blade_4.setRadius(0.1)
blade_4.setLocation(osim.Vec3(0.05, -0.6, -0.6))
blade_4.setFrame(blade)
blade_4.setName('blade_4')
bbaan.addContactGeometry(blade_4)

elbow_0 = osim.ContactSphere()
elbow_0.setRadius(0.1)
elbow_0.setLocation(osim.Vec3(0.0, 1.0, 0.0))
elbow_0.setFrame(upper)
elbow_0.setName('elbow_0')
bbaan.addContactGeometry(elbow_0)


"""  forces, actuators  """



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
dissipation         = 0.8;
staticFriction      = 0.1;
dynamicFriction     = 0.4;
viscousFriction     = 0.4;
transitionVelocity  = 0.1;

# Make a Hunt Crossley Force
cornerforce_1 = osim.HuntCrossleyForce()
cornerforce_1.setName('BoatForce_1')
cornerforce_1.addGeometry('corner_1')
cornerforce_1.addGeometry('baantje')
cornerforce_1.setStiffness(stiffness)
cornerforce_1.setDissipation(dissipation)
cornerforce_1.setStaticFriction(staticFriction)
cornerforce_1.setDynamicFriction(dynamicFriction)
cornerforce_1.setViscousFriction(viscousFriction)
cornerforce_1.setTransitionVelocity(transitionVelocity)
bbaan.addForce(cornerforce_1)

# Make a Hunt Crossley Force
cornerforce_2 = osim.HuntCrossleyForce()
cornerforce_2.setName('BoatForce_2')
cornerforce_2.addGeometry('corner_2')
cornerforce_2.addGeometry('baantje')
cornerforce_2.setStiffness(stiffness)
cornerforce_2.setDissipation(dissipation)
cornerforce_2.setStaticFriction(staticFriction)
cornerforce_2.setDynamicFriction(dynamicFriction)
cornerforce_2.setViscousFriction(viscousFriction)
cornerforce_2.setTransitionVelocity(transitionVelocity)
bbaan.addForce(cornerforce_2)

# Make a Hunt Crossley Force
cornerforce_3 = osim.HuntCrossleyForce()
cornerforce_3.setName('BoatForce_3')
cornerforce_3.addGeometry('corner_3')
cornerforce_3.addGeometry('baantje')
cornerforce_3.setStiffness(stiffness)
cornerforce_3.setDissipation(dissipation)
cornerforce_3.setStaticFriction(staticFriction)
cornerforce_3.setDynamicFriction(dynamicFriction)
cornerforce_3.setViscousFriction(viscousFriction)
cornerforce_3.setTransitionVelocity(transitionVelocity)
bbaan.addForce(cornerforce_3)

# Make a Hunt Crossley Force
cornerforce_4 = osim.HuntCrossleyForce()
cornerforce_4.setName('BoatForce_4')
cornerforce_4.addGeometry('corner_4')
cornerforce_4.addGeometry('baantje')
cornerforce_4.setStiffness(stiffness)
cornerforce_4.setDissipation(dissipation)
cornerforce_4.setStaticFriction(staticFriction)
cornerforce_4.setDynamicFriction(dynamicFriction)
cornerforce_4.setViscousFriction(viscousFriction)
cornerforce_4.setTransitionVelocity(transitionVelocity)
bbaan.addForce(cornerforce_4)

# Make a Hunt Crossley Force
bladeforce_1 = osim.HuntCrossleyForce()
bladeforce_1.setName('BladeForce_1')
bladeforce_1.addGeometry('blade_1')
bladeforce_1.addGeometry('baantje')
bladeforce_1.setStiffness(stiffness)
bladeforce_1.setDissipation(dissipation)
bladeforce_1.setStaticFriction(staticFriction)
bladeforce_1.setDynamicFriction(dynamicFriction)
bladeforce_1.setViscousFriction(viscousFriction)
bladeforce_1.setTransitionVelocity(transitionVelocity)
bbaan.addForce(bladeforce_1)

# Make a Hunt Crossley Force
bladeforce_2 = osim.HuntCrossleyForce()
bladeforce_2.setName('BladeForce_2')
bladeforce_2.addGeometry('blade_2')
bladeforce_2.addGeometry('baantje')
bladeforce_2.setStiffness(stiffness)
bladeforce_2.setDissipation(dissipation)
bladeforce_2.setStaticFriction(staticFriction)
bladeforce_2.setDynamicFriction(dynamicFriction)
bladeforce_2.setViscousFriction(viscousFriction)
bladeforce_2.setTransitionVelocity(transitionVelocity)
bbaan.addForce(bladeforce_2)

# Make a Hunt Crossley Force
bladeforce_3 = osim.HuntCrossleyForce()
bladeforce_3.setName('BladeForce_3')
bladeforce_3.addGeometry('blade_3')
bladeforce_3.addGeometry('baantje')
bladeforce_3.setStiffness(stiffness)
bladeforce_3.setDissipation(dissipation)
bladeforce_3.setStaticFriction(staticFriction)
bladeforce_3.setDynamicFriction(dynamicFriction)
bladeforce_3.setViscousFriction(viscousFriction)
bladeforce_3.setTransitionVelocity(transitionVelocity)
bbaan.addForce(bladeforce_3)

# Make a Hunt Crossley Force
bladeforce_4 = osim.HuntCrossleyForce()
bladeforce_4.setName('BladeForce_4')
bladeforce_4.addGeometry('blade_4')
bladeforce_4.addGeometry('baantje')
bladeforce_4.setStiffness(stiffness)
bladeforce_4.setDissipation(dissipation)
bladeforce_4.setStaticFriction(staticFriction)
bladeforce_4.setDynamicFriction(dynamicFriction)
bladeforce_4.setViscousFriction(viscousFriction)
bladeforce_4.setTransitionVelocity(transitionVelocity)
bbaan.addForce(bladeforce_4)

# Make a Hunt Crossley Force
elbow_force_0 = osim.HuntCrossleyForce()
elbow_force_0.setName('elbow_force_0')
elbow_force_0.addGeometry('elbow_0')
elbow_force_0.addGeometry('baantje')
elbow_force_0.setStiffness(stiffness)
elbow_force_0.setDissipation(dissipation)
elbow_force_0.setStaticFriction(staticFriction)
elbow_force_0.setDynamicFriction(dynamicFriction)
elbow_force_0.setViscousFriction(viscousFriction)
elbow_force_0.setTransitionVelocity(transitionVelocity)
bbaan.addForce(elbow_force_0)


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
