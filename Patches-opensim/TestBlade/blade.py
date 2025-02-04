import opensim as osim
import sys
import math
pi = math.pi


bbaan = osim.Model()

bbaan.setName("Blade")
bbaan.setGravity(osim.Vec3(0, -9.90665, 0))

"""   
 Test voor nieuwe force voor blad
   wrijving moet afhankelijk zijn van de richting "in het water"
   voorwaarts: veel wrijving: blad beweegt loodrecht op het water
   opzij: heel weinig  wrijving, blad beweegt in richting van de steel

"""
bottom = osim.Body("Bottom",
                   1.0,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(1, 1, 1, 0, 0, 0))
bbaan.addBody(bottom)

bottomGeometry = osim.Brick(osim.Vec3(2.5, 2.5, 0.005))
bottom.attachGeometry(bottomGeometry)
bottomGeometry.setColor(osim.Vec3(0, 0, 1))

upperleg = osim.Body("The__upper_leg",
                50.0,
                osim.Vec3(0, 0, 0),
                osim.Inertia(1, 1, 1, 0, 0, 0))
bbaan.addBody(upperleg)

upperlegGeometry = osim.Cylinder(0.025, 0.5)
upperleg.attachGeometry(upperlegGeometry)
upperlegGeometry.setColor(osim.Vec3(1, 0, 1))

lowerleg = osim.Body("The__lower_leg",
                50.0,
                osim.Vec3(0, 0, 0),
                osim.Inertia(1, 1, 1, 0, 0, 0))
bbaan.addBody(lowerleg)

lowerlegGeometry = osim.Cylinder(0.025, 0.5)
lowerleg.attachGeometry(lowerlegGeometry)
lowerlegGeometry.setColor(osim.Vec3(1, 0, 1))

# the foot
turnft = osim.Body()
turnft.setName('turnft')
turnft.setMass(1)
turnft.setInertia(osim.Inertia(1, 1, 1, 0, 0, 0))
bbaan.addBody(turnft)

theBlade = osim.Body()
theBlade.setName('TheBlade')
theBlade.setMass(100)
theBlade.setInertia(osim.Inertia(7, 7, 45, 0, 0, 0))
bbaan.addBody(theBlade)

theBladeGeometry = osim.Brick(osim.Vec3(0.3, 0.3, 0.025))
theBlade.attachGeometry(theBladeGeometry)
theBladeGeometry.setColor(osim.Vec3(0, 1, 1))


theBlade2 = osim.Body()
theBlade2.setName('TheBlade2')
theBlade2.setMass(100)
theBlade2.setInertia(osim.Inertia(7, 7, 45, 0, 0, 0))
bbaan.addBody(theBlade2)

theBladeGeometry = osim.Brick(osim.Vec3(0.3, 0.01, 0.3))
theBlade2.attachGeometry(theBladeGeometry)
theBladeGeometry.setColor(osim.Vec3(0, 1, 0.5))


"""  The joints   """
bottomJoint = osim.WeldJoint("bottomJoint",
                             bbaan.getGround(),
                             osim.Vec3(0, 0, 0),
                             osim.Vec3(pi/2, 0, 0),
                             bottom,
                             osim.Vec3(0, 0, 0),
                             osim.Vec3(0, 0, 0))
bbaan.addJoint(bottomJoint)


pelvisJoint = osim.PinJoint("pelvis_Joint",
                            bbaan.getGround(),
                            osim.Vec3(0, 0.1, 0),
                            osim.Vec3(0, 0, 0),
                            upperleg,
                            osim.Vec3(0, -0.5, 0),
                            osim.Vec3(0, 0, 0))
bbaan.addJoint(pelvisJoint)

coord = pelvisJoint.updCoordinate()
coord.setName('pelvisangle')
coord.setRangeMin(0.0)
coord.setRangeMax(pi/2)
coord.setDefaultValue(math.radians(47))
coord.set_clamped(True)


kneeJoint = osim.PinJoint("knee_Joint",
                          upperleg,
                          osim.Vec3(0, 0.5, 0),
                          osim.Vec3(0, 0, 0),
                          lowerleg,
                          osim.Vec3(0, -0.5, 0),
                          osim.Vec3(0, 0, 0))
bbaan.addJoint(kneeJoint)

coord = kneeJoint.updCoordinate()
coord.setName('kneeangle')
coord.setRangeMin(0.0)
coord.setRangeMax(pi/2)
coord.setDefaultValue(math.radians(90))
coord.set_clamped(True)

# werkt niet in Forward Dynamics
# osim.CoordinateLimitForce("kneeangle", pi/2, 1, 0, 1, 1, 0.2)

bladeJoint = osim.PinJoint("blade_Joint",
                           lowerleg,
                           osim.Vec3(0, 0.5, 0),
                           osim.Vec3(0, 0, 0),
                           turnft,
                           osim.Vec3(0, 0, 0),
                           osim.Vec3(0, 0, 0))
bbaan.addJoint(bladeJoint)

coord = bladeJoint.updCoordinate()
coord.setName('footangle')
coord.setRangeMin(0.0)
coord.setRangeMax(pi/2)
coord.setDefaultValue(math.radians(44))
coord.set_clamped(True)

turnJoint = osim.PinJoint("turn_Joint",
                          turnft,
                          osim.Vec3(0, 0.0, 0),
                          osim.Vec3(pi/2, 0, 0),
                          theBlade,
                          osim.Vec3(0, 0, 0),
                          osim.Vec3(0, 0, 0))
bbaan.addJoint(turnJoint)

coord = turnJoint.updCoordinate()
coord.setName('turnangle')
coord.setRangeMin(-pi/2)
coord.setRangeMax(pi/2)
coord.setDefaultValue(math.radians(0))
coord.set_clamped(True)

vertblade = osim.WeldJoint("vert_Joint",
                          theBlade,
                          osim.Vec3(0, 0, 0),
                          osim.Vec3(0, 0, 0),
                          theBlade2,
                          osim.Vec3(0, 0, 0),
                          osim.Vec3(0, 0, 0))
bbaan.addJoint(vertblade)

""" contact geometry  """
"""
#baan = osim.ContactMesh('box0.01_5_5_weinig.stl',
baan = osim.ContactMesh('box0.01_5_5.stl',
                        osim.Vec3(0, 0, 0),
                        osim.Vec3(0, 0, 0),
                        bottom)
"""
baan = osim.ContactHalfSpace(osim.Vec3(0, 0.0, 0),
                             osim.Vec3(0, -pi/2, 0),
                             bottom)


baan.setName("baantje")
bbaan.addContactGeometry(baan)


blade = osim.ContactMesh('box0.05_0.6_0.6.stl',
                         osim.Vec3(0, 0, 0),
                         osim.Vec3(0, pi, 0),
                         theBlade)
"""
blade = osim.ContactSphere(0.2,
                           osim.Vec3(0, 0, 0),
                           theBlade)
"""
blade.setName("bootje")
bbaan.addContactGeometry(blade)


# Define Contact Force Parameters
"""
stiffness           = 1000000
dissipation         = 2.0
staticFriction      = 0.8
dynamicFriction     = 0.4
viscousFriction     = 0.4
transitionVelocity  = 0.2
"""
stiffness           = 1000000
dissipation         = 2.0
staticFriction      = 0.8
dynamicFriction     = 0.4
viscousFriction     = 0.4
transitionVelocity  = 0.2

e_1 = osim.BladeForce()
#e_1 = osim.ElasticFoundationForce()
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

reporter = osim.ConsoleReporter()
reporter.set_report_time_interval(1.0)
bbaan.addComponent(reporter)

bbaan.finalizeConnections()
bbaan.printToXML("Test.osim")

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
