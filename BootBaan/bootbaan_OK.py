import opensim as osim
import math

from bb_common import *

"""   Auxiliary functions    """
"""
Inertia of solid cylinder of radiusr, height h and mass m
I_z = 1/2 mr^2
I_x = I_y = 1/12 m(3r_2 + h^2)


"""

bbaan = osim.Model()
bbaan.setName("BootBaan")
bbaan.setGravity(osim.Vec3(0, -9.90665, 0))


theCourse = osim.Body("Rowing_course",
                      1.0,
                      osim.Vec3(0, 0, 0),
                      osim.Inertia(1, 1, 1))

theCourseGeometry = osim.Brick(osim.Vec3(20, 0.05, 3.5))
theCourse.attachGeometry(theCourseGeometry)
theCourseGeometry.setColor(osim.Vec3(0, 0, 1))
bbaan.addBody(theCourse)

"""   The boat   """
####################################################
theBoat = osim.Body()
theBoat.setName('TheBoat')
theBoat.setMass(boatw)
I_xx = 1/12*boatw*(math.pow(boatHeight/2, 2)+math.pow(boatLength/2, 2))
I_yy = I_xx
I_zz = 1/2*boatw*math.pow(boatLength/2, 2)
theBoat.setInertia(osim.Inertia(I_xx, I_yy, I_zz))
theBoatGeometry = osim.Brick(osim.Vec3(boatLength/2, boatHeight/2, boatWidth/2))
theBoat.attachGeometry(theBoatGeometry)
theBoatGeometry.setColor(osim.Vec3(0, 1, 1))
bbaan.addBody(theBoat)


"""   Foot stretcher           """
####################################################
I_xx = 1/12*1*(math.pow(0.2, 2)+math.pow(boatWidth/2, 2))
I_yy = I_xx
I_zz = 1/2*1*math.pow(boatWidth/2, 2)

stretcher = osim.Body("Stretcher",
                      1.0,
                      osim.Vec3(0, 0, 0),
                      osim.Inertia(I_xx, I_yy, I_zz))

stretcherGeometry = osim.Brick(osim.Vec3(seatHeight/2, seatHeight/2, boatWidth/2))
stretcher.attachGeometry(stretcherGeometry)
stretcherGeometry.setColor(osim.Vec3(0.5,0.5,1))
bbaan.addBody(stretcher)

"""   Seat                     """
####################################################
seat = osim.Body("Seat",
                 1.0,
                 osim.Vec3(0, 0, 0),
                 osim.Inertia(1, 1, 1))

seatGeometry = osim.Brick(osim.Vec3(seatHeight/2, seatHeight/2, boatWidth/2))
seat.attachGeometry(seatGeometry)
seatGeometry.setColor(osim.Vec3(0.5,0.5,1))
bbaan.addBody(seat)

"""   Bow                     """
####################################################
bow = osim.Body("Bow",
                 0.1,
                 osim.Vec3(0, 0, 0),
                 osim.Inertia(1, 1, 1))

bowGeometry = osim.Sphere(0.05)
bow.attachGeometry(bowGeometry)
bowGeometry.setColor(osim.Vec3(0.5,0.5,1))
bbaan.addBody(bow)

"""   Starboard rigger              """
####################################################
"""
 - Zero for height is center of the boat.
 - Connection rigger to the boat (in boat coordinates)
        x=place, y =0, z= +boatWidth/2
 - Place of lock (lockHeight above seat)
        x=place, boatHeight/2+seatHeight+lockHeight, span/2
 - Length of rigger body
     sqrt(a^2 + b^2)
 - heigth difference port and starboard 2 cm
"""
a = span/2 - boatWidth/2
b = boatHeight/2+seatHeight+lockHeight+0.02
rigslength = math.sqrt(math.pow(a, 2)+math.pow(b, 2))
rigsangle = math.atan(b/a)
srigger = osim.Body("StarboardRigger",
                    1.0,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(1, 1, 1))

srigGeometry = osim.Cylinder(0.02, rigslength/2)
srigger.attachGeometry(srigGeometry)
srigGeometry.setColor(osim.Vec3(0.5, 0.5, 1))
bbaan.addBody(srigger)


"""   Starboard lock                """
####################################################
slock = osim.Body("StarboardLock",
                  1.0,
                  osim.Vec3(0, 0, 0),
                  osim.Inertia(1, 1, 1))

slockGeometry = osim.Cylinder(0.01, 0.025)
slock.attachGeometry(slockGeometry)
slockGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(slock)





"""   Port rigger         """
####################################################
a = span/2 - boatWidth/2
b = boatHeight/2+seatHeight+lockHeight
rigplength = math.sqrt(math.pow(a, 2)+math.pow(b, 2))
rigpangle = math.atan(b/a)
prigger = osim.Body("PortRigger",
                    1.0,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(1, 1, 1))

prigGeometry = osim.Cylinder(0.02, rigplength/2)
prigger.attachGeometry(prigGeometry)
prigGeometry.setColor(osim.Vec3(0.5, 0.5, 1))
bbaan.addBody(prigger)


"""   Port lock          """
####################################################
plock = osim.Body("PortLock",
                  1.0,
                  osim.Vec3(0, 0, 0),
                  osim.Inertia(1, 1, 1))

plockGeometry = osim.Cylinder(0.01, 0.025)
plock.attachGeometry(plockGeometry)
plockGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(plock)



"""   The joints               """
####################################################
courseJoint = osim.WeldJoint("courseJoint",
                             bbaan.getGround(),
                             osim.Vec3(0, -0.05, 0),
                             osim.Vec3(0, 0, 0),
                             theCourse,
                             osim.Vec3(0, 0, 0),
                             osim.Vec3(0, 0, 0))
bbaan.addJoint(courseJoint)

boattf = osim.SpatialTransform()
bj_3 = osim.ArrayStr()
bj_3.append("bJoint_3")
boattf.updTransformAxis(3).setCoordinateNames(bj_3)
boattf.updTransformAxis(3).set_function(osim.LinearFunction())
bj_4 = osim.ArrayStr()
bj_4.append("bJoint_4")
boattf.updTransformAxis(4).setCoordinateNames(bj_4)
boattf.updTransformAxis(4).set_function(osim.LinearFunction())

boatJoint = osim.CustomJoint("boatJoint",
                           bbaan.getGround(),
                           osim.Vec3(0, boatHeight/2, 0),
                           osim.Vec3(0, 0, 0),
                           theBoat,
                           osim.Vec3(0, 0, 0),
                           osim.Vec3(0, 0, 0),
                           boattf)
bbaan.addJoint(boatJoint)

act = osim.CoordinateActuator('bJoint_3')
act.setName('bJ_act_3')
bbaan.addForce(act)
act = osim.CoordinateActuator('bJoint_4')
act.setName('bJ_act_4')
bbaan.addForce(act)

stretcherJoint = osim.WeldJoint("stretcherJoint",
                                theBoat,
                                osim.Vec3(0, boatHeight/2, 0),
                                osim.Vec3(0, 0, 0),
                                stretcher,
                                osim.Vec3(0, 0, 0),
                                osim.Vec3(0, 0, 0))
bbaan.addJoint(stretcherJoint)

# seat with stretched leg
legangle = math.asin(seatHeight/(ulegl+llegl))
seatJoint = osim.SliderJoint("seatJoint",
                             theBoat,
                             osim.Vec3(ulegl+llegl*math.cos(legangle), boatHeight/2+seatHeight, 0),
                             osim.Vec3(0, 0, 0),
                             seat,
                             osim.Vec3(0, 0, 0),
                             osim.Vec3(0, 0, 0))
bbaan.addJoint(seatJoint)

coord = seatJoint.updCoordinate()
coord.setName('seatpos')
coord.setRangeMin(-(ulegl+llegl))
coord.setRangeMax(0.0)
coord.setDefaultValue(math.radians(-0.4226))

act = osim.CoordinateActuator('seatpos')
act.setName('seatact')
bbaan.addForce(act)

bowJoint = osim.WeldJoint("bowJoint",
                          theBoat,
                          osim.Vec3(boatLength/2, boatHeight/2, 0),
                          osim.Vec3(0, 0, 0),
                          bow,
                          osim.Vec3(0, 0, 0),
                          osim.Vec3(0, 0, 0))
bbaan.addJoint(bowJoint)

"""  Starboard joints     """
####################################################
srigJoint = osim.WeldJoint("starboardrigJoint",
                           theBoat,
                           osim.Vec3(place, 0, boatWidth/2),
                           osim.Vec3(-pi/2-rigsangle, 0, 0),
                           srigger,
                           osim.Vec3(0, rigslength/2, 0),
                           osim.Vec3(0, 0, 0))
bbaan.addJoint(srigJoint)


slocJoint = osim.PinJoint("starboardlockJoint",
                          srigger,
                          osim.Vec3(0, -rigslength/2, 0),
                          osim.Vec3(rigsangle, 0, 0),
                          slock,
                          osim.Vec3(0, 0.025, 0),
                          osim.Vec3(pi/2, 0, 0))
bbaan.addJoint(slocJoint)
coord = slocJoint.updCoordinate()
coord.setName('slocangle')
coord.setRangeMin(-pi/2)
coord.setRangeMax(pi/2)
coord.setDefaultValue(math.radians(-20.657))
coord.set_clamped(True)

act = osim.CoordinateActuator('slocangle')
act.setName('slocact')
bbaan.addForce(act)

""" Port joints    """
####################################################
prigJoint = osim.WeldJoint("portrigJoint",
                           theBoat,
                           osim.Vec3(place, 0, -boatWidth/2),
                           osim.Vec3(pi/2+rigsangle, 0, 0),
                           prigger,
                           osim.Vec3(0, rigslength/2, 0),
                           osim.Vec3(0, 0, 0))
bbaan.addJoint(prigJoint)

plocJoint = osim.PinJoint("portlockJoint",
                          prigger,
                          osim.Vec3(0, -rigslength/2, 0),
                          osim.Vec3(-rigsangle, 0, 0),
                          plock,
                          osim.Vec3(0, 0.025, 0),
                          osim.Vec3(-pi/2, 0, 0))
bbaan.addJoint(plocJoint)
coord = plocJoint.updCoordinate()
coord.setName('plocangle')
coord.setRangeMin(-pi/2)
coord.setRangeMax(pi/2)
coord.setDefaultValue(math.radians(-20.657))
coord.set_clamped(True)

act = osim.CoordinateActuator('plocangle')
act.setName('plocact')
bbaan.addForce(act)



"""       The Rower                              """
####################################################

# Lower leg (split in 2 to create a WeldContraint)
lower_l = osim.Body("Lower_leg_lower",
                  llegw/2,
                  osim.Vec3(0, 0, 0),
                  osim.Inertia(1, 1, 1))

lowerGeometry_l = osim.Cylinder(0.05, llegl/4)
lower_l.attachGeometry(lowerGeometry_l)
lowerGeometry_l.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(lower_l)

lower_u = osim.Body("Lower_leg_upper",
                  llegw/2,
                  osim.Vec3(0, 0, 0),
                  osim.Inertia(1, 1, 1))

lowerGeometry_u = osim.Cylinder(0.05, llegl/4)
lower_u.attachGeometry(lowerGeometry_u)
lowerGeometry_u.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(lower_u)

# Upper leg
upper = osim.Body("Upper_leg",
                  ulegw,
                  osim.Vec3(0, 0, 0),
                  osim.Inertia(1, 1, 1))

upperGeometry = osim.Cylinder(0.05, ulegl/2)
upper.attachGeometry(upperGeometry)
upperGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(upper)

# Lower back
lowerb = osim.Body("Lower_back",
                   lbackw,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(1, 1, 1))

lowerbGeometry = osim.Brick(osim.Vec3(0.04, lbackl/2, 0.08))
lowerb.attachGeometry(lowerbGeometry)
lowerbGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(lowerb)

# Upper back
upperb = osim.Body("Upper_back",
                   ubackw,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(1, 1, 1))

upperbGeometry = osim.Brick(osim.Vec3(0.05, ubackl/2, 0.10))
upperb.attachGeometry(upperbGeometry)
upperbGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(upperb)


# Shoulder
shoulder = osim.Body("Shoulder",
                     shoulderw,
                     osim.Vec3(0, 0, 0),
                     osim.Inertia(1, 1, 1))

shoulderGeometry = osim.Brick(osim.Vec3(shoulderth/2, shoulderth/2, shoulderl/2))
shoulder.attachGeometry(shoulderGeometry)
shoulderGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(shoulder)

# Head
head = osim.Body("Head",
                 headw,
                 osim.Vec3(0, 0, 0),
                 osim.Inertia(1, 1, 1))

headGeometry = osim.Brick(osim.Vec3(0.04, headl/2, headl/2))
head.attachGeometry(headGeometry)
headGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(head)



"""  Rower joints    """

hipJoint = osim.PinJoint("hipJoint",
                         seat,
                         osim.Vec3(0, seatHeight/2, 0),
                         osim.Vec3(0, 0, 0),
                         upper,
                         osim.Vec3(0, -ulegl/2, 0),
                         osim.Vec3(0, 0, 0))
bbaan.addJoint(hipJoint)
coord = hipJoint.updCoordinate()
coord.setName('hipangle')
coord.setRangeMin(-0.3)
coord.setRangeMax(pi/2+0.2)
coord.setDefaultValue(math.radians(78.9))
coord.set_clamped(True)

act = osim.CoordinateActuator('hipangle')
act.setName('hipact')
bbaan.addForce(act)

kneeJoint = osim.PinJoint("knee_Joint",
                          upper,
                          osim.Vec3(0, ulegl/2, 0),
                          osim.Vec3(0, 0, 0),
                          lower_u,
                          osim.Vec3(0, -llegl/4, 0),
                          osim.Vec3(0, 0, 0))
bbaan.addJoint(kneeJoint)

coord = kneeJoint.updCoordinate()
coord.setName('kneeangle')
coord.setRangeMin(0.0)
coord.setRangeMax(pi)
coord.setDefaultValue(math.radians(10))
coord.set_clamped(True)
""" nodig?
# clamped works in IK, for use in Forward Dynamics use CoordinateLimitForce
osim.CoordinateLimitForce('kneeangle',
                           pi,
                           10,
                           0,
                           10,
                           0.01,
                           2.0)
"""

act = osim.CoordinateActuator('kneeangle')
act.setName('kneeact')
bbaan.addForce(act)

footJoint = osim.PinJoint("foot_Joint",
                          stretcher,
                          osim.Vec3(0, seatHeight/2, 0),
                          osim.Vec3(0, 0, pi/2),
                          lower_l,
                          osim.Vec3(0, llegl/4, 0),
                          osim.Vec3(0, 0, 0))
bbaan.addJoint(footJoint)

coord = footJoint.updCoordinate()
coord.setName('footangle')
coord.setRangeMin(0.0)
coord.setRangeMax(pi/2)
coord.setDefaultValue(math.radians(5))
coord.set_clamped(True)

act = osim.CoordinateActuator('footangle')
act.setName('footact')
bbaan.addForce(act)

lbackJoint = osim.PinJoint("lbackJoint",
                           seat,
                           osim.Vec3(0, seatHeight/2, 0),
                           osim.Vec3(0, 0, 0),
                           lowerb,
                           osim.Vec3(0, -lbackl/2, 0),
                           osim.Vec3(0, 0, 0))
bbaan.addJoint(lbackJoint)
coord = lbackJoint.updCoordinate()
coord.setName('lbackangle')
coord.setRangeMin(-pi/2)
coord.setRangeMax(0.3)
coord.setDefaultValue(math.radians(-44.01))
coord.set_clamped(True)

act = osim.CoordinateActuator('lbackangle')
act.setName('lbackact')
bbaan.addForce(act)

# Eventually will become a PinJoint
ubackJoint = osim.WeldJoint("ubackJoint",
                            lowerb,
                            osim.Vec3(0, lbackl/2, 0),
                            osim.Vec3(0, 0, angleinb),
                            upperb,
                            osim.Vec3(0, -ubackl/2, 0),
                            osim.Vec3(0, 0, 0))
bbaan.addJoint(ubackJoint)

shoulderJoint = osim.WeldJoint("shoulderJoint",
                               upperb,
                               osim.Vec3(0, ubackl/2, 0),
                               osim.Vec3(0, 0, 0),
                               shoulder,
                               osim.Vec3(0, -shoulderth/2, 0),
                               osim.Vec3(0, 0, 0))
bbaan.addJoint(shoulderJoint)

headJoint = osim.WeldJoint("headJoint",
                           shoulder,
                           osim.Vec3(0, shoulderth/2, 0),
                           osim.Vec3(0, 0, 0),
                           head,
                           osim.Vec3(0, -headl/2, 0),
                           osim.Vec3(0, 0, 0))
bbaan.addJoint(headJoint)




# close the loop in the lower leg
legconstraint = osim.WeldConstraint("legconstraint", lower_l, osim.Transform(osim.Vec3(0, -llegl/4, 0)), lower_u, osim.Transform(osim.Vec3(0, llegl/4, 0)))
bbaan.addConstraint(legconstraint)

# close the loop in the left lower arm
#larmconstraint = osim.WeldConstraint("larmconstraint", loweral_l, osim.Transform(osim.Vec3(0, -larml/4, 0)), loweral_u, osim.Transform(osim.Vec3(0, larml/4, 0)))
#bbaan.addConstraint(larmconstraint)

# close the loop in the right lower arm
#rarmconstraint = osim.WeldConstraint("rarmconstraint", lowerar_u, osim.Transform(osim.Vec3(0, -larml/4, 0)), lowerar_l, osim.Transform(osim.Vec3(0, larml/4, 0)))
#bbaan.addConstraint(rarmconstraint)


"""  Contact Geometry    """
baan = osim.ContactHalfSpace(osim.Vec3(0, 0, 0),
                             osim.Vec3(0, 0, -pi/2),
                             bbaan.getGround())

baan.setName("baan")
bbaan.addContactGeometry(baan)


boot = osim.ContactMesh('box6_0.6_0.2.stl',
                        osim.Vec3(0, 0, 0),
                        osim.Vec3(-pi/2, 0, 0),
                        theBoat)
boot.setName("boot")
bbaan.addContactGeometry(boot)




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
e_1.setName('Boot')
e_1.addGeometry('boot')
e_1.addGeometry('baan')
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





# ook voor de handles om handles van elkaar te houden?
#   en de knie, back?

"""      Markers                                 """
####################################################

mboat = osim.Marker(markers[0], theBoat, osim.Vec3(0, 0, 0))
bbaan.addMarker(mboat)

mseat = osim.Marker(markers[1], seat, osim.Vec3(0))
bbaan.addMarker(mseat)

mshoulder = osim.Marker(markers[2], shoulder, osim.Vec3(0, 0, 0))
bbaan.addMarker(mshoulder)



"""      The rest                                """
####################################################

reporter = osim.ConsoleReporter()
reporter.set_report_time_interval(1.0)
bbaan.addComponent(reporter)

bbaan.finalizeConnections()
bbaan.printToXML("BootBaan.osim")

# uncomment the rest to directly get the simulation
"""
bbaan.setUseVisualizer(True)
state = bbaan.initSystem()

print(state.getY())
finalState = osim.simulate(bbaan, state, 5)


# Analyze a simulation.
print(coord.getValue(finalState))
bbaan.realizePosition(finalState)
print(bbaan.calcMassCenterPosition(finalState))
bbaan.realizeAcceleration(finalState)
print(uarmlJoint.calcReactionOnParentExpressedInGround(finalState))
"""

