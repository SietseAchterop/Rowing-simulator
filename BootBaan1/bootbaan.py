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

theCourseGeometry = osim.Brick(osim.Vec3(30, 0.00005, 3.5))
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



"""   Starboard oar                 """
####################################################
soar = osim.Body("StarboardOar",
                 1.0,
                 osim.Vec3(0, 0, 0),
                 osim.Inertia(1, 1, 1))

soarGeometry = osim.Cylinder(0.02, (inboard+outboard)/2)
soar.attachGeometry(soarGeometry)
soarGeometry.setColor(osim.Vec3(0.5, 0.5, 1))
bbaan.addBody(soar)


"""   Starboard blade               """
####################################################

I_xx = 1/12*1*(math.pow(bladepoint/2, 2)+math.pow(bladepoint/2, 2))

sblade = osim.Body("StarboardBlade",
                   1.0,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(0.1, 0.1, 0.1))

sbladeGeometry = osim.Brick(osim.Vec3(0.02, bladepoint/2, bladepoint/2))
sblade.attachGeometry(sbladeGeometry)
sbladeGeometry.setColor(osim.Vec3(0.5, 0.5, 1))
bbaan.addBody(sblade)


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


"""   Port oar            """
####################################################
poar = osim.Body("PortOar",
                 1.0,
                 osim.Vec3(0, 0, 0),
                 osim.Inertia(1, 1, 1))

poarGeometry = osim.Cylinder(0.02, (inboard+outboard)/2)
poar.attachGeometry(poarGeometry)
poarGeometry.setColor(osim.Vec3(0.5, 0.5, 1))
bbaan.addBody(poar)


"""   Port blade          """
####################################################
pblade = osim.Body("PortBlade",
                   1.0,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(0.1, 0.1, 0.1))

pbladeGeometry = osim.Brick(osim.Vec3(0.02, bladepoint/2, bladepoint/2))
pblade.attachGeometry(pbladeGeometry)
pbladeGeometry.setColor(osim.Vec3(0.5, 0.5, 1))
bbaan.addBody(pblade)


"""   The joints               """
####################################################

courseJoint = osim.WeldJoint("courseJoint",
                             bbaan.getGround(),
                             osim.Vec3(20, -0.0001, 0),
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

"""
act = osim.CoordinateActuator('bJoint_3')
act.setName('bJ_act_3')
act.set_optimal_force(1)
bbaan.addForce(act)

act = osim.CoordinateActuator('bJoint_4')
act.setName('bJ_act_4')
act.set_optimal_force(1)
bbaan.addForce(act)
"""

# close the loop between boat and stretcher
boatconstraint = osim.WeldConstraint("boatconstraint", theBoat, osim.Transform(osim.Vec3(0, boatHeight/2, 0)), stretcher, osim.Transform(osim.Vec3(0, 0, 0)))
bbaan.addConstraint(boatconstraint)

# seat
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
coord.setRangeMax(0.1)
coord.setDefaultValue(math.radians(-0.4226))

act = osim.CoordinateActuator('seatpos')
act.setName('seatact')
act.set_optimal_force(1)
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
"""
srigJoint = osim.WeldJoint("starboardrigJoint",
                           theBoat,
                           osim.Vec3(place, 0, boatWidth/2),
                           osim.Vec3(-pi/2-rigsangle, 0, 0),
                           srigger,
                           osim.Vec3(0, rigslength/2, 0),
                           osim.Vec3(0, 0, 0))
bbaan.addJoint(srigJoint)
"""

# close the loop in the starboard side
star_constraint = osim.WeldConstraint("star_constraint", theBoat, osim.Vec3(place, 0, boatWidth/2), osim.Vec3(-pi/2-rigsangle, 0, 0),
                                                         srigger, osim.Vec3(0, rigslength/2, 0),    osim.Vec3(0, 0, 0))
bbaan.addConstraint(star_constraint)

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
act.set_optimal_force(1)
bbaan.addForce(act)

soarJoint = osim.PinJoint("starboardoarJoint",
                          slock,
                          osim.Vec3(0, 0, 0),
                          osim.Vec3(pi/2, pi/2, 0),
                          soar,
                          osim.Vec3(0, (outboard-inboard)/2,  0),
                          osim.Vec3(0, 0, 0))
bbaan.addJoint(soarJoint)
coord = soarJoint.updCoordinate()
coord.setName('soarinout')
coord.setRangeMin(0.0)
coord.setRangeMax(1.0)
coord.setDefaultValue(math.radians(10.9))
coord.set_clamped(True)

act = osim.CoordinateActuator('soarinout')
act.setName('soarinoutact')
act.set_optimal_force(1)
bbaan.addForce(act)

sbladeJoint = osim.PinJoint("starboardbladeJoint",
                            soar,
                            osim.Vec3(0, -(inboard+outboard)/2+bladepoint,  0),
                            osim.Vec3(0, 0, 0),
                            sblade,
                            osim.Vec3(0, bladepoint/2,  0),
                            osim.Vec3(0, 0, 0))                            
bbaan.addJoint(sbladeJoint)
coord = sbladeJoint.updCoordinate()
coord.setName('sblpos')
coord.setRangeMin(-0.6)
coord.setRangeMax(0.1)
#coord.setDefaultValue(math.radians(2.345))
coord.set_clamped(True)

act = osim.CoordinateActuator('sblpos')
act.setName('sblact')
act.set_optimal_force(1)
bbaan.addForce(act)

# the actual starboard blade
stvertBlade = osim.Body("StVertBlade",
                   1.0,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(0.1, 0.1, 0.1))

theBladeGeometry = osim.Brick(osim.Vec3(0.1, bladepoint/2, 0.01))
stvertBlade.attachGeometry(theBladeGeometry)
theBladeGeometry.setColor(osim.Vec3(0, 1, 0.5))
bbaan.addBody(stvertBlade)

stvertbl = osim.WeldJoint("stvert_Joint",
                          sblade,
                          osim.Vec3(0, 0, 0),
                          osim.Vec3(0, 0, 0),
                          stvertBlade,
                          osim.Vec3(0, 0, 0),
                          osim.Vec3(0, 0, 0))
bbaan.addJoint(stvertbl)

""" Port joints """
####################################################
"""
prigJoint = osim.WeldJoint("portrigJoint",
                           theBoat,
                           osim.Vec3(place, 0, -boatWidth/2),
                           osim.Vec3(pi/2+rigpangle, 0, 0),
                           prigger,
                           osim.Vec3(0, rigplength/2, 0),
                           osim.Vec3(0, 0, 0))
bbaan.addJoint(prigJoint)
"""

# close the loop in the port side
port_constraint = osim.WeldConstraint("port_constraint", theBoat, osim.Vec3(place, 0, -boatWidth/2), osim.Vec3(pi/2+rigpangle, 0, 0),
                                                         prigger, osim.Vec3(0, rigplength/2, 0),    osim.Vec3(0, 0, 0))
bbaan.addConstraint(port_constraint)


plocJoint = osim.PinJoint("portlockJoint",
                          prigger,
                          osim.Vec3(0, -rigplength/2, 0),
                          osim.Vec3(-rigpangle, 0, 0),
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
act.set_optimal_force(1)
bbaan.addForce(act)

poarJoint = osim.PinJoint("portoarJoint",
                          plock,
                          osim.Vec3(0, 0, 0),
                          osim.Vec3(pi/2, -pi/2, pi),
                          poar,
                          osim.Vec3(0, (outboard-inboard)/2,  0),
                          osim.Vec3(0, 0, 0))

bbaan.addJoint(poarJoint)
coord = poarJoint.updCoordinate()
coord.setName('poarinout')
coord.setRangeMin(0.0)
coord.setRangeMax(1.0)
coord.setDefaultValue(math.radians(10.926))
coord.set_clamped(True)

act = osim.CoordinateActuator('poarinout')
act.setName('poarinoutact')
act.set_optimal_force(1)
bbaan.addForce(act)

pbladeJoint = osim.PinJoint("portbladeJoint",
                            poar,
                            osim.Vec3(0, -(inboard+outboard)/2+bladepoint,  0),
                            osim.Vec3(0, 0, pi),
                            pblade,
                            osim.Vec3(0, -bladepoint/2,  0),
                            osim.Vec3(0, 0, 0))                            
bbaan.addJoint(pbladeJoint)
coord = pbladeJoint.updCoordinate()
coord.setName('pblpos')
coord.setRangeMin(-0.6)
coord.setRangeMax(0.1)
#coord.setDefaultValue(math.radians(2.345))
coord.set_clamped(True)

act = osim.CoordinateActuator('pblpos')
act.setName('pblact')
act.set_optimal_force(1)
bbaan.addForce(act)

# the actual port blade
pvertBlade = osim.Body("PrVertBlade",
                   1.0,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(1, 1, 1))

theBladeGeometry = osim.Brick(osim.Vec3(0.1, bladepoint/2, 0.01))
pvertBlade.attachGeometry(theBladeGeometry)
theBladeGeometry.setColor(osim.Vec3(0, 1, 0.5))
bbaan.addBody(pvertBlade)

ptvertbl = osim.WeldJoint("ptvert_Joint",
                          pblade,
                          osim.Vec3(0, 0, 0),
                          osim.Vec3(0, 0, 0),
                          pvertBlade,
                          osim.Vec3(0, 0, 0),
                          osim.Vec3(0, 0, 0))
bbaan.addJoint(ptvertbl)

"""       The Rower                              """
####################################################

# Lower leg
lower_l = osim.Body("Lower_leg",
                  llegw,
                  osim.Vec3(0, 0, 0),
                  osim.Inertia(1, 1, 1))

lowerGeometry_l = osim.Cylinder(0.05, llegl/2)
lower_l.attachGeometry(lowerGeometry_l)
lowerGeometry_l.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(lower_l)

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

# Inertia was 1 voor de volgende 2. Maar moet 0.1 zijn omdat er vlak bij de uitzet een stukje heel snelle beweging
# inzit dat niet te volgen is. Zo lijkt het. 0.15 is al te hoog.

# Upper Arm left
upperal = osim.Body("Upper_Arm_Left",
                    uarmw,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(0.1, 0.1, 0.1))

upperalGeometry = osim.Brick(osim.Vec3(0.04, uarml/2, 0.04))
upperal.attachGeometry(upperalGeometry)
upperalGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(upperal)

# Lower Arm left
loweral = osim.Body("Lower_Arm_Left",
                    larmw,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(1, 1, 1))

loweral_Geometry = osim.Brick(osim.Vec3(0.04, larml/2, 0.04))
loweral.attachGeometry(loweral_Geometry)
loweral_Geometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(loweral)

# Upper Arm right
upperar = osim.Body("Upper_Arm_Right",
                    uarmw,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(1, 1, 1))

upperarGeometry = osim.Brick(osim.Vec3(0.04, uarml/2, 0.04))
upperar.attachGeometry(upperarGeometry)
upperarGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(upperar)


# Lower Arm right
lowerar = osim.Body("Lower_Arm_Right",
                    larmw,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(1, 1, 1))

lowerar_Geometry = osim.Brick(osim.Vec3(0.04, larml/2, 0.04))
lowerar.attachGeometry(lowerar_Geometry)
lowerar_Geometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(lowerar)

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
coord.setRangeMax(pi)
coord.setDefaultValue(math.radians(78.9))
coord.set_clamped(True)

act = osim.CoordinateActuator('hipangle')
act.setName('hipact')
act.set_optimal_force(1)
bbaan.addForce(act)

kneeJoint = osim.PinJoint("knee_Joint",
                          upper,
                          osim.Vec3(0, ulegl/2, 0),
                          osim.Vec3(0, 0, 0),
                          lower_l,
                          osim.Vec3(0, -llegl/2, 0),
                          osim.Vec3(0, 0, 0))
bbaan.addJoint(kneeJoint)

coord = kneeJoint.updCoordinate()
coord.setName('kneeangle')
coord.setRangeMin(-pi/4)
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
act.set_optimal_force(1)
bbaan.addForce(act)

footJoint = osim.PinJoint("foot_Joint",
                          stretcher,
                          osim.Vec3(0, seatHeight/2, 0),
                          osim.Vec3(0, 0, pi/2),
                          lower_l,
                          osim.Vec3(0, llegl/2, 0),
                          osim.Vec3(0, 0, 0))
bbaan.addJoint(footJoint)

coord = footJoint.updCoordinate()
coord.setName('footangle')
coord.setRangeMin(-pi/2)
coord.setRangeMax(pi/2)
coord.setDefaultValue(math.radians(5))
coord.set_clamped(True)

act = osim.CoordinateActuator('footangle')
act.setName('footact')
act.set_optimal_force(1)
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
act.set_optimal_force(1)
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

"""
uarmlJoint = osim.BallJoint("upper_arm_left_Joint",
                            shoulder,
                            osim.Vec3(0, 0, shoulderl/2),
                            osim.Vec3(0, 0, pi/2),
                            upperal,
                            osim.Vec3(0, -uarml/2, 0),
                            osim.Vec3(0, 0, 0))
"""

sp= osim.SpatialTransform()
arr = osim.ArrayStr()
arr.append('uarmleft_up')
sp.updTransformAxis(0).setCoordinateNames(arr)
sp.updTransformAxis(0).set_function(osim.LinearFunction())
sp.updTransformAxis(0).set_axis(osim.Vec3(0,0,1))
arr = osim.ArrayStr()
arr.append('uarmleft_out')
sp.updTransformAxis(1).setCoordinateNames(arr)
sp.updTransformAxis(1).set_function(osim.LinearFunction())
sp.updTransformAxis(1).set_axis(osim.Vec3(-1,0,0))
arr = osim.ArrayStr()
arr.append('uarmleft_trn')
sp.updTransformAxis(2).setCoordinateNames(arr)
sp.updTransformAxis(2).set_function(osim.LinearFunction())
sp.updTransformAxis(2).set_axis(osim.Vec3(0,1,0))

uarmlJoint = osim.CustomJoint("upper_arm_left_Joint",
                            shoulder,
                            osim.Vec3(0, 0, shoulderl/2),
                            osim.Vec3(0, 0, pi/2),
                            upperal,
                            osim.Vec3(0, -uarml/2, 0),                              
                            osim.Vec3(0, 0, 0), sp)

bbaan.addJoint(uarmlJoint)
coord = uarmlJoint.upd_coordinates(0)
coord.setDefaultValue(math.radians(0))
coord = uarmlJoint.upd_coordinates(1)
coord.setDefaultValue(math.radians(0))
coord = uarmlJoint.upd_coordinates(2)
coord.setDefaultValue(math.radians(0))

act = osim.CoordinateActuator('uarmleft_up')
act.setName('ualo_act')
act.set_optimal_force(1)
bbaan.addForce(act)
act = osim.CoordinateActuator('uarmleft_out')
act.setName('ualt_act')
act.set_optimal_force(1)
bbaan.addForce(act)
act = osim.CoordinateActuator('uarmleft_trn')
act.setName('ualu_act')
act.set_optimal_force(1)
bbaan.addForce(act)

larmltf = osim.SpatialTransform()
sinout = osim.ArrayStr()
sinout.append("se_inout")
larmltf.updTransformAxis(0).setCoordinateNames(sinout)
larmltf.updTransformAxis(0).set_function(osim.LinearFunction())
supdown = osim.ArrayStr()
supdown.append("se_updown")
larmltf.updTransformAxis(2).setCoordinateNames(supdown)
larmltf.updTransformAxis(2).set_function(osim.LinearFunction())

selbowJoint = osim.CustomJoint("elbow_left_Joint",
                              upperal,
                              osim.Vec3(0, uarml/2, 0),
                              osim.Vec3(0, 0, 0),
                              loweral,
                              osim.Vec3(0, -larml/2, 0),
                              osim.Vec3(0, 0, 0),
                              larmltf)
bbaan.addJoint(selbowJoint)

act = osim.CoordinateActuator('se_inout')
act.setName('sinout_act')
act.set_optimal_force(1)

bbaan.addForce(act)
act = osim.CoordinateActuator('se_updown')
act.setName('supdown_act')
act.set_optimal_force(1)
bbaan.addForce(act)

handlelJoint = osim.BallJoint("handle_left_Joint",
                            soar,
                            osim.Vec3(0, (inbhand+outboard)/2, 0),
                            osim.Vec3(pi/2, -pi/4, 0),
                            loweral,
                            osim.Vec3(0, larml/2, 0),
                            osim.Vec3(0, 0, pi))
bbaan.addJoint(handlelJoint)
coord = handlelJoint.upd_coordinates(0)
coord.setName('handleleft_out')
#coord.setDefaultValue(math.radians(-10.32))
coord = handlelJoint.upd_coordinates(1)
coord.setName('handleleft_trn')
#coord.setDefaultValue(math.radians(-0.08))
coord = handlelJoint.upd_coordinates(2)
coord.setName('handleleft_up')
#coord.setDefaultValue(math.radians(-0.54))

act = osim.CoordinateActuator('handleleft_out')
act.setName('hlout_act')
act.set_optimal_force(1)
bbaan.addForce(act)
act = osim.CoordinateActuator('handleleft_trn')
act.setName('hltrn_act')
act.set_optimal_force(1)
bbaan.addForce(act)
act = osim.CoordinateActuator('handleleft_up')
act.setName('hlup_act')
act.set_optimal_force(1)
bbaan.addForce(act)

"""
uarmrJoint = osim.BallJoint("upper_arm_right_Joint",
                            shoulder,
                            osim.Vec3(0, 0, -shoulderl/2),
                            osim.Vec3(0, 0, -pi/2),
                            upperar,
                            osim.Vec3(0, uarml/2, 0),
                            osim.Vec3(0, 0, 0))
"""

sp= osim.SpatialTransform()
arr = osim.ArrayStr()
arr.append('uarmright_up')
sp.updTransformAxis(0).setCoordinateNames(arr)
sp.updTransformAxis(0).set_function(osim.LinearFunction())
sp.updTransformAxis(0).set_axis(osim.Vec3(0,0,1))
arr = osim.ArrayStr()
arr.append('uarmright_out')
sp.updTransformAxis(1).setCoordinateNames(arr)
sp.updTransformAxis(1).set_function(osim.LinearFunction())
sp.updTransformAxis(1).set_axis(osim.Vec3(-1,0,0))
arr = osim.ArrayStr()
arr.append('uarmright_trn')
sp.updTransformAxis(2).setCoordinateNames(arr)
sp.updTransformAxis(2).set_function(osim.LinearFunction())
sp.updTransformAxis(2).set_axis(osim.Vec3(0,1,0))

uarmrJoint = osim.CustomJoint("upper_arm_right_Joint",
                            shoulder,
                            osim.Vec3(0, 0, -shoulderl/2),
                            osim.Vec3(0, 0, -pi/2),
                            upperar,
                            osim.Vec3(0, uarml/2, 0),                              
                            osim.Vec3(0, 0, 0), sp)

bbaan.addJoint(uarmrJoint)
coord = uarmrJoint.upd_coordinates(0)
coord.setDefaultValue(math.radians(0))
coord = uarmrJoint.upd_coordinates(1)
coord.setDefaultValue(math.radians(0))
coord = uarmrJoint.upd_coordinates(2)
coord.setDefaultValue(math.radians(0))

act = osim.CoordinateActuator('uarmright_up')
act.setName('uaro_act')
act.set_optimal_force(1)
bbaan.addForce(act)
act = osim.CoordinateActuator('uarmleft_out')
act.setName('uart_act')
act.set_optimal_force(1)
bbaan.addForce(act)
act = osim.CoordinateActuator('uarmleft_trn')
act.setName('uaru_act')
act.set_optimal_force(1)
bbaan.addForce(act)

larmrtf = osim.SpatialTransform()
pinout = osim.ArrayStr()
pinout.append("pe_inout")
larmrtf.updTransformAxis(0).setCoordinateNames(pinout)
larmrtf.updTransformAxis(0).set_function(osim.LinearFunction())
pupdown = osim.ArrayStr()
pupdown.append("pe_updown")
larmrtf.updTransformAxis(2).setCoordinateNames(pupdown)
larmrtf.updTransformAxis(2).set_function(osim.LinearFunction())

pelbowJoint = osim.CustomJoint("elbow_right_Joint",
                              upperar,
                              osim.Vec3(0, -uarml/2, 0),
                              osim.Vec3(0, 0, 0),
                              lowerar,
                              osim.Vec3(0, larml/2, 0),
                              osim.Vec3(0, 0, 0),
                              larmrtf)
bbaan.addJoint(pelbowJoint)

act = osim.CoordinateActuator('pe_inout')
act.setName('pinout_act')
bbaan.addForce(act)
act = osim.CoordinateActuator('pe_updown')
act.setName('pundown_act')
bbaan.addForce(act)


handlerJoint = osim.BallJoint("handle_right_Joint",
                            poar,
                            osim.Vec3(0, (inbhand+outboard)/2, 0),
                            osim.Vec3(-pi/2, pi/4, 0),
                            lowerar,
                            osim.Vec3(0, -larml/2, 0),
                            osim.Vec3(0, 0, 0))
bbaan.addJoint(handlerJoint)
coord = handlerJoint.upd_coordinates(0)
coord.setName('handleright_out')
#coord.setDefaultValue(math.radians(-10.32))
coord = handlerJoint.upd_coordinates(1)
coord.setName('handleright_trn')
#coord.setDefaultValue(math.radians(-0.08))
coord = handlerJoint.upd_coordinates(2)
coord.setName('handleright_up')
#coord.setDefaultValue(math.radians(-0.54))

act = osim.CoordinateActuator('handleright_out')
act.setName('hrout_act')
bbaan.addForce(act)
act = osim.CoordinateActuator('handleright_trn')
act.setName('hrtrn_act')
bbaan.addForce(act)
act = osim.CoordinateActuator('handleright_up')
act.setName('hrup_act')
bbaan.addForce(act)


#  alle demping hier ivm vergelijk met Delft. Zinnig?
"""
act = osim.SpringGeneralizedForce()
act.setName('hip_Damping')
act.set_coordinate('hipangle')
act.set_stiffness(1)
act.set_viscosity(1)
bbaan.addForce(act)
act = osim.SpringGeneralizedForce()
act.setName('knee_Damping')
act.set_coordinate('kneeangle')
act.set_stiffness(1)
act.set_viscosity(1)
bbaan.addForce(act)
act = osim.SpringGeneralizedForce()
act.setName('foot_Damping')
act.set_coordinate('footangle')
act.set_stiffness(1)
act.set_viscosity(1)
bbaan.addForce(act)
act = osim.SpringGeneralizedForce()
act.setName('seatpos_Damping')
act.set_coordinate('seatpos')
act.set_stiffness(1)
act.set_viscosity(1)
bbaan.addForce(act)
act = osim.SpringGeneralizedForce()
act.setName('lbackangle_Damping')
act.set_coordinate('lbackangle')
act.set_stiffness(1)
act.set_viscosity(1)
bbaan.addForce(act)
act = osim.SpringGeneralizedForce()
act.setName('uarmleft_out_Damping')
act.set_coordinate('uarmleft_up')
act.set_stiffness(1)
act.set_viscosity(1)
bbaan.addForce(act)
act = osim.SpringGeneralizedForce()
act.setName('uarmleft_up_Damping')
act.set_coordinate('uarmleft_out')
act.set_stiffness(1)
act.set_viscosity(1)
bbaan.addForce(act)
act = osim.SpringGeneralizedForce()
act.setName('uarmleft_trn_Damping')
act.set_coordinate('uarmleft_trn')
act.set_stiffness(1)
act.set_viscosity(1)
bbaan.addForce(act)
act = osim.SpringGeneralizedForce()
act.setName('uarmright_out_Damping')
act.set_coordinate('uarmright_up')
act.set_stiffness(1)
act.set_viscosity(1)
bbaan.addForce(act)
act = osim.SpringGeneralizedForce()
act.setName('uarmright_up_Damping')
act.set_coordinate('uarmright_out')
act.set_stiffness(1)
act.set_viscosity(1)
bbaan.addForce(act)
act = osim.SpringGeneralizedForce()
act.setName('uarmright_trn_Damping')
act.set_coordinate('uarmright_trn')
act.set_stiffness(1)
act.set_viscosity(1)
bbaan.addForce(act)
"""

"""   Keep both blades level using constraints   """

constraint = osim.CoordinateCouplerConstraint()
constraint.setName("s_blade")
independentCoords = osim.ArrayStr()
independentCoords.append('soarinout')
constraint.setIndependentCoordinateNames(independentCoords)
constraint.setDependentCoordinateName('sblpos')
constraint.setFunction(osim.LinearFunction(-1, 0))
bbaan.addConstraint(constraint)


constraint2 = osim.CoordinateCouplerConstraint()
constraint2.setName("p_blade")
independentCoords2 = osim.ArrayStr()
independentCoords2.append('poarinout')
constraint2.setIndependentCoordinateNames(independentCoords2)
constraint2.setDependentCoordinateName('pblpos')
constraint2.setFunction(osim.LinearFunction(-1, 0))
bbaan.addConstraint(constraint2)

"""  Contact Geometry  """
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

sblgeom = osim.ContactMesh('box0.05_0.45_0.45.stl',
                           osim.Vec3(0, 0, 0),
                           osim.Vec3(0, -pi/2, 0),
                           sblade)
sblgeom.setName("sblad")
bbaan.addContactGeometry(sblgeom)

pblgeom = osim.ContactMesh('box0.05_0.45_0.45.stl',
                           osim.Vec3(0, 0, 0),
                           osim.Vec3(0, pi/2, 0),
                           pblade)
pblgeom.setName("pblad")
bbaan.addContactGeometry(pblgeom)


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

# Blade values in the direction of the blade
# from BladeForce.cpp:       const Real stblade = 1e9, disblade = 0.8, statfblade = 0.01, dynfblade = 0.01, viscblade = 0.01, transvblade = 0.02;

stiffness           = 50000;
dissipation         = 0.8;
staticFriction      = 1.1;
dynamicFriction     = 1.1;
viscousFriction     = 1.04;
transitionVelocity  = 0.02;

#e_2 = osim.BladeForce()
# voorlopig
e_2 = osim.ElasticFoundationForce()
e_2.setName('SBlad')
e_2.addGeometry('sblad')
e_2.addGeometry('baan')
e_2.setStiffness(stiffness)
e_2.setDissipation(dissipation)
e_2.setStaticFriction(staticFriction)
e_2.setDynamicFriction(dynamicFriction)
e_2.setViscousFriction(viscousFriction)
e_2.setTransitionVelocity(transitionVelocity)
bbaan.addForce(e_2)

#e_3 = osim.BladeForce()
# voorlopig
e_3 = osim.ElasticFoundationForce()
e_3.setName('PBlad')
e_3.addGeometry('pblad')
e_3.addGeometry('baan')
e_3.setStiffness(stiffness)
e_3.setDissipation(dissipation)
e_3.setStaticFriction(staticFriction)
e_3.setDynamicFriction(dynamicFriction)
e_3.setViscousFriction(viscousFriction)
e_3.setTransitionVelocity(transitionVelocity)
bbaan.addForce(e_3)

"""      Markers                                 """
####################################################

mseat = osim.Marker(markers[1], seat, osim.Vec3(0))
bbaan.addMarker(mseat)

mshoulder = osim.Marker(markers[2], shoulder, osim.Vec3(0, 0, 0))
bbaan.addMarker(mshoulder)

mselbow = osim.Marker(markers[4], upperal, osim.Vec3(0, uarml/2, 0))
bbaan.addMarker(mselbow)

mpelbow = osim.Marker(markers[3], upperar, osim.Vec3(0, -uarml/2, 0))
bbaan.addMarker(mpelbow)

mshandle = osim.Marker(markers[6], soar, osim.Vec3(0, (outboard+inbhand)/2, 0))
bbaan.addMarker(mshandle)

mphandle = osim.Marker(markers[5], poar, osim.Vec3(0, (outboard+inbhand)/2, 0))
bbaan.addMarker(mphandle)

"""      The rest                                """
####################################################

reporter = osim.ConsoleReporter()
reporter.set_report_time_interval(1.0)
bbaan.addComponent(reporter)

bbaan.finalizeConnections()
bbaan.printToXML("BootBaan.osim")

