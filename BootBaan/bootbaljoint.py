import opensim as osim
import math
pi = math.pi

"""       Parameters of the model                """
####################################################
# rower
rowerw     = 80
rowerl     = 1.86

# Boat parameters
boatw      = 10
# rest rigging

# heigth of seat
seatHeight = 0.1
# width of boat
boatWidth = 0.6
# height of boat (above water)
boatHeight = 0.2
# distance between locks
span     = 1.60
# height lock above the seat
height   = 0.20
# horizontal distance between stretcher and lock
place    = 0.50
# inboard part of the oar minus half of the hand
inboard  = 0.88-0.05
# outboard part of the oar
outboard = 1.98
# begin blad vanaf einde
bladepoint = 0.4   # scull

"""   Derived parameters    """
llegw      = rowerw * 0.1
ulegw      = rowerw * 0.1
lbackw     = rowerw * 0.2
ubackw     = rowerw * 0.2
shoulderw  = rowerw * 0.1
uarmw      = rowerw * 0.05
larmw      = rowerw * 0.05
headw      = rowerw * 0.1

llegl      = rowerl * 0.29
ulegl      = rowerl * 0.29
lbackl     = rowerl * 0.10
ubackl     = rowerl * 0.26
shoulderl  = rowerl * 0.43
uarml      = rowerl * 0.18
larml      = rowerl * 0.17
headl      = rowerl * 0.15

"""   Auxiliary functions    """
####################################################

# global bbaan

"""  Define Contact Force Parameters for the boat
####################################################
The default values
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

def addContactForce(cs, item):
    """ Make a Hunt Crossley Force wrt to the ContactHalfspace 'baantje'
        Set parameters prior to calling!   
    """
    hf = osim.HuntCrossleyForce()
    hf.setName(item)
    hf.addGeometry(item)
    hf.addGeometry('baantje')
    hf.setStiffness(stiffness)
    hf.setDissipation(dissipation)
    hf.setStaticFriction(staticFriction)
    hf.setDynamicFriction(dynamicFriction)
    hf.setViscousFriction(viscousFriction)
    hf.setTransitionVelocity(transitionVelocity)
    cs.setName(name)
    bbaan.addContactGeometry(cs)
    bbaan.addForce(hf)


""""  The model and the rowing course   """
####################################################

bbaan = osim.Model()
bbaan.setName("BootBaan")
bbaan.setGravity(osim.Vec3(0, -9.90665, 0))

theCourse = osim.Body("The_rowing_course",
                      1.0,
                      osim.Vec3(0, 0, 0),
                      osim.Inertia(0, 0, 0))

theCourseGeometry = osim.Brick(osim.Vec3(40, 0.05, 5))
theCourse.attachGeometry(theCourseGeometry)
theCourseGeometry.setColor(osim.Vec3(0, 0, 1))

theCourseContactSpace = osim.ContactHalfSpace(osim.Vec3(0, 0.0, 0),
                                              osim.Vec3(0, 0, -pi/2),
                                              theCourse)
theCourseContactSpace.setName("baantje")
bbaan.addContactGeometry(theCourseContactSpace)
bbaan.addBody(theCourse)

"""   The boat   """
####################################################
theBoat = osim.Body()
theBoat.setName('TheBoat')
theBoat.setMass(boatw)
theBoat.setInertia(osim.Inertia(1, 1, 1, 0, 0, 0))

theBoatGeometry = osim.Brick(osim.Vec3(6, boatHeight/2, boatWidth/2))
theBoat.attachGeometry(theBoatGeometry)
theBoatGeometry.setColor(osim.Vec3(0, 1, 1))
bbaan.addBody(theBoat)

name = 'boatcorner_1'
cs = osim.ContactSphere()
cs.setRadius(0.1)
cs.setLocation(osim.Vec3(6, -boatHeight/2, -boatWidth/2))
cs.setFrame(theBoat)
addContactForce(cs, name)

name = 'boatcorner_2'
cs = osim.ContactSphere()
cs.setRadius(0.1)
cs.setLocation(osim.Vec3(-6, -boatHeight/2, -boatWidth/2))
cs.setFrame(theBoat)
addContactForce(cs, name)

name = 'boatcorner_3'
cs = osim.ContactSphere()
cs.setRadius(0.1)
cs.setLocation(osim.Vec3(6, -boatHeight/2, boatWidth/2))
cs.setFrame(theBoat)
addContactForce(cs, name)

name = 'boatcorner_4'
cs = osim.ContactSphere()
cs.setRadius(0.1)
cs.setLocation(osim.Vec3(-6, -boatHeight/2, boatWidth/2))
cs.setFrame(theBoat)
addContactForce(cs, name)

"""   Foot stretcher           """
####################################################
stretcher = osim.Body("Stretcher",
                      1.0,
                      osim.Vec3(0, 0, 0),
                      osim.Inertia(0, 0, 0))

stretcherGeometry = osim.Brick(osim.Vec3(seatHeight/2, seatHeight/2, boatWidth/2))
stretcher.attachGeometry(stretcherGeometry)
stretcherGeometry.setColor(osim.Vec3(0.5,0.5,1))
bbaan.addBody(stretcher)

"""   Seat                     """
####################################################
seat = osim.Body("Seat",
                 1.0,
                 osim.Vec3(0, 0, 0),
                 osim.Inertia(0, 0, 0))

seatGeometry = osim.Brick(osim.Vec3(seatHeight/2, seatHeight/2, boatWidth/2))
seat.attachGeometry(seatGeometry)
seatGeometry.setColor(osim.Vec3(0.5,0.5,1))
bbaan.addBody(seat)

"""   Bow                     """
####################################################
bow = osim.Body("Bow",
                 0.1,
                 osim.Vec3(0, 0, 0),
                 osim.Inertia(0, 0, 0))

bowGeometry = osim.Sphere(0.05)
bow.attachGeometry(bowGeometry)
bowGeometry.setColor(osim.Vec3(0.5,0.5,1))
bbaan.addBody(bow)

"""   Port rigger              """
####################################################
"""
 - Zero for height is on top of the seat. 
 - Connection rigger to the boat (in boat coordinates)
        x=0.5, y =0, z= +boatWidth/2
 - Place of lock
        x=0.5, boatHeight/2+seatHeight/2, span/2
 - Length of rigger body
     sqrt(a^2 + b^2)
"""
a = span/2 - boatWidth/2
b = boatHeight/2+seatHeight/2
riglength = math.sqrt(math.pow(a, 2)+math.pow(b, 2))
rigangle = math.atan(b/a)
prigger = osim.Body("PortRigger",
                    1.0,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(0, 0, 0))

prigGeometry = osim.Cylinder(0.02, riglength/2)
prigger.attachGeometry(prigGeometry)
prigGeometry.setColor(osim.Vec3(0.5, 0.5, 1))
bbaan.addBody(prigger)


"""   Port lock                """
####################################################
plock = osim.Body("PortLock",
                  1.0,
                  osim.Vec3(0, 0, 0),
                  osim.Inertia(0, 0, 0))

plockGeometry = osim.Cylinder(0.01, 0.025)
plock.attachGeometry(plockGeometry)
plockGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(plock)



"""   Port oar                 """
####################################################
poar = osim.Body("PortOar",
                 1.0,
                 osim.Vec3(0, 0, 0),
                 osim.Inertia(0, 0, 0))

poarGeometry = osim.Cylinder(0.02, (inboard+outboard)/2)
poar.attachGeometry(poarGeometry)
poarGeometry.setColor(osim.Vec3(0.5, 0.5, 1))
bbaan.addBody(poar)


"""   Port blade               """
# weight? 
####################################################
pblade = osim.Body("PortBlade",
                   1.0,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(0, 0, 0))

pbladeGeometry = osim.Brick(osim.Vec3(0.02, bladepoint/2, bladepoint/2))
pblade.attachGeometry(pbladeGeometry)
pbladeGeometry.setColor(osim.Vec3(0.5, 0.5, 1))
bbaan.addBody(pblade)

""" Friction values for the blades  """
stiffness           = 1000000;
dissipation         = 0.8;
staticFriction      = 0.1;
dynamicFriction     = 0.4;
viscousFriction     = 0.4;
transitionVelocity  = 0.1;

name = 'pblcorner_1'
cs = osim.ContactSphere()
cs.setRadius(0.05)
cs.setLocation(osim.Vec3(0, bladepoint/2, bladepoint/2))
cs.setFrame(pblade)
addContactForce(cs, name)

name = 'pblcorner_2'
cs = osim.ContactSphere()
cs.setRadius(0.05)
cs.setLocation(osim.Vec3(0, bladepoint/2, -bladepoint/2))
cs.setFrame(pblade)
addContactForce(cs, name)

name = 'pblcorner_3'
cs = osim.ContactSphere()
cs.setRadius(0.05)
cs.setLocation(osim.Vec3(0, -bladepoint/2, bladepoint/2))
cs.setFrame(pblade)
addContactForce(cs, name)

name = 'pblcorner_4'
cs = osim.ContactSphere()
cs.setRadius(0.05)
cs.setLocation(osim.Vec3(0, -bladepoint/2, -bladepoint/2))
cs.setFrame(pblade)
addContactForce(cs, name)


"""   Starboard rigger         """
####################################################
srigger = osim.Body("StarboardRigger",
                    1.0,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(0, 0, 0))

srigGeometry = osim.Cylinder(0.02, riglength/2)
srigger.attachGeometry(srigGeometry)
srigGeometry.setColor(osim.Vec3(0.5, 0.5, 1))
bbaan.addBody(srigger)


"""   Starboard lock          """
####################################################
slock = osim.Body("StarboardLock",
                  1.0,
                  osim.Vec3(0, 0, 0),
                  osim.Inertia(0, 0, 0))

slockGeometry = osim.Cylinder(0.01, 0.025)
slock.attachGeometry(slockGeometry)
slockGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(slock)


"""   Starboard oar            """
####################################################
soar = osim.Body("StarboardOar",
                 1.0,
                 osim.Vec3(0, 0, 0),
                 osim.Inertia(0, 0, 0))

soarGeometry = osim.Cylinder(0.02, (inboard+outboard)/2)
soar.attachGeometry(soarGeometry)
soarGeometry.setColor(osim.Vec3(0.5, 0.5, 1))
bbaan.addBody(soar)

"""   Starboard blade          """
####################################################
sblade = osim.Body("StarboardBlade",
                   1.0,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(0, 0, 0))

sbladeGeometry = osim.Brick(osim.Vec3(0.02, bladepoint/2, bladepoint/2))
sblade.attachGeometry(sbladeGeometry)
sbladeGeometry.setColor(osim.Vec3(0.5, 0.5, 1))
bbaan.addBody(sblade)

name = 'sblcorner_1'
cs = osim.ContactSphere()
cs.setRadius(0.05)
cs.setLocation(osim.Vec3(0, bladepoint/2, bladepoint/2))
cs.setFrame(sblade)
addContactForce(cs, name)

name = 'sblcorner_2'
cs = osim.ContactSphere()
cs.setRadius(0.05)
cs.setLocation(osim.Vec3(0, bladepoint/2, -bladepoint/2))
cs.setFrame(sblade)
addContactForce(cs, name)

name = 'sblcorner_3'
cs = osim.ContactSphere()
cs.setRadius(0.05)
cs.setLocation(osim.Vec3(0, -bladepoint/2, bladepoint/2))
cs.setFrame(sblade)
addContactForce(cs, name)

name = 'sblcorner_4'
cs = osim.ContactSphere()
cs.setRadius(0.05)
cs.setLocation(osim.Vec3(0, -bladepoint/2, -bladepoint/2))
cs.setFrame(sblade)
addContactForce(cs, name)

"""   The joints               """
####################################################
courseJoint = osim.WeldJoint("courseJoint",
                             bbaan.getGround(),
                             osim.Vec3(0, 0.025, 0),
                             osim.Vec3(0, 0, 0),
                             theCourse,
                             osim.Vec3(0, 0, 0),
                             osim.Vec3(0, 0, 0))
bbaan.addJoint(courseJoint)

boatJoint = osim.FreeJoint("boatJoint",
                           theCourse,
                           osim.Vec3(0, boatHeight, 0),
                           osim.Vec3(0, 0, 0),
                           theBoat,
                           osim.Vec3(0, 0, 0),
                           osim.Vec3(0, 0, 0))
bbaan.addJoint(boatJoint)

stretcherJoint = osim.WeldJoint("stretcherJoint",
                                theBoat,
                                osim.Vec3(0, boatHeight/2, 0),
                                osim.Vec3(0, 0, 0),
                                stretcher,
                                osim.Vec3(0, 0, 0),
                                osim.Vec3(0, 0, 0))
bbaan.addJoint(stretcherJoint)

seatJoint = osim.SliderJoint("seatJoint",
                             theBoat,
                             osim.Vec3(ulegl+llegl, boatHeight/2+seatHeight/2, 0),
                             osim.Vec3(0, 0, 0),
                             seat,
                             osim.Vec3(0, 0, 0),
                             osim.Vec3(0, 0, 0))
bbaan.addJoint(seatJoint)

"""
seatact = osim.CoordinateActuator('seatpos')
seatact.setName('seatact')
bbaan.addForce(seatact)
"""

bowJoint = osim.WeldJoint("bowJoint",
                          theBoat,
                          osim.Vec3(6, boatHeight/2, 0),
                          osim.Vec3(0, 0, 0),
                          bow,
                          osim.Vec3(0, 0, 0),
                          osim.Vec3(0, 0, 0))
bbaan.addJoint(bowJoint)

seatcoord = seatJoint.updCoordinate()
seatcoord.setName('seatpos')
seatcoord.setRangeMin(-(ulegl+llegl))
seatcoord.setRangeMax(0.0)


"""  Port joints     """
####################################################
prigJoint = osim.WeldJoint("portrigJoint",
                           theBoat,
                           osim.Vec3(0.5, 0, boatWidth/2),
                           osim.Vec3(-pi/2-rigangle, 0, 0),
                           prigger,
                           osim.Vec3(0, riglength/2, 0),
                           osim.Vec3(0, 0, 0))
bbaan.addJoint(prigJoint)


plocJoint = osim.PinJoint("portlockJoint",
                          prigger,
                          osim.Vec3(0, -riglength/2, 0),
                          osim.Vec3(rigangle, 0, 0),
                          plock,
                          osim.Vec3(0, 0.025, 0),
                          osim.Vec3(pi/2, 0, 0))
bbaan.addJoint(plocJoint)
pblcoord = plocJoint.updCoordinate()
pblcoord.setName('plocangle')
pblcoord.setRangeMin(-pi/2)
pblcoord.setRangeMax(pi/2)


poarJoint = osim.PinJoint("portoarJoint",
                          plock,
                          osim.Vec3(0, 0, 0),
                          osim.Vec3(pi/2, pi/2, 0),
                          poar,
                          osim.Vec3(0, (outboard-inboard)/2,  0),
                          osim.Vec3(0, 0, 0))
bbaan.addJoint(poarJoint)
pblcoord = poarJoint.updCoordinate()
pblcoord.setName('poarinout')
pblcoord.setRangeMin(0.0)
pblcoord.setRangeMax(0.3)

pbladeJoint = osim.PinJoint("portbladeJoint",
                            poar,
                            osim.Vec3(0, -(inboard+outboard)/2+bladepoint,  0),
                            osim.Vec3(0, 0, 0),
                            pblade,
                            osim.Vec3(0, bladepoint/2,  0),
                            osim.Vec3(0, 0, 0))                            

pblcoord = pbladeJoint.updCoordinate()
pblcoord.setName('pblpos')
pblcoord.setRangeMin(-0.3)
pblcoord.setRangeMax(0.0)

bbaan.addJoint(pbladeJoint)



""" Starboard joints    """
####################################################
srigJoint = osim.WeldJoint("starboardrigJoint",
                           theBoat,
                           osim.Vec3(0.5, 0, -boatWidth/2),
                           osim.Vec3(pi/2+rigangle, 0, 0),
                           srigger,
                           osim.Vec3(0, riglength/2, 0),
                           osim.Vec3(0, 0, 0))
bbaan.addJoint(srigJoint)

slocJoint = osim.PinJoint("starboardlockJoint",
                          srigger,
                          osim.Vec3(0, -riglength/2, 0),
                          osim.Vec3(-rigangle, 0, 0),
                          slock,
                          osim.Vec3(0, 0.025, 0),
                          osim.Vec3(-pi/2, 0, 0))
bbaan.addJoint(slocJoint)
pblcoord = slocJoint.updCoordinate()
pblcoord.setName('slocangle')
pblcoord.setRangeMin(-pi/2)
pblcoord.setRangeMax(pi/2)


"""  for CustomJoint
soartf = osim.SpatialTransform()
pupdown= osim.ArrayStr()
pupdown.append("s_updown")
soartf.updTransformAxis(0).setCoordinateNames(pupdown)
soartf.updTransformAxis(0).set_function(osim.LinearFunction())
pangle = osim.ArrayStr()
pangle.append("s_angle")
soartf.updTransformAxis(2).setCoordinateNames(pangle)
soartf.updTransformAxis(2).set_function(osim.LinearFunction())
"""

soarJoint = osim.PinJoint("starboardoarJoint",
                          slock,
                          osim.Vec3(0, 0, 0),
                          osim.Vec3(pi/2, -pi/2, pi),
                          soar,
                          osim.Vec3(0, (outboard-inboard)/2,  0),
                          osim.Vec3(0, 0, 0))

bbaan.addJoint(soarJoint)
pblcoord = soarJoint.updCoordinate()
pblcoord.setName('soarinout')
pblcoord.setRangeMin(0.0)
pblcoord.setRangeMax(0.3)


sbladeJoint = osim.PinJoint("starboardbladeJoint",
                            soar,
                            osim.Vec3(0, -(inboard+outboard)/2+bladepoint,  0),
                            osim.Vec3(0, 0, pi),
                            sblade,
                            osim.Vec3(0, -bladepoint/2,  0),
                            osim.Vec3(0, 0, 0))                            

sblcoord = sbladeJoint.updCoordinate()
sblcoord.setName('sblpos')
sblcoord.setRangeMin(-0.3)
sblcoord.setRangeMax(0.0)

bbaan.addJoint(sbladeJoint)

"""       The Rower                              """
####################################################

# Lower leg
lower = osim.Body("Lower_leg",
                  llegw,
                  osim.Vec3(0, 0, 0),
                  osim.Inertia(0, 0, 0))

lowerGeometry = osim.Cylinder(0.05, llegl/2)
lower.attachGeometry(lowerGeometry)
lowerGeometry.setColor(osim.Vec3(1, 0, 1))
bbaan.addBody(lower)

name = 'knee'
cs = osim.ContactSphere()
cs.setRadius(0.02)
cs.setLocation(osim.Vec3(-0.3, llegl/2, 0))
cs.setFrame(lower)
addContactForce(cs, name)

# Upper leg
upper = osim.Body("The_upper_leg",
                  ulegw,
                  osim.Vec3(0, 0, 0),
                  osim.Inertia(0, 0, 0))

upperGeometry = osim.Cylinder(0.05, ulegl/2)
upper.attachGeometry(upperGeometry)
upperGeometry.setColor(osim.Vec3(0.5, 0.5, 1))
bbaan.addBody(upper)

# Lower back
lowerb = osim.Body("The_lower_back",
                   lbackw,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(0, 0, 0))

lowerbGeometry = osim.Brick(osim.Vec3(0.04, lbackl/2, 0.08))
lowerb.attachGeometry(lowerbGeometry)
lowerbGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(lowerb)

# Upper back
upperb = osim.Body("The_upper_back",
                   ubackw,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(0, 0, 0))

upperbGeometry = osim.Brick(osim.Vec3(0.05, ubackl/2, 0.10))
upperb.attachGeometry(upperbGeometry)
upperbGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(upperb)

# Shoulder
shoulder = osim.Body("Shoulder",
                     shoulderw,
                     osim.Vec3(0, 0, 0),
                     osim.Inertia(0, 0, 0))

shoulderGeometry = osim.Brick(osim.Vec3(0.04, 0.04, shoulderl/2))
shoulder.attachGeometry(shoulderGeometry)
shoulderGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(shoulder)

# Head
head = osim.Body("Head",
                 headw,
                 osim.Vec3(0, 0, 0),
                 osim.Inertia(0, 0, 0))

headGeometry = osim.Brick(osim.Vec3(0.04, headl/2, headl/2))
head.attachGeometry(headGeometry)
headGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(head)


# Upper Arm left
upperal = osim.Body("Upper_Arm_Left",
                    uarmw,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(0, 0, 0))

upperalGeometry = osim.Brick(osim.Vec3(0.04, uarml/2, 0.04))
upperal.attachGeometry(upperalGeometry)
upperalGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(upperal)
"""

# Lower Arm left
loweral = osim.Body("Lower_Arm_Left",
                    larmw,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(0, 0, 0))

loweralGeometry = osim.Brick(osim.Vec3(0.04, larml/2, 0.04))
loweral.attachGeometry(loweralGeometry)
loweralGeometry.setColor(osim.Vec3(1, 1, 0))
bbaan.addBody(loweral)

# Upper Arm right
upperar = osim.Body("Upper_Arm_Right",
                    uarmw,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(0, 0, 0))

upperarGeometry = osim.Brick(osim.Vec3(0.04, uarml/2, 0.04))
upperar.attachGeometry(upperarGeometry)
upperarGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(upperar)


# Lower Arm right
lowerar = osim.Body("Lower_Arm_Right",
                    larmw,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(0, 0, 0))

lowerarGeometry = osim.Brick(osim.Vec3(0.04, larml/2, 0.04))
lowerar.attachGeometry(lowerarGeometry)
lowerarGeometry.setColor(osim.Vec3(1, 1, 0))
bbaan.addBody(lowerar)
"""


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
coord.setRangeMax(pi/2)

kneeJoint = osim.PinJoint("knee_Joint",
                          upper,
                          osim.Vec3(0, ulegl/2, 0),
                          osim.Vec3(0, pi, 0),
                          lower,
                          osim.Vec3(0, llegl/2, 0),
                          osim.Vec3(0, 0, 0))
bbaan.addJoint(kneeJoint)

coord = kneeJoint.updCoordinate()
coord.setName('kneeangle')
coord.setRangeMin(0.2)
coord.setRangeMax(pi)

# pointconstraint to close the kinematic loop?
#legtostretcher = osim.PointConstraint(lower, osim.Vec3(0,-llegl/2,0), stretcher, osim.Vec3(0,seatHeight/2,0))
#bbaan.addConstraint(legtostretcher)


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

ubackJoint = osim.WeldJoint("ubackJoint",
                            lowerb,
                            osim.Vec3(0, lbackl/2, 0),
                            osim.Vec3(0, 0, 0.3),
                            upperb,
                            osim.Vec3(0, -ubackl/2, 0),
                            osim.Vec3(0, 0, 0))
bbaan.addJoint(ubackJoint)

shoulderJoint = osim.WeldJoint("shoulderJoint",
                               upperb,
                               osim.Vec3(0, ubackl/2, 0),
                               osim.Vec3(0, 0, 0),
                               shoulder,
                               osim.Vec3(0, -0.04, 0),
                               osim.Vec3(0, 0, 0))
bbaan.addJoint(shoulderJoint)

headJoint = osim.WeldJoint("headJoint",
                           shoulder,
                           osim.Vec3(0, 0.04, 0),
                           osim.Vec3(0, 0, 0),
                           head,
                           osim.Vec3(0, -headl/2, 0),
                           osim.Vec3(0, 0, 0))
bbaan.addJoint(headJoint)


uarmlJoint = osim.BallJoint("upper_arm_left_Joint",
                            shoulder,
                            osim.Vec3(0, 0, shoulderl/2),
                            osim.Vec3(0, 0, pi/2),
                            upperal,
                            osim.Vec3(0, -uarml/2, 0),
                            osim.Vec3(0, 0, 0))
bbaan.addJoint(uarmlJoint)
"""

larmlJoint = osim.BallJoint("lower_arm_left_Joint",
                            upperal,
                            osim.Vec3(0, uarml/2, 0),
                            osim.Vec3(0, 0, 0),
                            loweral,
                            osim.Vec3(0, -larml/2, 0),
                            osim.Vec3(0, 0, 0))
bbaan.addJoint(larmlJoint)

uarmrJoint = osim.BallJoint("upper_arm_right_Joint",
                            shoulder,
                            osim.Vec3(0, 0, -shoulderl/2),
                            osim.Vec3(0, 0, -pi/2),
                            upperar,
                            osim.Vec3(0, uarml/2, 0),
                            osim.Vec3(0, 0, 0))
bbaan.addJoint(uarmrJoint)


larmrJoint = osim.BallJoint("lower_arm_right_Joint",
                            upperar,
                            osim.Vec3(0, -uarml/2, 0),
                            osim.Vec3(0, 0, 0),
                            lowerar,
                            osim.Vec3(0, larml/2, 0),
                            osim.Vec3(0, 0, 0))
bbaan.addJoint(larmrJoint)
"""

"""      The rest        """
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
finalState = osim.simulate(bbaan, state, 10)

# Analyze a simulation.
print(coord.getValue(finalState))
model.realizePosition(finalState)
print(model.calcMassCenterPosition(finalState))
model.realizeAcceleration(finalState)
print(joint.calcReactionOnParentExpressedInGround(finalState))

"""
