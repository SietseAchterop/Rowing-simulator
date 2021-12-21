import opensim as osim
import math

from bb_common import *

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

def addContactForceGnd(cs, item):
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

def addContactForce(cs, item, wrt):
    """ Make a Hunt Crossley Force wrt to the ContactHalfspace 'baantje'
        Set parameters prior to calling!   
    """
    hf = osim.HuntCrossleyForce()
    hf.setName(item)
    hf.addGeometry(item)
    hf.addGeometry(wrt)
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


theCourse = osim.Body("Rowing_course",
                      1.0,
                      osim.Vec3(0, 0, 0),
                      osim.Inertia(1, 1, 1))

theCourseGeometry = osim.Brick(osim.Vec3(6, 0.05, 5))
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
I_xx = 1/12*boatw*(math.pow(boatHeight, 2)+math.pow(boatLength/2, 2))
I_yy = I_xx
I_zz = 1/2*boatw*math.pow(boatLength/2, 2)
theBoat.setInertia(osim.Inertia(I_xx, I_yy, I_zz))

theBoatGeometry = osim.Brick(osim.Vec3(boatLength, boatHeight/2, boatWidth/2))
theBoat.attachGeometry(theBoatGeometry)
theBoatGeometry.setColor(osim.Vec3(0, 1, 1))
bbaan.addBody(theBoat)


name = 'boatcorner_1'
cs = osim.ContactSphere()
cs.setRadius(0.1)
cs.setLocation(osim.Vec3(6, -boatHeight/2, -boatWidth/2))
cs.setFrame(theBoat)
addContactForceGnd(cs, name)

name = 'boatcorner_2'
cs = osim.ContactSphere()
cs.setRadius(0.1)
cs.setLocation(osim.Vec3(-6, -boatHeight/2, -boatWidth/2))
cs.setFrame(theBoat)
addContactForceGnd(cs, name)

name = 'boatcorner_3'
cs = osim.ContactSphere()
cs.setRadius(0.1)
cs.setLocation(osim.Vec3(6, -boatHeight/2, boatWidth/2))
cs.setFrame(theBoat)
addContactForceGnd(cs, name)

name = 'boatcorner_4'
cs = osim.ContactSphere()
cs.setRadius(0.1)
cs.setLocation(osim.Vec3(-6, -boatHeight/2, boatWidth/2))
cs.setFrame(theBoat)
addContactForceGnd(cs, name)

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

name = 'handlesb'
cs = osim.ContactSphere()
cs.setRadius(0.02)
cs.setLocation(osim.Vec3(0.0, (inboard+outboard)/2, 0))
cs.setFrame(soar)
cs.setName('handlesb')
addContactForce(cs, name, 'handlep')


"""   Starboard blade               """
####################################################
sblade = osim.Body("StarboardBlade",
                   1.0,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(1, 1, 1))

sbladeGeometry = osim.Brick(osim.Vec3(0.02, bladepoint/2, bladepoint/2))
sblade.attachGeometry(sbladeGeometry)
sbladeGeometry.setColor(osim.Vec3(0.5, 0.5, 1))
bbaan.addBody(sblade)

""" Friction values for the blades  """
stiffness           = 1000000;
dissipation         = 0.8;
staticFriction      = 0.1;
dynamicFriction     = 0.4;
viscousFriction     = 0.4;
transitionVelocity  = 0.1;

name = 'sblcorner_1'
cs = osim.ContactSphere()
cs.setRadius(0.05)
cs.setLocation(osim.Vec3(0, bladepoint/2, bladepoint/2))
cs.setFrame(sblade)
addContactForceGnd(cs, name)

name = 'sblcorner_2'
cs = osim.ContactSphere()
cs.setRadius(0.05)
cs.setLocation(osim.Vec3(0, bladepoint/2, -bladepoint/2))
cs.setFrame(sblade)
addContactForceGnd(cs, name)

name = 'sblcorner_3'
cs = osim.ContactSphere()
cs.setRadius(0.05)
cs.setLocation(osim.Vec3(0, -bladepoint/2, bladepoint/2))
cs.setFrame(sblade)
addContactForceGnd(cs, name)

name = 'sblcorner_4'
cs = osim.ContactSphere()
cs.setRadius(0.05)
cs.setLocation(osim.Vec3(0, -bladepoint/2, -bladepoint/2))
cs.setFrame(sblade)
addContactForceGnd(cs, name)


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

# helpt dit om de assemblysolver fouten te vermijden die lijken te komen als contraintpoint overlappen. Is dat zo?
#   en dit moet een mesh worden van de het deel van de riem dat zou kunnen overlappen.
#   evt de knie er ook bij betrekken.
name = 'handlep'
cs = osim.ContactSphere()
cs.setRadius(0.02)
cs.setLocation(osim.Vec3(0.0, (inboard+outboard)/2, 0))
cs.setFrame(poar)
cs.setName('handlep')
addContactForce(cs, name, 'handlesb')

"""   Port blade          """
####################################################
pblade = osim.Body("PortBlade",
                   1.0,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(1, 1, 1))

pbladeGeometry = osim.Brick(osim.Vec3(0.02, bladepoint/2, bladepoint/2))
pblade.attachGeometry(pbladeGeometry)
pbladeGeometry.setColor(osim.Vec3(0.5, 0.5, 1))
bbaan.addBody(pblade)

name = 'pblcorner_1'
cs = osim.ContactSphere()
cs.setRadius(0.05)
cs.setLocation(osim.Vec3(0, bladepoint/2, bladepoint/2))
cs.setFrame(pblade)
addContactForceGnd(cs, name)

name = 'pblcorner_2'
cs = osim.ContactSphere()
cs.setRadius(0.05)
cs.setLocation(osim.Vec3(0, bladepoint/2, -bladepoint/2))
cs.setFrame(pblade)
addContactForceGnd(cs, name)

name = 'pblcorner_3'
cs = osim.ContactSphere()
cs.setRadius(0.05)
cs.setLocation(osim.Vec3(0, -bladepoint/2, bladepoint/2))
cs.setFrame(pblade)
addContactForceGnd(cs, name)

name = 'pblcorner_4'
cs = osim.ContactSphere()
cs.setRadius(0.05)
cs.setLocation(osim.Vec3(0, -bladepoint/2, -bladepoint/2))
cs.setFrame(pblade)
addContactForceGnd(cs, name)

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

# later make it a custom joint with only 2 degrees of freedom. forward and up
boatJoint = osim.FreeJoint("boatJoint",
                           bbaan.getGround(),
                           osim.Vec3(0, boatHeight/2, 0),
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

# x te groot voor goede aansluiting met de stretcher. Coordinate zal dus nooit de waarde 0 halen.
#  Wordt via de constraint bij stretcher opgelost.
seatJoint = osim.SliderJoint("seatJoint",
                             theBoat,
                             osim.Vec3(ulegl+llegl, boatHeight/2+seatHeight, 0),
                             osim.Vec3(0, 0, 0),
                             seat,
                             osim.Vec3(0, 0, 0),
                             osim.Vec3(0, 0, 0))
bbaan.addJoint(seatJoint)

seatcoord = seatJoint.updCoordinate()
seatcoord.setName('seatpos')
seatcoord.setRangeMin(-(ulegl+llegl))
seatcoord.setRangeMax(0.0)
seatcoord.setDefaultValue(-0.2)
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
pblcoord = slocJoint.updCoordinate()
pblcoord.setName('slocangle')
pblcoord.setRangeMin(-pi/2)
pblcoord.setRangeMax(pi/2)


soarJoint = osim.PinJoint("starboardoarJoint",
                          slock,
                          osim.Vec3(0, 0, 0),
                          osim.Vec3(pi/2, pi/2, 0),
                          soar,
                          osim.Vec3(0, (outboard-inboard)/2,  0),
                          osim.Vec3(0, 0, 0))
bbaan.addJoint(soarJoint)
pblcoord = soarJoint.updCoordinate()
pblcoord.setName('soarinout')
pblcoord.setRangeMin(0.0)
pblcoord.setRangeMax(0.4)

sbladeJoint = osim.PinJoint("starboardbladeJoint",
                            soar,
                            osim.Vec3(0, -(inboard+outboard)/2+bladepoint,  0),
                            osim.Vec3(0, 0, 0),
                            sblade,
                            osim.Vec3(0, bladepoint/2,  0),
                            osim.Vec3(0, 0, 0))                            
bbaan.addJoint(sbladeJoint)
pblcoord = sbladeJoint.updCoordinate()
pblcoord.setName('sblpos')
pblcoord.setRangeMin(-0.3)
pblcoord.setRangeMax(0.0)



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
pblcoord = plocJoint.updCoordinate()
pblcoord.setName('plocangle')
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

poarJoint = osim.PinJoint("portoarJoint",
                          plock,
                          osim.Vec3(0, 0, 0),
                          osim.Vec3(pi/2, -pi/2, pi),
                          poar,
                          osim.Vec3(0, (outboard-inboard)/2,  0),
                          osim.Vec3(0, 0, 0))

bbaan.addJoint(poarJoint)
pblcoord = poarJoint.updCoordinate()
pblcoord.setName('poarinout')
pblcoord.setRangeMin(0.0)
pblcoord.setRangeMax(0.4)


pbladeJoint = osim.PinJoint("portbladeJoint",
                            poar,
                            osim.Vec3(0, -(inboard+outboard)/2+bladepoint,  0),
                            osim.Vec3(0, 0, pi),
                            pblade,
                            osim.Vec3(0, -bladepoint/2,  0),
                            osim.Vec3(0, 0, 0))                            
bbaan.addJoint(pbladeJoint)
sblcoord = pbladeJoint.updCoordinate()
sblcoord.setName('pblpos')
sblcoord.setRangeMin(-0.3)
sblcoord.setRangeMax(0.0)


"""       The Rower                              """
####################################################

# Lower leg
lower = osim.Body("Lower_leg",
                  llegw,
                  osim.Vec3(0, 0, 0),
                  osim.Inertia(1, 1, 1))

lowerGeometry = osim.Cylinder(0.05, llegl/2)
lower.attachGeometry(lowerGeometry)
lowerGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(lower)

name = 'knee'
cs = osim.ContactSphere()
cs.setRadius(0.02)
cs.setLocation(osim.Vec3(-0.3, llegl/2, 0))
cs.setFrame(lower)
addContactForceGnd(cs, name)

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

# waarom werkt clamping niet bij gewone simulatie?
name = 'back'
cs = osim.ContactSphere()
cs.setRadius(0.02)
cs.setLocation(osim.Vec3(0.4, llegl/2, 0))
cs.setFrame(upperb)
addContactForceGnd(cs, name)

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


# Upper Arm left
upperal = osim.Body("Upper_Arm_Left",
                    uarmw,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(1, 1, 1))

upperalGeometry = osim.Brick(osim.Vec3(0.04, uarml/2, 0.04))
upperal.attachGeometry(upperalGeometry)
upperalGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(upperal)

# Lower Arm left
loweral = osim.Body("Lower_Arm_Left",
                    larmw,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(1, 1, 1))

loweralGeometry = osim.Brick(osim.Vec3(0.04, larml/2, 0.04))
loweral.attachGeometry(loweralGeometry)
loweralGeometry.setColor(osim.Vec3(1, 1, 1))
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

lowerarGeometry = osim.Brick(osim.Vec3(0.04, larml/2, 0.04))
lowerar.attachGeometry(lowerarGeometry)
lowerarGeometry.setColor(osim.Vec3(1, 1, 1))
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
coord.setRangeMax(pi/2+0.1)
coord.setDefaultValue(0.8)

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
coord.setDefaultValue(3.0)

# AssemblySolver: wat is a good value?
bbaan.set_assembly_accuracy(1e-07)

# close the loop between lower leg and stretcher
legtostretcher = osim.PointConstraint(lower, osim.Vec3(0, -llegl/2, 0), stretcher, osim.Vec3(0, seatHeight/2, 0))
legtostretcher.setName("leg2stretcher")
bbaan.addConstraint(legtostretcher)

# close the loop via port oar
parmtooar = osim.PointConstraint(lowerar, osim.Vec3(0, -larml/2, 0), poar, osim.Vec3(0, (inboard+outboard)/2, 0))
parmtooar.setName("parm2oar")
bbaan.addConstraint(parmtooar)


# close the loop via starboard oar
sarmtooar = osim.PointConstraint(loweral, osim.Vec3(0, larml/2, 0), soar, osim.Vec3(0, (inboard+outboard)/2, 0))
sarmtooar.setName("sarm2oar")
bbaan.addConstraint(sarmtooar)


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
coord.setDefaultValue(-1.0)

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


uarmlJoint = osim.BallJoint("upper_arm_left_Joint",
                            shoulder,
                            osim.Vec3(0, 0, shoulderl/2),
                            osim.Vec3(0, 0, pi/2),
                            upperal,
                            osim.Vec3(0, -uarml/2, 0),
                            osim.Vec3(0, 0, 0))
bbaan.addJoint(uarmlJoint)
coord = uarmlJoint.upd_coordinates(0)
coord.setName('uarmleft_out')   # zwenk naar buiten/binnen
#coord.setDefaultValue(0.8)
coord = uarmlJoint.upd_coordinates(1)
coord.setName('uarmleft_trn')   # draai naar binnen/buiten
#coord.setDefaultValue(0.8)
coord = uarmlJoint.upd_coordinates(2)
coord.setName('uarmleft_up')   # zwenk naar beneden/boven
coord.setDefaultValue(0.8)


larmlJoint = osim.BallJoint("lower_arm_left_Joint",
                            upperal,
                            osim.Vec3(0, uarml/2, 0),
                            osim.Vec3(0, 0, 0),
                            loweral,
                            osim.Vec3(0, -larml/2, 0),
                            osim.Vec3(0, 0, 0))
bbaan.addJoint(larmlJoint)
coord = larmlJoint.upd_coordinates(0)
coord.setName('larmleft_out')
#coord.setDefaultValue(0.8)
coord = larmlJoint.upd_coordinates(1)
coord.setName('larmleft_trn')
#coord.setDefaultValue(0.8)
coord = larmlJoint.upd_coordinates(2)
coord.setName('larmleft_up')
#coord.setDefaultValue(0.8)


uarmrJoint = osim.BallJoint("upper_arm_right_Joint",
                            shoulder,
                            osim.Vec3(0, 0, -shoulderl/2),
                            osim.Vec3(0, 0, -pi/2),
                            upperar,
                            osim.Vec3(0, uarml/2, 0),
                            osim.Vec3(0, 0, 0))
bbaan.addJoint(uarmrJoint)
coord = uarmrJoint.upd_coordinates(0)
coord.setName('uarmright_out')
#coord.setDefaultValue(0.8)
coord = uarmrJoint.upd_coordinates(1)
coord.setName('uarmright_trn')
#coord.setDefaultValue(0.8)
coord = uarmrJoint.upd_coordinates(2)
coord.setName('uarmright_up')
coord.setDefaultValue(0.8)


larmrJoint = osim.BallJoint("lower_arm_right_Joint",
                            upperar,
                            osim.Vec3(0, -uarml/2, 0),
                            osim.Vec3(0, 0, 0),
                            lowerar,
                            osim.Vec3(0, larml/2, 0),
                            osim.Vec3(0, 0, 0))
bbaan.addJoint(larmrJoint)
coord = larmrJoint.upd_coordinates(0)
coord.setName('larmright_out')
#coord.setDefaultValue(0.8)
coord = larmrJoint.upd_coordinates(1)
coord.setName('larmright_trn')
#coord.setDefaultValue(0.8)
coord = larmrJoint.upd_coordinates(2)
coord.setName('larmright_up')
#coord.setDefaultValue(0.8)


"""      Markers                                 """
####################################################

mboat = osim.Marker(markers[0], theBoat, osim.Vec3(0, 0, 0))
bbaan.addMarker(mboat)

mseat = osim.Marker(markers[1], seat, osim.Vec3(0, 0, 0))
bbaan.addMarker(mseat)

mshoulder = osim.Marker(markers[2], shoulder, osim.Vec3(0, 0, 0))
bbaan.addMarker(mshoulder)

mpelbow = osim.Marker(markers[3], upperar, osim.Vec3(0, -uarml/2, 0))
bbaan.addMarker(mpelbow)

mselbow = osim.Marker(markers[4], upperal, osim.Vec3(0, uarml/2, 0))
bbaan.addMarker(mselbow)

mphandle = osim.Marker(markers[5], poar, osim.Vec3(0, (outboard+inboard)/2, 0))
bbaan.addMarker(mphandle)

mshandle = osim.Marker(markers[6], soar, osim.Vec3(0, (outboard+inboard)/2, 0))
bbaan.addMarker(mshandle)

mpblade = osim.Marker(markers[7], pblade, osim.Vec3(0, bladepoint/2, 0))
bbaan.addMarker(mpblade)

msblade = osim.Marker(markers[8], sblade, osim.Vec3(0, -bladepoint/2, 0))
bbaan.addMarker(msblade)


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

