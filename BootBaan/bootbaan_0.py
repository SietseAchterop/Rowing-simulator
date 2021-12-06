import opensim as osim
import math
pi = math.pi

# how to structure this better?

"""       Parameters of the model                """
####################################################

# Boat parameters
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

# Rower parameters
#   ...
llegl      = 0.53
ulegl      = 0.45
lbackl     = 0.20
ubackl     = 0.40
shouderl   = 0.37
uarml      = 0.35
larml      = 0.35

# weights
boatw      = 14
rowerw     = 80  # divide over the bodies.

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
theBoat.setMass(1)
theBoat.setInertia(osim.Inertia(1, 1, 1, 0, 0, 0))

# For the moment a simple brick.
theBoatGeometry = osim.Brick(osim.Vec3(6, boatHeight/2, boatWidth/2))
theBoat.attachGeometry(theBoatGeometry)
theBoatGeometry.setColor(osim.Vec3(0, 1, 1))

# For the moment friction via 4 spheres at the corners of the brick.
corner_1 = osim.ContactSphere()
corner_1.setRadius(0.1)
corner_1.setLocation(osim.Vec3(6, -boatHeight/2, -boatWidth/2))
corner_1.setFrame(theBoat)
corner_1.setName('corner_1')
bbaan.addContactGeometry(corner_1)

corner_2 = osim.ContactSphere()
corner_2.setRadius(0.1)
corner_2.setLocation(osim.Vec3(-6, -boatHeight/2, -boatWidth/2))
corner_2.setFrame(theBoat)
corner_2.setName('corner_2')
bbaan.addContactGeometry(corner_2)

corner_3 = osim.ContactSphere()
corner_3.setRadius(0.1)
corner_3.setLocation(osim.Vec3(6, -boatHeight/2, boatWidth/2))
corner_3.setFrame(theBoat)
corner_3.setName('corner_3')
bbaan.addContactGeometry(corner_3)

corner_4 = osim.ContactSphere()
corner_4.setRadius(0.1)
corner_4.setLocation(osim.Vec3(-6, -boatHeight/2, boatWidth/2))
corner_4.setFrame(theBoat)
corner_4.setName('corner_4')
bbaan.addContactGeometry(corner_4)
bbaan.addBody(theBoat)

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
                 1.0,
                 osim.Vec3(0, 0, 0),
                 osim.Inertia(0, 0, 0))

bowGeometry = osim.Sphere(0.05)
bow.attachGeometry(bowGeometry)
bowGeometry.setColor(osim.Vec3(0.5,0.5,1))
bbaan.addBody(bow)

"""   Port rigger
 - Zero for height is on top of the seat. 
 - Connection rigger to the boat (in boat coordinates)
        x=0.5, y =0, z= +boatWidth/2
 - Place of lock
        x=0.5, boatHeight/2+seatHeight/2, span/2
 - Length of rigger body
     sqrt(a^2 + b^2)
                               """
####################################################
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
# de plaat aan riem bevestigen daar waar riem het water in gaat 50cm bij scull
# einde riem laat einde blad zien. 
####################################################
pblade = osim.Body("PortBlade",
                   1.0,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(0, 0, 0))

pbladeGeometry = osim.Brick(osim.Vec3(0.02, bladepoint/2, bladepoint/2))
pblade.attachGeometry(pbladeGeometry)
pbladeGeometry.setColor(osim.Vec3(0.5, 0.5, 1))

# For the moment friction via 4 spheres at the corners of the brick.
pblcorner_1 = osim.ContactSphere()
pblcorner_1.setRadius(0.05)
pblcorner_1.setLocation(osim.Vec3(0, bladepoint/2, bladepoint/2))
pblcorner_1.setFrame(pblade)
pblcorner_1.setName('pblcorner_1')
bbaan.addContactGeometry(pblcorner_1)

# For the moment friction via 4 spheres at the corners of the brick.
pblcorner_2 = osim.ContactSphere()
pblcorner_2.setRadius(0.05)
pblcorner_2.setLocation(osim.Vec3(0, bladepoint/2, -bladepoint/2))
pblcorner_2.setFrame(pblade)
pblcorner_2.setName('pblcorner_2')
bbaan.addContactGeometry(pblcorner_2)

# For the moment friction via 4 spheres at the corners of the brick.
pblcorner_3 = osim.ContactSphere()
pblcorner_3.setRadius(0.05)
pblcorner_3.setLocation(osim.Vec3(0, -bladepoint/2, bladepoint/2))
pblcorner_3.setFrame(pblade)
pblcorner_3.setName('pblcorner_3')
bbaan.addContactGeometry(pblcorner_3)

# For the moment friction via 4 spheres at the corners of the brick.
pblcorner_4 = osim.ContactSphere()
pblcorner_4.setRadius(0.05)
pblcorner_4.setLocation(osim.Vec3(0, -bladepoint/2, -bladepoint/2))
pblcorner_4.setFrame(pblade)
pblcorner_4.setName('pblcorner_4')
bbaan.addContactGeometry(pblcorner_4)

bbaan.addBody(pblade)


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
# de plaat aan riem bevestigen daar waar riem het water in gaat 50cm bij scull
# einde riem laat einde blad zien. 
sblade = osim.Body("StarboardBlade",
                   1.0,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(0, 0, 0))

sbladeGeometry = osim.Brick(osim.Vec3(0.02, bladepoint/2, bladepoint/2))
sblade.attachGeometry(sbladeGeometry)
sbladeGeometry.setColor(osim.Vec3(0.5, 0.5, 1))

# For the moment friction via 4 spheres at the corners of the brick.
sblcorner_1 = osim.ContactSphere()
sblcorner_1.setRadius(0.05)
sblcorner_1.setLocation(osim.Vec3(0, bladepoint/2, bladepoint/2))
sblcorner_1.setFrame(sblade)
sblcorner_1.setName('sblcorner_1')
bbaan.addContactGeometry(sblcorner_1)

# For the moment friction via 4 spheres at the corners of the brick.
sblcorner_2 = osim.ContactSphere()
sblcorner_2.setRadius(0.05)
sblcorner_2.setLocation(osim.Vec3(0, bladepoint/2, -bladepoint/2))
sblcorner_2.setFrame(sblade)
sblcorner_2.setName('sblcorner_2')
bbaan.addContactGeometry(sblcorner_2)

# For the moment friction via 4 spheres at the corners of the brick.
sblcorner_3 = osim.ContactSphere()
sblcorner_3.setRadius(0.05)
sblcorner_3.setLocation(osim.Vec3(0, -bladepoint/2, bladepoint/2))
sblcorner_3.setFrame(sblade)
sblcorner_3.setName('sblcorner_3')
bbaan.addContactGeometry(sblcorner_3)

# For the moment friction via 4 spheres at the corners of the brick.
sblcorner_4 = osim.ContactSphere()
sblcorner_4.setRadius(0.05)
sblcorner_4.setLocation(osim.Vec3(0, -bladepoint/2, -bladepoint/2))
sblcorner_4.setFrame(sblade)
sblcorner_4.setName('sblcorner_4')
bbaan.addContactGeometry(sblcorner_4)

bbaan.addBody(sblade)


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

seatact = osim.CoordinateActuator('seatpos')
seatact.setName('seatact')
bbaan.addForce(seatact)

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


# Define Contact Force Parameters for the boat
####################################################
"""  The default values
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

# values for the blades
####################################################
stiffness           = 1000000;
dissipation         = 0.8;
staticFriction      = 0.1;
dynamicFriction     = 0.4;
viscousFriction     = 0.4;
transitionVelocity  = 0.1;

# Make a Hunt Crossley Force
pblcornerforce_1 = osim.HuntCrossleyForce()
pblcornerforce_1.setName('BladeForce_1')
pblcornerforce_1.addGeometry('pblcorner_1')
pblcornerforce_1.addGeometry('baantje')
pblcornerforce_1.setStiffness(stiffness)
pblcornerforce_1.setDissipation(dissipation)
pblcornerforce_1.setStaticFriction(staticFriction)
pblcornerforce_1.setDynamicFriction(dynamicFriction)
pblcornerforce_1.setViscousFriction(viscousFriction)
pblcornerforce_1.setTransitionVelocity(transitionVelocity)
bbaan.addForce(pblcornerforce_1)


# Make a Hunt Crossley Force
pblcornerforce_2 = osim.HuntCrossleyForce()
pblcornerforce_2.setName('BladeForce_2')
pblcornerforce_2.addGeometry('pblcorner_2')
pblcornerforce_2.addGeometry('baantje')
pblcornerforce_2.setStiffness(stiffness)
pblcornerforce_2.setDissipation(dissipation)
pblcornerforce_2.setStaticFriction(staticFriction)
pblcornerforce_2.setDynamicFriction(dynamicFriction)
pblcornerforce_2.setViscousFriction(viscousFriction)
pblcornerforce_2.setTransitionVelocity(transitionVelocity)
bbaan.addForce(pblcornerforce_2)

# Make a Hunt Crossley Force
pblcornerforce_3 = osim.HuntCrossleyForce()
pblcornerforce_3.setName('BladeForce_3')
pblcornerforce_3.addGeometry('pblcorner_3')
pblcornerforce_3.addGeometry('baantje')
pblcornerforce_3.setStiffness(stiffness)
pblcornerforce_3.setDissipation(dissipation)
pblcornerforce_3.setStaticFriction(staticFriction)
pblcornerforce_3.setDynamicFriction(dynamicFriction)
pblcornerforce_3.setViscousFriction(viscousFriction)
pblcornerforce_3.setTransitionVelocity(transitionVelocity)
bbaan.addForce(pblcornerforce_3)


# Make a Hunt Crossley Force
pblcornerforce_4 = osim.HuntCrossleyForce()
pblcornerforce_4.setName('BladeForce_4')
pblcornerforce_4.addGeometry('pblcorner_4')
pblcornerforce_4.addGeometry('baantje')
pblcornerforce_4.setStiffness(stiffness)
pblcornerforce_4.setDissipation(dissipation)
pblcornerforce_4.setStaticFriction(staticFriction)
pblcornerforce_4.setDynamicFriction(dynamicFriction)
pblcornerforce_4.setViscousFriction(viscousFriction)
pblcornerforce_4.setTransitionVelocity(transitionVelocity)
bbaan.addForce(pblcornerforce_4)


# Make a Hunt Crossley Force
sblcornerforce_1 = osim.HuntCrossleyForce()
sblcornerforce_1.setName('SBladeForce_1')
sblcornerforce_1.addGeometry('sblcorner_1')
sblcornerforce_1.addGeometry('baantje')
sblcornerforce_1.setStiffness(stiffness)
sblcornerforce_1.setDissipation(dissipation)
sblcornerforce_1.setStaticFriction(staticFriction)
sblcornerforce_1.setDynamicFriction(dynamicFriction)
sblcornerforce_1.setViscousFriction(viscousFriction)
sblcornerforce_1.setTransitionVelocity(transitionVelocity)
bbaan.addForce(sblcornerforce_1)


# Make a Hunt Crossley Force
sblcornerforce_2 = osim.HuntCrossleyForce()
sblcornerforce_2.setName('SBladeForce_2')
sblcornerforce_2.addGeometry('sblcorner_2')
sblcornerforce_2.addGeometry('baantje')
sblcornerforce_2.setStiffness(stiffness)
sblcornerforce_2.setDissipation(dissipation)
sblcornerforce_2.setStaticFriction(staticFriction)
sblcornerforce_2.setDynamicFriction(dynamicFriction)
sblcornerforce_2.setViscousFriction(viscousFriction)
sblcornerforce_2.setTransitionVelocity(transitionVelocity)
bbaan.addForce(sblcornerforce_2)

# Make a Hunt Crossley Force
sblcornerforce_3 = osim.HuntCrossleyForce()
sblcornerforce_3.setName('SBladeForce_3')
sblcornerforce_3.addGeometry('sblcorner_3')
sblcornerforce_3.addGeometry('baantje')
sblcornerforce_3.setStiffness(stiffness)
sblcornerforce_3.setDissipation(dissipation)
sblcornerforce_3.setStaticFriction(staticFriction)
sblcornerforce_3.setDynamicFriction(dynamicFriction)
sblcornerforce_3.setViscousFriction(viscousFriction)
sblcornerforce_3.setTransitionVelocity(transitionVelocity)
bbaan.addForce(sblcornerforce_3)


# Make a Hunt Crossley Force
sblcornerforce_4 = osim.HuntCrossleyForce()
sblcornerforce_4.setName('SBladeForce_4')
sblcornerforce_4.addGeometry('sblcorner_4')
sblcornerforce_4.addGeometry('baantje')
sblcornerforce_4.setStiffness(stiffness)
sblcornerforce_4.setDissipation(dissipation)
sblcornerforce_4.setStaticFriction(staticFriction)
sblcornerforce_4.setDynamicFriction(dynamicFriction)
sblcornerforce_4.setViscousFriction(viscousFriction)
sblcornerforce_4.setTransitionVelocity(transitionVelocity)
bbaan.addForce(sblcornerforce_4)


"""       The Rower         """
####################################################

# Lower leg
lower = osim.Body("Lower_leg",
                     1.0,
                     osim.Vec3(0, 0, 0),
                     osim.Inertia(0, 0, 0))

lowerGeometry = osim.Cylinder(0.05, llegl/2)
lower.attachGeometry(lowerGeometry)
lowerGeometry.setColor(osim.Vec3(1,0,1))
bbaan.addBody(lower)

corner = osim.ContactSphere()
corner.setRadius(0.02)
corner.setLocation(osim.Vec3(-0.2, llegl/2, 0))
corner.setFrame(lower)
corner.setName('lower_leg')
bbaan.addContactGeometry(corner)

# Upper leg
upper = osim.Body("The_upper_leg",
                     1.0,
                     osim.Vec3(0, 0, 0),
                     osim.Inertia(0, 0, 0))

upperGeometry = osim.Cylinder(0.05, ulegl/2)
upper.attachGeometry(upperGeometry)
upperGeometry.setColor(osim.Vec3(0.5,0.5,1))
bbaan.addBody(upper)

#  de rest ....

"""  Rower joints    """

hip1Joint = osim.PinJoint("hip1Joint",
                        seat,
                        osim.Vec3(0, seatHeight/2, 0),
                        osim.Vec3(0, 0, 0),
                        upper,
                        osim.Vec3(0, -ulegl/2, 0),
                        osim.Vec3(0, 0, 0))
bbaan.addJoint(hip1Joint)
coord = hip1Joint.updCoordinate()
coord.setName('hip1angle')
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
legtostretcher = osim.PointConstraint(lower, osim.Vec3(0,-llegl/2,0), stretcher, osim.Vec3(0,seatHeight/2,0))
bbaan.addConstraint(legtostretcher)

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
