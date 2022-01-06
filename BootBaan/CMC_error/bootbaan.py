import opensim as osim
import math

from bb_common import *

bbaan = osim.Model()
bbaan.setName("BootBaan")
bbaan.setGravity(osim.Vec3(0, -9.90665, 0))


"""   The boat   """
theBoat = osim.Body()
theBoat.setName('TheBoat')
theBoat.setMass(boatw)
theBoat.setInertia(osim.Inertia(1, 1, 1))
theBoatGeometry = osim.Brick(osim.Vec3(boatLength/2, boatHeight/2, boatWidth/2))
theBoat.attachGeometry(theBoatGeometry)
theBoatGeometry.setColor(osim.Vec3(0, 1, 1))
bbaan.addBody(theBoat)

"""   Foot stretcher           """
stretcher = osim.Body("Stretcher",
                      1.0,
                      osim.Vec3(0, 0, 0),
                      osim.Inertia(1, 1, 1))
stretcherGeometry = osim.Brick(osim.Vec3(seatHeight/2, seatHeight/2, boatWidth/2))
stretcher.attachGeometry(stretcherGeometry)
stretcherGeometry.setColor(osim.Vec3(0.5,0.5,1))
bbaan.addBody(stretcher)

"""   Seat                     """
seat = osim.Body("Seat",
                 1.0,
                 osim.Vec3(0, 0, 0),
                 osim.Inertia(1, 1, 1))
seatGeometry = osim.Brick(osim.Vec3(seatHeight/2, seatHeight/2, boatWidth/2))
seat.attachGeometry(seatGeometry)
seatGeometry.setColor(osim.Vec3(0.5,0.5,1))
bbaan.addBody(seat)

"""   The joints               """
boattf = osim.SpatialTransform()
bj_1 = osim.ArrayStr()
bj_1.append("bJoint_1")
boattf.updTransformAxis(1).setCoordinateNames(bj_1)
boattf.updTransformAxis(1).set_function(osim.LinearFunction())
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

act = osim.CoordinateActuator('bJoint_1')
act.setName('bJ_act_1')
bbaan.addForce(act)
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

act = osim.CoordinateActuator('seatpos')
act.setName('seatact')
bbaan.addForce(act)


"""       The Rower                              """
# Lower leg (split in 2 to use a WeldContraint)
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
coord.set_clamped(True)
osim.CoordinateLimitForce('hipangle',
                           pi/2+0.1,
                           10,
                           -0.3,
                           10,
                           0.01,
                           2.0)


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
coord.set_clamped(True)
osim.CoordinateLimitForce('kneeangle',
                           pi,
                           10,
                           0,
                           10,
                           0.01,
                           2.0)


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
coord.set_clamped(True)
# clamped works in IK, for use in Forward Dynamics use CoordinateLimitForce
osim.CoordinateLimitForce('footangle',
                           pi/2,
                           10,
                           0,
                           10,
                           0.01,
                           2.0)

act = osim.CoordinateActuator('footangle')
act.setName('footact')
bbaan.addForce(act)


# close the loop in the lower leg
legconstraint = osim.WeldConstraint("legconstraint", lower_l, osim.Transform(osim.Vec3(0, -llegl/4, 0)), lower_u, osim.Transform(osim.Vec3(0, llegl/4, 0)))
bbaan.addConstraint(legconstraint)


"""      Markers                                 """
mseat = osim.Marker(markers[0], seat, osim.Vec3(0, 0, 0))
bbaan.addMarker(mseat)


"""      The rest                                """
reporter = osim.ConsoleReporter()
reporter.set_report_time_interval(1.0)
bbaan.addComponent(reporter)

bbaan.finalizeConnections()
bbaan.printToXML("BootBaan.osim")

