import opensim as osim
import math

from bb_common import *

bbaan = osim.Model()
bbaan.setName("BootBaan")
bbaan.setGravity(osim.Vec3(0, -9.90665, 0))


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



"""   The joints               """
####################################################

# seat with stretched leg
legangle = math.asin(seatHeight/(ulegl+llegl))
seatJoint = osim.SliderJoint("seatJoint",
                             bbaan.getGround(),
                             osim.Vec3(ulegl+llegl*math.cos(legangle), boatHeight+seatHeight, 0),
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


"""       The Rower                              """
####################################################

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


# Upper Arm left
upperal = osim.Body("Upper_Arm_Left",
                    uarmw,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(1, 1, 1))

upperalGeometry = osim.Brick(osim.Vec3(0.04, uarml/2, 0.04))
upperal.attachGeometry(upperalGeometry)
upperalGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(upperal)


"""  Rower joints    """
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
coord.setDefaultValue(math.radians(-14.27))
coord = uarmlJoint.upd_coordinates(1)
coord.setName('uarmleft_trn')   # draai naar binnen/buiten
coord.setDefaultValue(math.radians(-16.08))
coord = uarmlJoint.upd_coordinates(2)
coord.setName('uarmleft_up')   # zwenk naar beneden/boven
coord.setDefaultValue(math.radians(45.9))


act = osim.CoordinateActuator('uarmleft_out')
act.setName('ualo_act')
bbaan.addForce(act)
act = osim.CoordinateActuator('uarmleft_trn')
act.setName('ualt_act')
bbaan.addForce(act)
act = osim.CoordinateActuator('uarmleft_up')
act.setName('ualu_act')
bbaan.addForce(act)


"""      Markers                                 """
####################################################

mseat = osim.Marker(markers[1], seat, osim.Vec3(0))
bbaan.addMarker(mseat)

mshoulder = osim.Marker(markers[2], shoulder, osim.Vec3(0, 0, 0))
bbaan.addMarker(mshoulder)

mselbow = osim.Marker(markers[4], upperal, osim.Vec3(0, uarml/2, 0))
bbaan.addMarker(mselbow)



"""      The rest                                """
####################################################

reporter = osim.ConsoleReporter()
reporter.set_report_time_interval(1.0)
bbaan.addComponent(reporter)

bbaan.finalizeConnections()
bbaan.printToXML("BootBaan.osim")

