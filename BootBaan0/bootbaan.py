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
"""
bj_4 = osim.ArrayStr()
bj_4.append("bJoint_4")
boattf.updTransformAxis(4).setCoordinateNames(bj_4)
boattf.updTransformAxis(4).set_function(osim.LinearFunction())
"""
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
act.set_optimal_force(10000)
bbaan.addForce(act)

act = osim.CoordinateActuator('bJoint_4')
act.setName('bJ_act_4')
act.set_optimal_force(10000)
bbaan.addForce(act)
"""

# close the loop between boat and stretcher
boatconstraint = osim.WeldConstraint("boatconstraint", theBoat, osim.Transform(osim.Vec3(0, boatHeight/2, 0)), stretcher, osim.Transform(osim.Vec3(0, 0, 0)))
bbaan.addConstraint(boatconstraint)


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
act.set_optimal_force(10000)
bbaan.addForce(act)

bowJoint = osim.WeldJoint("bowJoint",
                          theBoat,
                          osim.Vec3(boatLength/2, boatHeight/2, 0),
                          osim.Vec3(0, 0, 0),
                          bow,
                          osim.Vec3(0, 0, 0),
                          osim.Vec3(0, 0, 0))
bbaan.addJoint(bowJoint)


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
#   snap ik niet: optimal_force verhogen helpt niet...

# Upper Arm left
upperal = osim.Body("Upper_Arm_Left",
                    uarmw,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(0.1, 0.1, 0.1))

upperalGeometry = osim.Brick(osim.Vec3(0.04, uarml/2, 0.04))
upperal.attachGeometry(upperalGeometry)
upperalGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(upperal)


# Upper Arm right
upperar = osim.Body("Upper_Arm_Right",
                    uarmw,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(0.1, 0.1, 0.1))

upperarGeometry = osim.Brick(osim.Vec3(0.04, uarml/2, 0.04))
upperar.attachGeometry(upperarGeometry)
upperarGeometry.setColor(osim.Vec3(1, 1, 1))
bbaan.addBody(upperar)


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
act.set_optimal_force(10000)
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
act.set_optimal_force(10000)
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
coord.setRangeMin(0.0)
coord.setRangeMax(pi/2)
coord.setDefaultValue(math.radians(5))
coord.set_clamped(True)

act = osim.CoordinateActuator('footangle')
act.setName('footact')
act.set_optimal_force(10000)
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
act.set_optimal_force(10000)
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
arr.append('uarmleft_out')
sp.updTransformAxis(0).setCoordinateNames(arr)
sp.updTransformAxis(0).set_function(osim.LinearFunction())
sp.updTransformAxis(0).set_axis(osim.Vec3(0,0,1))
arr = osim.ArrayStr()
arr.append('uarmleft_trn')
sp.updTransformAxis(1).setCoordinateNames(arr)
sp.updTransformAxis(1).set_function(osim.LinearFunction())
sp.updTransformAxis(1).set_axis(osim.Vec3(-1,0,0))
arr = osim.ArrayStr()
arr.append('uarmleft_up')
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

act = osim.CoordinateActuator('uarmleft_out')
act.setName('ualo_act')
act.set_optimal_force(10000)
bbaan.addForce(act)
act = osim.CoordinateActuator('uarmleft_trn')
act.setName('ualt_act')
act.set_optimal_force(10000)
bbaan.addForce(act)
act = osim.CoordinateActuator('uarmleft_up')
act.setName('ualu_act')
act.set_optimal_force(10000)
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
arr.append('uarmright_out')
sp.updTransformAxis(0).setCoordinateNames(arr)
sp.updTransformAxis(0).set_function(osim.LinearFunction())
sp.updTransformAxis(0).set_axis(osim.Vec3(0,0,1))
arr = osim.ArrayStr()
arr.append('uarmright_trn')
sp.updTransformAxis(1).setCoordinateNames(arr)
sp.updTransformAxis(1).set_function(osim.LinearFunction())
sp.updTransformAxis(1).set_axis(osim.Vec3(-1,0,0))
arr = osim.ArrayStr()
arr.append('uarmright_up')
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

act = osim.CoordinateActuator('uarmright_out')
act.setName('uaro_act')
act.set_optimal_force(10000)
bbaan.addForce(act)
act = osim.CoordinateActuator('uarmright_trn')
act.setName('uart_act')
act.set_optimal_force(10000)
bbaan.addForce(act)
act = osim.CoordinateActuator('uarmright_up')
act.setName('uaru_act')
act.set_optimal_force(10000)
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
act.setName('uarmleft_up_Damping')
act.set_coordinate('uarmleft_up')
act.set_stiffness(1)
act.set_viscosity(1)
bbaan.addForce(act)
act = osim.SpringGeneralizedForce()
act.setName('uarmright_out_Damping')
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
act = osim.SpringGeneralizedForce()
act.setName('uarmright_up_Damping')
act.set_coordinate('uarmright_up')
act.set_stiffness(1)
act.set_viscosity(1)
bbaan.addForce(act)
"""

"""  Contact Geometry   en ElasticFoundationForce()
       maakt niet uit

"""

"""      Markers                                 """
####################################################

mboat = osim.Marker(markers[0], theBoat, osim.Vec3(0, 0, 0))
bbaan.addMarker(mboat)

mseat = osim.Marker(markers[1], seat, osim.Vec3(0))
bbaan.addMarker(mseat)

mshoulder = osim.Marker(markers[2], shoulder, osim.Vec3(0, 0, 0))
bbaan.addMarker(mshoulder)

mpelbow = osim.Marker(markers[3], upperar, osim.Vec3(0, -uarml/2, 0))
bbaan.addMarker(mpelbow)

mselbow = osim.Marker(markers[4], upperal, osim.Vec3(0, uarml/2, 0))
bbaan.addMarker(mselbow)



"""      The rest                                """
####################################################

reporter = osim.ConsoleReporter()
reporter.set_report_time_interval(1.0)
bbaan.addComponent(reporter)

bbaan.finalizeConnections()
bbaan.printToXML("BootBaan.osim")

