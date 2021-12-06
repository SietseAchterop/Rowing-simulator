# ----------------------------------------------------------------------- #
# The OpenSim API is a toolkit for musculoskeletal modeling and           #
# simulation. See http://opensim.stanford.edu and the NOTICE file         #
# for more information. OpenSim is developed at Stanford University       #
# and supported by the US National Institutes of Health (U54 GM072970,    #
# R24 HD065690) and by DARPA through the Warrior Web program.             #
#                                                                         #
# Copyright (c) 2005-2017 Stanford University and the Authors             #
# Author(s): Neil Dhir                                                    #
# Contributor(s): Christopher Dembia                                      #
#                                                                         #
# Licensed under the Apache License, Version 2.0 (the "License");         #
# you may not use this file except in compliance with the License.        #
# You may obtain a copy of the License at                                 #
# http://www.apache.org/licenses/LICENSE-2.0.                             #
#                                                                         #
# Unless required by applicable law or agreed to in writing, software     #
# distributed under the License is distributed on an "AS IS" BASIS,       #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         #
# implied. See the License for the specific language governing            #
# permissions and limitations under the License.                          #
# ----------------------------------------------------------------------- #

# build_simple_arm_model.py
# Author: Neil Dhir
# ------------------------------------------------------------------------#
# ABSTRACT: This short piece of OpenSim python API example demonstrates a #
# simple arm which consists of two bodies, two joints, a muscle and a     #
# controller. All model elements are labeled with their appropriate       #
# biomechanical namesakes for easy identification and clarity of          #
# demonstration.                                                          #
# ------------------------------------------------------------------------#

import opensim as osim

import sys
# Are we running this script as a test? Users can ignore this line!
running_as_test = 'unittest' in str().join(sys.argv)

# Define global model where the arm lives.
arm = osim.Model()
if not running_as_test: arm.setUseVisualizer(True)

arm.setName("RowTheBoat:")
arm.setGravity(osim.Vec3(0, -9.90665, 0))

deBaan = osim.Body("De_baan",
                     1.0,
                     osim.Vec3(0, 0, 0),
                     osim.Inertia(0, 0, 0))
                     
deBaanGeometry = osim.Brick(osim.Vec3(20, 0.01, 2))
deBaan.attachGeometry(deBaanGeometry)
deBaanGeometry.setColor(osim.Vec3(0,0,1))

deBaanContactSpace = osim.ContactHalfSpace(osim.Vec3(0, 0.2, 0),
                                           osim.Vec3(0, 0, -1.57),
                                           deBaan)
deBaanContactSpace.setName("baantje")
arm.addContactGeometry(deBaanContactSpace)



deBoot = osim.Body("De_boot",
                     1.0,
                     osim.Vec3(0, 0, 0),
                     osim.Inertia(0, 0, 0))
                     
deBootGeometry = osim.Brick(osim.Vec3(7, 0.05, 1))
deBoot.attachGeometry(deBootGeometry)
deBootGeometry.setColor(osim.Vec3(0,1,1))

deBootContactSpace = osim.ContactHalfSpace(osim.Vec3(0, 0.2, 0),
                                           osim.Vec3(0, 0, -1.57),
                                           deBoot)
deBootContactSpace.setName("bootje")
arm.addContactGeometry(deBootContactSpace)


# Define Contact Force Parameters
stiffness           = 1000000;
dissipation         = 2.0;
staticFriction      = 0.8;
dynamicFriction     = 0.4;
viscousFriction     = 0.4;
transitionVelocity  = 0.2;

# Make a Hunt Crossley Force for Boat and update parameters
HuntCrossleyBoat = osim.HuntCrossleyForce();
HuntCrossleyBoat.setName('bootjeForce');
HuntCrossleyBoat.addGeometry('bootje');
HuntCrossleyBoat.addGeometry('baantje');
HuntCrossleyBoat.setStiffness(stiffness);
HuntCrossleyBoat.setDissipation(dissipation);
HuntCrossleyBoat.setStaticFriction(staticFriction);
HuntCrossleyBoat.setDynamicFriction(dynamicFriction);
HuntCrossleyBoat.setViscousFriction(viscousFriction);
HuntCrossleyBoat.setTransitionVelocity(transitionVelocity);
arm.addForce(HuntCrossleyBoat);


                     
# ---------------------------------------------------------------------------
# Create two links, each with a mass of 1 kg, centre of mass at the body's
# origin, and moments and products of inertia of zero.
# ---------------------------------------------------------------------------

humerus = osim.Body("humerus",
                    1.0,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(0,  0, 0))
radius = osim.Body("radius",
                   1.0,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(0, 0, 0))

# ---------------------------------------------------------------------------
# Connect the bodies with pin joints. Assume each body is 1m long.
# ---------------------------------------------------------------------------

baanJoint = osim.WeldJoint("baanJoint",
                        arm.getGround(),
                        osim.Vec3(0, 0.005, 0),
                        osim.Vec3(0, 0, 0),
                        deBaan,
                        osim.Vec3(0, 0, 0),
                        osim.Vec3(0, 0, 0))
                        
bootJoint = osim.PinJoint("bootJoint",
                        deBaan,
                        osim.Vec3(0, 0.5, 0),
                        osim.Vec3(0, 0, 0),
                        deBoot,
                        osim.Vec3(0, 0, 0),
                        osim.Vec3(0, 0, 0))
                        
shoulder = osim.PinJoint("shoulder",
                         deBoot, # PhysicalFrame
                         osim.Vec3(0, 3, 0),
                         osim.Vec3(0, 0, 0),
                         humerus, # PhysicalFrame
                         osim.Vec3(0, 1, 0),
                         osim.Vec3(0, 0, 0))

elbow = osim.PinJoint("elbow",
                      humerus, # PhysicalFrame
                      osim.Vec3(0, 0, 0),
                      osim.Vec3(0, 0, 0),
                      radius, # PhysicalFrame
                      osim.Vec3(0, 1, 0),
                      osim.Vec3(0, 0, 0))

# ---------------------------------------------------------------------------
# Add a muscle that flexes the elbow (actuator for robotics people).
# ---------------------------------------------------------------------------

biceps = osim.Millard2012EquilibriumMuscle("biceps",  # Muscle name
                                           200.0,  # Max isometric force
                                           0.6,  # Optimal fibre length
                                           0.55,  # Tendon slack length
                                           0.0)  # Pennation angle
biceps.addNewPathPoint("origin",
                       humerus,
                       osim.Vec3(0, 0.8, 0))

biceps.addNewPathPoint("insertion",
                       radius,
                       osim.Vec3(0, 0.7, 0))

# ---------------------------------------------------------------------------
# Add a controller that specifies the excitation of the muscle.
# ---------------------------------------------------------------------------

brain = osim.PrescribedController()
brain.addActuator(biceps)
brain.prescribeControlForActuator("biceps",
                                  osim.StepFunction(0.5, 3.0, 0.3, 1.0))

# ---------------------------------------------------------------------------
# Build model with components created above.
# ---------------------------------------------------------------------------

arm.addBody(deBaan)
arm.addBody(deBoot)
arm.addBody(humerus)
arm.addBody(radius)
arm.addJoint(baanJoint)
arm.addJoint(bootJoint)
arm.addJoint(shoulder) # Now required in OpenSim4.0
arm.addJoint(elbow)
arm.addForce(biceps)
arm.addController(brain)

# ---------------------------------------------------------------------------
# Add a console reporter to print the muscle fibre force and elbow angle.
# ---------------------------------------------------------------------------

# We want to write our simulation results to the console.
reporter = osim.ConsoleReporter()
reporter.set_report_time_interval(1.0)
reporter.addToReport(biceps.getOutput("fiber_force"))
elbow_coord = elbow.getCoordinate().getOutput("value")
reporter.addToReport(elbow_coord, "elbow_angle")
arm.addComponent(reporter)

# ---------------------------------------------------------------------------
# Add display geometry. 
# ---------------------------------------------------------------------------

bodyGeometry = osim.Ellipsoid(0.1, 0.5, 0.1)
bodyGeometry.setColor(osim.Gray)
humerusCenter = osim.PhysicalOffsetFrame()
humerusCenter.setName("humerusCenter")
humerusCenter.setParentFrame(humerus)
humerusCenter.setOffsetTransform(osim.Transform(osim.Vec3(0, 0.5, 0)))
humerus.addComponent(humerusCenter)
humerusCenter.attachGeometry(bodyGeometry.clone())

radiusCenter = osim.PhysicalOffsetFrame()
radiusCenter.setName("radiusCenter")
radiusCenter.setParentFrame(radius)
radiusCenter.setOffsetTransform(osim.Transform(osim.Vec3(0, 0.5, 0)))
radius.addComponent(radiusCenter)
radiusCenter.attachGeometry(bodyGeometry.clone())

# ---------------------------------------------------------------------------
# Configure the model.
# ---------------------------------------------------------------------------

state = arm.initSystem()
# Fix the shoulder at its default angle and begin with the elbow flexed.
shoulder.getCoordinate().setLocked(state, True)
elbow.getCoordinate().setValue(state, 0.5 * osim.SimTK_PI)
arm.equilibrateMuscles(state)

# ---------------------------------------------------------------------------
# Simulate.
# ---------------------------------------------------------------------------

manager = osim.Manager(arm)
state.setTime(0)
manager.initialize(state)
state = manager.integrate(10.0)

# ---------------------------------------------------------------------------
# Print/save model file
# ---------------------------------------------------------------------------

arm.printToXML("SimpleArm.osim")
