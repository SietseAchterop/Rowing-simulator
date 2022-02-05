# -------------------------------------------------------------------------- #
# OpenSim Moco: exampleMocoTrack.py                                          #
# -------------------------------------------------------------------------- #
# Copyright (c) 2019 Stanford University and the Authors                     #
#                                                                            #
# Author(s): Nicholas Bianco                                                 #
#                                                                            #
# Licensed under the Apache License, Version 2.0 (the "License") you may     #
# not use this file except in compliance with the License. You may obtain a  #
# copy of the License at http://www.apache.org/licenses/LICENSE-2.0          #
#                                                                            #
# Unless required by applicable law or agreed to in writing, software        #
# distributed under the License is distributed on an "AS IS" BASIS,          #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
# See the License for the specific language governing permissions and        #
# limitations under the License.                                             #
# -------------------------------------------------------------------------- #

# This example features two different tracking problems solved using the
# MocoTrack tool. 
#  - The first problem demonstrates the basic usage of the tool interface
#    to solve a torque-driven marker tracking problem.
#  - The second problem shows how to customize a muscle-driven state tracking
#    problem using more advanced features of the tool interface.
#
# See the README.txt next to this file for more information.

import opensim as osim


def muscleDrivenStateTracking():

    # Create and name an instance of the MocoTrack tool.
    track = osim.MocoTrack()
    track.setName("muscle_driven_state_tracking")

    # Construct a ModelProcessor and set it on the tool. The default
    # muscles in the model are replaced with optimization-friendly
    # DeGrooteFregly2016Muscles, and adjustments are made to the default muscle
    # parameters.
    modelProcessor = osim.ModelProcessor("Pusher_noC.osim")
    # modelProcessor.append(osim.ModOpAddExternalLoads("grf_walk.xml"))
    modelProcessor.append(osim.ModOpIgnoreTendonCompliance())
    modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
    # Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(osim.ModOpIgnorePassiveFiberForcesDGF())
    # Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
    track.setModel(modelProcessor)

    # Construct a TableProcessor of the coordinate data and pass it to the 
    # tracking tool. TableProcessors can be used in the same way as
    # ModelProcessors by appending TableOperators to modify the base table.
    # A TableProcessor with no operators, as we have here, simply returns the
    # base table.
    track.setStatesReference(osim.TableProcessor("trajectory.mot"))
    track.set_states_global_tracking_weight(10)

    coordWeights = osim.MocoWeightSet()
    coordWeights.cloneAndAppend(osim.MocoWeight("bJoint_3", 0))
    coordWeights.cloneAndAppend(osim.MocoWeight("bJoint_4", 0))
    coordWeights.cloneAndAppend(osim.MocoWeight("baseangle", 20))
    coordWeights.cloneAndAppend(osim.MocoWeight("kneeangle", 20))
    coordWeights.cloneAndAppend(osim.MocoWeight("bladeangle", 20))
    track.set_states_weight_set(coordWeights)

    # This setting allows extra data columns contained in the states
    # reference that don't correspond to model coordinates.
    track.set_allow_unused_references(True)

    # Since there is only coordinate position data the states references, this
    # setting is enabled to fill in the missing coordinate speed data using
    # the derivative of splined position data.
    track.set_track_reference_position_derivatives(True)

    # Initial time, final time, and mesh interval.
    track.set_initial_time(0.0)
    track.set_final_time(1.0)
    track.set_mesh_interval(0.08)

    # sol = track.solve()
    # sol.write("deze.sto")
    # return

    # Instead of calling solve(), call initialize() to receive a pre-configured
    # MocoStudy object based on the settings above. Use this to customize the
    # problem beyond the MocoTrack interface.
    study = track.initialize()

    # Get a reference to the MocoControlCost that is added to every MocoTrack
    # problem by default.
    problem = study.updProblem()
    effort = osim.MocoControlGoal.safeDownCast(problem.updGoal("control_effort"))

    # Put a large weight on the pelvis CoordinateActuators, which act as the
    # residual, or 'hand-of-god', forces which we would like to keep as small
    # as possible.
    model = modelProcessor.process()
    model.initSystem()
    forceSet = model.getForceSet()
    for i in range(forceSet.getSize()):
        forcePath = forceSet.get(i).getAbsolutePathString()
        print(forcePath)
        if 'bj_act' in str(forcePath):
            # is this relevant?
            effort.setWeightForControl(forcePath, 0)
    # Solve and visualize.
    solution = study.solve()
    solution.write('mocotrack_solution.sto')
    study.visualize(solution)


# Solve the muscle-driven state tracking problem.
# This problem could take an hour or more to solve, depending on the number of
# processor cores available for parallelization. With 12 cores, it takes around
# 25 minutes.
muscleDrivenStateTracking()
