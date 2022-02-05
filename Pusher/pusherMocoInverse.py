""" Derived from the example3DWalking example from Moco  """

import opensim as osim


def solveMocoInverse():

    # Construct the MocoInverse tool.
    inverse = osim.MocoInverse()

    modelProcessor = osim.ModelProcessor('Pusher.osim')
    modelProcessor.append(osim.ModOpIgnoreTendonCompliance())
    modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
    # Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(osim.ModOpIgnorePassiveFiberForcesDGF())
    # Only valid for DeGrooteFregly2016Muscles.
    modelProcessor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
    modelProcessor.append(osim.ModOpAddReserves(1.0))
    inverse.setModel(modelProcessor)

    # Construct a TableProcessor of the coordinate data and pass it to the
    # inverse tool. TableProcessors can be used in the same way as
    # ModelProcessors by appending TableOperators to modify the base table.
    # A TableProcessor with no operators, as we have here, simply returns the
    # base table.
    inverse.setKinematics(osim.TableProcessor('trajectory.mot'))

    # Initial time, final time, and mesh interval.
    inverse.set_initial_time(0.0)
    inverse.set_final_time(11.98)
    inverse.set_mesh_interval(0.02)

    # By default, Moco gives an error if the kinematics contains extra columns.
    # Here, we tell Moco to allow (and ignore) those extra columns.
    inverse.set_kinematics_allow_extra_columns(True)

    # Solve the problem and write the solution to a Storage file.
    solution = inverse.solve()
    solution.getMocoSolution().write('moco_solution.sto')


solveMocoInverse()
