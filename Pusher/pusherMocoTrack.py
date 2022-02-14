import opensim as osim

def muscleDrivenStateTracking():
    track = osim.MocoTrack()
    track.setName("muscle_driven_state_tracking")
    modelProcessor = osim.ModelProcessor("Pusher.osim")
    """
    modelProcessor.append(osim.ModOpIgnoreTendonCompliance())
    modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
    modelProcessor.append(osim.ModOpIgnorePassiveFiberForcesDGF())
    modelProcessor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
    modelProcessor.append(osim.ModOpAddReserves(250))
    """
    modelProcessor.append(osim.ModOpAddReserves(250))
    track.setModel(modelProcessor)
    track.setStatesReference(osim.TableProcessor("trajectory.mot"))
    track.set_states_global_tracking_weight(100)

    coordWeights = osim.MocoWeightSet()
    coordWeights.cloneAndAppend(osim.MocoWeight("bJoint_3", 10))
    coordWeights.cloneAndAppend(osim.MocoWeight("bJoint_4", 10))
    coordWeights.cloneAndAppend(osim.MocoWeight("baseangle", 20))
    coordWeights.cloneAndAppend(osim.MocoWeight("kneeangle", 20))
    coordWeights.cloneAndAppend(osim.MocoWeight("bladeangle", 20))
    track.set_states_weight_set(coordWeights)
    track.set_allow_unused_references(True)
    track.set_track_reference_position_derivatives(True)
    track.set_initial_time(0.0)
    track.set_final_time(5.0)
    track.set_mesh_interval(0.08)
    sol = track.solve()
    sol.write("mocotrack_solution.sto")
    return

muscleDrivenStateTracking()
