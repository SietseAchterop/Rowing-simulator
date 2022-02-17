import opensim as osim

def muscleDrivenStateTracking():
    track = osim.MocoTrack()
    track.setName("muscle_driven_state_tracking")
    modelProcessor = osim.ModelProcessor("Pusher.osim")
    track.setModel(modelProcessor)

    tableProcessor = osim.TableProcessor("trajectory.mot")
    tableProcessor.append(osim.TabOpUseAbsoluteStateNames())
    track.setStatesReference(tableProcessor)

    track.set_states_global_tracking_weight(100)

    coordWeights = osim.MocoWeightSet()
    coordWeights.cloneAndAppend(osim.MocoWeight("/jointset/boatJoint/bJoint_3/value", 0))
    coordWeights.cloneAndAppend(osim.MocoWeight("/jointset/boatJoint/bJoint_4/value", 0))
    coordWeights.cloneAndAppend(osim.MocoWeight("/jointset/boatJoint/baseangle/value", 200))
    coordWeights.cloneAndAppend(osim.MocoWeight("/jointset/boatJoint/kneeangle/value", 200))
    coordWeights.cloneAndAppend(osim.MocoWeight("/jointset/boatJoint/bladeangle/value", 200))
    track.set_states_weight_set(coordWeights)
    track.set_allow_unused_references(True)
    track.set_track_reference_position_derivatives(True)
    track.set_initial_time(0.0)
    track.set_final_time(0.4)
    track.set_mesh_interval(0.025)
    sol = track.solve()
    sol.write("mocotrack_solution.sto")
    return

muscleDrivenStateTracking()
