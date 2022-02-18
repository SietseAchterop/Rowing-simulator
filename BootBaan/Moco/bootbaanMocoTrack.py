import opensim as osim

def muscleDrivenStateTracking():
    track = osim.MocoTrack()
    track.setName("muscle_driven_state_tracking")
    modelProcessor = osim.ModelProcessor("BootBaan.osim")
    track.setModel(modelProcessor)

    tableProcessor = osim.TableProcessor("trajectory.mot")
    tableProcessor.append(osim.TabOpUseAbsoluteStateNames())
    track.setStatesReference(tableProcessor)

    track.set_states_global_tracking_weight(30)

    coordWeights = osim.MocoWeightSet()
    coordWeights.cloneAndAppend(osim.MocoWeight("/jointset/boatJoint/bJoint_3/value", 0))
    coordWeights.cloneAndAppend(osim.MocoWeight("/jointset/boatJoint/bJoint_3/speed", 0))
    coordWeights.cloneAndAppend(osim.MocoWeight("/jointset/boatJoint/bJoint_4/value", 0))
    coordWeights.cloneAndAppend(osim.MocoWeight("/jointset/boatJoint/bJoint_4/speed", 0))

    coordWeights.cloneAndAppend(osim.MocoWeight("/jointset/boatJoint/seatpos/value", 20))
    coordWeights.cloneAndAppend(osim.MocoWeight("/jointset/boatJoint/hipangle/value", 20))
    coordWeights.cloneAndAppend(osim.MocoWeight("/jointset/boatJoint/kneeangle/value", 20))
    coordWeights.cloneAndAppend(osim.MocoWeight("/jointset/boatJoint/footangle/value", 20))
    coordWeights.cloneAndAppend(osim.MocoWeight("/jointset/boatJoint/lbackangle/value", 20))

    track.set_states_weight_set(coordWeights)
    track.set_allow_unused_references(True)
    track.set_track_reference_position_derivatives(True)
    track.set_initial_time(0.02)
    track.set_final_time(0.2)
    track.set_mesh_interval(0.025)
    sol = track.solve()
    sol.write("mocotrack_solution.sto")
    return

muscleDrivenStateTracking()
