/*
  Test for ElasticFoundationForce.

 */

#include "Simbody.h"

using namespace SimTK;
using namespace std;

// some useful rotation
const Rotation YtoX(-Pi/2,ZAxis);
const Rotation YtoZ( Pi/2,XAxis);
const Rotation ZtoY( -Pi/2,XAxis);
    

int main() {
  try
  { // Create the system.
    
    MultibodySystem         system; system.setUseUniformBackground(true);
    SimbodyMatterSubsystem  matter(system);
    GeneralContactSubsystem contacts(system);
    GeneralForceSubsystem   forces(system);
    Force::UniformGravity   gravity(forces, matter, Vec3(0, -9.8, 0));

    Visualizer viz(system);
    system.addEventReporter(new Visualizer::Reporter(viz, 1./30));
    viz.setBackgroundType(Visualizer::GroundAndSky);
    viz.setDesiredFrameRate(200);
    viz.setShowSimTime(true);

    const Vec3 bricksize = Vec3(0.3, 0.02, 0.15);

  
    // Add leg
    Body::Rigid leg(MassProperties(10.0, Vec3(0,0,0), Inertia(1)));
    leg.addDecoration(Transform(), DecorativeCylinder(0.075, 0.5).setColor(Red)); 
    MobilizedBody::Pin upperleg(matter.updGround(), Transform(Rotation(0.3, ZAxis), Vec3(0, 0, 0)), leg, Transform(Vec3(0, -0.5, 0)));
    MobilizedBody::Pin lowerleg(upperleg, Transform(Rotation(1, ZAxis), Vec3(0, 0.5, 0)), leg, Transform(Vec3(0, -0.5, 0)));

    // Add ankle as a platform for the foot to turn on for testing
    Body::Rigid ankle(MassProperties(1, Vec3(0,0,0), Inertia(1)));
    MobilizedBody::Pin anklemod(lowerleg, Transform(Vec3(0, 0.5, 0)), ankle, Transform(Rotation(1, ZAxis), Vec3(0)));
    // Constraint to fix the orientation of the ankle
    Constraint::ConstantOrientation constraint(matter.updGround(), Rotation(), anklemod, Rotation());

    // Add foot with optional turn
    const Real turn = 0.5; //Pi/2;
    Body::Rigid foot(MassProperties(1.234, Vec3(0,0,0), Inertia(1)));
    foot.addDecoration(Transform(), DecorativeBrick(bricksize).setColor(Vec3(0.3, 0.4, 0.8)));
    MobilizedBody::Pin blad(anklemod, Transform(Vec3(0, 0, 0)), foot, Transform(Rotation(turn, YAxis), Vec3(0)));

    // Contacts
    const ContactSetIndex setIndex = contacts.createContactSet();

    // Material properties for sphere
    //  richting van de riem
    //Real stiffness = 1e9;
    //Real dissipation = 0.8, us = 0.0, ud = 0.0, uv = 0.0, vt = 0.0;
    // loodrecht op het blad
    Real stiffness = 1e6;
    Real dissipation = 2.0, us = 0.8, ud = 0.4, uv = 0.4, vt = 0.2;

    contacts.addBody(setIndex, blad, ContactGeometry::TriangleMesh(PolygonalMesh::createBrickMesh(bricksize, 1)), Transform( Vec3(0)));    
    contacts.addBody(setIndex, matter.updGround(), ContactGeometry::HalfSpace(),
		     Transform(YtoX, Vec3(0.0, 0, 0.0)));    

    BladeForce ef(forces, contacts, setIndex);
    ef.setBodyParameters(ContactSurfaceIndex(0), stiffness, dissipation, us, ud, uv);
    ef.setTransitionVelocity(vt);

    system.realizeTopology();
    State state = system.getDefaultState();

    // Simulate it.
    RungeKuttaMersonIntegrator integ(system);
    TimeStepper ts(system, integ);
    ts.initialize(state);
    ts.stepTo(10.0);


  } catch (const std::exception& e) {
    std::printf("EXCEPTION THROWN: %s\n", e.what());
    exit(1);

  } catch (...) {
    std::printf("UNKNOWN EXCEPTION THROWN\n");
    exit(1);
  }

    return 0;
}
    


