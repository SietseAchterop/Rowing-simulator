/* -------------------------------------------------------------------------- *
 *                               Simbody(tm)                                  *
 * -------------------------------------------------------------------------- *
 * This is part of the SimTK biosimulation toolkit originating from           *
 * Simbios, the NIH National Center for Physics-Based Simulation of           *
 * Biological Structures at Stanford, funded under the NIH Roadmap for        *
 * Medical Research, grant U54 GM072970. See https://simtk.org/home/simbody.  *
 *                                                                            *
 * Portions copyright (c) 2008-12 Stanford University and the Authors.        *
 * Authors: Peter Eastman                                                     *
 * Contributors:                                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "SimTKcommon.h"

#include "simbody/internal/common.h"
#include "simbody/internal/GeneralContactSubsystem.h"
#include "simbody/internal/MobilizedBody.h"
#include "BladeForceImpl.h"
#include <map>
#include <set>

#include <iomanip>

namespace SimTK {

SimTK_INSERT_DERIVED_HANDLE_DEFINITIONS(BladeForce, BladeForceImpl, Force);

BladeForce::BladeForce(GeneralForceSubsystem& forces, GeneralContactSubsystem& contacts, ContactSetIndex set) :
        Force(new BladeForceImpl(contacts, set)) {
    updImpl().setForceSubsystem(forces, forces.adoptForce(*this));
}

void BladeForce::setBodyParameters
   (ContactSurfaceIndex bodyIndex, Real stiffness, Real dissipation, 
    Real staticFriction, Real dynamicFriction, Real viscousFriction) {
    updImpl().setBodyParameters(bodyIndex, stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction);
}

Real BladeForce::getTransitionVelocity() const {
    return getImpl().transitionVelocity;
}

void BladeForce::setTransitionVelocity(Real v) {
    updImpl().transitionVelocity = v;
}

BladeForceImpl::BladeForceImpl
   (GeneralContactSubsystem& subsystem, ContactSetIndex set) : 
        subsystem(subsystem), set(set), transitionVelocity(Real(0.01)) {
}

void BladeForceImpl::setBodyParameters
   (ContactSurfaceIndex bodyIndex, Real stiffness, Real dissipation, 
    Real staticFriction, Real dynamicFriction, Real viscousFriction) {
    SimTK_APIARGCHECK1(bodyIndex >= 0 && bodyIndex < subsystem.getNumBodies(set), "BladeForceImpl", "setBodyParameters",
            "Illegal body index: %d", (int)bodyIndex);
    SimTK_APIARGCHECK1(subsystem.getBodyGeometry(set, bodyIndex).getTypeId() 
                        == ContactGeometry::TriangleMesh::classTypeId(), 
        "BladeForceImpl", "setBodyParameters",
        "Body %d is not a triangle mesh", (int)bodyIndex);
    parameters[bodyIndex] = 
        Parameters(stiffness, dissipation, staticFriction, dynamicFriction, 
                   viscousFriction);
    const ContactGeometry::TriangleMesh& mesh = 
        ContactGeometry::TriangleMesh::getAs
                (subsystem.getBodyGeometry(set, bodyIndex));
    Parameters& param = parameters[bodyIndex];
    param.springPosition.resize(mesh.getNumFaces());
    param.springNormal.resize(mesh.getNumFaces());
    param.springArea.resize(mesh.getNumFaces());
    Vec2 uv(Real(1./3.), Real(1./3.));
    for (int i = 0; i < (int) param.springPosition.size(); i++) {
        param.springPosition[i] = 
           (mesh.getVertexPosition(mesh.getFaceVertex(i, 0))
            +mesh.getVertexPosition(mesh.getFaceVertex(i, 1))
            +mesh.getVertexPosition(mesh.getFaceVertex(i, 2)))/3;
        param.springNormal[i] = -mesh.findNormalAtPoint(i, uv);
        param.springArea[i] = mesh.getFaceArea(i);
    }
    subsystem.invalidateSubsystemTopologyCache();
}

void BladeForceImpl::calcForce
   (const State& state, Vector_<SpatialVec>& bodyForces, 
    Vector_<Vec3>& particleForces, Vector& mobilityForces) const 
{
    const Array_<Contact>& contacts = subsystem.getContacts(state, set);
    Real& pe = Value<Real>::updDowncast
                (subsystem.updCacheEntry(state, energyCacheIndex));
    pe = 0.0;
    for (int i = 0; i < (int) contacts.size(); i++) {
        std::map<ContactSurfaceIndex, Parameters>::const_iterator iter1 = 
            parameters.find(contacts[i].getSurface1());
        std::map<ContactSurfaceIndex, Parameters>::const_iterator iter2 = 
            parameters.find(contacts[i].getSurface2());

        // If there are two meshes, scale each one's contributions by 50%.
        Real areaScale = (iter1==parameters.end() || iter2==parameters.end())
                         ? Real(1) : Real(0.5);

        // in ons geval geldt iter1 == parameters.end().
        if (iter1 != parameters.end()) {
	      // A crude check
	      std::cout << "Should not happen in BladeForce!" << std::endl;
	      exit(-1);

	  const TriangleMeshContact& contact = 
	    static_cast<const TriangleMeshContact&>(contacts[i]);
	  processContact(state, contact.getSurface1(), 
			 contact.getSurface2(), iter1->second, 
			 contact.getSurface1Faces(), areaScale, bodyForces, pe);
        }

        // Only this branch will be taken
        if (iter2 != parameters.end()) {
            const TriangleMeshContact& contact = 
                static_cast<const TriangleMeshContact&>(contacts[i]);
	    std::cout << "call processContact, i:" << i << "  "  << contacts[i].nameOfCondition << std::endl;
            processContact(state, contact.getSurface2(), 
                contact.getSurface1(), iter2->second, 
                contact.getSurface2Faces(), areaScale, bodyForces, pe);
        }
    }
}

void BladeForceImpl::processContact
   (const State& state, 
    ContactSurfaceIndex meshIndex, ContactSurfaceIndex otherBodyIndex, 
    const Parameters& param, const std::set<int>& insideFaces,
    Real areaScale, Vector_<SpatialVec>& bodyForces, Real& pe) const 
{
    const ContactGeometry& otherObject = subsystem.getBodyGeometry(set, otherBodyIndex);
    const MobilizedBody& body1 = subsystem.getBody(set, meshIndex);
    const MobilizedBody& body2 = subsystem.getBody(set, otherBodyIndex);
    const Transform t1g = body1.getBodyTransform(state)*subsystem.getBodyTransform(set, meshIndex); // mesh to ground
    const Transform t2g = body2.getBodyTransform(state)*subsystem.getBodyTransform(set, otherBodyIndex); // other object to ground
    const Transform t12 = ~t2g*t1g; // mesh to other object

    // Added wrt ElasticFoundationForce
    const Rotation body1_rot = body1.getBodyRotation(state);
    // Direction of X-axis of body1 wrt the ground frame (normalized)
    const Vec3 xdir = body1_rot*Vec3(0,1,0);
    // speed of body1 wrt the ground plane
    Vec3 vel = body1.getBodyOriginVelocity(state);
    // snelheiden zijn wel heel klein! 

    // beter gewoon de hoek met de x-as van body1 tov ground frame nemen? 
    // kan omdat de boot steeds op de X-as blijft.
    vel = Vec3(1, 0, 0);

    // angle between blade and velocity of blade, only use XZ plane.
    // DIT IS GEWOON FOUT!! sin + cos is te simpel

    int bofs;
    Vec3 poss = body1.getBodyOriginLocation(state);
    std::cout << "position body: " << poss << std::endl;
    if (poss[2] > 0) bofs = 1; else bofs = 0;
    
    const Real tijd = state.getTime();
 
    if (bofs == 0)
      std::cout << "=====  body1_rot: " << std::setprecision(4) <<  body1_rot << " xdir " << xdir << " vel: "  <<  vel << " Tijd: " << std::setprecision(4) << tijd << std::endl;
    else
      std::cout << "++++  body1_rot: " << std::setprecision(4) <<  body1_rot << " xdir " << xdir << " vel: "  <<  vel << " Tijd: " << std::setprecision(4) << tijd << std::endl;

    Real angle;
    if (vel.norm() < 0.0001) {
      angle = 0;
      std::cout << "BladForce: no speed." << std::endl;
      // we get here using the Inverse Kinematics tool
    } else {
      // dot product in the XZ plane
      const Real dotprod = vel[0]*xdir[0] + vel[2]*xdir[2];
      const Real prod = std::sqrt(vel[0]*vel[0]+vel[2]*vel[2])  *  std::sqrt(xdir[0]*xdir[0]+xdir[2]*xdir[2]);
      angle = std::acos(dotprod/prod);
      if (angle > Pi/2) angle = Pi - angle;
      if (bofs == 0)
	std::cout << "==== prContact dotprod: " << dotprod << " prod: " << prod <<  " angle: " << std::setprecision(3) <<  angle *(180/3.14159) << std::endl;
      else
	std::cout << "++++ prContact dotprod: " << dotprod << " prod: " << prod <<  " angle: " << std::setprecision(3) <<  angle *(180/3.14159) << std::endl;
	
    }

    // Set param values according to angle
    // parameters in the direction of the blade
    const Real stblade = 50000, disblade = 0.8, statfblade = 0.01, dynfblade = 0.01, viscblade = 0.01, transvblade = 0.02;
    // kan dit niet beter?
    Real stiffness = stblade*std::cos(angle) + param.stiffness*std::sin(angle);
    Real dissip = disblade*std::cos(angle) + param.dissipation*std::sin(angle);
    Real staticf = statfblade*std::cos(angle) + param.staticFriction*std::sin(angle);
    Real dynamf = dynfblade*std::cos(angle) + param.dynamicFriction*std::sin(angle);
    Real viscf = viscblade*std::cos(angle) + param.viscousFriction*std::sin(angle);
    Real transvel = transvblade*std::cos(angle) + transitionVelocity*std::sin(angle);

    if (bofs == 0)
      std::cout << "==== boord Parameters: angle: " << angle*(180/3.14159) << " stiffness: " << stiffness << " dissipation " << dissip << " staticF " << staticf << " dynamicF " << dynamf << " viscousF " << viscf  << " transV " << transvel << std::endl;
    else
      std::cout << "++++ boord Parameters: angle: " << angle*(180/3.14159) << " stiffness: " << stiffness << " dissipation " << dissip << " staticF " << staticf << " dynamicF " << dynamf << " viscousF " << viscf  << " transV " << transvel << std::endl;
      

    // Loop over all the springs, and evaluate the force from each one.

    for (std::set<int>::const_iterator iter = insideFaces.begin(); 
                                       iter != insideFaces.end(); ++iter) {
        int face = *iter;
        UnitVec3 normal;
        bool inside;
        Vec3 nearestPoint = otherObject.findNearestPoint(t12*param.springPosition[face], inside, normal);
        if (!inside)
            continue;
        
        // Find how much the spring is displaced.
        
        nearestPoint = t2g*nearestPoint;
        const Vec3 springPosInGround = t1g*param.springPosition[face];
        const Vec3 displacement = nearestPoint-springPosInGround;
        const Real distance = displacement.norm();
        if (distance == 0.0)
            continue;
        const Vec3 forceDir = displacement/distance;
        
        // Calculate the relative velocity of the two bodies at the contact point.
        
        const Vec3 station1 = body1.findStationAtGroundPoint(state, nearestPoint);
        const Vec3 station2 = body2.findStationAtGroundPoint(state, nearestPoint);
        // als ik gebruik maak van dat de mesh een brick is?
        //   en dat body2 ground is? dus v2 == 0
        const Vec3 v1 = body1.findStationVelocityInGround(state, station1);
        const Vec3 v2 = body2.findStationVelocityInGround(state, station2);
        const Vec3 v = v2-v1;
        const Real vnormal = dot(v, forceDir);
        const Vec3 vtangent = v-vnormal*forceDir;
        
        // Calculate the damping force.

        const Real area = areaScale * param.springArea[face];
        const Real f = stiffness*area*distance*(1+dissip*vnormal);
        Vec3 force = (f > 0 ? f*forceDir : Vec3(0));
        
        // Calculate the friction force.
        
        const Real vslip = vtangent.norm();
        if (f > 0 && vslip != 0) {
            const Real vrel = vslip/transvel;
            const Real ffriction = 
                f*(std::min(vrel, Real(1))
                 *(dynamf+2*(staticf-dynamf)
                 /(1+vrel*vrel))+viscf*vslip);
            force += ffriction*vtangent/vslip;
        }

        body1.applyForceToBodyPoint(state, station1, force, bodyForces);
        body2.applyForceToBodyPoint(state, station2, -force, bodyForces);
        pe += stiffness*area*displacement.normSqr()/2;
    }
}

Real BladeForceImpl::calcPotentialEnergy(const State& state) const {
    return Value<Real>::downcast
            (subsystem.getCacheEntry(state, energyCacheIndex));
}

void BladeForceImpl::realizeTopology(State& state) const {
    energyCacheIndex = subsystem.allocateCacheEntry
                        (state, Stage::Dynamics, new Value<Real>());
}


} // namespace SimTK

