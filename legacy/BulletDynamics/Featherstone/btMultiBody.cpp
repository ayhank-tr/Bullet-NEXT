/*
 * PURPOSE:
 *   Class representing an articulated rigid body. Stores the body's
 *   current state, allows forces and torques to be set, handles
 *   timestepping and implements Featherstone's algorithm.
 *   
 * COPYRIGHT:
 *   Copyright (C) Stephen Thompson, <stephen@solarflare.org.uk>, 2011-2013
 *   Portions written By Erwin Coumans: connection to LCP solver, various multibody constraints, replacing Eigen math library by Bullet LinearMath and a dedicated 6x6 matrix inverse (solveImatrix)
 *   Portions written By Jakub Stepien: support for multi-DOF constraints, introduction of spatial algebra and several other improvements

 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 
 */


#include "btMultiBody.h"
#include "btMultiBodyLink.h"
#include "btMultiBodyLinkCollider.h"
#include "btMultiBodyJointFeedback.h"
#include "LinearMath/btTransformUtil.h"
#include "LinearMath/btSerializer.h"
// #define INCLUDE_GYRO_TERM 

///todo: determine if we need these options. If so, make a proper API, otherwise delete those globals
bool btl_gJointFeedbackInWorldSpace = false;
bool btl_gJointFeedbackInJointFrame = false;

namespace {
    const btlScalar SLEEP_EPSILON = btlScalar(0.05);  // this is a squared velocity (m^2 s^-2)
    const btlScalar SLEEP_TIMEOUT = btlScalar(2);     // in seconds
}

namespace {
    void SpatialTransform(const btlMatrix3x3 &rotation_matrix,  // rotates vectors in 'from' frame to vectors in 'to' frame
                          const btlVector3 &displacement,     // vector from origin of 'from' frame to origin of 'to' frame, in 'to' coordinates
                          const btlVector3 &top_in,       // top part of input vector
                          const btlVector3 &bottom_in,    // bottom part of input vector
                          btlVector3 &top_out,         // top part of output vector
                          btlVector3 &bottom_out)      // bottom part of output vector
    {
        top_out = rotation_matrix * top_in;
        bottom_out = -displacement.cross(top_out) + rotation_matrix * bottom_in;
    }

/*
    void InverseSpatialTransform(const btMatrix3x3 &rotation_matrix,
                                 const btVector3 &displacement,
                                 const btVector3 &top_in,
                                 const btVector3 &bottom_in,
                                 btVector3 &top_out,
                                 btVector3 &bottom_out)
    {
        top_out = rotation_matrix.transpose() * top_in;
        bottom_out = rotation_matrix.transpose() * (bottom_in + displacement.cross(top_in));		
    }

    btScalar SpatialDotProduct(const btVector3 &a_top,
                            const btVector3 &a_bottom,
                            const btVector3 &b_top,
                            const btVector3 &b_bottom)
    {
        return a_bottom.dot(b_top) + a_top.dot(b_bottom);
    }

	void SpatialCrossProduct(const btVector3 &a_top,
                            const btVector3 &a_bottom,
                            const btVector3 &b_top,
                            const btVector3 &b_bottom,
							btVector3 &top_out,
							btVector3 &bottom_out)
	{
		top_out = a_top.cross(b_top);
		bottom_out = a_bottom.cross(b_top) + a_top.cross(b_bottom);
	}
*/
}


//
// Implementation of class btMultiBody
//

btlMultiBody::btlMultiBody(int n_links,
                     btlScalar mass,
                     const btlVector3 &inertia,
                     bool fixedBase,
                     bool canSleep,
		     bool /*deprecatedUseMultiDof*/)
    : 
    	m_baseCollider(0),
		m_baseName(0),
    	m_basePos(0,0,0),
    	m_baseQuat(0, 0, 0, 1),
      m_baseMass(mass),
      m_baseInertia(inertia),
    
		m_fixedBase(fixedBase),
		m_awake(true),
		m_canSleep(canSleep),
		m_sleepTimer(0),
		
		m_linearDamping(0.04f),
		m_angularDamping(0.04f),
		m_useGyroTerm(true),
			m_maxAppliedImpulse(1000.f),
		m_maxCoordinateVelocity(100.f),
			m_hasSelfCollision(true),		
		__posUpdated(false),
			m_dofCount(0),
		m_posVarCnt(0),
		m_useRK4(false), 	
		m_useGlobalVelocities(false),
		m_internalNeedsJointFeedback(false)
{
	m_links.resize(n_links);
	m_matrixBuf.resize(n_links + 1);


    m_baseForce.setValue(0, 0, 0);
    m_baseTorque.setValue(0, 0, 0);
}

btlMultiBody::~btlMultiBody()
{
}

void btlMultiBody::setupFixed(int i,
						   btlScalar mass,
						   const btlVector3 &inertia,
						   int parent,
						   const btlQuaternion &rotParentToThis,
						   const btlVector3 &parentComToThisPivotOffset,
                           const btlVector3 &thisPivotToThisComOffset, bool /*deprecatedDisableParentCollision*/)
{
	
	m_links[i].m_mass = mass;
    m_links[i].m_inertiaLocal = inertia;
    m_links[i].m_parent = parent;
    m_links[i].m_zeroRotParentToThis = rotParentToThis;
	m_links[i].m_dVector = thisPivotToThisComOffset;
    m_links[i].m_eVector = parentComToThisPivotOffset;    

	m_links[i].m_jointType = btlMultibodyLink::eFixed;
	m_links[i].m_dofCount = 0;
	m_links[i].m_posVarCount = 0;

	m_links[i].m_flags |=BT_MULTIBODYLINKFLAGS_DISABLE_PARENT_COLLISION;    
	
	m_links[i].updateCacheMultiDof();

	updateLinksDofOffsets();

}


void btlMultiBody::setupPrismatic(int i,
                               btlScalar mass,
                               const btlVector3 &inertia,
                               int parent,
                               const btlQuaternion &rotParentToThis,
                               const btlVector3 &jointAxis,
                               const btlVector3 &parentComToThisPivotOffset,
							   const btlVector3 &thisPivotToThisComOffset,
							   bool disableParentCollision)
{
	m_dofCount += 1;
	m_posVarCnt += 1;
	
    m_links[i].m_mass = mass;
    m_links[i].m_inertiaLocal = inertia;
    m_links[i].m_parent = parent;
    m_links[i].m_zeroRotParentToThis = rotParentToThis;
    m_links[i].setAxisTop(0, 0., 0., 0.);
    m_links[i].setAxisBottom(0, jointAxis);
    m_links[i].m_eVector = parentComToThisPivotOffset;
	m_links[i].m_dVector = thisPivotToThisComOffset;
    m_links[i].m_cachedRotParentToThis = rotParentToThis;

	m_links[i].m_jointType = btlMultibodyLink::ePrismatic;
	m_links[i].m_dofCount = 1;
	m_links[i].m_posVarCount = 1;	
	m_links[i].m_jointPos[0] = 0.f;
	m_links[i].m_jointTorque[0] = 0.f;

	if (disableParentCollision)
		m_links[i].m_flags |=BT_MULTIBODYLINKFLAGS_DISABLE_PARENT_COLLISION;
	//
	
	m_links[i].updateCacheMultiDof();
	
	updateLinksDofOffsets();
}

void btlMultiBody::setupRevolute(int i,
                              btlScalar mass,
                              const btlVector3 &inertia,
                              int parent,
                              const btlQuaternion &rotParentToThis,
                              const btlVector3 &jointAxis,
                              const btlVector3 &parentComToThisPivotOffset,
                              const btlVector3 &thisPivotToThisComOffset,
							  bool disableParentCollision)
{
	m_dofCount += 1;
	m_posVarCnt += 1;
	
    m_links[i].m_mass = mass;
    m_links[i].m_inertiaLocal = inertia;
    m_links[i].m_parent = parent;
    m_links[i].m_zeroRotParentToThis = rotParentToThis;
    m_links[i].setAxisTop(0, jointAxis);
    m_links[i].setAxisBottom(0, jointAxis.cross(thisPivotToThisComOffset));
    m_links[i].m_dVector = thisPivotToThisComOffset;
    m_links[i].m_eVector = parentComToThisPivotOffset;

	m_links[i].m_jointType = btlMultibodyLink::eRevolute;
	m_links[i].m_dofCount = 1;
	m_links[i].m_posVarCount = 1;	
	m_links[i].m_jointPos[0] = 0.f;
	m_links[i].m_jointTorque[0] = 0.f;

	if (disableParentCollision)
		m_links[i].m_flags |=BT_MULTIBODYLINKFLAGS_DISABLE_PARENT_COLLISION;
    //
	m_links[i].updateCacheMultiDof();
	//
	updateLinksDofOffsets();
}



void btlMultiBody::setupSpherical(int i,
						   btlScalar mass,
						   const btlVector3 &inertia,
						   int parent,
						   const btlQuaternion &rotParentToThis,
						   const btlVector3 &parentComToThisPivotOffset,
						   const btlVector3 &thisPivotToThisComOffset,
						   bool disableParentCollision)
{
	
	m_dofCount += 3;
	m_posVarCnt += 4;

	m_links[i].m_mass = mass;
    m_links[i].m_inertiaLocal = inertia;
    m_links[i].m_parent = parent;
    m_links[i].m_zeroRotParentToThis = rotParentToThis;    
    m_links[i].m_dVector = thisPivotToThisComOffset;
    m_links[i].m_eVector = parentComToThisPivotOffset;    

	m_links[i].m_jointType = btlMultibodyLink::eSpherical;
	m_links[i].m_dofCount = 3;
	m_links[i].m_posVarCount = 4;
	m_links[i].setAxisTop(0, 1.f, 0.f, 0.f);
	m_links[i].setAxisTop(1, 0.f, 1.f, 0.f);
	m_links[i].setAxisTop(2, 0.f, 0.f, 1.f);
	m_links[i].setAxisBottom(0, m_links[i].getAxisTop(0).cross(thisPivotToThisComOffset));
	m_links[i].setAxisBottom(1, m_links[i].getAxisTop(1).cross(thisPivotToThisComOffset));
	m_links[i].setAxisBottom(2, m_links[i].getAxisTop(2).cross(thisPivotToThisComOffset));
	m_links[i].m_jointPos[0] = m_links[i].m_jointPos[1] = m_links[i].m_jointPos[2] = 0.f; m_links[i].m_jointPos[3] = 1.f;
	m_links[i].m_jointTorque[0] = m_links[i].m_jointTorque[1] = m_links[i].m_jointTorque[2] = 0.f;


	if (disableParentCollision)
		m_links[i].m_flags |=BT_MULTIBODYLINKFLAGS_DISABLE_PARENT_COLLISION;    
	//
	m_links[i].updateCacheMultiDof();	
	//
	updateLinksDofOffsets();
}

void btlMultiBody::setupPlanar(int i,
						   btlScalar mass,
						   const btlVector3 &inertia,
						   int parent,
						   const btlQuaternion &rotParentToThis,
						   const btlVector3 &rotationAxis,
						   const btlVector3 &parentComToThisComOffset,						   
						   bool disableParentCollision)
{
	
	m_dofCount += 3;
	m_posVarCnt += 3;

	m_links[i].m_mass = mass;
    m_links[i].m_inertiaLocal = inertia;
    m_links[i].m_parent = parent;
    m_links[i].m_zeroRotParentToThis = rotParentToThis;    
	m_links[i].m_dVector.setZero();
    m_links[i].m_eVector = parentComToThisComOffset;

	//
	static btlVector3 vecNonParallelToRotAxis(1, 0, 0);
	if(rotationAxis.normalized().dot(vecNonParallelToRotAxis) > 0.999)
		vecNonParallelToRotAxis.setValue(0, 1, 0);
	//

	m_links[i].m_jointType = btlMultibodyLink::ePlanar;
	m_links[i].m_dofCount = 3;
	m_links[i].m_posVarCount = 3;
	btlVector3 n=rotationAxis.normalized();
	m_links[i].setAxisTop(0, n[0],n[1],n[2]);
	m_links[i].setAxisTop(1,0,0,0);
	m_links[i].setAxisTop(2,0,0,0);
	m_links[i].setAxisBottom(0,0,0,0);
	btlVector3 cr = m_links[i].getAxisTop(0).cross(vecNonParallelToRotAxis);
	m_links[i].setAxisBottom(1,cr[0],cr[1],cr[2]);
	cr = m_links[i].getAxisBottom(1).cross(m_links[i].getAxisTop(0));
	m_links[i].setAxisBottom(2,cr[0],cr[1],cr[2]);
	m_links[i].m_jointPos[0] = m_links[i].m_jointPos[1] = m_links[i].m_jointPos[2] = 0.f;
	m_links[i].m_jointTorque[0] = m_links[i].m_jointTorque[1] = m_links[i].m_jointTorque[2] = 0.f;

	if (disableParentCollision)
		m_links[i].m_flags |=BT_MULTIBODYLINKFLAGS_DISABLE_PARENT_COLLISION;
    //
	m_links[i].updateCacheMultiDof();
	//
	updateLinksDofOffsets();
}

void btlMultiBody::finalizeMultiDof()
{
	m_deltaV.resize(0);
	m_deltaV.resize(6 + m_dofCount);
	m_realBuf.resize(6 + m_dofCount + m_dofCount*m_dofCount + 6 + m_dofCount);			//m_dofCount for joint-space vels + m_dofCount^2 for "D" matrices + delta-pos vector (6 base "vels" + joint "vels")
	m_vectorBuf.resize(2 * m_dofCount);													//two 3-vectors (i.e. one six-vector) for each system dof	("h" matrices)

	updateLinksDofOffsets();
}
	
int btlMultiBody::getParent(int i) const
{
    return m_links[i].m_parent;
}

btlScalar btlMultiBody::getLinkMass(int i) const
{
    return m_links[i].m_mass;
}

const btlVector3 & btlMultiBody::getLinkInertia(int i) const
{
    return m_links[i].m_inertiaLocal;
}

btlScalar btlMultiBody::getJointPos(int i) const
{
    return m_links[i].m_jointPos[0];
}

btlScalar btlMultiBody::getJointVel(int i) const
{
    return m_realBuf[6 + m_links[i].m_dofOffset];
}

btlScalar * btlMultiBody::getJointPosMultiDof(int i)
{
	return &m_links[i].m_jointPos[0];
}

btlScalar * btlMultiBody::getJointVelMultiDof(int i)
{
	return &m_realBuf[6 + m_links[i].m_dofOffset];
}

const btlScalar * btlMultiBody::getJointPosMultiDof(int i) const 
{
	return &m_links[i].m_jointPos[0];
}

const btlScalar * btlMultiBody::getJointVelMultiDof(int i) const 
{
	return &m_realBuf[6 + m_links[i].m_dofOffset];
}


void btlMultiBody::setJointPos(int i, btlScalar q)
{
    m_links[i].m_jointPos[0] = q;
    m_links[i].updateCacheMultiDof();
}

void btlMultiBody::setJointPosMultiDof(int i, btlScalar *q)
{
	for(int pos = 0; pos < m_links[i].m_posVarCount; ++pos)
		m_links[i].m_jointPos[pos] = q[pos];

    m_links[i].updateCacheMultiDof();
}

void btlMultiBody::setJointVel(int i, btlScalar qdot)
{
    m_realBuf[6 + m_links[i].m_dofOffset] = qdot;
}

void btlMultiBody::setJointVelMultiDof(int i, btlScalar *qdot)
{
	for(int dof = 0; dof < m_links[i].m_dofCount; ++dof)
		m_realBuf[6 + m_links[i].m_dofOffset + dof] = qdot[dof];
}

const btlVector3 & btlMultiBody::getRVector(int i) const
{
    return m_links[i].m_cachedRVector;
}

const btlQuaternion & btlMultiBody::getParentToLocalRot(int i) const
{
    return m_links[i].m_cachedRotParentToThis;
}

btlVector3 btlMultiBody::localPosToWorld(int i, const btlVector3 &local_pos) const
{
    btlVector3 result = local_pos;
    while (i != -1) {
        // 'result' is in frame i. transform it to frame parent(i)
        result += getRVector(i);
        result = btl_quatRotate(getParentToLocalRot(i).inverse(),result);
        i = getParent(i);
    }

    // 'result' is now in the base frame. transform it to world frame
    result = btl_quatRotate(getWorldToBaseRot().inverse() ,result);
    result += getBasePos();

    return result;
}

btlVector3 btlMultiBody::worldPosToLocal(int i, const btlVector3 &world_pos) const
{
    if (i == -1) {
        // world to base
        return btl_quatRotate(getWorldToBaseRot(),(world_pos - getBasePos()));
    } else {
        // find position in parent frame, then transform to current frame
        return btl_quatRotate(getParentToLocalRot(i),worldPosToLocal(getParent(i), world_pos)) - getRVector(i);
    }
}

btlVector3 btlMultiBody::localDirToWorld(int i, const btlVector3 &local_dir) const
{
    btlVector3 result = local_dir;
    while (i != -1) {
        result = btl_quatRotate(getParentToLocalRot(i).inverse() , result);
        i = getParent(i);
    }
    result = btl_quatRotate(getWorldToBaseRot().inverse() , result);
    return result;
}

btlVector3 btlMultiBody::worldDirToLocal(int i, const btlVector3 &world_dir) const
{
    if (i == -1) {
        return btl_quatRotate(getWorldToBaseRot(), world_dir);
    } else {
        return btl_quatRotate(getParentToLocalRot(i) ,worldDirToLocal(getParent(i), world_dir));
    }
}

void btlMultiBody::compTreeLinkVelocities(btlVector3 *omega, btlVector3 *vel) const
{
	int num_links = getNumLinks();
    // Calculates the velocities of each link (and the base) in its local frame
    omega[0] = btl_quatRotate(m_baseQuat ,getBaseOmega());
    vel[0] = btl_quatRotate(m_baseQuat ,getBaseVel());
    
    for (int i = 0; i < num_links; ++i) {
        const int parent = m_links[i].m_parent;

        // transform parent vel into this frame, store in omega[i+1], vel[i+1]
        SpatialTransform(btlMatrix3x3(m_links[i].m_cachedRotParentToThis), m_links[i].m_cachedRVector,
                         omega[parent+1], vel[parent+1],
                         omega[i+1], vel[i+1]);

        // now add qidot * shat_i
        omega[i+1] += getJointVel(i) * m_links[i].getAxisTop(0);
        vel[i+1] += getJointVel(i) * m_links[i].getAxisBottom(0);
    }
}

btlScalar btlMultiBody::getKineticEnergy() const
{
	int num_links = getNumLinks();
    // TODO: would be better not to allocate memory here
    btlAlignedObjectArray<btlVector3> omega;omega.resize(num_links+1);
	btlAlignedObjectArray<btlVector3> vel;vel.resize(num_links+1);
    compTreeLinkVelocities(&omega[0], &vel[0]);

    // we will do the factor of 0.5 at the end
    btlScalar result = m_baseMass * vel[0].dot(vel[0]);
    result += omega[0].dot(m_baseInertia * omega[0]);
    
    for (int i = 0; i < num_links; ++i) {
        result += m_links[i].m_mass * vel[i+1].dot(vel[i+1]);
        result += omega[i+1].dot(m_links[i].m_inertiaLocal * omega[i+1]);
    }

    return 0.5f * result;
}

btlVector3 btlMultiBody::getAngularMomentum() const
{
	int num_links = getNumLinks();
    // TODO: would be better not to allocate memory here
    btlAlignedObjectArray<btlVector3> omega;omega.resize(num_links+1);
	btlAlignedObjectArray<btlVector3> vel;vel.resize(num_links+1);
    btlAlignedObjectArray<btlQuaternion> rot_from_world;rot_from_world.resize(num_links+1);
    compTreeLinkVelocities(&omega[0], &vel[0]);

    rot_from_world[0] = m_baseQuat;
    btlVector3 result = btl_quatRotate(rot_from_world[0].inverse() , (m_baseInertia * omega[0]));

    for (int i = 0; i < num_links; ++i) {
        rot_from_world[i+1] = m_links[i].m_cachedRotParentToThis * rot_from_world[m_links[i].m_parent+1];
        result += (btl_quatRotate(rot_from_world[i+1].inverse() , (m_links[i].m_inertiaLocal * omega[i+1])));
    }

    return result;
}

void btlMultiBody::clearConstraintForces()
{
	m_baseConstraintForce.setValue(0, 0, 0);
	m_baseConstraintTorque.setValue(0, 0, 0);


    for (int i = 0; i < getNumLinks(); ++i) {
        m_links[i].m_appliedConstraintForce.setValue(0, 0, 0);
        m_links[i].m_appliedConstraintTorque.setValue(0, 0, 0);
    }
}
void btlMultiBody::clearForcesAndTorques()
{
    m_baseForce.setValue(0, 0, 0);
    m_baseTorque.setValue(0, 0, 0);

	
    for (int i = 0; i < getNumLinks(); ++i) {
        m_links[i].m_appliedForce.setValue(0, 0, 0);
        m_links[i].m_appliedTorque.setValue(0, 0, 0);
		m_links[i].m_jointTorque[0] = m_links[i].m_jointTorque[1] = m_links[i].m_jointTorque[2] = m_links[i].m_jointTorque[3] = m_links[i].m_jointTorque[4] = m_links[i].m_jointTorque[5] = 0.f;
    }
}

void btlMultiBody::clearVelocities()
{
	for (int i = 0; i < 6 + getNumLinks(); ++i) 
	{
		m_realBuf[i] = 0.f;
	}
}
void btlMultiBody::addLinkForce(int i, const btlVector3 &f)
{
    m_links[i].m_appliedForce += f;
}

void btlMultiBody::addLinkTorque(int i, const btlVector3 &t)
{
    m_links[i].m_appliedTorque += t;
}

void btlMultiBody::addLinkConstraintForce(int i, const btlVector3 &f)
{
    m_links[i].m_appliedConstraintForce += f;
}

void btlMultiBody::addLinkConstraintTorque(int i, const btlVector3 &t)
{
    m_links[i].m_appliedConstraintTorque += t;
}



void btlMultiBody::addJointTorque(int i, btlScalar Q)
{
    m_links[i].m_jointTorque[0] += Q;
}

void btlMultiBody::addJointTorqueMultiDof(int i, int dof, btlScalar Q)
{
	m_links[i].m_jointTorque[dof] += Q;
}

void btlMultiBody::addJointTorqueMultiDof(int i, const btlScalar *Q)
{
	for(int dof = 0; dof < m_links[i].m_dofCount; ++dof)
		m_links[i].m_jointTorque[dof] = Q[dof];
}

const btlVector3 & btlMultiBody::getLinkForce(int i) const
{
    return m_links[i].m_appliedForce;
}

const btlVector3 & btlMultiBody::getLinkTorque(int i) const
{
    return m_links[i].m_appliedTorque;
}

btlScalar btlMultiBody::getJointTorque(int i) const
{
    return m_links[i].m_jointTorque[0];
}

btlScalar * btlMultiBody::getJointTorqueMultiDof(int i)
{
    return &m_links[i].m_jointTorque[0];
}

inline btlMatrix3x3 outerProduct(const btlVector3& v0, const btlVector3& v1)				//renamed it from vecMulVecTranspose (http://en.wikipedia.org/wiki/Outer_product); maybe it should be moved to btVector3 like dot and cross?
{
		btlVector3 row0 = btlVector3( 
			v0.x() * v1.x(),
			v0.x() * v1.y(),
			v0.x() * v1.z());
		btlVector3 row1 = btlVector3( 
			v0.y() * v1.x(),
			v0.y() * v1.y(),
			v0.y() * v1.z());
		btlVector3 row2 = btlVector3( 
			v0.z() * v1.x(),
			v0.z() * v1.y(),
			v0.z() * v1.z());

        btlMatrix3x3 m(row0[0],row0[1],row0[2],
						row1[0],row1[1],row1[2],
						row2[0],row2[1],row2[2]);
		return m;
}

#define vecMulVecTranspose(v0, v1Transposed) outerProduct(v0, v1Transposed)
//

void btlMultiBody::computeAccelerationsArticulatedBodyAlgorithmMultiDof(btlScalar dt,
                               btlAlignedObjectArray<btlScalar> &scratch_r,
                               btlAlignedObjectArray<btlVector3> &scratch_v,
                               btlAlignedObjectArray<btlMatrix3x3> &scratch_m,
				bool isConstraintPass)
{
    // Implement Featherstone's algorithm to calculate joint accelerations (q_double_dot)
    // and the base linear & angular accelerations.

    // We apply damping forces in this routine as well as any external forces specified by the 
    // caller (via addBaseForce etc).

    // output should point to an array of 6 + num_links reals.
    // Format is: 3 angular accelerations (in world frame), 3 linear accelerations (in world frame),
    // num_links joint acceleration values.
    
	// We added support for multi degree of freedom (multi dof) joints.
	// In addition we also can compute the joint reaction forces. This is performed in a second pass,
	// so that we can include the effect of the constraint solver forces (computed in the PGS LCP solver)

	m_internalNeedsJointFeedback = false;

	int num_links = getNumLinks();

    const btlScalar DAMPING_K1_LINEAR = m_linearDamping;
	const btlScalar DAMPING_K2_LINEAR = m_linearDamping;

	const btlScalar DAMPING_K1_ANGULAR = m_angularDamping;
	const btlScalar DAMPING_K2_ANGULAR= m_angularDamping;

    btlVector3 base_vel = getBaseVel();
    btlVector3 base_omega = getBaseOmega();

    // Temporary matrices/vectors -- use scratch space from caller
    // so that we don't have to keep reallocating every frame

    scratch_r.resize(2*m_dofCount + 6);				//multidof? ("Y"s use it and it is used to store qdd) => 2 x m_dofCount
    scratch_v.resize(8*num_links + 6);
    scratch_m.resize(4*num_links + 4);

	//btScalar * r_ptr = &scratch_r[0];
    btlScalar * output = &scratch_r[m_dofCount];  // "output" holds the q_double_dot results
    btlVector3 * v_ptr = &scratch_v[0];
    
    // vhat_i  (top = angular, bottom = linear part)	
	btlSpatialMotionVector *spatVel = (btlSpatialMotionVector *)v_ptr;
	v_ptr += num_links * 2 + 2;
	//
    // zhat_i^A    
	btlSpatialForceVector * zeroAccSpatFrc = (btlSpatialForceVector *)v_ptr;
	v_ptr += num_links * 2 + 2;
	//
    // chat_i  (note NOT defined for the base)    
	btlSpatialMotionVector * spatCoriolisAcc = (btlSpatialMotionVector *)v_ptr;
	v_ptr += num_links * 2;
	//
    // Ihat_i^A.    
	btlSymmetricSpatialDyad * spatInertia = (btlSymmetricSpatialDyad *)&scratch_m[num_links + 1];

    // Cached 3x3 rotation matrices from parent frame to this frame.
    btlMatrix3x3 * rot_from_parent = &m_matrixBuf[0];
    btlMatrix3x3 * rot_from_world = &scratch_m[0];

    // hhat_i, ahat_i
    // hhat is NOT stored for the base (but ahat is)    
	btlSpatialForceVector * h = (btlSpatialForceVector *)(m_dofCount > 0 ? &m_vectorBuf[0] : 0);
	btlSpatialMotionVector * spatAcc = (btlSpatialMotionVector *)v_ptr;
	v_ptr += num_links * 2 + 2;
	//
    // Y_i, invD_i
    btlScalar * invD = m_dofCount > 0 ? &m_realBuf[6 + m_dofCount] : 0;
	btlScalar * Y = &scratch_r[0];
	//
	//aux variables	
	static btlSpatialMotionVector spatJointVel;					//spatial velocity due to the joint motion (i.e. without predecessors' influence)
	static btlScalar D[36];										//"D" matrix; it's dofxdof for each body so asingle 6x6 D matrix will do	
	static btlScalar invD_times_Y[6];							//D^{-1} * Y [dofxdof x dofx1 = dofx1] <=> D^{-1} * u; better moved to buffers since it is recalced in calcAccelerationDeltasMultiDof; num_dof of btScalar would cover all bodies	
	static btlSpatialMotionVector result;							//holds results of the SolveImatrix op; it is a spatial motion vector (accel)
	static btlScalar Y_minus_hT_a[6];							//Y - h^{T} * a; it's dofx1 for each body so a single 6x1 temp is enough	
	static btlSpatialForceVector spatForceVecTemps[6];				//6 temporary spatial force vectors
	static btlSpatialTransformationMatrix fromParent;				//spatial transform from parent to child
	static btlSymmetricSpatialDyad dyadTemp;						//inertia matrix temp
	static btlSpatialTransformationMatrix fromWorld;
	fromWorld.m_trnVec.setZero();
	/////////////////

    // ptr to the joint accel part of the output
    btlScalar * joint_accel = output + 6;

    // Start of the algorithm proper.
    
    // First 'upward' loop.
    // Combines CompTreeLinkVelocities and InitTreeLinks from Mirtich.

    rot_from_parent[0] = btlMatrix3x3(m_baseQuat);				//m_baseQuat assumed to be alias!?

	//create the vector of spatial velocity of the base by transforming global-coor linear and angular velocities into base-local coordinates
	spatVel[0].setVector(rot_from_parent[0] * base_omega, rot_from_parent[0] * base_vel);

    if (m_fixedBase) 
	{        
		zeroAccSpatFrc[0].setZero();
    }
	else 
	{
		btlVector3 baseForce = isConstraintPass? m_baseConstraintForce : m_baseForce;
		btlVector3 baseTorque = isConstraintPass? m_baseConstraintTorque : m_baseTorque;
		//external forces		
		zeroAccSpatFrc[0].setVector(-(rot_from_parent[0] * baseTorque), -(rot_from_parent[0] * baseForce));	

		//adding damping terms (only)
		btlScalar linDampMult = 1., angDampMult = 1.;
		zeroAccSpatFrc[0].addVector(angDampMult * m_baseInertia * spatVel[0].getAngular() * (DAMPING_K1_ANGULAR + DAMPING_K2_ANGULAR * spatVel[0].getAngular().norm()),
								   linDampMult * m_baseMass * spatVel[0].getLinear() * (DAMPING_K1_LINEAR + DAMPING_K2_LINEAR * spatVel[0].getLinear().norm()));

		//
		//p += vhat x Ihat vhat - done in a simpler way
		if (m_useGyroTerm)
			zeroAccSpatFrc[0].addAngular(spatVel[0].getAngular().cross(m_baseInertia * spatVel[0].getAngular()));
		//
		zeroAccSpatFrc[0].addLinear(m_baseMass * spatVel[0].getAngular().cross(spatVel[0].getLinear()));
    }


	//init the spatial AB inertia (it has the simple form thanks to choosing local body frames origins at their COMs)
	spatInertia[0].setMatrix(	btlMatrix3x3(0,0,0,0,0,0,0,0,0),
								//
								btlMatrix3x3(m_baseMass, 0, 0,
											0, m_baseMass, 0,
											0, 0, m_baseMass),
								//
								btlMatrix3x3(m_baseInertia[0], 0, 0,
											0, m_baseInertia[1], 0,
											0, 0, m_baseInertia[2])
							);

    rot_from_world[0] = rot_from_parent[0];

	//
    for (int i = 0; i < num_links; ++i) {		
        const int parent = m_links[i].m_parent;
        rot_from_parent[i+1] = btlMatrix3x3(m_links[i].m_cachedRotParentToThis);
        rot_from_world[i+1] = rot_from_parent[i+1] * rot_from_world[parent+1];

		fromParent.m_rotMat = rot_from_parent[i+1]; fromParent.m_trnVec = m_links[i].m_cachedRVector;
		fromWorld.m_rotMat = rot_from_world[i+1];
		fromParent.transform(spatVel[parent+1], spatVel[i+1]);

		// now set vhat_i to its true value by doing
        // vhat_i += qidot * shat_i			
		if(!m_useGlobalVelocities)
		{
			spatJointVel.setZero();

			for(int dof = 0; dof < m_links[i].m_dofCount; ++dof)		
				spatJointVel += m_links[i].m_axes[dof] * getJointVelMultiDof(i)[dof];

			// remember vhat_i is really vhat_p(i) (but in current frame) at this point	=> we need to add velocity across the inboard joint
			spatVel[i+1] += spatJointVel;

			//
			// vhat_i is vhat_p(i) transformed to local coors + the velocity across the i-th inboard joint
			//spatVel[i+1] = fromParent * spatVel[parent+1] + spatJointVel;

		}
		else
		{
			fromWorld.transformRotationOnly(m_links[i].m_absFrameTotVelocity, spatVel[i+1]);
			fromWorld.transformRotationOnly(m_links[i].m_absFrameLocVelocity, spatJointVel);
		}

		// we can now calculate chat_i 		
		spatVel[i+1].cross(spatJointVel, spatCoriolisAcc[i]);		

        // calculate zhat_i^A
		//
		//external forces		
		btlVector3 linkAppliedForce = isConstraintPass? m_links[i].m_appliedConstraintForce : m_links[i].m_appliedForce;
		btlVector3 linkAppliedTorque =isConstraintPass ? m_links[i].m_appliedConstraintTorque : m_links[i].m_appliedTorque;
 
		zeroAccSpatFrc[i+1].setVector(-(rot_from_world[i+1] * linkAppliedTorque), -(rot_from_world[i+1] * linkAppliedForce ));
	
#if 0	
		{

			b3Printf("stepVelocitiesMultiDof zeroAccSpatFrc[%d] linear:%f,%f,%f, angular:%f,%f,%f",
			i+1,
			zeroAccSpatFrc[i+1].m_topVec[0],
			zeroAccSpatFrc[i+1].m_topVec[1],
			zeroAccSpatFrc[i+1].m_topVec[2],

			zeroAccSpatFrc[i+1].m_bottomVec[0],
			zeroAccSpatFrc[i+1].m_bottomVec[1],
			zeroAccSpatFrc[i+1].m_bottomVec[2]);
		}
#endif
		//
		//adding damping terms (only)
		btlScalar linDampMult = 1., angDampMult = 1.;
		zeroAccSpatFrc[i+1].addVector(angDampMult * m_links[i].m_inertiaLocal * spatVel[i+1].getAngular() * (DAMPING_K1_ANGULAR + DAMPING_K2_ANGULAR * spatVel[i+1].getAngular().norm()),
									 linDampMult * m_links[i].m_mass * spatVel[i+1].getLinear() * (DAMPING_K1_LINEAR + DAMPING_K2_LINEAR * spatVel[i+1].getLinear().norm()));
		
        // calculate Ihat_i^A
		//init the spatial AB inertia (it has the simple form thanks to choosing local body frames origins at their COMs)
		spatInertia[i+1].setMatrix(	btlMatrix3x3(0,0,0,0,0,0,0,0,0),
									//
									btlMatrix3x3(m_links[i].m_mass, 0, 0,
												0, m_links[i].m_mass, 0,
												0, 0, m_links[i].m_mass),
									//
									btlMatrix3x3(m_links[i].m_inertiaLocal[0], 0, 0,
												0, m_links[i].m_inertiaLocal[1], 0,
												0, 0, m_links[i].m_inertiaLocal[2])
								);
		//
		//p += vhat x Ihat vhat - done in a simpler way
		if(m_useGyroTerm)
			zeroAccSpatFrc[i+1].addAngular(spatVel[i+1].getAngular().cross(m_links[i].m_inertiaLocal * spatVel[i+1].getAngular()));			
		//		
		zeroAccSpatFrc[i+1].addLinear(m_links[i].m_mass * spatVel[i+1].getAngular().cross(spatVel[i+1].getLinear()));
		//btVector3 temp = m_links[i].m_mass * spatVel[i+1].getAngular().cross(spatVel[i+1].getLinear());
		////clamp parent's omega
		//btScalar parOmegaMod = temp.length();
		//btScalar parOmegaModMax = 1000;
		//if(parOmegaMod > parOmegaModMax)
		//	temp *= parOmegaModMax / parOmegaMod;
		//zeroAccSpatFrc[i+1].addLinear(temp);
		//printf("|zeroAccSpatFrc[%d]| = %.4f\n", i+1, temp.length());
		//temp = spatCoriolisAcc[i].getLinear();
		//printf("|spatCoriolisAcc[%d]| = %.4f\n", i+1, temp.length());
		
		

		//printf("w[%d] = [%.4f %.4f %.4f]\n", i, vel_top_angular[i+1].x(), vel_top_angular[i+1].y(), vel_top_angular[i+1].z());
		//printf("v[%d] = [%.4f %.4f %.4f]\n", i, vel_bottom_linear[i+1].x(), vel_bottom_linear[i+1].y(), vel_bottom_linear[i+1].z());		
		//printf("c[%d] = [%.4f %.4f %.4f]\n", i, coriolis_bottom_linear[i].x(), coriolis_bottom_linear[i].y(), coriolis_bottom_linear[i].z());
    }
	
    // 'Downward' loop.
    // (part of TreeForwardDynamics in Mirtich.)
    for (int i = num_links - 1; i >= 0; --i)
	{
		const int parent = m_links[i].m_parent;
		fromParent.m_rotMat = rot_from_parent[i+1]; fromParent.m_trnVec = m_links[i].m_cachedRVector;

		for(int dof = 0; dof < m_links[i].m_dofCount; ++dof)
		{
			btlSpatialForceVector &hDof = h[m_links[i].m_dofOffset + dof];
			//
			hDof = spatInertia[i+1] * m_links[i].m_axes[dof];
			//
			Y[m_links[i].m_dofOffset + dof] = m_links[i].m_jointTorque[dof]
			- m_links[i].m_axes[dof].dot(zeroAccSpatFrc[i+1])
			- spatCoriolisAcc[i].dot(hDof)
			;
		}

		for(int dof = 0; dof < m_links[i].m_dofCount; ++dof)
		{
			btlScalar *D_row = &D[dof * m_links[i].m_dofCount];			
			for(int dof2 = 0; dof2 < m_links[i].m_dofCount; ++dof2)
			{
				btlSpatialForceVector &hDof2 = h[m_links[i].m_dofOffset + dof2];
				D_row[dof2] = m_links[i].m_axes[dof].dot(hDof2);
			}
		}

        btlScalar *invDi = &invD[m_links[i].m_dofOffset*m_links[i].m_dofOffset];
		switch(m_links[i].m_jointType)
		{
			case btlMultibodyLink::ePrismatic:
			case btlMultibodyLink::eRevolute:
			{
				invDi[0] = 1.0f / D[0];
				break;
			}
			case btlMultibodyLink::eSpherical:
			case btlMultibodyLink::ePlanar:
			{
				static btlMatrix3x3 D3x3; D3x3.setValue(D[0], D[1], D[2], D[3], D[4], D[5], D[6], D[7], D[8]);
				static btlMatrix3x3 invD3x3; invD3x3 = D3x3.inverse();

				//unroll the loop?
				for(int row = 0; row < 3; ++row)
				{
					for(int col = 0; col < 3; ++col)
					{						
						invDi[row * 3 + col] = invD3x3[row][col];
					}
				}

				break;
			}
			default:
			{
			
			}
		}

		//determine h*D^{-1}
		for(int dof = 0; dof < m_links[i].m_dofCount; ++dof)
		{
			spatForceVecTemps[dof].setZero();

			for(int dof2 = 0; dof2 < m_links[i].m_dofCount; ++dof2)
			{				
				btlSpatialForceVector &hDof2 = h[m_links[i].m_dofOffset + dof2];
				//				
				spatForceVecTemps[dof] += hDof2 * invDi[dof2 * m_links[i].m_dofCount + dof];
			}
		}

		dyadTemp = spatInertia[i+1];

		//determine (h*D^{-1}) * h^{T}
		for(int dof = 0; dof < m_links[i].m_dofCount; ++dof)
		{			
			btlSpatialForceVector &hDof = h[m_links[i].m_dofOffset + dof];
			//
			dyadTemp -= symmetricSpatialOuterProduct(hDof, spatForceVecTemps[dof]);
		}

		fromParent.transformInverse(dyadTemp, spatInertia[parent+1], btlSpatialTransformationMatrix::Add);
        
		for(int dof = 0; dof < m_links[i].m_dofCount; ++dof)
		{
			invD_times_Y[dof] = 0.f;

			for(int dof2 = 0; dof2 < m_links[i].m_dofCount; ++dof2)
			{
				invD_times_Y[dof] += invDi[dof * m_links[i].m_dofCount + dof2] * Y[m_links[i].m_dofOffset + dof2];				
			}	
		}
		
		spatForceVecTemps[0] = zeroAccSpatFrc[i+1] + spatInertia[i+1] * spatCoriolisAcc[i];		

		for(int dof = 0; dof < m_links[i].m_dofCount; ++dof)
		{				
			btlSpatialForceVector &hDof = h[m_links[i].m_dofOffset + dof];
			//
			spatForceVecTemps[0] += hDof * invD_times_Y[dof];			
		}
		
		fromParent.transformInverse(spatForceVecTemps[0], spatForceVecTemps[1]);
		
		zeroAccSpatFrc[parent+1] += spatForceVecTemps[1];
    }


    // Second 'upward' loop
    // (part of TreeForwardDynamics in Mirtich)

    if (m_fixedBase) 
	{
        spatAcc[0].setZero();
    } 
	else 
	{
        if (num_links > 0) 
		{
			m_cachedInertiaTopLeft = spatInertia[0].m_topLeftMat;
			m_cachedInertiaTopRight = spatInertia[0].m_topRightMat;
			m_cachedInertiaLowerLeft = spatInertia[0].m_bottomLeftMat;
			m_cachedInertiaLowerRight= spatInertia[0].m_topLeftMat.transpose();

        }		
		
		solveImatrix(zeroAccSpatFrc[0], result);
		spatAcc[0] = -result;
    }
	
	
    // now do the loop over the m_links
    for (int i = 0; i < num_links; ++i) 
	{
		//	qdd = D^{-1} * (Y - h^{T}*apar) = (S^{T}*I*S)^{-1} * (tau - S^{T}*I*cor - S^{T}*zeroAccFrc - S^{T}*I*apar)
		//	a = apar + cor + Sqdd
		//or
		//	qdd = D^{-1} * (Y - h^{T}*(apar+cor))
		//	a = apar + Sqdd

        const int parent = m_links[i].m_parent;
		fromParent.m_rotMat = rot_from_parent[i+1]; fromParent.m_trnVec = m_links[i].m_cachedRVector;

		fromParent.transform(spatAcc[parent+1], spatAcc[i+1]);
		
		for(int dof = 0; dof < m_links[i].m_dofCount; ++dof)
		{
			btlSpatialForceVector &hDof = h[m_links[i].m_dofOffset + dof];
			//			
			Y_minus_hT_a[dof] = Y[m_links[i].m_dofOffset + dof] - spatAcc[i+1].dot(hDof);			
		}

		btlScalar *invDi = &invD[m_links[i].m_dofOffset*m_links[i].m_dofOffset];
		//D^{-1} * (Y - h^{T}*apar)
		mulMatrix(invDi, Y_minus_hT_a, m_links[i].m_dofCount, m_links[i].m_dofCount, m_links[i].m_dofCount, 1, &joint_accel[m_links[i].m_dofOffset]);

		spatAcc[i+1] += spatCoriolisAcc[i];		

		for(int dof = 0; dof < m_links[i].m_dofCount; ++dof)		
			spatAcc[i+1] += m_links[i].m_axes[dof] * joint_accel[m_links[i].m_dofOffset + dof];

		if (m_links[i].m_jointFeedback)
		{
			m_internalNeedsJointFeedback = true;

			btlVector3 angularBotVec = (spatInertia[i+1]*spatAcc[i+1]+zeroAccSpatFrc[i+1]).m_bottomVec;
			btlVector3 linearTopVec = (spatInertia[i+1]*spatAcc[i+1]+zeroAccSpatFrc[i+1]).m_topVec;

			if (btl_gJointFeedbackInJointFrame)
			{
				//shift the reaction forces to the joint frame
				//linear (force) component is the same
				//shift the angular (torque, moment) component using the relative position,  m_links[i].m_dVector
				 angularBotVec = angularBotVec - linearTopVec.cross(m_links[i].m_dVector);
			}
			

			if (btl_gJointFeedbackInWorldSpace)
			{
				if (isConstraintPass)
				{
 m_links[i].m_jointFeedback->m_reactionForces.m_bottomVec += m_links[i].m_cachedWorldTransform.getBasis()*angularBotVec;
                                        m_links[i].m_jointFeedback->m_reactionForces.m_topVec += m_links[i].m_cachedWorldTransform.getBasis()*linearTopVec;
				} else
				{
					m_links[i].m_jointFeedback->m_reactionForces.m_bottomVec = m_links[i].m_cachedWorldTransform.getBasis()*angularBotVec;
					m_links[i].m_jointFeedback->m_reactionForces.m_topVec = m_links[i].m_cachedWorldTransform.getBasis()*linearTopVec;
				}
			} else
			{
				if (isConstraintPass)
				{
					  m_links[i].m_jointFeedback->m_reactionForces.m_bottomVec += angularBotVec;                        
                                m_links[i].m_jointFeedback->m_reactionForces.m_topVec += linearTopVec;

				}
				else
				{
				m_links[i].m_jointFeedback->m_reactionForces.m_bottomVec = angularBotVec;
				m_links[i].m_jointFeedback->m_reactionForces.m_topVec = linearTopVec;
				}		
			}	
	}

    }

    // transform base accelerations back to the world frame.
    btlVector3 omegadot_out = rot_from_parent[0].transpose() * spatAcc[0].getAngular();
	output[0] = omegadot_out[0];
	output[1] = omegadot_out[1];
	output[2] = omegadot_out[2];

    btlVector3 vdot_out = rot_from_parent[0].transpose() * (spatAcc[0].getLinear() + spatVel[0].getAngular().cross(spatVel[0].getLinear()));
	output[3] = vdot_out[0];
	output[4] = vdot_out[1];
	output[5] = vdot_out[2];

	/////////////////
	//printf("q = [");
	//printf("%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f ", m_baseQuat.x(), m_baseQuat.y(), m_baseQuat.z(), m_baseQuat.w(), m_basePos.x(), m_basePos.y(), m_basePos.z());
	//for(int link = 0; link < getNumLinks(); ++link)
	//	for(int dof = 0; dof < m_links[link].m_dofCount; ++dof)
	//		printf("%.6f ", m_links[link].m_jointPos[dof]);
	//printf("]\n");
	////
	//printf("qd = [");
	//for(int dof = 0; dof < getNumDofs() + 6; ++dof)
	//	printf("%.6f ", m_realBuf[dof]);
	//printf("]\n");
	//printf("qdd = [");
	//for(int dof = 0; dof < getNumDofs() + 6; ++dof)
	//	printf("%.6f ", output[dof]);
	//printf("]\n");
	/////////////////

    // Final step: add the accelerations (times dt) to the velocities.

	if (!isConstraintPass)
	{
	if(dt > 0.)
		applyDeltaVeeMultiDof(output, dt);

	}
	/////
	//btScalar angularThres = 1;
	//btScalar maxAngVel = 0.;		
	//bool scaleDown = 1.;
	//for(int link = 0; link < m_links.size(); ++link)
	//{		
	//	if(spatVel[link+1].getAngular().length() > maxAngVel)
	//	{
	//		maxAngVel = spatVel[link+1].getAngular().length();
	//		scaleDown = angularThres / spatVel[link+1].getAngular().length();
	//		break;
	//	}		
	//}

	//if(scaleDown != 1.)
	//{
	//	for(int link = 0; link < m_links.size(); ++link)
	//	{
	//		if(m_links[link].m_jointType == btMultibodyLink::eRevolute || m_links[link].m_jointType == btMultibodyLink::eSpherical)
	//		{
	//			for(int dof = 0; dof < m_links[link].m_dofCount; ++dof)
	//				getJointVelMultiDof(link)[dof] *= scaleDown;
	//		}
	//	}
	//}
	/////

	/////////////////////
	if(m_useGlobalVelocities)
	{
		for (int i = 0; i < num_links; ++i) 
		{
			const int parent = m_links[i].m_parent;
			//rot_from_parent[i+1] = btMatrix3x3(m_links[i].m_cachedRotParentToThis);    /// <- done
			//rot_from_world[i+1] = rot_from_parent[i+1] * rot_from_world[parent+1];		/// <- done
		
			fromParent.m_rotMat = rot_from_parent[i+1]; fromParent.m_trnVec = m_links[i].m_cachedRVector;
			fromWorld.m_rotMat = rot_from_world[i+1];			
        
			// vhat_i = i_xhat_p(i) * vhat_p(i)		
			fromParent.transform(spatVel[parent+1], spatVel[i+1]);
			//nice alternative below (using operator *) but it generates temps
			/////////////////////////////////////////////////////////////

			// now set vhat_i to its true value by doing
			// vhat_i += qidot * shat_i			
			spatJointVel.setZero();

			for(int dof = 0; dof < m_links[i].m_dofCount; ++dof)		
				spatJointVel += m_links[i].m_axes[dof] * getJointVelMultiDof(i)[dof];
		
			// remember vhat_i is really vhat_p(i) (but in current frame) at this point	=> we need to add velocity across the inboard joint
			spatVel[i+1] += spatJointVel;


			fromWorld.transformInverseRotationOnly(spatVel[i+1], m_links[i].m_absFrameTotVelocity);
			fromWorld.transformInverseRotationOnly(spatJointVel, m_links[i].m_absFrameLocVelocity);
		}
	}
	
}



void btlMultiBody::solveImatrix(const btlVector3& rhs_top, const btlVector3& rhs_bot, float result[6]) const
{
	int num_links = getNumLinks();
	///solve I * x = rhs, so the result = invI * rhs
    if (num_links == 0) 
	{
		// in the case of 0 m_links (i.e. a plain rigid body, not a multibody) rhs * invI is easier
        result[0] = rhs_bot[0] / m_baseInertia[0];
        result[1] = rhs_bot[1] / m_baseInertia[1];
        result[2] = rhs_bot[2] / m_baseInertia[2];
        result[3] = rhs_top[0] / m_baseMass;
        result[4] = rhs_top[1] / m_baseMass;
        result[5] = rhs_top[2] / m_baseMass;
    } else 
	{
		/// Special routine for calculating the inverse of a spatial inertia matrix
		///the 6x6 matrix is stored as 4 blocks of 3x3 matrices
		btlMatrix3x3 Binv = m_cachedInertiaTopRight.inverse()*-1.f;
		btlMatrix3x3 tmp = m_cachedInertiaLowerRight * Binv;
		btlMatrix3x3 invIupper_right = (tmp * m_cachedInertiaTopLeft + m_cachedInertiaLowerLeft).inverse();
		tmp = invIupper_right * m_cachedInertiaLowerRight;
		btlMatrix3x3 invI_upper_left = (tmp * Binv);
		btlMatrix3x3 invI_lower_right = (invI_upper_left).transpose();
		tmp = m_cachedInertiaTopLeft  * invI_upper_left;
		tmp[0][0]-= 1.0;
		tmp[1][1]-= 1.0;
		tmp[2][2]-= 1.0;
		btlMatrix3x3 invI_lower_left = (Binv * tmp);

		//multiply result = invI * rhs
		{
		  btlVector3 vtop = invI_upper_left*rhs_top;
		  btlVector3 tmp;
		  tmp = invIupper_right * rhs_bot;
		  vtop += tmp;
		  btlVector3 vbot = invI_lower_left*rhs_top;
		  tmp = invI_lower_right * rhs_bot;
		  vbot += tmp;
		  result[0] = vtop[0];
		  result[1] = vtop[1];
		  result[2] = vtop[2];
		  result[3] = vbot[0];
		  result[4] = vbot[1];
		  result[5] = vbot[2];
		}
		
    }
}
void btlMultiBody::solveImatrix(const btlSpatialForceVector &rhs, btlSpatialMotionVector &result) const
{
	int num_links = getNumLinks();
	///solve I * x = rhs, so the result = invI * rhs
    if (num_links == 0) 
	{
		// in the case of 0 m_links (i.e. a plain rigid body, not a multibody) rhs * invI is easier
		result.setAngular(rhs.getAngular() / m_baseInertia);
		result.setLinear(rhs.getLinear() / m_baseMass);		
    } else 
	{
		/// Special routine for calculating the inverse of a spatial inertia matrix
		///the 6x6 matrix is stored as 4 blocks of 3x3 matrices
		btlMatrix3x3 Binv = m_cachedInertiaTopRight.inverse()*-1.f;
		btlMatrix3x3 tmp = m_cachedInertiaLowerRight * Binv;
		btlMatrix3x3 invIupper_right = (tmp * m_cachedInertiaTopLeft + m_cachedInertiaLowerLeft).inverse();
		tmp = invIupper_right * m_cachedInertiaLowerRight;
		btlMatrix3x3 invI_upper_left = (tmp * Binv);
		btlMatrix3x3 invI_lower_right = (invI_upper_left).transpose();
		tmp = m_cachedInertiaTopLeft  * invI_upper_left;
		tmp[0][0]-= 1.0;
		tmp[1][1]-= 1.0;
		tmp[2][2]-= 1.0;
		btlMatrix3x3 invI_lower_left = (Binv * tmp);

		//multiply result = invI * rhs
		{
		  btlVector3 vtop = invI_upper_left*rhs.getLinear();
		  btlVector3 tmp;
		  tmp = invIupper_right * rhs.getAngular();
		  vtop += tmp;
		  btlVector3 vbot = invI_lower_left*rhs.getLinear();
		  tmp = invI_lower_right * rhs.getAngular();
		  vbot += tmp;
		  result.setVector(vtop, vbot);		  
		}
		
    }
}

void btlMultiBody::mulMatrix(btlScalar *pA, btlScalar *pB, int rowsA, int colsA, int rowsB, int colsB, btlScalar *pC) const
{
	for (int row = 0; row < rowsA; row++)
	{
		for (int col = 0; col < colsB; col++)
		{
			pC[row * colsB + col] = 0.f;
			for (int inner = 0; inner < rowsB; inner++)
			{
				pC[row * colsB + col] += pA[row * colsA + inner] * pB[col + inner * colsB];
			}
		}
	}
}

void btlMultiBody::calcAccelerationDeltasMultiDof(const btlScalar *force, btlScalar *output,
                                       btlAlignedObjectArray<btlScalar> &scratch_r, btlAlignedObjectArray<btlVector3> &scratch_v) const
{
    // Temporary matrices/vectors -- use scratch space from caller
    // so that we don't have to keep reallocating every frame

	
	int num_links = getNumLinks();	
    scratch_r.resize(m_dofCount);
    scratch_v.resize(4*num_links + 4);	    

    btlScalar * r_ptr = m_dofCount ? &scratch_r[0] : 0;
    btlVector3 * v_ptr = &scratch_v[0];

    // zhat_i^A (scratch space)
    btlSpatialForceVector * zeroAccSpatFrc = (btlSpatialForceVector *)v_ptr;
	v_ptr += num_links * 2 + 2;

    // rot_from_parent (cached from calcAccelerations)
    const btlMatrix3x3 * rot_from_parent = &m_matrixBuf[0];

    // hhat (cached), accel (scratch)
    // hhat is NOT stored for the base (but ahat is) 
	const btlSpatialForceVector * h = (btlSpatialForceVector *)(m_dofCount > 0 ? &m_vectorBuf[0] : 0);
	btlSpatialMotionVector * spatAcc = (btlSpatialMotionVector *)v_ptr;
	v_ptr += num_links * 2 + 2;

    // Y_i (scratch), invD_i (cached)
    const btlScalar * invD = m_dofCount > 0 ? &m_realBuf[6 + m_dofCount] : 0;
	btlScalar * Y = r_ptr; 
	////////////////
	//aux variables
	static btlScalar invD_times_Y[6];							//D^{-1} * Y [dofxdof x dofx1 = dofx1] <=> D^{-1} * u; better moved to buffers since it is recalced in calcAccelerationDeltasMultiDof; num_dof of btScalar would cover all bodies
	static btlSpatialMotionVector result;							//holds results of the SolveImatrix op; it is a spatial motion vector (accel)
	static btlScalar Y_minus_hT_a[6];							//Y - h^{T} * a; it's dofx1 for each body so a single 6x1 temp is enough	
	static btlSpatialForceVector spatForceVecTemps[6];				//6 temporary spatial force vectors
	static btlSpatialTransformationMatrix fromParent;	
	/////////////////

    // First 'upward' loop.
    // Combines CompTreeLinkVelocities and InitTreeLinks from Mirtich.
	
	// Fill in zero_acc
    // -- set to force/torque on the base, zero otherwise
    if (m_fixedBase) 
	{
        zeroAccSpatFrc[0].setZero();
    } else 
	{	
		//test forces
		fromParent.m_rotMat = rot_from_parent[0];
		fromParent.transformRotationOnly(btlSpatialForceVector(-force[0],-force[1],-force[2], -force[3],-force[4],-force[5]), zeroAccSpatFrc[0]);
    }
    for (int i = 0; i < num_links; ++i) 
	{
		zeroAccSpatFrc[i+1].setZero();
    }    

	// 'Downward' loop.
    // (part of TreeForwardDynamics in Mirtich.)
    for (int i = num_links - 1; i >= 0; --i)
	{
		const int parent = m_links[i].m_parent;
		fromParent.m_rotMat = rot_from_parent[i+1]; fromParent.m_trnVec = m_links[i].m_cachedRVector;

		for(int dof = 0; dof < m_links[i].m_dofCount; ++dof)
		{
			Y[m_links[i].m_dofOffset + dof] = force[6 + m_links[i].m_dofOffset + dof]
											- m_links[i].m_axes[dof].dot(zeroAccSpatFrc[i+1])
											;
		}

		btlVector3 in_top, in_bottom, out_top, out_bottom;
		const btlScalar *invDi = &invD[m_links[i].m_dofOffset*m_links[i].m_dofOffset];
		
		for(int dof = 0; dof < m_links[i].m_dofCount; ++dof)
		{
			invD_times_Y[dof] = 0.f;

			for(int dof2 = 0; dof2 < m_links[i].m_dofCount; ++dof2)
			{
				invD_times_Y[dof] += invDi[dof * m_links[i].m_dofCount + dof2] * Y[m_links[i].m_dofOffset + dof2];				
			}	
		}
		
		 // Zp += pXi * (Zi + hi*Yi/Di)
		spatForceVecTemps[0] = zeroAccSpatFrc[i+1];

		for(int dof = 0; dof < m_links[i].m_dofCount; ++dof)
		{
			const btlSpatialForceVector &hDof = h[m_links[i].m_dofOffset + dof];
			//
			spatForceVecTemps[0] += hDof * invD_times_Y[dof];		
		}
		

		fromParent.transformInverse(spatForceVecTemps[0], spatForceVecTemps[1]);
			
		zeroAccSpatFrc[parent+1] += spatForceVecTemps[1];
    }
	
	// ptr to the joint accel part of the output
    btlScalar * joint_accel = output + 6;


    // Second 'upward' loop
    // (part of TreeForwardDynamics in Mirtich)

    if (m_fixedBase) 
	{
        spatAcc[0].setZero();
    } 
	else 
	{
		solveImatrix(zeroAccSpatFrc[0], result);
		spatAcc[0] = -result;

    }
	
    // now do the loop over the m_links
    for (int i = 0; i < num_links; ++i)
	{
        const int parent = m_links[i].m_parent;
		fromParent.m_rotMat = rot_from_parent[i+1]; fromParent.m_trnVec = m_links[i].m_cachedRVector;

		fromParent.transform(spatAcc[parent+1], spatAcc[i+1]);
		
		for(int dof = 0; dof < m_links[i].m_dofCount; ++dof)
		{
			const btlSpatialForceVector &hDof = h[m_links[i].m_dofOffset + dof];
			//			
			Y_minus_hT_a[dof] = Y[m_links[i].m_dofOffset + dof] - spatAcc[i+1].dot(hDof);
		}

		const btlScalar *invDi = &invD[m_links[i].m_dofOffset*m_links[i].m_dofOffset];
		mulMatrix(const_cast<btlScalar*>(invDi), Y_minus_hT_a, m_links[i].m_dofCount, m_links[i].m_dofCount, m_links[i].m_dofCount, 1, &joint_accel[m_links[i].m_dofOffset]);

		for(int dof = 0; dof < m_links[i].m_dofCount; ++dof)		
			spatAcc[i+1] += m_links[i].m_axes[dof] * joint_accel[m_links[i].m_dofOffset + dof];      
    }

    // transform base accelerations back to the world frame.
    btlVector3 omegadot_out;
    omegadot_out = rot_from_parent[0].transpose() * spatAcc[0].getAngular();
	output[0] = omegadot_out[0];
	output[1] = omegadot_out[1];
	output[2] = omegadot_out[2];

    btlVector3 vdot_out;
    vdot_out = rot_from_parent[0].transpose() * spatAcc[0].getLinear();
	output[3] = vdot_out[0];
	output[4] = vdot_out[1];
	output[5] = vdot_out[2];

	/////////////////
	//printf("delta = [");
	//for(int dof = 0; dof < getNumDofs() + 6; ++dof)
	//	printf("%.2f ", output[dof]);
	//printf("]\n");
	/////////////////
}




void btlMultiBody::stepPositionsMultiDof(btlScalar dt, btlScalar *pq, btlScalar *pqd)
{	
	int num_links = getNumLinks();
    // step position by adding dt * velocity
	//btVector3 v = getBaseVel();	
    //m_basePos += dt * v;
	//
	btlScalar *pBasePos = (pq ? &pq[4] : m_basePos);
	btlScalar *pBaseVel = (pqd ? &pqd[3] : &m_realBuf[3]);			//note: the !pqd case assumes m_realBuf holds with base velocity at 3,4,5 (should be wrapped for safety)
	//	
	pBasePos[0] += dt * pBaseVel[0];
	pBasePos[1] += dt * pBaseVel[1];
	pBasePos[2] += dt * pBaseVel[2];

	///////////////////////////////
	//local functor for quaternion integration (to avoid error prone redundancy)
	struct
	{
		//"exponential map" based on btTransformUtil::integrateTransform(..)
		void operator() (const btlVector3 &omega, btlQuaternion &quat, bool baseBody, btlScalar dt)
		{
			//baseBody	=>	quat is alias and omega is global coor
			//!baseBody	=>	quat is alibi and omega is local coor	
			
			btlVector3 axis;
			btlVector3 angvel;

			if(!baseBody)			
				angvel = btl_quatRotate(quat, omega);				//if quat is not m_baseQuat, it is alibi => ok			
			else
				angvel = omega;
		
			btlScalar fAngle = angvel.length(); 		
			//limit the angular motion
			if (fAngle * dt > ANGULAR_MOTION_THRESHOLD)
			{
				fAngle = btlScalar(0.5)*SIMD_HALF_PI / dt;
			}

			if ( fAngle < btlScalar(0.001) )
			{
				// use Taylor's expansions of sync function
				axis   = angvel*( btlScalar(0.5)*dt-(dt*dt*dt)*(btlScalar(0.020833333333))*fAngle*fAngle );
			}
			else
			{
				// sync(fAngle) = sin(c*fAngle)/t
				axis   = angvel*( btlSin(btlScalar(0.5)*fAngle*dt)/fAngle );
			}
			
			if(!baseBody)				
				quat = btlQuaternion(axis.x(),axis.y(),axis.z(),btlCos( fAngle*dt*btlScalar(0.5) )) * quat;			
			else			
				quat = quat * btlQuaternion(-axis.x(),-axis.y(),-axis.z(),btlCos( fAngle*dt*btlScalar(0.5) ));
				//equivalent to: quat = (btQuaternion(axis.x(),axis.y(),axis.z(),btCos( fAngle*dt*btScalar(0.5) )) * quat.inverse()).inverse();			
		
			quat.normalize();
		}
	} pQuatUpdateFun;
	///////////////////////////////

	//pQuatUpdateFun(getBaseOmega(), m_baseQuat, true, dt);
	//	
	btlScalar *pBaseQuat = pq ? pq : m_baseQuat;	
	btlScalar *pBaseOmega = pqd ? pqd : &m_realBuf[0];		//note: the !pqd case assumes m_realBuf starts with base omega (should be wrapped for safety)
	//
	static btlQuaternion baseQuat; baseQuat.setValue(pBaseQuat[0], pBaseQuat[1], pBaseQuat[2], pBaseQuat[3]);
	static btlVector3 baseOmega; baseOmega.setValue(pBaseOmega[0], pBaseOmega[1], pBaseOmega[2]);
	pQuatUpdateFun(baseOmega, baseQuat, true, dt);
	pBaseQuat[0] = baseQuat.x();
	pBaseQuat[1] = baseQuat.y();
	pBaseQuat[2] = baseQuat.z();
	pBaseQuat[3] = baseQuat.w();


	//printf("pBaseOmega = %.4f %.4f %.4f\n", pBaseOmega->x(), pBaseOmega->y(), pBaseOmega->z());
	//printf("pBaseVel = %.4f %.4f %.4f\n", pBaseVel->x(), pBaseVel->y(), pBaseVel->z());
	//printf("baseQuat = %.4f %.4f %.4f %.4f\n", pBaseQuat->x(), pBaseQuat->y(), pBaseQuat->z(), pBaseQuat->w());

	if(pq)		
		pq += 7;
	if(pqd)
		pqd += 6;

	// Finally we can update m_jointPos for each of the m_links
    for (int i = 0; i < num_links; ++i) 
	{
		btlScalar *pJointPos = (pq ? pq : &m_links[i].m_jointPos[0]);		
		btlScalar *pJointVel = (pqd ? pqd : getJointVelMultiDof(i));

		switch(m_links[i].m_jointType)
		{
			case btlMultibodyLink::ePrismatic:
			case btlMultibodyLink::eRevolute:
			{
				btlScalar jointVel = pJointVel[0];	
				pJointPos[0] += dt * jointVel;
				break;
			}
			case btlMultibodyLink::eSpherical:
			{
				static btlVector3 jointVel; jointVel.setValue(pJointVel[0], pJointVel[1], pJointVel[2]);
				static btlQuaternion jointOri; jointOri.setValue(pJointPos[0], pJointPos[1], pJointPos[2], pJointPos[3]);
				pQuatUpdateFun(jointVel, jointOri, false, dt);
				pJointPos[0] = jointOri.x(); pJointPos[1] = jointOri.y(); pJointPos[2] = jointOri.z(); pJointPos[3] = jointOri.w();
				break;
			}
			case btlMultibodyLink::ePlanar:
			{
				pJointPos[0] += dt * getJointVelMultiDof(i)[0];

				btlVector3 q0_coors_qd1qd2 = getJointVelMultiDof(i)[1] * m_links[i].getAxisBottom(1) + getJointVelMultiDof(i)[2] * m_links[i].getAxisBottom(2);
				btlVector3 no_q0_coors_qd1qd2 = btl_quatRotate(btlQuaternion(m_links[i].getAxisTop(0), pJointPos[0]), q0_coors_qd1qd2);
				pJointPos[1] += m_links[i].getAxisBottom(1).dot(no_q0_coors_qd1qd2) * dt;
				pJointPos[2] += m_links[i].getAxisBottom(2).dot(no_q0_coors_qd1qd2) * dt;

				break;
			}
			default:
			{
			}

		}

		m_links[i].updateCacheMultiDof(pq);

		if(pq)		
			pq += m_links[i].m_posVarCount;
		if(pqd)
			pqd += m_links[i].m_dofCount;
    }
}

void btlMultiBody::fillConstraintJacobianMultiDof(int link,
                                    const btlVector3 &contact_point,
									const btlVector3 &normal_ang,
                                    const btlVector3 &normal_lin,
                                    btlScalar *jac,
                                    btlAlignedObjectArray<btlScalar> &scratch_r,
                                    btlAlignedObjectArray<btlVector3> &scratch_v,
                                    btlAlignedObjectArray<btlMatrix3x3> &scratch_m) const
{
    // temporary space
	int num_links = getNumLinks();
	int m_dofCount = getNumDofs();
    scratch_v.resize(3*num_links + 3);			//(num_links + base) offsets + (num_links + base) normals_lin + (num_links + base) normals_ang
    scratch_m.resize(num_links + 1);

    btlVector3 * v_ptr = &scratch_v[0];
    btlVector3 * p_minus_com_local = v_ptr; v_ptr += num_links + 1;
    btlVector3 * n_local_lin = v_ptr; v_ptr += num_links + 1;
	btlVector3 * n_local_ang = v_ptr; v_ptr += num_links + 1;
    btlAssert(v_ptr - &scratch_v[0] == scratch_v.size());

    scratch_r.resize(m_dofCount);
    btlScalar * results = m_dofCount > 0 ? &scratch_r[0] : 0;

    btlMatrix3x3 * rot_from_world = &scratch_m[0];

    const btlVector3 p_minus_com_world = contact_point - m_basePos;
	const btlVector3 &normal_lin_world = normal_lin;							//convenience
	const btlVector3 &normal_ang_world = normal_ang;

    rot_from_world[0] = btlMatrix3x3(m_baseQuat);    
    
    // omega coeffients first.
    btlVector3 omega_coeffs_world;
    omega_coeffs_world = p_minus_com_world.cross(normal_lin_world);
	jac[0] = omega_coeffs_world[0] + normal_ang_world[0];
	jac[1] = omega_coeffs_world[1] + normal_ang_world[1];
	jac[2] = omega_coeffs_world[2] + normal_ang_world[2];
    // then v coefficients
    jac[3] = normal_lin_world[0];
    jac[4] = normal_lin_world[1];
    jac[5] = normal_lin_world[2];

	//create link-local versions of p_minus_com and normal
	p_minus_com_local[0] = rot_from_world[0] * p_minus_com_world;
    n_local_lin[0] = rot_from_world[0] * normal_lin_world;
	n_local_ang[0] = rot_from_world[0] * normal_ang_world;

    // Set remaining jac values to zero for now.
    for (int i = 6; i < 6 + m_dofCount; ++i) 
	{
        jac[i] = 0;
    }

    // Qdot coefficients, if necessary.
    if (num_links > 0 && link > -1) {

        // TODO: speed this up -- don't calculate for m_links we don't need.
        // (Also, we are making 3 separate calls to this function, for the normal & the 2 friction directions,
        // which is resulting in repeated work being done...)

        // calculate required normals & positions in the local frames.
        for (int i = 0; i < num_links; ++i) {

            // transform to local frame
            const int parent = m_links[i].m_parent;
            const btlMatrix3x3 mtx(m_links[i].m_cachedRotParentToThis);
            rot_from_world[i+1] = mtx * rot_from_world[parent+1];

            n_local_lin[i+1] = mtx * n_local_lin[parent+1];
			n_local_ang[i+1] = mtx * n_local_ang[parent+1];
            p_minus_com_local[i+1] = mtx * p_minus_com_local[parent+1] - m_links[i].m_cachedRVector;

			// calculate the jacobian entry
			switch(m_links[i].m_jointType)
			{
				case btlMultibodyLink::eRevolute:
				{
					results[m_links[i].m_dofOffset] = n_local_lin[i+1].dot(m_links[i].getAxisTop(0).cross(p_minus_com_local[i+1]) + m_links[i].getAxisBottom(0));
					results[m_links[i].m_dofOffset] += n_local_ang[i+1].dot(m_links[i].getAxisTop(0));
					break;
				}
				case btlMultibodyLink::ePrismatic:
				{
					results[m_links[i].m_dofOffset] = n_local_lin[i+1].dot(m_links[i].getAxisBottom(0));
					break;
				}
				case btlMultibodyLink::eSpherical:
				{
					results[m_links[i].m_dofOffset + 0] = n_local_lin[i+1].dot(m_links[i].getAxisTop(0).cross(p_minus_com_local[i+1]) + m_links[i].getAxisBottom(0));
					results[m_links[i].m_dofOffset + 1] = n_local_lin[i+1].dot(m_links[i].getAxisTop(1).cross(p_minus_com_local[i+1]) + m_links[i].getAxisBottom(1));
					results[m_links[i].m_dofOffset + 2] = n_local_lin[i+1].dot(m_links[i].getAxisTop(2).cross(p_minus_com_local[i+1]) + m_links[i].getAxisBottom(2));
										
					results[m_links[i].m_dofOffset + 0] += n_local_ang[i+1].dot(m_links[i].getAxisTop(0));
					results[m_links[i].m_dofOffset + 1] += n_local_ang[i+1].dot(m_links[i].getAxisTop(1));
					results[m_links[i].m_dofOffset + 2] += n_local_ang[i+1].dot(m_links[i].getAxisTop(2));

					break;
				}
				case btlMultibodyLink::ePlanar:
				{
					results[m_links[i].m_dofOffset + 0] = n_local_lin[i+1].dot(m_links[i].getAxisTop(0).cross(p_minus_com_local[i+1]));// + m_links[i].getAxisBottom(0));
					results[m_links[i].m_dofOffset + 1] = n_local_lin[i+1].dot(m_links[i].getAxisBottom(1));
					results[m_links[i].m_dofOffset + 2] = n_local_lin[i+1].dot(m_links[i].getAxisBottom(2));

					break;
				}
				default:
				{
				}
			}
            
        }

        // Now copy through to output.
		//printf("jac[%d] = ", link);
        while (link != -1) 
		{
			for(int dof = 0; dof < m_links[link].m_dofCount; ++dof)
			{
				jac[6 + m_links[link].m_dofOffset + dof] = results[m_links[link].m_dofOffset + dof];
				//printf("%.2f\t", jac[6 + m_links[link].m_dofOffset + dof]);
			}
            
			link = m_links[link].m_parent;
        }
		//printf("]\n");
    }
}


void btlMultiBody::wakeUp()
{
    m_awake = true;
}

void btlMultiBody::goToSleep()
{
    m_awake = false;
}

void btlMultiBody::checkMotionAndSleepIfRequired(btlScalar timestep)
{
	extern bool btl_gDisableDeactivation;
    if (!m_canSleep || btl_gDisableDeactivation) 
	{
		m_awake = true;
		m_sleepTimer = 0;
		return;
	}

    // motion is computed as omega^2 + v^2 + (sum of squares of joint velocities)
    btlScalar motion = 0;
	{
		for (int i = 0; i < 6 + m_dofCount; ++i) 		
			motion += m_realBuf[i] * m_realBuf[i];
	}
	

    if (motion < SLEEP_EPSILON) {
        m_sleepTimer += timestep;
        if (m_sleepTimer > SLEEP_TIMEOUT) {
            goToSleep();
        }
    } else {
        m_sleepTimer = 0;
		if (!m_awake)
			wakeUp();
    }
}


void	btlMultiBody::forwardKinematics(btlAlignedObjectArray<btlQuaternion>& world_to_local,btlAlignedObjectArray<btlVector3>& local_origin)
{
	
	int num_links = getNumLinks();

	// Cached 3x3 rotation matrices from parent frame to this frame.
	btlMatrix3x3* rot_from_parent =(btlMatrix3x3 *) &m_matrixBuf[0];

	rot_from_parent[0] = btlMatrix3x3(m_baseQuat);				//m_baseQuat assumed to be alias!?
	
	for (int i = 0; i < num_links; ++i) 
	{
		rot_from_parent[i+1] = btlMatrix3x3(m_links[i].m_cachedRotParentToThis);
	}
		
	int nLinks = getNumLinks();
	///base + num m_links
	world_to_local.resize(nLinks+1);
	local_origin.resize(nLinks+1);

	world_to_local[0] = getWorldToBaseRot();
	local_origin[0] = getBasePos();
	
	for (int k=0;k<getNumLinks();k++)
	{
		const int parent = getParent(k);
		world_to_local[k+1] = getParentToLocalRot(k) * world_to_local[parent+1];
		local_origin[k+1] = local_origin[parent+1] + (btl_quatRotate(world_to_local[k+1].inverse() , getRVector(k)));
	}

	for (int link=0;link<getNumLinks();link++)
	{
		int index = link+1;

		btlVector3 posr = local_origin[index];
		btlScalar quat[4]={-world_to_local[index].x(),-world_to_local[index].y(),-world_to_local[index].z(),world_to_local[index].w()};
		btlTransform tr;
		tr.setIdentity();
		tr.setOrigin(posr);
		tr.setRotation(btlQuaternion(quat[0],quat[1],quat[2],quat[3]));
		getLink(link).m_cachedWorldTransform = tr;
			
	}

}

void	btlMultiBody::updateCollisionObjectWorldTransforms(btlAlignedObjectArray<btlQuaternion>& world_to_local,btlAlignedObjectArray<btlVector3>& local_origin)
{
	world_to_local.resize(getNumLinks()+1);
	local_origin.resize(getNumLinks()+1);
	
	world_to_local[0] = getWorldToBaseRot();
	local_origin[0] = getBasePos();
	
	if (getBaseCollider())
	{
		btlVector3 posr = local_origin[0];
		//	float pos[4]={posr.x(),posr.y(),posr.z(),1};
		btlScalar quat[4]={-world_to_local[0].x(),-world_to_local[0].y(),-world_to_local[0].z(),world_to_local[0].w()};
		btlTransform tr;
		tr.setIdentity();
		tr.setOrigin(posr);
		tr.setRotation(btlQuaternion(quat[0],quat[1],quat[2],quat[3]));
		
		getBaseCollider()->setWorldTransform(tr);
		
	}
	
	for (int k=0;k<getNumLinks();k++)
	{
		const int parent = getParent(k);
		world_to_local[k+1] = getParentToLocalRot(k) * world_to_local[parent+1];
		local_origin[k+1] = local_origin[parent+1] + (btl_quatRotate(world_to_local[k+1].inverse() , getRVector(k)));
	}
	
	
	for (int m=0;m<getNumLinks();m++)
	{
		btlMultiBodyLinkCollider* col = getLink(m).m_collider;
		if (col)
		{
			int link = col->m_link;
			btlAssert(link == m);
			
			int index = link+1;
			
			btlVector3 posr = local_origin[index];
			//			float pos[4]={posr.x(),posr.y(),posr.z(),1};
			btlScalar quat[4]={-world_to_local[index].x(),-world_to_local[index].y(),-world_to_local[index].z(),world_to_local[index].w()};
			btlTransform tr;
			tr.setIdentity();
			tr.setOrigin(posr);
			tr.setRotation(btlQuaternion(quat[0],quat[1],quat[2],quat[3]));
			
			col->setWorldTransform(tr);
		}
	}
}

int	btlMultiBody::calculateSerializeBufferSize()	const
{
	int sz = sizeof(btlMultiBodyData);
	return sz;
}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
const char*	btlMultiBody::serialize(void* dataBuffer, class btlSerializer* serializer) const
{
		btlMultiBodyData* mbd = (btlMultiBodyData*) dataBuffer;
		getBaseWorldTransform().serialize(mbd->m_baseWorldTransform);
		mbd->m_baseMass = this->getBaseMass();
		getBaseInertia().serialize(mbd->m_baseInertia);
		{
			char* name = (char*) serializer->findNameForPointer(m_baseName);
			mbd->m_baseName = (char*)serializer->getUniquePointer(name);
			if (mbd->m_baseName)
			{
				serializer->serializeName(name);
			}
		}
		mbd->m_numLinks = this->getNumLinks();
		if (mbd->m_numLinks)
		{
			int sz = sizeof(btlMultiBodyLinkData);
			int numElem = mbd->m_numLinks;
			btlChunk* chunk = serializer->allocate(sz,numElem);
			btlMultiBodyLinkData* memPtr = (btlMultiBodyLinkData*)chunk->m_oldPtr;
			for (int i=0;i<numElem;i++,memPtr++)
			{

				memPtr->m_jointType = getLink(i).m_jointType;
				memPtr->m_dofCount = getLink(i).m_dofCount;
				memPtr->m_posVarCount = getLink(i).m_posVarCount;
				
				getLink(i).m_inertiaLocal.serialize(memPtr->m_linkInertia);
				memPtr->m_linkMass = getLink(i).m_mass;
				memPtr->m_parentIndex = getLink(i).m_parent;
				getLink(i).m_eVector.serialize(memPtr->m_parentComToThisComOffset);
				getLink(i).m_dVector.serialize(memPtr->m_thisPivotToThisComOffset);
				getLink(i).m_zeroRotParentToThis.serialize(memPtr->m_zeroRotParentToThis);
				btlAssert(memPtr->m_dofCount<=3);
				for (int dof = 0;dof<getLink(i).m_dofCount;dof++)
				{
					getLink(i).getAxisBottom(dof).serialize(memPtr->m_jointAxisBottom[dof]);
					getLink(i).getAxisTop(dof).serialize(memPtr->m_jointAxisTop[dof]);
					
					memPtr->m_jointTorque[dof] = getLink(i).m_jointTorque[dof];
					memPtr->m_jointVel[dof] = getJointVelMultiDof(i)[dof];

				}
				int numPosVar = getLink(i).m_posVarCount;
				for (int posvar = 0; posvar < numPosVar;posvar++)
				{
					memPtr->m_jointPos[posvar] = getLink(i).m_jointPos[posvar];
				}
				
				
				{
					char* name = (char*) serializer->findNameForPointer(m_links[i].m_linkName);
					memPtr->m_linkName = (char*)serializer->getUniquePointer(name);
					if (memPtr->m_linkName)
					{
						serializer->serializeName(name);
					}
				}
				{
					char* name = (char*) serializer->findNameForPointer(m_links[i].m_jointName);
					memPtr->m_jointName = (char*)serializer->getUniquePointer(name);
					if (memPtr->m_jointName)
					{
						serializer->serializeName(name);
					}
				}
				memPtr->m_linkCollider = (btlCollisionObjectData*)serializer->getUniquePointer(getLink(i).m_collider);

			}
			serializer->finalizeChunk(chunk,btlMultiBodyLinkDataName,BT_ARRAY_CODE,(void*) &m_links[0]);
		}
		mbd->m_links = mbd->m_numLinks? (btlMultiBodyLinkData*) serializer->getUniquePointer((void*)&m_links[0]):0;

		return btlMultiBodyDataName;
}
