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


#ifndef BT_MULTIBODY_H
#define BT_MULTIBODY_H

#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btAlignedObjectArray.h"


///serialization data, don't change them if you are not familiar with the details of the serialization mechanisms
#ifdef BT_USE_DOUBLE_PRECISION
	#define btlMultiBodyData	btlMultiBodyDoubleData
	#define btlMultiBodyDataName	"btMultiBodyDoubleData"
	#define btlMultiBodyLinkData btlMultiBodyLinkDoubleData
	#define btlMultiBodyLinkDataName	"btMultiBodyLinkDoubleData"
#else
	#define btlMultiBodyData	btlMultiBodyFloatData
	#define btlMultiBodyDataName	"btMultiBodyFloatData"
	#define btlMultiBodyLinkData btlMultiBodyLinkFloatData
	#define btlMultiBodyLinkDataName	"btMultiBodyLinkFloatData"
#endif //BT_USE_DOUBLE_PRECISION

#include "btMultiBodyLink.h"
class btlMultiBodyLinkCollider;

ATTRIBUTE_ALIGNED16(class) btlMultiBody 
{
public:

    
	BT_DECLARE_ALIGNED_ALLOCATOR();

    //
    // initialization
    //
    
	btlMultiBody(int n_links,             // NOT including the base
		btlScalar mass,                // mass of base
		const btlVector3 &inertia,    // inertia of base, in base frame; assumed diagonal
		bool fixedBase,           // whether the base is fixed (true) or can move (false)
		bool canSleep, bool deprecatedMultiDof=true);


	virtual ~btlMultiBody();
    
	//note: fixed link collision with parent is always disabled
	void setupFixed(int linkIndex,
						   btlScalar mass,
						   const btlVector3 &inertia,
						   int parent,
						   const btlQuaternion &rotParentToThis,
						   const btlVector3 &parentComToThisPivotOffset,
                           const btlVector3 &thisPivotToThisComOffset, bool deprecatedDisableParentCollision=true);

						
	void setupPrismatic(int i,
                               btlScalar mass,
                               const btlVector3 &inertia,
                               int parent,
                               const btlQuaternion &rotParentToThis,
                               const btlVector3 &jointAxis,
                               const btlVector3 &parentComToThisPivotOffset,
							   const btlVector3 &thisPivotToThisComOffset,
							   bool disableParentCollision);

    void setupRevolute(int linkIndex,            // 0 to num_links-1
                       btlScalar mass,
                       const btlVector3 &inertia,
                       int parentIndex,
                       const btlQuaternion &rotParentToThis,  // rotate points in parent frame to this frame, when q = 0
                       const btlVector3 &jointAxis,    // in my frame
                       const btlVector3 &parentComToThisPivotOffset,    // vector from parent COM to joint axis, in PARENT frame
                       const btlVector3 &thisPivotToThisComOffset,       // vector from joint axis to my COM, in MY frame
					   bool disableParentCollision=false);

	void setupSpherical(int linkIndex,											// 0 to num_links-1
                       btlScalar mass,
                       const btlVector3 &inertia,
                       int parent,
                       const btlQuaternion &rotParentToThis,		// rotate points in parent frame to this frame, when q = 0                       
                       const btlVector3 &parentComToThisPivotOffset,			// vector from parent COM to joint axis, in PARENT frame
                       const btlVector3 &thisPivotToThisComOffset,				// vector from joint axis to my COM, in MY frame
					   bool disableParentCollision=false);		

	void setupPlanar(int i,											// 0 to num_links-1
                       btlScalar mass,
                       const btlVector3 &inertia,
                       int parent,
                       const btlQuaternion &rotParentToThis,		// rotate points in parent frame to this frame, when q = 0                       
					   const btlVector3 &rotationAxis,
                       const btlVector3 &parentComToThisComOffset,			// vector from parent COM to this COM, in PARENT frame                       
					   bool disableParentCollision=false);		
	
	const btlMultibodyLink& getLink(int index) const
	{
		return m_links[index];
	}

	btlMultibodyLink& getLink(int index)
	{
		return m_links[index];
	}


	void setBaseCollider(btlMultiBodyLinkCollider* collider)//collider can be NULL to disable collision for the base
	{
		m_baseCollider = collider;
	}
	const btlMultiBodyLinkCollider* getBaseCollider() const
	{
		return m_baseCollider;
	}
	btlMultiBodyLinkCollider* getBaseCollider()
	{
		return m_baseCollider;
	}

    //
    // get parent
    // input: link num from 0 to num_links-1
    // output: link num from 0 to num_links-1, OR -1 to mean the base.
    //
    int getParent(int link_num) const;
    
    
    //
    // get number of m_links, masses, moments of inertia
    //

    int getNumLinks() const { return m_links.size(); }
	int getNumDofs() const { return m_dofCount; }
	int getNumPosVars() const { return m_posVarCnt; }
    btlScalar getBaseMass() const { return m_baseMass; }
    const btlVector3 & getBaseInertia() const { return m_baseInertia; }
    btlScalar getLinkMass(int i) const;
    const btlVector3 & getLinkInertia(int i) const;
    
	

    //
    // change mass (incomplete: can only change base mass and inertia at present)
    //

    void setBaseMass(btlScalar mass) { m_baseMass = mass; }
    void setBaseInertia(const btlVector3 &inertia) { m_baseInertia = inertia; }


    //
    // get/set pos/vel/rot/omega for the base link
    //

    const btlVector3 & getBasePos() const { return m_basePos; }    // in world frame
    const btlVector3 getBaseVel() const 
	{ 
		return btlVector3(m_realBuf[3],m_realBuf[4],m_realBuf[5]); 
	}     // in world frame
    const btlQuaternion & getWorldToBaseRot() const 
	{ 
		return m_baseQuat; 
	}     // rotates world vectors into base frame
    btlVector3 getBaseOmega() const { return btlVector3(m_realBuf[0],m_realBuf[1],m_realBuf[2]); }   // in world frame

    void setBasePos(const btlVector3 &pos) 
	{ 
		m_basePos = pos; 
	}

	void setBaseWorldTransform(const btlTransform& tr)
	{
		setBasePos(tr.getOrigin());
		setWorldToBaseRot(tr.getRotation().inverse());

	}

	btlTransform getBaseWorldTransform() const
	{
		btlTransform tr;
		tr.setOrigin(getBasePos());
		tr.setRotation(getWorldToBaseRot().inverse());
		return tr;
	}

    void setBaseVel(const btlVector3 &vel) 
	{ 

		m_realBuf[3]=vel[0]; m_realBuf[4]=vel[1]; m_realBuf[5]=vel[2]; 
	}
    void setWorldToBaseRot(const btlQuaternion &rot) 
	{ 
		m_baseQuat = rot;					//m_baseQuat asumed to ba alias!?
	}
    void setBaseOmega(const btlVector3 &omega) 
	{ 
		m_realBuf[0]=omega[0]; 
		m_realBuf[1]=omega[1]; 
		m_realBuf[2]=omega[2]; 
	}


    //
    // get/set pos/vel for child m_links (i = 0 to num_links-1)
    //

    btlScalar getJointPos(int i) const;
    btlScalar getJointVel(int i) const;

	btlScalar * getJointVelMultiDof(int i);
	btlScalar * getJointPosMultiDof(int i);

	const btlScalar * getJointVelMultiDof(int i) const ;
	const btlScalar * getJointPosMultiDof(int i) const ;

    void setJointPos(int i, btlScalar q);
    void setJointVel(int i, btlScalar qdot);
	void setJointPosMultiDof(int i, btlScalar *q);
    void setJointVelMultiDof(int i, btlScalar *qdot);	



    //
    // direct access to velocities as a vector of 6 + num_links elements.
    // (omega first, then v, then joint velocities.)
    //
    const btlScalar * getVelocityVector() const 
	{ 
		return &m_realBuf[0]; 
	}
/*    btScalar * getVelocityVector() 
	{ 
		return &real_buf[0]; 
	}
  */  

    //
    // get the frames of reference (positions and orientations) of the child m_links
    // (i = 0 to num_links-1)
    //

    const btlVector3 & getRVector(int i) const;   // vector from COM(parent(i)) to COM(i), in frame i's coords
    const btlQuaternion & getParentToLocalRot(int i) const;   // rotates vectors in frame parent(i) to vectors in frame i.


    //
    // transform vectors in local frame of link i to world frame (or vice versa)
    //
    btlVector3 localPosToWorld(int i, const btlVector3 &vec) const;
    btlVector3 localDirToWorld(int i, const btlVector3 &vec) const;
    btlVector3 worldPosToLocal(int i, const btlVector3 &vec) const;
    btlVector3 worldDirToLocal(int i, const btlVector3 &vec) const;
    

    //
    // calculate kinetic energy and angular momentum
    // useful for debugging.
    //

    btlScalar getKineticEnergy() const;
    btlVector3 getAngularMomentum() const;
    

    //
    // set external forces and torques. Note all external forces/torques are given in the WORLD frame.
    //

    void clearForcesAndTorques();
   void clearConstraintForces();

	void clearVelocities();

    void addBaseForce(const btlVector3 &f) 
	{ 
		m_baseForce += f; 
	}
    void addBaseTorque(const btlVector3 &t) { m_baseTorque += t; }
    void addLinkForce(int i, const btlVector3 &f);
    void addLinkTorque(int i, const btlVector3 &t);

 void addBaseConstraintForce(const btlVector3 &f)
        {
                m_baseConstraintForce += f;
        }
    void addBaseConstraintTorque(const btlVector3 &t) { m_baseConstraintTorque += t; }
    void addLinkConstraintForce(int i, const btlVector3 &f);
    void addLinkConstraintTorque(int i, const btlVector3 &t);
       

void addJointTorque(int i, btlScalar Q);
	void addJointTorqueMultiDof(int i, int dof, btlScalar Q);
	void addJointTorqueMultiDof(int i, const btlScalar *Q);

    const btlVector3 & getBaseForce() const { return m_baseForce; }
    const btlVector3 & getBaseTorque() const { return m_baseTorque; }
    const btlVector3 & getLinkForce(int i) const;
    const btlVector3 & getLinkTorque(int i) const;
    btlScalar getJointTorque(int i) const;
	btlScalar * getJointTorqueMultiDof(int i);


    //
    // dynamics routines.
    //

    // timestep the velocities (given the external forces/torques set using addBaseForce etc).
    // also sets up caches for calcAccelerationDeltas.
    //
    // Note: the caller must provide three vectors which are used as
    // temporary scratch space. The idea here is to reduce dynamic
    // memory allocation: the same scratch vectors can be re-used
    // again and again for different Multibodies, instead of each
    // btMultiBody allocating (and then deallocating) their own
    // individual scratch buffers. This gives a considerable speed
    // improvement, at least on Windows (where dynamic memory
    // allocation appears to be fairly slow).
    //
   
 
	void computeAccelerationsArticulatedBodyAlgorithmMultiDof(btlScalar dt,
                        btlAlignedObjectArray<btlScalar> &scratch_r,
                        btlAlignedObjectArray<btlVector3> &scratch_v,
                        btlAlignedObjectArray<btlMatrix3x3> &scratch_m,
			bool isConstraintPass=false
		);

///stepVelocitiesMultiDof is deprecated, use computeAccelerationsArticulatedBodyAlgorithmMultiDof instead
        void stepVelocitiesMultiDof(btlScalar dt,
                        btlAlignedObjectArray<btlScalar> &scratch_r,
                        btlAlignedObjectArray<btlVector3> &scratch_v,
                        btlAlignedObjectArray<btlMatrix3x3> &scratch_m,
                        bool isConstraintPass=false)
	{
		computeAccelerationsArticulatedBodyAlgorithmMultiDof(dt,scratch_r,scratch_v,scratch_m,isConstraintPass);
        }

    // calcAccelerationDeltasMultiDof
    // input: force vector (in same format as jacobian, i.e.:
    //                      3 torque values, 3 force values, num_links joint torque values)
    // output: 3 omegadot values, 3 vdot values, num_links q_double_dot values
    // (existing contents of output array are replaced)
    // calcAccelerationDeltasMultiDof must have been called first.
	void calcAccelerationDeltasMultiDof(const btlScalar *force, btlScalar *output,
                                btlAlignedObjectArray<btlScalar> &scratch_r,
                                btlAlignedObjectArray<btlVector3> &scratch_v) const;
	
  
	void applyDeltaVeeMultiDof2(const btlScalar * delta_vee, btlScalar multiplier)
	{
		for (int dof = 0; dof < 6 + getNumDofs(); ++dof)
                {
                        m_deltaV[dof] += delta_vee[dof] * multiplier;
                }
	}
	void processDeltaVeeMultiDof2()
	{
		applyDeltaVeeMultiDof(&m_deltaV[0],1);

		for (int dof = 0; dof < 6 + getNumDofs(); ++dof)
                {
			m_deltaV[dof] = 0.f;
		}
	}

	void applyDeltaVeeMultiDof(const btlScalar * delta_vee, btlScalar multiplier) 
	{
		//for (int dof = 0; dof < 6 + getNumDofs(); ++dof)
		//	printf("%.4f ", delta_vee[dof]*multiplier);
		//printf("\n");

		//btScalar sum = 0;
		//for (int dof = 0; dof < 6 + getNumDofs(); ++dof)
		//{
		//	sum += delta_vee[dof]*multiplier*delta_vee[dof]*multiplier;
		//}
		//btScalar l = btSqrt(sum);

		//if (l>m_maxAppliedImpulse)
		//{
		//	multiplier *= m_maxAppliedImpulse/l;
		//}

		for (int dof = 0; dof < 6 + getNumDofs(); ++dof)
		{
			m_realBuf[dof] += delta_vee[dof] * multiplier;
			btlClamp(m_realBuf[dof],-m_maxCoordinateVelocity,m_maxCoordinateVelocity);
		}
    }

	
	
    // timestep the positions (given current velocities).
	void stepPositionsMultiDof(btlScalar dt, btlScalar *pq = 0, btlScalar *pqd = 0);


    //
    // contacts
    //

    // This routine fills out a contact constraint jacobian for this body.
    // the 'normal' supplied must be -n for body1 or +n for body2 of the contact.
    // 'normal' & 'contact_point' are both given in world coordinates.
	
	void fillContactJacobianMultiDof(int link,
                             const btlVector3 &contact_point,
                             const btlVector3 &normal,
                             btlScalar *jac,
                             btlAlignedObjectArray<btlScalar> &scratch_r,
                             btlAlignedObjectArray<btlVector3> &scratch_v,
							 btlAlignedObjectArray<btlMatrix3x3> &scratch_m) const { fillConstraintJacobianMultiDof(link, contact_point, btlVector3(0, 0, 0), normal, jac, scratch_r, scratch_v, scratch_m); }

	//a more general version of fillContactJacobianMultiDof which does not assume..
	//.. that the constraint in question is contact or, to be more precise, constrains linear velocity only
	void fillConstraintJacobianMultiDof(int link,
                             const btlVector3 &contact_point,
							 const btlVector3 &normal_ang,
                             const btlVector3 &normal_lin,
                             btlScalar *jac,
                             btlAlignedObjectArray<btlScalar> &scratch_r,
                             btlAlignedObjectArray<btlVector3> &scratch_v,
                             btlAlignedObjectArray<btlMatrix3x3> &scratch_m) const;


    //
    // sleeping
    //
	void	setCanSleep(bool canSleep)
	{
		m_canSleep = canSleep;
	}

	bool getCanSleep()const
	{
		return m_canSleep;
	}

    bool isAwake() const { return m_awake; }
    void wakeUp();
    void goToSleep();
    void checkMotionAndSleepIfRequired(btlScalar timestep);
    
	bool hasFixedBase() const
	{
		    return m_fixedBase;
	}

	int getCompanionId() const
	{
		return m_companionId;
	}
	void setCompanionId(int id)
	{
		//printf("for %p setCompanionId(%d)\n",this, id);
		m_companionId = id;
	}

	void setNumLinks(int numLinks)//careful: when changing the number of m_links, make sure to re-initialize or update existing m_links
	{
		m_links.resize(numLinks);
	}

	btlScalar getLinearDamping() const
	{
			return m_linearDamping;
	}
	void setLinearDamping( btlScalar damp)
	{
		m_linearDamping = damp;
	}
	btlScalar getAngularDamping() const
	{
		return m_angularDamping;
	}
	void setAngularDamping( btlScalar damp)
	{
		m_angularDamping = damp;
	}
		
	bool getUseGyroTerm() const
	{
		return m_useGyroTerm;
	}
	void setUseGyroTerm(bool useGyro)
	{
		m_useGyroTerm = useGyro;
	}
	btlScalar	getMaxCoordinateVelocity() const
	{
		return m_maxCoordinateVelocity ;
	}
	void	setMaxCoordinateVelocity(btlScalar maxVel)
	{
		m_maxCoordinateVelocity = maxVel;
	}

	btlScalar	getMaxAppliedImpulse() const
	{
		return m_maxAppliedImpulse;
	}
	void	setMaxAppliedImpulse(btlScalar maxImp)
	{
		m_maxAppliedImpulse = maxImp;
	}
	void	setHasSelfCollision(bool hasSelfCollision)
	{
		m_hasSelfCollision = hasSelfCollision;
	}
	bool hasSelfCollision() const
	{
		return m_hasSelfCollision;
	}

	
	void finalizeMultiDof();

	void useRK4Integration(bool use) { m_useRK4 = use; }
	bool isUsingRK4Integration() const { return m_useRK4; }
	void useGlobalVelocities(bool use) { m_useGlobalVelocities = use; }
	bool isUsingGlobalVelocities() const { return m_useGlobalVelocities; }

	bool isPosUpdated() const
	{
		return __posUpdated;
	}
	void setPosUpdated(bool updated)
	{
		__posUpdated = updated;
	}
	
	//internalNeedsJointFeedback is for internal use only
	bool internalNeedsJointFeedback() const
	{
		return m_internalNeedsJointFeedback;
	}
	void	forwardKinematics(btlAlignedObjectArray<btlQuaternion>& scratch_q,btlAlignedObjectArray<btlVector3>& scratch_m);

	void	updateCollisionObjectWorldTransforms(btlAlignedObjectArray<btlQuaternion>& scratch_q,btlAlignedObjectArray<btlVector3>& scratch_m);
	
	virtual	int	calculateSerializeBufferSize()	const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	const char*	serialize(void* dataBuffer,  class btlSerializer* serializer) const;

	const char*				getBaseName() const
	{
		return m_baseName;
	}
	///memory of setBaseName needs to be manager by user
	void	setBaseName(const char* name)
	{
		m_baseName = name;
	}

private:
    btlMultiBody(const btlMultiBody &);  // not implemented
    void operator=(const btlMultiBody &);  // not implemented

    void compTreeLinkVelocities(btlVector3 *omega, btlVector3 *vel) const;

	void solveImatrix(const btlVector3& rhs_top, const btlVector3& rhs_bot, float result[6]) const;
	void solveImatrix(const btlSpatialForceVector &rhs, btlSpatialMotionVector &result) const;
	
	void updateLinksDofOffsets()
	{
		int dofOffset = 0, cfgOffset = 0;
		for(int bidx = 0; bidx < m_links.size(); ++bidx)
		{
			m_links[bidx].m_dofOffset = dofOffset; m_links[bidx].m_cfgOffset = cfgOffset;
			dofOffset += m_links[bidx].m_dofCount; cfgOffset += m_links[bidx].m_posVarCount;
		}
	}

	void mulMatrix(btlScalar *pA, btlScalar *pB, int rowsA, int colsA, int rowsB, int colsB, btlScalar *pC) const;
	
	
private:

	btlMultiBodyLinkCollider* m_baseCollider;//can be NULL
	const char*				m_baseName;//memory needs to be manager by user!

    btlVector3 m_basePos;       // position of COM of base (world frame)
    btlQuaternion m_baseQuat;   // rotates world points into base frame

    btlScalar m_baseMass;         // mass of the base
    btlVector3 m_baseInertia;   // inertia of the base (in local frame; diagonal)

    btlVector3 m_baseForce;     // external force applied to base. World frame.
    btlVector3 m_baseTorque;    // external torque applied to base. World frame.
   
    btlVector3 m_baseConstraintForce;     // external force applied to base. World frame.
    btlVector3 m_baseConstraintTorque;    // external torque applied to base. World frame.
 
    btlAlignedObjectArray<btlMultibodyLink> m_links;    // array of m_links, excluding the base. index from 0 to num_links-1.
	btlAlignedObjectArray<btlMultiBodyLinkCollider*> m_colliders;

    
    //
    // realBuf:
    //  offset         size            array
    //   0              6 + num_links   v (base_omega; base_vel; joint_vels)					MULTIDOF [sysdof x sysdof for D matrices (TOO MUCH!) + pos_delta which is sys-cfg sized]
    //   6+num_links    num_links       D
    //
    // vectorBuf:
    //  offset         size         array
    //   0              num_links    h_top
    //   num_links      num_links    h_bottom
    //
    // matrixBuf:
    //  offset         size         array
    //   0              num_links+1  rot_from_parent
    //
   btlAlignedObjectArray<btlScalar> m_deltaV; 
    btlAlignedObjectArray<btlScalar> m_realBuf;
    btlAlignedObjectArray<btlVector3> m_vectorBuf;
    btlAlignedObjectArray<btlMatrix3x3> m_matrixBuf;


	btlMatrix3x3 m_cachedInertiaTopLeft;
	btlMatrix3x3 m_cachedInertiaTopRight;
	btlMatrix3x3 m_cachedInertiaLowerLeft;
	btlMatrix3x3 m_cachedInertiaLowerRight;

    bool m_fixedBase;

    // Sleep parameters.
    bool m_awake;
    bool m_canSleep;
    btlScalar m_sleepTimer;

	int	m_companionId;
	btlScalar	m_linearDamping;
	btlScalar	m_angularDamping;
	bool	m_useGyroTerm;
	btlScalar	m_maxAppliedImpulse;
	btlScalar	m_maxCoordinateVelocity;
	bool		m_hasSelfCollision;
	
		bool __posUpdated;
		int m_dofCount, m_posVarCnt;
	bool m_useRK4, m_useGlobalVelocities;
	
	///the m_needsJointFeedback gets updated/computed during the stepVelocitiesMultiDof and it for internal usage only
	bool m_internalNeedsJointFeedback;
};

struct btlMultiBodyLinkDoubleData
{
	btlQuaternionDoubleData	m_zeroRotParentToThis;
	btlVector3DoubleData		m_parentComToThisComOffset;
	btlVector3DoubleData		m_thisPivotToThisComOffset;
	btlVector3DoubleData		m_jointAxisTop[6];
	btlVector3DoubleData		m_jointAxisBottom[6];


	char					*m_linkName;
	char					*m_jointName;
	btlCollisionObjectDoubleData	*m_linkCollider;
	
	btlVector3DoubleData		m_linkInertia;   // inertia of the base (in local frame; diagonal)
	double					m_linkMass;
	int						m_parentIndex;
	int						m_jointType;
	

	

	int						m_dofCount;
	int						m_posVarCount;
	double					m_jointPos[7];
	double					m_jointVel[6];
	double					m_jointTorque[6];
	
	
	
};

struct btlMultiBodyLinkFloatData
{
	btlQuaternionFloatData	m_zeroRotParentToThis;
	btlVector3FloatData		m_parentComToThisComOffset;
	btlVector3FloatData		m_thisPivotToThisComOffset;
	btlVector3FloatData		m_jointAxisTop[6];
	btlVector3FloatData		m_jointAxisBottom[6];
	

	char				*m_linkName;
	char				*m_jointName;
	btlCollisionObjectFloatData	*m_linkCollider;
	
	btlVector3FloatData	m_linkInertia;   // inertia of the base (in local frame; diagonal)
	int						m_dofCount;
	float				m_linkMass;
	int					m_parentIndex;
	int					m_jointType;
	

		
	float					m_jointPos[7];
	float					m_jointVel[6];
	float					m_jointTorque[6];
	int						m_posVarCount;
	

};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btlMultiBodyDoubleData
{
	char	*m_baseName;
	btlMultiBodyLinkDoubleData	*m_links;
	btlCollisionObjectDoubleData	*m_baseCollider;

	btlTransformDoubleData m_baseWorldTransform;
	btlVector3DoubleData m_baseInertia;   // inertia of the base (in local frame; diagonal)
	
	int		m_numLinks;
	double	m_baseMass;

	char m_padding[4];
	
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btlMultiBodyFloatData
{
	char	*m_baseName;
	btlMultiBodyLinkFloatData	*m_links;
	btlCollisionObjectFloatData	*m_baseCollider;
	btlTransformFloatData m_baseWorldTransform;
	btlVector3FloatData m_baseInertia;   // inertia of the base (in local frame; diagonal)
	
	float	m_baseMass;
	int		m_numLinks;
};



#endif
