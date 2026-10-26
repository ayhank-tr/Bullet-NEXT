/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_SOFT_RIGID_DYNAMICS_WORLD_H
#define BT_SOFT_RIGID_DYNAMICS_WORLD_H

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "btSoftBody.h"

typedef	btlAlignedObjectArray<btlSoftBody*> btlSoftBodyArray;

class btlSoftBodySolver;

class btlSoftRigidDynamicsWorld : public btlDiscreteDynamicsWorld
{

	btlSoftBodyArray	m_softBodies;
	int				m_drawFlags;
	bool			m_drawNodeTree;
	bool			m_drawFaceTree;
	bool			m_drawClusterTree;
	btlSoftBodyWorldInfo m_sbi;
	///Solver classes that encapsulate multiple soft bodies for solving
	btlSoftBodySolver *m_softBodySolver;
	bool			m_ownsSolver;

protected:

	virtual void	predictUnconstraintMotion(btlScalar timeStep);

	virtual void	internalSingleStepSimulation( btlScalar timeStep);

	void	solveSoftBodiesConstraints( btlScalar timeStep );

	void	serializeSoftBodies(btlSerializer* serializer);

public:

	btlSoftRigidDynamicsWorld(btlDispatcher* dispatcher,btlBroadphaseInterface* pairCache,btlConstraintSolver* constraintSolver, btlCollisionConfiguration* collisionConfiguration, btlSoftBodySolver *softBodySolver = 0 );

	virtual ~btlSoftRigidDynamicsWorld();

	virtual void	debugDrawWorld();

	void	addSoftBody(btlSoftBody* body,short int collisionFilterGroup=btlBroadphaseProxy::DefaultFilter,short int collisionFilterMask=btlBroadphaseProxy::AllFilter);

	void	removeSoftBody(btlSoftBody* body);

	///removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btDiscreteDynamicsWorld::removeCollisionObject
	virtual void	removeCollisionObject(btlCollisionObject* collisionObject);

	int		getDrawFlags() const { return(m_drawFlags); }
	void	setDrawFlags(int f)	{ m_drawFlags=f; }

	btlSoftBodyWorldInfo&	getWorldInfo()
	{
		return m_sbi;
	}
	const btlSoftBodyWorldInfo&	getWorldInfo() const
	{
		return m_sbi;
	}

	virtual btlDynamicsWorldType	getWorldType() const
	{
		return	BT_SOFT_RIGID_DYNAMICS_WORLD;
	}

	btlSoftBodyArray& getSoftBodyArray()
	{
		return m_softBodies;
	}

	const btlSoftBodyArray& getSoftBodyArray() const
	{
		return m_softBodies;
	}


	virtual void rayTest(const btlVector3& rayFromWorld, const btlVector3& rayToWorld, RayResultCallback& resultCallback) const; 

	/// rayTestSingle performs a raycast call and calls the resultCallback. It is used internally by rayTest.
	/// In a future implementation, we consider moving the ray test as a virtual method in btCollisionShape.
	/// This allows more customization.
	static void	rayTestSingle(const btlTransform& rayFromTrans,const btlTransform& rayToTrans,
					  btlCollisionObject* collisionObject,
					  const btlCollisionShape* collisionShape,
					  const btlTransform& colObjWorldTransform,
					  RayResultCallback& resultCallback);

	virtual	void	serialize(btlSerializer* serializer);

};

#endif //BT_SOFT_RIGID_DYNAMICS_WORLD_H
